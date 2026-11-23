#include "http_server.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <fcntl.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/beast.hpp>

namespace web_control
{
HttpResponse HttpResponse::json(nlohmann::json value, unsigned status)
{
  auto result = bytes(std::make_shared<const std::string>(
      value.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace)),
      "application/json; charset=utf-8");
  result.status = status;
  return result;
}

HttpResponse HttpResponse::bytes(std::shared_ptr<const std::string> body, std::string content_type)
{
  HttpResponse result;
  result.body = std::move(body);
  result.content_type = std::move(content_type);
  return result;
}

HttpResponse HttpResponse::stream(std::function<std::string()> next_event, StreamSubscribe subscribe)
{
  HttpResponse result;
  result.next_event = std::move(next_event);
  result.subscribe = std::move(subscribe);
  return result;
}

HttpResponse HttpResponse::file(int descriptor, std::string content_type)
{
  auto result = bytes(nullptr, std::move(content_type));
  result.descriptor = std::shared_ptr<int>(new int(-1), [](int * handle) {
      if (*handle >= 0) {
        ::close(*handle);
      }
      delete handle;
    });
  *result.descriptor = ::fcntl(descriptor, F_DUPFD_CLOEXEC, 0);
  if (*result.descriptor < 0) {
    throw std::runtime_error("could not duplicate file descriptor");
  }
  return result;
}

const std::string & query_value(const Query & query, const std::string & name)
{
  static const std::string empty;
  const auto found = query.find(name);
  return found == query.end() ? empty : found->second;
}

namespace
{
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using asio::ip::tcp;
using namespace std::chrono_literals;

struct NotificationGate
{
  std::mutex mutex;
  bool accepting = true;
};

int hex_value(char ch)
{
  if (ch >= '0' && ch <= '9') {
    return ch - '0';
  }
  if (ch >= 'a' && ch <= 'f') {
    return ch - 'a' + 10;
  }
  if (ch >= 'A' && ch <= 'F') {
    return ch - 'A' + 10;
  }
  throw std::invalid_argument("invalid percent escape");
}

std::string decode(std::string_view input, bool plus_as_space)
{
  std::string output;
  for (std::size_t i = 0; i < input.size(); ++i) {
    char ch = input[i];
    if (ch == '%') {
      if (input.size() - i < 3) {
        throw std::invalid_argument("incomplete percent escape");
      }
      ch = static_cast<char>(16 * hex_value(input[i + 1]) + hex_value(input[i + 2]));
      i += 2;
    } else if (plus_as_space && ch == '+') {
      ch = ' ';
    }
    if (ch == '\0') {
      throw std::invalid_argument("invalid URL");
    }
    output.push_back(ch);
  }
  return output;
}

HttpRequest to_request(const http::request<http::string_body> & request)
{
  HttpRequest result;
  result.method.assign(request.method_string().data(), request.method_string().size());
  const std::string target(request.target().data(), request.target().size());
  const auto delimiter = target.find('?');
  const std::string_view url(target);
  result.path = decode(url.substr(0, delimiter), false);
  if (delimiter != std::string::npos) {
    for (auto start = delimiter + 1; start < target.size(); ) {
      const auto end = std::min(target.find('&', start), target.size());
      const auto equals = target.find('=', start);
      if (equals < end) {
        result.query.emplace(decode(url.substr(start, equals - start), true),
          decode(url.substr(equals + 1, end - equals - 1), true));
      }
      start = end + 1;
    }
  }
  if (request.method() == http::verb::post) {
    result.body = request.body().empty() ? nlohmann::json::object() :
      nlohmann::json::parse(request.body());
  }
  return result;
}

class Session : public std::enable_shared_from_this<Session>
{
public:
  Session(tcp::socket socket, asio::thread_pool & workers, asio::thread_pool & command_worker,
    HttpServer::Handler handler, std::chrono::duration<double> command_timeout,
    std::shared_ptr<NotificationGate> notification_gate)
  : stream_(std::move(socket)), timer_(stream_.get_executor()), workers_(workers),
    command_worker_(command_worker), handler_(std::move(handler)), command_timeout_(command_timeout),
    notification_gate_(std::move(notification_gate))
  {
    parser_.body_limit(1024 * 1024);
  }

  void start()
  {
    stream_.expires_after(5s);
    http::async_read(stream_, buffer_, parser_, [self = shared_from_this()](
      beast::error_code error, std::size_t)
    {
      if (!error) {
        const auto received = std::chrono::steady_clock::now();
        const auto & message = self->parser_.get();
        const bool teleop = message.method() == http::verb::post &&
          message.target().substr(0, message.target().find('?')) == "/api/teleop";
        asio::post(teleop ? self->command_worker_ : self->workers_,
          [self, received, teleop]() {self->dispatch(received, teleop);});
      }
    });
  }

private:
  void dispatch(std::chrono::steady_clock::time_point received, bool teleop)
  {
    HttpResponse result;
    try {
      auto request = to_request(parser_.get());
      if (request.method == "POST" && request.path == "/api/teleop" &&
        (!teleop || std::chrono::steady_clock::now() - received >= command_timeout_))
      {
        result = HttpResponse::json({{"ok", false}, {"message", "teleop request expired"}}, 408);
      } else {
        result = handler_(std::move(request));
      }
    } catch (const nlohmann::json::exception & error) {
      result = HttpResponse::json({{"ok", false}, {"message", error.what()}}, 400);
    } catch (const std::invalid_argument & error) {
      result = HttpResponse::json({{"ok", false}, {"message", error.what()}}, 400);
    } catch (const std::exception &) {
      result = HttpResponse::json({{"ok", false}, {"message", "internal server error"}}, 500);
    }
    asio::post(stream_.get_executor(),
      [self = shared_from_this(), result = std::move(result)]() mutable {
        self->respond(std::move(result));
      });
  }

  void respond(HttpResponse result)
  {
    if (result.next_event) {
      next_event_ = std::move(result.next_event);
      if (result.subscribe) {
        subscription_ = result.subscribe([session = weak_from_this(), gate = notification_gate_]() {
            std::lock_guard<std::mutex> lock(gate->mutex);
            if (!gate->accepting) {
              return;
            }
            if (const auto self = session.lock(); self && !self->wake_posted_.exchange(true)) {
              asio::post(self->stream_.get_executor(), [session]() {
                  if (const auto active = session.lock()) {
                    active->wake_posted_ = false;
                    active->wake();
                  }
                });
            }
          });
      }
      send_event(true, false);
      return;
    }
    if (result.descriptor) {
      beast::file file;
      file.native_handle(std::exchange(*result.descriptor, -1));
      beast::error_code error;
      file_response_.body().reset(std::move(file), error);
      if (error) {
        respond(HttpResponse::json({{"ok", false}, {"message", "file unavailable"}}, 503));
        return;
      }
      headers(file_response_, result);
      file_response_.prepare_payload();
      stream_.expires_after(30s);
      http::async_write(stream_, file_response_,
        [self = shared_from_this()](beast::error_code, std::size_t) {});
      return;
    }
    body_ = std::move(result.body);
    headers(response_, result);
    if (body_) {
      response_.body() = {body_->data(), body_->size()};
    }
    response_.prepare_payload();
    stream_.expires_after(5s);
    http::async_write(stream_, response_,
      [self = shared_from_this()](beast::error_code, std::size_t) {});
  }

  template<typename Response>
  void headers(Response & response, const HttpResponse & result)
  {
    response.version(parser_.get().version());
    response.keep_alive(false);
    response.result(result.status);
    response.set(http::field::cache_control, "no-store");
    response.set(http::field::content_type, result.content_type);
    for (const auto & header : result.headers) {
      response.set(header.first, header.second);
    }
  }

  void wake()
  {
    if (writing_) {
      event_pending_ = true;
      return;
    }
    timer_.cancel();
    send_event(false, false);
  }

  void wait_for_heartbeat()
  {
    timer_.expires_after(1s);
    timer_.async_wait([self = shared_from_this()](beast::error_code error) {
      if (!error) {
        self->send_event(false, true);
      }
    });
  }

  void send_event(bool initial, bool heartbeat)
  {
    try {
      event_ = next_event_();
    } catch (const std::exception &) {
      return;
    }
    if (initial) {
      event_.insert(0, "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream; charset=utf-8\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\nX-Accel-Buffering: no\r\n\r\n");
    } else if (event_.empty()) {
      if (!heartbeat) {
        wait_for_heartbeat();
        return;
      }
      event_ = ": keepalive\n\n";
    }
    writing_ = true;
    stream_.expires_after(5s);
    asio::async_write(stream_, asio::buffer(event_), [self = shared_from_this()](
      beast::error_code error, std::size_t)
    {
      self->writing_ = false;
      if (!error) {
        if (std::exchange(self->event_pending_, false)) {
          self->send_event(false, false);
        } else {
          self->wait_for_heartbeat();
        }
      } else {
        self->subscription_.reset();
      }
    });
  }

  beast::tcp_stream stream_;
  asio::steady_timer timer_;
  beast::flat_buffer buffer_;
  http::request_parser<http::string_body> parser_;
  http::response<http::span_body<const char>> response_;
  http::response<http::file_body> file_response_;
  asio::thread_pool & workers_;
  asio::thread_pool & command_worker_;
  HttpServer::Handler handler_;
  std::chrono::duration<double> command_timeout_;
  std::shared_ptr<NotificationGate> notification_gate_;
  std::shared_ptr<const std::string> body_;
  std::function<std::string()> next_event_;
  std::shared_ptr<void> subscription_;
  std::atomic_bool wake_posted_{false};
  bool writing_ = false;
  bool event_pending_ = false;
  std::string event_;
};
}  // namespace

class HttpServer::Implementation
{
public:
  Implementation(const std::string & host, std::uint16_t port, Handler handler,
    std::chrono::duration<double> command_timeout)
  : acceptor_(context_, tcp::endpoint(asio::ip::make_address(host), port)),
    accept_retry_(context_), workers_(4), command_worker_(1), handler_(std::move(handler)),
    command_timeout_(command_timeout)
  {
    accept();
    thread_ = std::thread([this]() {context_.run();});
  }

  ~Implementation()
  {
    {
      std::lock_guard<std::mutex> lock(notification_gate_->mutex);
      notification_gate_->accepting = false;
    }
    context_.stop();
    command_worker_.stop();
    workers_.stop();
    thread_.join();
    command_worker_.join();
    workers_.join();
  }

private:
  void accept()
  {
    acceptor_.async_accept([this](beast::error_code error, tcp::socket socket) {
      if (!error) {
        std::make_shared<Session>(std::move(socket), workers_, command_worker_, handler_,
          command_timeout_, notification_gate_)->start();
        accept();
      } else if (error != asio::error::operation_aborted) {
        accept_retry_.expires_after(100ms);
        accept_retry_.async_wait([this](beast::error_code retry_error) {
          if (!retry_error) {
            accept();
          }
        });
      }
    });
  }

  asio::io_context context_;
  tcp::acceptor acceptor_;
  asio::steady_timer accept_retry_;
  asio::thread_pool workers_;
  asio::thread_pool command_worker_;
  Handler handler_;
  std::chrono::duration<double> command_timeout_;
  std::shared_ptr<NotificationGate> notification_gate_ = std::make_shared<NotificationGate>();
  std::thread thread_;
};

HttpServer::HttpServer(const std::string & host, std::uint16_t port, Handler handler,
  std::chrono::duration<double> command_timeout)
: implementation_(std::make_unique<Implementation>(host, port, std::move(handler), command_timeout)) {}

HttpServer::~HttpServer() = default;
}  // namespace web_control
