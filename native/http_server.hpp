#pragma once

#include <cstdint>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

namespace web_control
{
using Query = std::map<std::string, std::string>;
using StreamSubscribe = std::function<std::shared_ptr<void>(std::function<void()>)>;

struct HttpRequest
{
  std::string method;
  std::string path;
  Query query;
  nlohmann::json body;
};

struct HttpResponse
{
  unsigned status = 200;
  std::string content_type = "application/json; charset=utf-8";
  std::shared_ptr<const std::string> body;
  std::shared_ptr<int> descriptor;
  std::map<std::string, std::string> headers;
  // Called once for the initial snapshot and then periodically for changes.
  std::function<std::string()> next_event;
  StreamSubscribe subscribe;

  static HttpResponse json(nlohmann::json body, unsigned status = 200);
  static HttpResponse bytes(std::shared_ptr<const std::string> body, std::string content_type);
  static HttpResponse file(int descriptor, std::string content_type);
  static HttpResponse stream(std::function<std::string()> next_event, StreamSubscribe subscribe = {});
};

const std::string & query_value(const Query & query, const std::string & name);

class HttpServer
{
public:
  using Handler = std::function<HttpResponse(HttpRequest)>;

  HttpServer(const std::string & host, std::uint16_t port, Handler handler,
    std::chrono::duration<double> command_timeout);
  ~HttpServer();

  HttpServer(const HttpServer &) = delete;
  HttpServer & operator=(const HttpServer &) = delete;

private:
  class Implementation;
  std::unique_ptr<Implementation> implementation_;
};
}
