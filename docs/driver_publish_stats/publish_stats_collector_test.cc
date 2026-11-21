// Standalone C++17 test for the ROS-free driver publish-statistics collector.
// Build/run without ROS or live hardware:
//   g++ -std=c++17 -O2 -pthread -I <package>/src \
//       publish_stats_collector_test.cc -o /tmp/publish_stats_test
//   /tmp/publish_stats_test
// All clock values are injected/frozen; only the worker join test uses a short
// real wait so it cannot depend on wall-clock jitter for rate assertions.

#include "utility/publish_stats_collector.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

using hesai_publish_stats::PublishStatsCollector;
using hesai_publish_stats::PublishStatsReporter;
using hesai_publish_stats::PublishStatsSnapshot;

static void ExpectNear(double actual, double expected, double epsilon = 1e-6)
{
  assert(std::fabs(actual - expected) <= epsilon);
}

int main()
{
  // Initial state: no frames is explicit, not a fake active/zero measurement.
  double now = 100.0;
  PublishStatsCollector empty([&now] { return now; });
  PublishStatsSnapshot snapshot = empty.Report();
  assert(snapshot.total_frames == 0);
  assert(snapshot.window_count == 0);
  ExpectNear(snapshot.publish_rate_hz, 0.0);
  ExpectNear(snapshot.last_publish_age_sec, -1.0);
  assert(snapshot.state == "waiting");

  // Ten publishes over exactly one second: window rate and exact totals.
  now = 200.0;
  PublishStatsCollector collector([&now] { return now; });
  for (int index = 0; index < 10; ++index) {
    now = 200.0 + index * 0.1;
    collector.NotePublish();
  }
  now = 201.0;
  snapshot = collector.Report();
  assert(snapshot.total_frames == 10);
  assert(snapshot.window_count == 10);
  ExpectNear(snapshot.window_sec, 1.0);
  ExpectNear(snapshot.publish_rate_hz, 10.0);
  ExpectNear(snapshot.last_publish_age_sec, 0.1);
  assert(snapshot.state == "publishing");

  // Heartbeat continues after frames stop: measured 0 Hz + growing age.
  now = 202.0;
  snapshot = collector.Report();
  assert(snapshot.total_frames == 10);
  assert(snapshot.window_count == 0);
  ExpectNear(snapshot.publish_rate_hz, 0.0);
  ExpectNear(snapshot.last_publish_age_sec, 1.1);
  assert(snapshot.state == "idle");

  now = 210.0;
  snapshot = collector.Report();
  assert(snapshot.total_frames == 10);
  assert(snapshot.window_count == 0);
  ExpectNear(snapshot.last_publish_age_sec, 9.1);
  assert(snapshot.state == "idle");

  // A driver restart is a fresh collector: counters/age do not leak across it.
  now = 300.0;
  PublishStatsCollector restarted([&now] { return now; });
  snapshot = restarted.Report();
  assert(snapshot.total_frames == 0);
  ExpectNear(snapshot.last_publish_age_sec, -1.0);
  assert(snapshot.state == "waiting");
  restarted.NotePublish();
  now = 301.0;
  snapshot = restarted.Report();
  assert(snapshot.total_frames == 1);
  assert(snapshot.window_count == 1);
  assert(snapshot.state == "publishing");

  // Concurrent publish callbacks increment exactly once each.
  now = 400.0;
  PublishStatsCollector concurrent([&now] { return now; });
  std::vector<std::thread> workers;
  for (int thread = 0; thread < 4; ++thread) {
    workers.emplace_back([&concurrent] {
      for (int index = 0; index < 25000; ++index) {
        concurrent.NotePublish();
      }
    });
  }
  for (auto& worker : workers) {
    worker.join();
  }
  assert(concurrent.total_frames() == 100000);
  now = 401.0;
  snapshot = concurrent.Report();
  assert(snapshot.total_frames == 100000);
  assert(snapshot.window_count == 100000);
  ExpectNear(snapshot.publish_rate_hz, 100000.0);
  assert(snapshot.state == "publishing");

  // Reporter start/stop: heartbeat fires with no clouds, then joins and stops
  // publishing.  The sink only reads snapshots, so republishing stats can never
  // increment the point-cloud counter.
  now = 500.0;
  PublishStatsCollector worker_collector([&now] { return now; });
  std::mutex mutex;
  std::condition_variable condition;
  int reports = 0;
  PublishStatsReporter reporter(
      worker_collector,
      [&mutex, &condition, &reports](const PublishStatsSnapshot&) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          ++reports;
        }
        condition.notify_all();
      },
      0.01);
  reporter.Start();
  {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait_for(lock, std::chrono::seconds(2),
                       [&reports] { return reports >= 2; });
  }
  assert(reports >= 2);
  reporter.Stop();
  assert(!reporter.started());
  int after_stop = 0;
  {
    std::lock_guard<std::mutex> lock(mutex);
    after_stop = reports;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  {
    std::lock_guard<std::mutex> lock(mutex);
    assert(reports == after_stop);
  }
  assert(worker_collector.total_frames() == 0);

  // Stop is idempotent and safe on a never-started reporter.
  reporter.Stop();
  PublishStatsReporter unused(worker_collector, [](const PublishStatsSnapshot&) {});
  unused.Stop();

  std::printf("publish_stats_collector tests passed\n");
  return 0;
}
