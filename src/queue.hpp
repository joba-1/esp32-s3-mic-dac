#pragma once

#include <Arduino.h>
#include <atomic>

template <typename ITEM>
class Queue {
public:
  class Stats {
  public:
    Stats( size_t limit ) : _limit(limit) { init(); }

    void init() {
      _min = _limit;
      _max = 0;
      _sum = 0;
      _count = 0;
    }

    void update( size_t len ) {
      if (_max < len) _max = len;
      if (_min > len) _min = len;
      _count++;
      _sum = _sum + len;
    }

    size_t max() const { return _max; }
    size_t min() const { return _min; }
    size_t avg() const { size_t count = _count; return count ? _sum/count : 0; }

  private:
    Stats();
    Stats( const Stats & );

    std::atomic<size_t> _min, _max, _sum, _count;
    const size_t _limit;
  };

  Queue(size_t size) : _get_stats(size), _put_stats(size) {
    _queue = xQueueCreate(size, sizeof(ITEM));
  }

  ~Queue() { vQueueDelete(_queue); }

  size_t length() { return uxQueueMessagesWaiting(_queue); }

  bool get(ITEM &item, TickType_t timeout = 0) {
    _get_stats.update(length());
    return xQueueReceive(_queue, &item, timeout);
  }

  bool put(const ITEM &item, TickType_t timeout = portMAX_DELAY) {
    _put_stats.update(length());
    return xQueueSend(_queue, &item, timeout);
  }

  void reset() { xQueueReset(_queue); }

  const Stats &get_stats() const { return _get_stats; }
  const Stats &put_stats() const { return _put_stats; }

private:
  Queue();

  QueueHandle_t _queue;
  Stats _get_stats, _put_stats;
};
