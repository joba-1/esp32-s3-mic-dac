#pragma once

#include <Arduino.h>
#include <atomic>

template <typename ITEM>
class Queue {
public:
  class Stats {
  public:
    Stats() { init(); }

    void init() {
      _min_ = (size_t)-1;
      _max_ = 0;
      _sum = 0;
      _count = 0;
    }

    void update( size_t len ) {
      if (_max_ < len) _max_ = len;
      if (_min_ > len) _min_ = len;
      _count++;
      _sum = _sum + len;
    }

    size_t count() const { return _count; }
    size_t max() const { return _max_; }
    size_t min() const { return _min_; }
    size_t avg() const { size_t count = _count; return count ? _sum/count : 0; }

  private:
    std::atomic<size_t> _min_, _max_, _sum, _count;
  };

  Queue(size_t size) : _size(size), _get_stats(), _put_stats() {
    _queue = xQueueCreate(size, sizeof(ITEM));
  }

  ~Queue() { vQueueDelete(_queue); }

  size_t used() const { return uxQueueMessagesWaiting(_queue); }
  size_t free() const { return uxQueueSpacesAvailable(_queue); }
  size_t size() const { return _size; }

  bool get(ITEM &item, TickType_t timeout = 0) {
    _get_stats.update(used());
    return xQueueReceive(_queue, &item, timeout);
  }

  bool put(const ITEM &item, TickType_t timeout = portMAX_DELAY) {
    _put_stats.update(free());
    return xQueueSend(_queue, &item, timeout);
  }

  void reset() { xQueueReset(_queue); }

  Stats &get_stats() { return _get_stats; }
  Stats &put_stats() { return _put_stats; }

private:
  Queue();

  QueueHandle_t _queue;
  const size_t _size;
  Stats _get_stats, _put_stats;
};
