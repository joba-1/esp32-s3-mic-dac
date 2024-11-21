#pragma once

#include <Arduino.h>

// Mutex
class Mutex {
public:
    Mutex() { _sem = xSemaphoreCreateMutex(); }

    ~Mutex() { if(_sem) { vSemaphoreDelete(_sem); } }

    bool take() { return _sem && xSemaphoreTake(_sem, portMAX_DELAY); }
    void give() { if (_sem) { xSemaphoreGive(_sem); } }

    operator bool() { return _sem != NULL; }

private:
    Mutex(const Mutex &);  // no copies

    SemaphoreHandle_t _sem;
};


// Locks
class Lock {
public:
    Lock( Mutex &mutex ) : _mutex(mutex) { _locked = _mutex.take(); }
    ~Lock() { if (_locked) { _mutex.give(); } };

    operator bool() { return _locked; }

    void unlock() { if (_locked) { _mutex.give(); _locked = false; } }
    bool lock() { if (!_locked) { _locked = _mutex.take(); } return _locked; }

private:
    Lock(const Lock &);

    Mutex &_mutex;
    bool _locked;
};

