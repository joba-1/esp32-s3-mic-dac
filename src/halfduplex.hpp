#pragma once

#include <Arduino.h>

/* WIP

Usage: 
* master periodically sends ping (message with size 0) to potential slaves (id 0-254)
* existing slaves that answer are now known and can be used as recipients of messages
* master sends messages to known slaves as required
* slaves need to respond within TIMEOUT_MS
* slaves whos responses time out MAX_OPEN times are treated as unknown again
*/


class Halfduplex : public Stream {
public:
    using id_t = uint8_t;

    Halfduplex( Stream &s ) : Stream(s) {}
    virtual ~Halfduplex() {}

    virtual uint8_t recv( char *msg, uint8_t max_size ) = 0;
    
    const uint8_t HD_MAGIC = 'H';     // first byte to send to distinguish from other types of devices on the bus
};


class HalfduplexSlave : public Halfduplex {
public:
    HalfduplexSlave( Stream &s, id_t id ) : Halfduplex(s), _id(id) {}


    bool send(char *msg, uint8_t size) {
        uint8_t header[] = {HD_MAGIC, _id, size};

        return write(header, sizeof(header)) == sizeof(header)
            && (size == 0 || write(msg, size) == size);
    }


    uint8_t recv(char *msg, uint8_t max_size) {
        uint8_t header[3];
        if (readBytes(header, sizeof(header)) == sizeof(header)
         && header[0] == HD_MAGIC 
         && header[1] == _id) {
            if (header[2] == 0) {  // valid ping for us
                send(NULL, 0);  // send ping ack
            }
            else {
                uint8_t len = min(max_size, header[2]);
                size_t read_bytes = readBytes(msg, len);
                if (read_bytes == len) {
                    return len;
                }
            }
        }
        else {  // not for us
            while (available()) {
                read();  // discard
            }
        }
        return 0;
    }

private:
    id_t _id;
};


class HalfduplexMaster : public Halfduplex {
public:
    const uint8_t MAX_OPEN = 3;       // at max unanswered requests master assumes client is gone (NEVER_SEEN)
    const uint8_t NEVER_SEEN = 0xff;  // client never seen, try detect if bus is idle
    const uint32_t TIMEOUT_MS = 2;    // stop waiting for response


    HalfduplexMaster( Stream &s ) : Halfduplex(s), _id(0), _requested(false), _since(0) {
        _open.fill(NEVER_SEEN);
    }


    // write message to stream if no other request is pending
    // if size == 0 it is a ping test  to unknown client 
    bool send(id_t id, const char *msg, uint8_t size) {
        if (_requested && (millis() - _since > TIMEOUT_MS)) {
            _requested = false;  // previous request timed out. Ok to send.
        }
        else {
            return false;  // still waiting for open request to respond
        }

        if (!size) {  // hello to unknown client
            _open[id] = MAX_OPEN-1;  // one chance to respond
        }
        else if ( _open[id] >= MAX_OPEN) {  // don't waste time on unknown or unresponsive client
            return false;
        }

        _requested = true;  // even if we fail now, we probably have sent out something
        _since = millis();  // so block for some time
        _id = 0xff;         // unused id, -> will timeout

        uint8_t header[] = {HD_MAGIC, id, size};
        if (write(header, sizeof(header)) != sizeof(header)) {
            return false;
        }

        if (size == 0 || write(msg, size) == size) {
            _id = id;     // sent message, wait for id to respond
            _open[id]++;
            return true;
        }

        return false;
    }


    uint8_t recv( char *msg, uint8_t max_size ) {
        if (millis() - _since > TIMEOUT_MS) {  // request or block timed out
            _requested = false;
        }

        if (!_requested) {  // no request pending
            int bytes = available();
            if (bytes) {  // bus is busy, hold back own requests
                _requested = true;
                _since = millis();
                _id = 0xff;
                while (bytes--) read();  // discard data which is not for us
            }
            return 0;
        } 

        // we requested something, try to read the response
        uint8_t header[3];
        size_t read_bytes = readBytes(header, sizeof(header));
        if (header[0] != HD_MAGIC || header[1] != _id) {  // message is not for us
            _requested = true;  // hands off the bus
            _since = millis();  // for a while
            _id = 0xff;         // invalid id -> will timeout
            return 0;
        }
    
        // header ok and for us, request is pending -> read message
        uint8_t len = min(max_size, header[2]);
        size_t read_bytes = readBytes(msg, len);
        _requested = false;
        if (len == read_bytes) {
            _open[_id] = 0;
            return len;
        }

        return 0;
    }

private:
    id_t _id;         // client id of current request
    bool _requested;  // request active
    uint32_t _since;  // request active since
    std::array<int8_t, 256> _open;  // 0 - max, -1 = never seen
};
