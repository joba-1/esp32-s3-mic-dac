#include <Arduino.h>

#define SAMPLE_RATE        16000
#define BUFFER_TIME_MS     2
#define BUFFER_SIZE        (sizeof(sample_t)*SAMPLE_RATE*BUFFER_TIME_MS/1000)
#define BUFFER_TIMEOUT_MS  (3*BUFFER_TIME_MS)

// Baud for RS485 r/w at least SAMPLE_RATE * SAMPLE_SIZE * 2 + some headroom...
#define RS485_BAUD  (460800*2)

#define DAC_16_BITS
#define MIC_16_BITS

#if defined(DAC_16_BITS)
  #define DAC_SAMPLE_SIZE  I2S_DATA_BIT_WIDTH_16BIT
  #define DAC_SAMPLE_FMT   "%04hx"
  typedef int16_t          dac_sample_t;
#else
  #define DAC_SAMPLE_SIZE  I2S_DATA_BIT_WIDTH_32BIT
  #define DAC_SAMPLE_FMT   "%08x"
  typedef int32_t          dac_sample_t;
#endif

#if defined(MIC_16_BITS)
  #define MIC_SAMPLE_SIZE  I2S_DATA_BIT_WIDTH_16BIT
  #define MIC_SAMPLE_FMT   "%04hx"
  typedef int16_t          mic_sample_t;
#else
  #define MIC_SAMPLE_SIZE  I2S_DATA_BIT_WIDTH_32BIT
  #define MIC_SAMPLE_FMT   "%08x"
  typedef int32_t          mic_sample_t;
#endif

typedef union sample {
  mic_sample_t mic;
  dac_sample_t dac;
} sample_t;

#if MIC_SAMPLE_SIZE >= DAC_SAMPLE_SIZE
  #define SAMPLE_SIZE  MIC_SAMPLE_SIZE
  #define SAMPLE_FMT   MIC_SAMPLE_FMT
#else
  #define SAMPLE_SIZE  DAC_SAMPLE_SIZE
  #define SAMPLE_FMT   DAC_SAMPLE_FMT
#endif


// see http://wiki.fluidnc.com/en/hardware/ESP32-S3_Pin_Reference
// normally (H)SPI2
#define CONFIG_I2S_OUT_BCK_PIN   12
#define CONFIG_I2S_OUT_LRCK_PIN  13
#define CONFIG_I2S_OUT_DATA_PIN  11
// normally (V)SPI3
#define CONFIG_I2S_IN_BCK_PIN   36
#define CONFIG_I2S_IN_LRCK_PIN  37
#define CONFIG_I2S_IN_DATA_PIN  35

#define RS485_TX    17
#define RS485_RX    18

#define LED_PIN     48
#define BTN_PIN     0


#include <rgb_circle.hpp>
using Circle = rgb::NeoRawCircle<0xff/6>;
Circle circle(LED_PIN);


#include "queue.hpp"
#include <array>

enum Q { Q_mic, Q_tx, Q_rx, Q_dac, Q_print, Q_end };
const char *Q[] = { "Q_mic", "Q_tx", "Q_rx", "Q_dac", "Q_print" };

using buffer_t = std::array<uint8_t, BUFFER_SIZE>;
using buffers_t = std::array<buffer_t, 4>;  // 2 per queue
using queue_t = Queue<buffer_t *>;
using queues_t = std::array<queue_t, Q_end>;

buffers_t mic_buffers, dac_buffers;
queues_t queues{mic_buffers.size(), mic_buffers.size()/2, dac_buffers.size(), dac_buffers.size()/2, 1};


#include "lock.hpp"
using shared_t = struct shared {
  Mutex led;
  Mutex serial;
};

shared_t shared;


template <typename S>
void print_sample( const char *fmt, S s ) {
  Serial.printf(fmt, s);
}

template <typename S>
void print_samples( const char *tag, const char *fmt, uint8_t *buffer, size_t size ) {
  const S *sample = (const S *)buffer;
  const S *end = (const S *)(buffer + size);
  size_t count = 0;

  Serial.printf("\n%s[%u] @ %p:\n", tag, size, buffer);
  while (sample < end) {
    print_sample<S>(fmt, *sample);
    count++;
    if (count & 1) {
      Serial.print('|');
    }
    else if (count == 8) {
      count = 0;
      Serial.print('\n');
    }
    else {
      Serial.print(' ');
    }
    sample++;
  }
}


#include <ESP_I2S.h>

I2SClass I2S_MIC;
I2SClass I2S_DAC;


void start_mic() {
    I2S_MIC.setPins(  //SCK, WS, SDOUT, SDIN, MCLK
      CONFIG_I2S_IN_BCK_PIN, 
      CONFIG_I2S_IN_LRCK_PIN, 
      NOT_A_PIN, 
      CONFIG_I2S_IN_DATA_PIN, 
      NOT_A_PIN);

    if (!I2S_MIC.begin(I2S_MODE_STD, SAMPLE_RATE, MIC_SAMPLE_SIZE, I2S_SLOT_MODE_STEREO)) {
      Serial.println("mic i2s init failed");
    }
}


void stop_mic() {
  I2S_MIC.end();
}


void start_dac() {
    I2S_DAC.setPins(  //SCK, WS, SDOUT, SDIN, MCLK
      CONFIG_I2S_OUT_BCK_PIN,  
      CONFIG_I2S_OUT_LRCK_PIN, 
      CONFIG_I2S_OUT_DATA_PIN, 
      NOT_A_PIN, 
      NOT_A_PIN);

    if (!I2S_DAC.begin(I2S_MODE_STD, SAMPLE_RATE, DAC_SAMPLE_SIZE, I2S_SLOT_MODE_STEREO)) {
      Serial.println("dac i2s init failed");
    }
}


void stop_dac() {
  I2S_DAC.end();
}


void micTaskCode( void *parameter ) {
  shared_t *shared = (shared_t *)parameter;
  buffer_t *buffer;

  Lock lock(shared->serial);
  Serial.printf("Mic  start at %u with prio %u on core %d\n", millis(), uxTaskPriorityGet(NULL), xPortGetCoreID());
  lock.unlock();

  for(;;) {
    if (queues[Q_mic].get(buffer, BUFFER_TIMEOUT_MS/portTICK_PERIOD_MS)) {
      size_t bytes_read = 0;
      size_t bytes_to_read = buffer->size();

      uint32_t start = millis();
      do {
        if (bytes_read != 0) {
          lock.lock();
          Serial.println("notice: i2s read cut");  // not an error, but interesting...
          lock.unlock();
          delay(1);
        }
        bytes_read += I2S_MIC.readBytes((char *)&buffer[bytes_read], bytes_to_read - bytes_read);
      } while ((bytes_read != bytes_to_read) && ((millis() - start) < BUFFER_TIMEOUT_MS));
      
      if (bytes_read != bytes_to_read) {
        memset(&buffer[bytes_read], 0, bytes_to_read - bytes_read);
      }

      // 24bit r -> 32bit mono (or 12->16) for transport
      const size_t scale_bits = 3;  // good for inmp441 
      size_t num_samples = buffer->size() / sizeof(mic_sample_t) / 2;
      mic_sample_t *src = (mic_sample_t *)buffer->data();
      mic_sample_t *dst = src;
      while (num_samples--) {
        src++;                    // skip empty channel
        *dst = *(src++);          // copy other channel
        *(dst++) <<= scale_bits;  // and scale it
      }

      queues[Q_tx].put(buffer);

      Lock lock(shared->led);
      circle.next(Circle::R);
    }

    delay(BUFFER_TIME_MS/2);
  }
}


void txTaskCode( void *parameter ) {
  shared_t *shared = (shared_t *)parameter;
  buffer_t *buffer;

  Lock lock(shared->serial);
  Serial.printf("Tx   start at %u with prio %u on core %d\n", millis(), uxTaskPriorityGet(NULL), xPortGetCoreID());
  lock.unlock();

  for(;;) {
    if (queues[Q_tx].get(buffer, BUFFER_TIMEOUT_MS/portTICK_PERIOD_MS)) {
      size_t bytes_written = 0;
      size_t bytes_to_write = buffer->size()/2;

      uint32_t start = millis();
      do {
        if (bytes_written != 0) {
          lock.lock();
          Serial.println("notice: tx write cut");  // not an error, but interesting...
          lock.unlock();
          delay(1);
        }
        bytes_written += Serial1.write((uint8_t *)&buffer[bytes_written], bytes_to_write - bytes_written);
      } while ((bytes_written != bytes_to_write) && ((millis() - start) < BUFFER_TIMEOUT_MS));

      queues[Q_mic].put(buffer);
    }
    delay(BUFFER_TIME_MS/2);
  }
}


void rxTaskCode( void *parameter ) {
  shared_t *shared = (shared_t *)parameter;
  buffer_t *buffer;

  Lock lock(shared->serial);
  Serial.printf("Rx   start at %u with prio %u on core %d\n", millis(), uxTaskPriorityGet(NULL), xPortGetCoreID());
  lock.unlock();

  for(;;) {
    if (queues[Q_rx].get(buffer, BUFFER_TIMEOUT_MS/portTICK_PERIOD_MS)) {
      size_t bytes_read = 0;
      size_t bytes_to_read = buffer->size()/2;

      uint32_t start = millis();
      do {
        if (bytes_read != 0) {
          lock.lock();
          Serial.println("notice: rx read cut");  // not an error, but interesting...
          lock.unlock();
          delay(1);
        }
        bytes_read += Serial1.read((uint8_t *)&buffer[bytes_read], bytes_to_read - bytes_read);
      } while ((bytes_read != bytes_to_read) && ((millis() - start) < BUFFER_TIMEOUT_MS));

      if (bytes_read != bytes_to_read) {
        memset(&buffer[bytes_read], 0, bytes_to_read - bytes_read);
      }

      queues[Q_dac].put(buffer);
    }
    delay(BUFFER_TIME_MS/2);
  }
}


void dacTaskCode( void *parameter ) {
  shared_t *shared = (shared_t *)parameter;
  buffer_t *buffer;
  const uint32_t elapsed_print = 10000;
  static uint32_t prev_print = -elapsed_print;

  Lock lock(shared->serial);
  Serial.printf("Dac  start at %u with prio %u on core %d\n", millis(), uxTaskPriorityGet(NULL), xPortGetCoreID());
  lock.unlock();

  for(;;) {
    if (queues[Q_dac].get(buffer, BUFFER_TIMEOUT_MS/portTICK_PERIOD_MS)) {
      // convert mono to stereo for pcm5101a
      size_t num_samples = buffer->size() / sizeof(dac_sample_t) / 2;
      dac_sample_t *src = ((dac_sample_t *)buffer->data()) + num_samples;  // end of valid mono samples
      dac_sample_t *dst = src + num_samples;                               // end of all samples
      while (num_samples--) {
        *(--dst) = *(--src);
        *(--dst) = *src; 
      }

      size_t bytes_written = 0;
      size_t bytes_to_write = buffer->size();

      uint32_t start = millis();
      do {
        if (bytes_written != 0) {
          lock.lock();
          Serial.println("notice: i2s write cut");  // not an error, but interesting...
          lock.unlock();
          delay(1);
        }
        bytes_written += I2S_DAC.write((uint8_t *)&buffer[bytes_written], bytes_to_write - bytes_written);
      } while ((bytes_written != bytes_to_write) && ((millis() - start) < BUFFER_TIMEOUT_MS));

      uint32_t now = millis();
      if (now - prev_print > elapsed_print) {
        prev_print = now;
        queues[Q_print].put(buffer);
      }
      else {
        queues[Q_rx].put(buffer);
      }

      Lock lock(shared->led);
      circle.next(Circle::B);
    }

    delay(BUFFER_TIME_MS/2);
  }
}


void setup() {
  circle.set(0xff, 0xff, 0xff);
  
  #if ARDUINO_USB_CDC_ON_BOOT == 1
    if (ESP_RST_POWERON == esp_reset_reason()) {
      delay(6500);  // let host recognize the new usb device...
    }
  #endif

  Serial.begin(BAUD);
  while (!Serial);
  delay(10);
  Serial.printf("Main start at %u with prio %u on core %d\n", millis(), uxTaskPriorityGet(NULL), xPortGetCoreID());

  Serial1.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX, false, BUFFER_TIMEOUT_MS);

  pinMode(BTN_PIN, INPUT_PULLUP);  // press to gnd

  Serial.printf("Rate %u samples/s, buffer %u bytes, timeout %u ms\n", 
    SAMPLE_RATE, BUFFER_SIZE, BUFFER_TIMEOUT_MS);
  Serial.printf("Mic BCK %d, LRCK %d, DATA %d\n",
    CONFIG_I2S_IN_BCK_PIN, CONFIG_I2S_IN_LRCK_PIN, CONFIG_I2S_IN_DATA_PIN);
  Serial.printf("Dac BCK %d, LRCK %d, DATA %d\n",
    CONFIG_I2S_OUT_BCK_PIN, CONFIG_I2S_OUT_LRCK_PIN, CONFIG_I2S_OUT_DATA_PIN);
  Serial.printf("Pins LED %d, BTN %d\n", LED_PIN, BTN_PIN);

  start_mic();
  start_dac();

  for (auto &buffer : mic_buffers) {
    queues[Q_mic].put(&buffer);
  }

  for (auto &buffer : dac_buffers) {
    queues[Q_rx].put(&buffer);
  }

  circle.set(0, 0, 0);
  
  UBaseType_t i2s_prio = uxTaskPriorityGet(NULL) + 2;
  UBaseType_t rxtx_prio = uxTaskPriorityGet(NULL) + 1;
  BaseType_t mic_core = 1 - xPortGetCoreID();
  BaseType_t dac_core = 1 - mic_core;
  xTaskCreatePinnedToCore(micTaskCode, "MicTask", 10000, &shared, i2s_prio,  NULL, mic_core);
  xTaskCreatePinnedToCore(txTaskCode,  "TxTask",  10000, &shared, rxtx_prio, NULL, mic_core);
  xTaskCreatePinnedToCore(dacTaskCode, "DacTask", 10000, &shared, i2s_prio,  NULL, dac_core);
  xTaskCreatePinnedToCore(rxTaskCode,  "RxTask",  10000, &shared, rxtx_prio, NULL, dac_core);

  Lock lock(shared.serial);
  Serial.println("Init done");
}


void loop() {
  // green led breathing
  const uint32_t elapsed_led = 20;
  static uint32_t prev_led = 0;
  uint32_t now = millis();
  if (now - prev_led > elapsed_led) {
    prev_led = now;
    Lock lock(shared.led);
    circle.next(Circle::G);
  }

  // print buffer content
  const char q_fmt[]  = "%5s[%u]: %4u %s, %2d min, %2u avg, %2u max items %s\n";
  buffer_t *buffer;
  if (queues[Q_print].get(buffer)) {
    Lock lock(shared.serial);
    print_samples<sample_t>("buf", SAMPLE_FMT, buffer->data(), buffer->size());
    lock.unlock();
    queues[Q_rx].put(buffer);
    delay(1);
  }

  // print queue stats
  const uint32_t elapsed_stats = 10000;
  static uint32_t prev_stats = -elapsed_stats;
  if (now - prev_stats > elapsed_stats) {
    prev_stats = now;
    Serial.printf("\nqueue stats @%u\n", now);
    for (size_t i=0; i<queues.size(); i++) {
      auto size = queues[i].size();
      auto &get_stats = queues[i].get_stats();
      auto &put_stats = queues[i].put_stats();

      auto get_count = get_stats.count();
      auto get_min = get_stats.min();
      auto get_avg = get_stats.avg();
      auto get_max = get_stats.max();

      auto put_count = put_stats.count();
      auto put_min = put_stats.min();
      auto put_avg = put_stats.avg();
      auto put_max = put_stats.max();

      Lock lock(shared.serial);
      Serial.printf(q_fmt, &Q[i][2], size, get_count, "get", (int)get_min, get_avg, get_max, "used");
      Serial.printf(q_fmt, &Q[i][2], size, put_count, "put", (int)put_min, put_avg, put_max, "free");
      lock.unlock();

      get_stats.init();
      put_stats.init();
      
      delay(1);
    }
  }

  // reboot key
  if (!digitalRead(BTN_PIN)) {
    do { delay(10); } while (!digitalRead(BTN_PIN));
    ESP.restart();
  }

  delay(BUFFER_TIME_MS/2);
}
