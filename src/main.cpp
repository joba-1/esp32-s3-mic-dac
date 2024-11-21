#include <Arduino.h>

//#define DAC_16_BITS
//#define MIC_16_BITS

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


#define SAMPLE_RATE        16000
#define HZ                 400
#define BUFFER_SIZE        (sizeof(sample_t)*SAMPLE_RATE/HZ)
#define BUFFER_TIMEOUT_MS  (3*1000*BUFFER_SIZE/sizeof(sample_t)/SAMPLE_RATE)


// see http://wiki.fluidnc.com/en/hardware/ESP32-S3_Pin_Reference
// normally (H)SPI2
#define CONFIG_I2S_OUT_BCK_PIN   12
#define CONFIG_I2S_OUT_LRCK_PIN  13
#define CONFIG_I2S_OUT_DATA_PIN  11
// normally (V)SPI3
#define CONFIG_I2S_IN_BCK_PIN   36
#define CONFIG_I2S_IN_LRCK_PIN  37
#define CONFIG_I2S_IN_DATA_PIN  35

#define RS485_TX  17
#define RS485_RX  18

#define LED_PIN    48
#define BTN_PIN     0


#include <rgb_circle.hpp>
using Circle = rgb::NeoRawCircle<0xff/6>;
Circle circle(LED_PIN);


#include "queue.hpp"
#include <array>
using buffer_t = std::array<uint8_t, BUFFER_SIZE>;
using buffers_t = std::array<buffer_t, 4>;
using queue_t = Queue<buffer_t *>;

buffers_t buffers;
queue_t q_useable(buffers.size()), q_filled(buffers.size()/2), q_print(1);


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

    if (!I2S_MIC.begin(I2S_MODE_STD, SAMPLE_RATE, MIC_SAMPLE_SIZE, I2S_SLOT_MODE_MONO)) {
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

    if (!I2S_DAC.begin(I2S_MODE_STD, SAMPLE_RATE, DAC_SAMPLE_SIZE, I2S_SLOT_MODE_MONO)) {
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
    if (q_useable.get(buffer, BUFFER_TIMEOUT_MS/portTICK_PERIOD_MS)) {

      size_t bytes_read = 0;
      do {
        if (bytes_read != 0) {
          lock.lock();
          Serial.println("notice: i2s read cut");  // not an error, but interesting...
          lock.unlock();
        }
        bytes_read += I2S_DAC.readBytes((char *)buffer->data(), buffer->size() - bytes_read);
        delay(1);
      } while (bytes_read != buffer->size());
      
      q_filled.put(buffer);

      Lock lock(shared->led);
      circle.next(Circle::R);
    }
    else {
      delay(1);
    }
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
    if (q_filled.get(buffer, BUFFER_TIMEOUT_MS/portTICK_PERIOD_MS)) {
      size_t bytes_written = 0;
      do {
        if (bytes_written != 0) {
          lock.lock();
          Serial.println("notice: i2s write cut");  // not an error, but interesting...
          lock.unlock();
        }
        bytes_written += I2S_DAC.write(buffer->data(), buffer->size() - bytes_written);
        delay(1);
      } while (bytes_written != buffer->size());

      uint32_t now = millis();
      if (now - prev_print > elapsed_print) {
        prev_print = now;
        q_print.put(buffer);
      }
      else {
        q_useable.put(buffer);
      }

      Lock lock(shared->led);
      circle.next(Circle::B);
    }
    else {
      delay(1);
    }
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

  for (auto &buffer : buffers) {
    q_useable.put(&buffer);
  }

  circle.set(0, 0, 0);

  UBaseType_t prio = uxTaskPriorityGet(NULL) + 1;
  BaseType_t core = 1 - xPortGetCoreID();
  xTaskCreatePinnedToCore(micTaskCode, "MicTask", 10000, &shared, prio, NULL, core);
  xTaskCreatePinnedToCore(dacTaskCode, "DacTask", 10000, &shared, prio, NULL, core);

  Lock lock(shared.serial);
  Serial.println("Init done");
}


void loop() {
  const uint32_t elapsed_led = 20;
  static uint32_t prev_led = 0;
  uint32_t now = millis();
  if (now - prev_led > elapsed_led) {
    prev_led = now;
    Lock lock(shared.led);
    circle.next(Circle::G);
  }

  buffer_t *buffer;
  if (q_print.get(buffer)) {
    Lock lock(shared.serial);
    print_samples<sample_t>("buf", SAMPLE_FMT, buffer->data(), buffer->size());
    q_useable.put(buffer);
  }

  if (!digitalRead(BTN_PIN)) {
    do { delay(10); } while (!digitalRead(BTN_PIN));
    ESP.restart();
  }

  delay(2);
}
