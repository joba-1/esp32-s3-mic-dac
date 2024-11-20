#include <Arduino.h>

//#define DAC_16_BITS

#if defined(DAC_16_BITS)
  #define DAC_SAMPLE_SIZE  I2S_DATA_BIT_WIDTH_16BIT
  #define DAC_SAMPLE_FMT   "%04hx"
  typedef int16_t          dac_sample_t;
#else
  #define DAC_SAMPLE_SIZE  I2S_DATA_BIT_WIDTH_32BIT
  #define DAC_SAMPLE_FMT   "%08x"
  typedef int32_t          dac_sample_t;
#endif

#define DAC_I2S_NUMBER     I2S_NUM_1

#define SAMPLE_RATE        16000
#define HZ                 400
#define BUFFER_SIZE        (sizeof(dac_sample_t)*SAMPLE_RATE/HZ)
#define BUFFER_TIMEOUT_MS  (3*1000*used_buffer/sizeof(dac_sample_t)/SAMPLE_RATE)


#if defined(BOARD_S3C1)

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

// S3 avoid use
// Flash/PSRAM: 26-32
// Strapping: 0, 3, 45, 46
// Debug/USB: 19, 20

// UART0 TX, RX defaults: 43, 44, fixed
// UART1 TX, RX defaults: 17, 18, any
// UART2 TX, RX defaults: --, --, any

// Solder bridges for my pcm5101a
// SCK  -> Gnd = internal clock
// FLT  -> Gnd = normal, fir 0.5ms latency
// DEMP -> Gnd = deactivate because no one uses this...
// XSMT -> Vcc = force unmute, deactivate mute via pin,
// FMT  -> Gnd = I2S as used by the pico

#define CONFIG_I2S_OUT_GAIN_PIN  10
#define GAIN_12DB  LOW
#define GAIN_6DB   HIGH

#define LED_PIN    48
#define BTN_PIN     0

#elif defined(BOARD_MINI)

#define CONFIG_I2S_OUT_BCK_PIN   21
#define CONFIG_I2S_OUT_LRCK_PIN  16
#define CONFIG_I2S_OUT_DATA_PIN  17

#define LED_PIN     2
#define BTN_PIN     0

#endif


#include <rgb_circle.hpp>
rgb::NeoRawCircle<0xff/6> circle(LED_PIN);


char buffer[BUFFER_SIZE];
size_t used_buffer = 0;  // bytes of buffer used by complete sine wave samples


template <typename S>
void print_sample( const char *fmt, S s ) {
  Serial.printf(fmt, s);
}

template <typename S>
void print_samples( const char *tag, const char *fmt, const char *buffer, size_t size ) {
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


template <typename S>
size_t gen_sine( char *buffer, size_t size, size_t hz, size_t rate ) {
  size_t buffer_samples = size/sizeof(S);
  size_t min_hz = rate / buffer_samples;  // at least one sine wave should fit into the buffer
  if (hz < min_hz) hz = min_hz;
  size_t waves = hz * buffer_samples / rate;
  size_t wave_samples = rate / hz;
  size_t amplitude = (1U << (sizeof(S)*8 - 1)) - 1;  // max amplitude

  size_t used_bytes = 0;
  for (size_t w = 0; w < waves; w++) {
    for (size_t s = 0; s < wave_samples; s++) {
      ((S *)buffer)[w*wave_samples + s] = (S)(sin(2*PI*s/wave_samples)*amplitude/8);
      used_bytes += sizeof(S);
    }
  }

  size_t buffer_time = 1000000*waves/hz;
  Serial.printf("Sine wave %u hz, %u waves/buffer, %u samples/wave, %u bytes = %u us\n", 
    hz, waves, wave_samples, used_bytes, buffer_time);
  
  return used_bytes;
}


#include <ESP_I2S.h>

// I2SClass I2S_MIC;
I2SClass I2S_DAC;


// const int buff_size = 128;
// int available_bytes, read_bytes;
// uint8_t buffer[buff_size];
// I2SClass I2S;

// void setup() {
//   I2S.setPins(5, 25, 26, 35, 0); //SCK, WS, SDOUT, SDIN, MCLK
//   I2S.begin(I2S_MODE_STD, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
//   I2S.read();
//   available_bytes = I2S.available();
//   if(available_bytes < buff_size) {
//     read_bytes = I2S.readBytes(buffer, available_bytes);
//   } else {
//     read_bytes = I2S.readBytes(buffer, buff_size);
//   }
//   I2S.write(buffer, read_bytes);
//   I2S.end();
// }


void InstallI2SDriver( i2s_port_t port ) {
  if (port == DAC_I2S_NUMBER) {
    if (!I2S_DAC.begin(I2S_MODE_STD, SAMPLE_RATE, DAC_SAMPLE_SIZE, I2S_SLOT_MODE_MONO)) {
      Serial.println("dac i2s init failed");
    }
  }
  else {
    // I2S_MIC.begin(I2S_MODE_STD, SAMPLE_RATE, MIC_SAMPLE_SIZE, I2S_SLOT_MODE_MONO);
  }
}


void SetI2SPins( i2s_port_t port ) {
  if (port == DAC_I2S_NUMBER) {
    I2S_DAC.setPins(  //SCK, WS, SDOUT, SDIN, MCLK
      CONFIG_I2S_OUT_BCK_PIN, 
      CONFIG_I2S_OUT_LRCK_PIN, 
      CONFIG_I2S_OUT_DATA_PIN, 
      NOT_A_PIN, 
      NOT_A_PIN);
  }
  else {
    // I2S_MIC.setPins(  //SCK, WS, SDOUT, SDIN, MCLK
    //   CONFIG_I2S_IN_BCK_PIN, 
    //   CONFIG_I2S_IN_LRCK_PIN, 
    //   NOT_A_PIN, 
    //   CONFIG_I2S_IN_DATA_PIN, 
    //   NOT_A_PIN);
  }
}


void start_dac() {
  InstallI2SDriver(DAC_I2S_NUMBER);
  SetI2SPins(DAC_I2S_NUMBER);
}


void stop_dac() {
  I2S_DAC.end();
}


void dacTaskCode( void * parameter ) {
  Serial.printf("Dac  start at %u with prio %u on core %d\n", millis(), uxTaskPriorityGet(NULL), xPortGetCoreID());

  for(;;) {
    size_t bytes_written = 0;
    do {
      if (bytes_written != 0) {
        Serial.println("notice: i2s write cut");  // not an error, but interesting...
      }
      bytes_written += I2S_DAC.write((uint8_t *)buffer, used_buffer - bytes_written);
      delay(1);
    } while (bytes_written != used_buffer);
  }
}


void setup() {
  ++circle;
  
  #if ARDUINO_USB_CDC_ON_BOOT == 1
    if (ESP_RST_POWERON == esp_reset_reason()) {
      delay(6500);  // let host recognize the new usb device...
    }
  #endif

  Serial.begin(115200);
  while (!Serial);
  delay(10);
  Serial.printf("Main start at %u with prio %u on core %d\n", millis(), uxTaskPriorityGet(NULL), xPortGetCoreID());

  #ifdef CONFIG_I2S_OUT_GAIN_PIN
    pinMode(CONFIG_I2S_OUT_GAIN_PIN, OUTPUT);
    digitalWrite(CONFIG_I2S_OUT_GAIN_PIN, GAIN_12DB);
  #endif
  pinMode(BTN_PIN, INPUT_PULLUP);  // press to gnd

  used_buffer = gen_sine<dac_sample_t>(buffer, BUFFER_SIZE, HZ, SAMPLE_RATE);

  Serial.printf("Rate %u samples/s, buffer %u bytes, timeout %u ms\n", 
    SAMPLE_RATE, BUFFER_SIZE, BUFFER_TIMEOUT_MS);
  Serial.printf("Pins BCK %d, LRCK %d, DATA %d, LED %d, BTN %d\n",
    CONFIG_I2S_OUT_BCK_PIN, CONFIG_I2S_OUT_LRCK_PIN, 
    CONFIG_I2S_OUT_DATA_PIN, LED_PIN, BTN_PIN);

  start_dac();
  stop_dac();
  start_dac();

  UBaseType_t prio = uxTaskPriorityGet(NULL) + 1;
  BaseType_t core = 1 - xPortGetCoreID();
  xTaskCreatePinnedToCore(dacTaskCode, "DacTask", 10000, NULL, prio, NULL, core);
  delay(20);

  Serial.println("Init done");
  Serial.flush();
  delay(10);
}


void loop() {
  const uint32_t elapsed_led = 20;
  static uint32_t prev_led = 0;
  uint32_t now = millis();
  if (now - prev_led > elapsed_led) {
    prev_led = now;
    ++circle;
  }

  const uint32_t elapsed_print = 10000;
  static uint32_t prev_print = -elapsed_print;
  if (now - prev_print > elapsed_print) {
    prev_print = now;
    print_samples<dac_sample_t>("buf", DAC_SAMPLE_FMT, buffer, used_buffer);
  }

  if (!digitalRead(BTN_PIN)) {
    do { delay(10); } while (!digitalRead(BTN_PIN));
    ESP.restart();
  }

  delay(2);
}
