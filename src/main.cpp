#include <Arduino.h>

//#define DAC_16_BITS

#define DAC_SIGNAL_MODE    0
#define DAC_CHAN_SIZE      I2S_BITS_PER_CHAN_DEFAULT

#if defined(DAC_16_BITS)
  #define DAC_SAMPLE_SIZE  I2S_BITS_PER_SAMPLE_16BIT
  #define DAC_SAMPLE_FMT   "%04hx"
  typedef int16_t          dac_sample_t;
#else
  #define DAC_SAMPLE_SIZE  I2S_BITS_PER_SAMPLE_32BIT
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


#include <driver/i2s.h>

void InstallI2SDriver( i2s_port_t port, size_t buffer_bytes ) {
  esp_err_t err = ESP_OK;

  i2s_config_t i2s_config = {
    .sample_rate          = SAMPLE_RATE,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    // .dma_desc_num         = 3,
    .dma_buf_count        = 3,
  };

  i2s_config.mode               = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2s_config.bits_per_sample    = DAC_SAMPLE_SIZE;
  i2s_config.bits_per_chan      = DAC_CHAN_SIZE;
  // i2s_config.dma_frame_num      = buffer_bytes/sizeof(dac_sample_t);
  i2s_config.dma_buf_len        = buffer_bytes/sizeof(dac_sample_t);
  i2s_config.use_apll           = true;
  i2s_config.tx_desc_auto_clear = true;

  err = i2s_driver_install(port, &i2s_config, 0, NULL);

  if (err != ESP_OK) {
    Serial.printf("Install I2S driver %d failed with rc = %d\n", port, err);
  }

  uint32_t bits = (i2s_config.bits_per_chan << 16) | i2s_config.bits_per_sample;
  err = i2s_set_clk(port, SAMPLE_RATE, bits, I2S_CHANNEL_MONO);
  if (err != ESP_OK) {
    Serial.printf("Set I2S clock %d failed with rc = %d\n", port, err);
  }
}


void SetI2SPins( i2s_port_t port ) {
  esp_err_t err = ESP_OK;

  i2s_pin_config_t tx_pin_config;

  tx_pin_config.mck_io_num = I2S_PIN_NO_CHANGE;

  tx_pin_config.bck_io_num   = CONFIG_I2S_OUT_BCK_PIN;
  tx_pin_config.ws_io_num    = CONFIG_I2S_OUT_LRCK_PIN;

  tx_pin_config.data_out_num = CONFIG_I2S_OUT_DATA_PIN;
  tx_pin_config.data_in_num  = I2S_PIN_NO_CHANGE;

  err = i2s_set_pin(port, &tx_pin_config);
  if (err != ESP_OK) {
    Serial.printf("Set I2S pins %d failed with rc = %d\n", port, err);
  }
}


void start_dac( size_t size ) {
  InstallI2SDriver(DAC_I2S_NUMBER, size);
  SetI2SPins(DAC_I2S_NUMBER);
}


void stop_dac() {
  i2s_driver_uninstall(DAC_I2S_NUMBER);
}


void dacTaskCode( void * parameter ) {
  Serial.printf("Dac  start at %u with prio %u on core %d\n", millis(), uxTaskPriorityGet(NULL), xPortGetCoreID());

  size_t bytes_written;

  for(;;) {
    if (i2s_write(DAC_I2S_NUMBER, buffer, used_buffer,
      &bytes_written, BUFFER_TIMEOUT_MS/portTICK_PERIOD_MS) == ESP_OK) {
      if (bytes_written != used_buffer) {
        Serial.println("i2s write cut");
      }
    }
    else {
      Serial.println("i2s write failed");
    }

    delay(1);
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

  used_buffer = gen_sine<dac_sample_t>(buffer, BUFFER_SIZE, HZ, SAMPLE_RATE);

  Serial.printf("Rate %u samples/s, buffer %u bytes, timeout %u ms\n", 
    SAMPLE_RATE, BUFFER_SIZE, BUFFER_TIMEOUT_MS);
  Serial.printf("Pins BCK %d, LRCK %d, DATA %d, LED %d, BTN %d\n",
    CONFIG_I2S_OUT_BCK_PIN, CONFIG_I2S_OUT_LRCK_PIN, 
    CONFIG_I2S_OUT_DATA_PIN, LED_PIN, BTN_PIN);

  start_dac(used_buffer);
  stop_dac();
  start_dac(used_buffer);

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
