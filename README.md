# Simple I2S MIC to DAC Player

2 way mic/dac intercom over serial line 
WIP, used to test new Arduino 3.x I2S API on my ESP32 S3s

## Done
* send mic over serial (1 channel stereo mic frames reduced to mono)
* receive dac over serial (mono duplicated to both dac channels)
* sync serial comms on buffer borders with magic bytes
* combine send and receive for normal serial full duplex connection.
* different sampling rates and sample sizes (tested up to 16k/32bit)
* rgb breathing feedback: green=maintenance loop, red=mic loop, blue=dac loop
* low latency with few and small buffers

## Todo
* half duplex RS485 with protocol for collision avoidance
* test shutdown and restart mic and dac

These pin combinations work well:

```
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
```
