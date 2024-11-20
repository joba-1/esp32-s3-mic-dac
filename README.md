# Simple Sine Wave I2S DAC Player

used to find suitable pins for I2S on my ESP32 S3s

Works on classic ESP32 and on S3 :)

Tried pins 11, 12, 13 or 4, 5, 6. They do not work but produce different mixed sine wave patterns. Probably BCK pin too slow?

Thanks to a hint from PSchatzmann :) I switched roles of pins and chose other pins based on their normal usage for SPI:

These two combinations work well:

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

Success was independent from platform versions. 
Now that I found a working fork of arduino-espressif32 platform,
where Arduino 3.x and IDF 5.x are running, I also moved to new I2S API.
Much simpler, way less possibilities to shoot oneself ito the foot... :) 
 
