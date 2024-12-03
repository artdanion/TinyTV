#include <WiFi.h>
#include <FS.h>
#include <SD_MMC.h>
#include <JPEGDEC.h>
#include <Audio.h>
#include <Arduino_GFX_Library.h>
#include "player.h"

#define DECODEASSIGNCORE 0
#define DRAWASSIGNCORE 0

#define READ_BUFFER_SIZE 4096
#define MAXOUTPUTSIZE (288 / 3 / 16)
#define NUMBER_OF_DECODE_BUFFER 4
#define NUMBER_OF_DRAW_BUFFER 24

#define FPS 25
#define MJPEG_BUFFER_SIZE (288 * 250 * 2 / 8)

#define GFX_RST 48
#define GFX_BL 42
#define GFX_DC 40
#define GFX_CS 41
#define GFX_SCK 21
#define GFX_MOSI 47

#define SD_MMC_CLK 3
#define SD_MMC_CMD 4
#define SD_MMC_D0 2
#define SD_MMC_D1 1
#define SD_MMC_D2 6
#define SD_MMC_D3 5

Arduino_DataBus *bus = new Arduino_ESP32SPIDMA(GFX_DC, GFX_CS, GFX_SCK, GFX_MOSI, GFX_NOT_DEFINED, HSPI, false);
Arduino_GFX *gfx = new Arduino_ST7789(bus, GFX_RST, 1 /* rotation */, true /* IPS */, 240 /* width */, 288 /* height */, 0 /* col offset 1 */, 20 /* row offset 1 */, 0 /* col offset 2 */, 12 /* row offset 2 */);

VideoPlayer *videoPlayer;

void videoTask(void *param) {
  videoPlayer->task();
}

void setup() {
  WiFi.mode(WIFI_OFF);

  Serial.setTxTimeoutMs(5);  // set USB CDC Time TX
  Serial.begin(115200);

  delay(5000);

  esp_log_level_set("*", ESP_LOG_NONE);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  LeftButton.begin();
  RightButton.begin();
  delay(100);

  xTaskCreatePinnedToCore(input_task, "Button Task", 4096, NULL, (UBaseType_t)configMAX_PRIORITIES - 1, &inputHandle, INPUTASSIGNCORE);

  player.init();

  delay(100);

  current_video++;
  current_audio++;

  player.start(videoFiles[current_video].c_str());
  player.setVolume(15);
}

void loop()
{
  esp_task_wdt_reset(); // Reset watchdog in case of long operations
}

void input_task(void *param)
{
  for (;;)
  {
    if (LeftButton.pressed())
    {
      debugln("Button pressed");
      current_video++;
      current_audio++;

      if (current_video >= videoFiles.size())
        current_video = 0;

      // if (current_audio >= audioFiles.size())
      //   current_audio = 0;

      player.stop();
     
      debugln("Starting player");
      debugln("next Video");
      player.start(videoFiles[current_video].c_str());
    }

    if (LeftButton.released())
    {
      debugln("Button released");
      if (millis() - LeftButtonsMillis > 1000)
      {
        is_muted = !is_muted;
        debugln("mute");
      }
      if (is_muted)
        player.setVolume(0);
      else
        player.setVolume(15);
      LeftButtonsMillis = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Delay for 20 milliseconds
  }
}
