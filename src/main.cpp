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

  Serial.println("Init FS");

  if (!SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0, SD_MMC_D1, SD_MMC_D2, SD_MMC_D3)) {
    Serial.println("Pin change failed!");
    return;
  }

  // SD-Karte initialisieren
  if ((!SD_MMC.begin("/root", false)) && (!SD_MMC.begin("/root", false)) && (!SD_MMC.begin("/root", false)) && (!SD_MMC.begin("/root", false))) {
    Serial.println("ERROR: File system mount failed!");
  }

  // VideoPlayer initialisieren
  videoPlayer = new VideoPlayer(gfx, MJPEG_BUFFER_SIZE, READ_BUFFER_SIZE);
  videoPlayer->init();

  // Task erstellen
  xTaskCreate(videoTask, "Video Task", 8192, nullptr, 1, nullptr);

  // Video abspielen
  videoPlayer->play("/rick.mjpeg");
}

void loop() {
  delay(6000);

  videoPlayer->stop();

  videoPlayer->play("/rick.mjpeg");
  delay(6000);

  videoPlayer->stop();
  delay(6000);
}
