/***
 * based on the project of: https://github.com/galbraithmedia1/Mini-Tv-ESP32
 *
 * Changes:
 *
 * 2024 artdanion
 *
 */

// Version for ESP32-S3 Dev Board with SD_MMC Card

//  Audio and video code

// ffmpeg -i  office1.mp4 -ar 44100 -ac 1 -ab 24k -filter:a loudnorm -filter:a "volume=-5dB" office1.aac
// ffmpeg -i office1.mp4 -vf "fps=25,scale=-1:240:flags=lanczos,crop=288:in_h:(in_w-288)/2:0" -q:v 11 office1.mjpeg

#define AAC_FILENAME "/kirk.aac"
#define MJPEG_FILENAME "/kirk.mjpeg"

#define INPUTASSIGNCORE 1

#define DEBUG true

#if DEBUG == true
#define debug(x) Serial.print(x)
#define debugf(...) Serial.printf(__VA_ARGS__)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugf(...)
#define debugln(x)
#endif

#define BATSENS 11
#define BUTTON1 10
#define BUTTON2 12

#include <Arduino.h>
#include <WiFi.h>
#include <FS.h>
#include <vector>
#include <map>
#include <string>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>
#include <player.h>
#include <Button.h>

/* functions */
void input_task(void *param);

bool is_muted = false;
float volume_level = 0.5; // Startlautstärke

unsigned long LeftButtonsMillis = 0;
unsigned long RightButtonsMillis = 0;

TaskHandle_t TaskHandle_1;

Button LeftButton(BUTTON1);
Button RightButton(BUTTON2);

Player player;

void setup()
{
  disableCore0WDT();
  disableCore1WDT();

  WiFi.mode(WIFI_OFF);

  Serial.setTxTimeoutMs(5); // set USB CDC Time TX
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

  xTaskCreatePinnedToCore(input_task, "Button Task", 4096, NULL, (UBaseType_t)configMAX_PRIORITIES - 1, &TaskHandle_1, INPUTASSIGNCORE);

  player.init();
  delay(100);
  
  player.start(videoFiles[current_video].c_str());
  //player.set_volume(volume_level);
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

      if (current_audio >= audioFiles.size())
        current_audio = 0;

      debugln("Stopping player");

      player.stop();
      delay(200);
      debugln("Starting player");
      player.start(videoFiles[current_video].c_str());
      debugln("next Video");
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
        player.set_volume(0.0);
      else
        player.set_volume(0.6);
      LeftButtonsMillis = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Delay for 20 milliseconds
  }
}

