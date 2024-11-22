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

// auto fall back to MP3 if AAC file not available
#define AAC_FILENAME "/kirk.aac"
#define MP3_FILENAME "/360.mp3"
#define MJPEG_FILENAME "/kirk.mjpeg"

// #define AAC_FILENAME "/nacho.aac"
// #define MJPEG_FILENAME "/nacho.mjpeg"

#define AUDIOASSIGNCORE 0
#define DECODEASSIGNCORE 0
#define DRAWASSIGNCORE 1

#define DEBUG true

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugf(...) Serial.printf(__VA_ARGS__)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugf(...)
#define debugln(x)
#endif

#define FPS 25
#define MJPEG_BUFFER_SIZE (288 * 250 * 2 / 8)

#define BATSENS 11
#define BUTTON1 10
#define BUTTON2 12

#define GFX_RST 48  // 42
#define GFX_BL 42   // 48
#define GFX_DC 40   // 41
#define GFX_CS 41   // 10
#define GFX_SCK 21  // 12
#define GFX_MOSI 47 // 11

#define I2S_MCLK -1
#define I2S_SCLK 8 // 5
#define I2S_LRCK 9 // 6
#define I2S_DOUT 7 // 4

#define SD_MMC_CLK 3 // 7
#define SD_MMC_CMD 4 // 15
#define SD_MMC_D0 2  // 16
#define SD_MMC_D1 1  // 17
#define SD_MMC_D2 6  // 18
#define SD_MMC_D3 5  // 45

#define CHART_MARGIN 64
#define LEGEND_A_COLOR 0x1BB6
#define LEGEND_B_COLOR 0xFBE1
#define LEGEND_C_COLOR 0x2D05
#define LEGEND_D_COLOR 0xD125
#define LEGEND_E_COLOR 0x9337
#define LEGEND_F_COLOR 0x8AA9
#define LEGEND_G_COLOR 0xE3B8
#define LEGEND_H_COLOR 0x7BEF
#define LEGEND_I_COLOR 0xBDE4
#define LEGEND_J_COLOR 0x15F9

#include <Arduino.h>
#include <WiFi.h>
#include <FS.h>
#include <LittleFS.h>
#include <SPIFFS.h>
#include <FFat.h>
#include <SD_MMC.h>
#include <vector>
#include <string>
#include <esp_log.h>
#include <esp_task_wdt.h> // Include watchdog control
/* audio */
#include "esp32_audio_task.h"
/* MJPEG Video */
#include "mjpeg_decode_draw_task.h"
/* Arduino_GFX */
#include <Arduino_GFX_Library.h>
#include <Button.h>

/* functions */
void showStats();
static int drawMCU(JPEGDRAW *pDraw); // pixel drawing callback
void listFilesByExtension(fs::FS &fs, const char *dirname, std::vector<String> &videoFiles, std::vector<String> &audioFiles);

/* variables */
static int next_frame = 0;
static int skipped_frames = 0;
static unsigned long start_ms, curr_ms, next_frame_ms;

int current_video = 0;
int current_audio = 0;
bool is_playing = true;
bool is_muted = false;
int volume_level = 5; // Startlautstärke

unsigned long LeftButtonsMillis = 0;
unsigned long RightButtonsMillis = 0;

std::vector<String> videoFiles;
std::vector<String> audioFiles;

// Arduino_DataBus *bus = new Arduino_HWSPI(GFX_DC /* DC */, GFX_CS /* CS */); //, GFX_SCK /* SCK */, GFX_MOSI /* MOSI */, GFX_NOT_DEFINED /* MISO */);
// Arduino_DataBus *bus = new Arduino_ESP32SPI(GFX_DC /* DC */, GFX_CS /* CS */, GFX_SCK /* SCK */, GFX_MOSI /* MOSI */, GFX_NOT_DEFINED /* MISO */, FSPI, true);
Arduino_DataBus *bus = new Arduino_ESP32SPIDMA(GFX_DC, GFX_CS, GFX_SCK, GFX_MOSI, GFX_NOT_DEFINED, HSPI, false);
Arduino_GFX *gfx = new Arduino_ST7789(bus, GFX_RST, 1 /* rotation */, true /* IPS */, 240 /* width */, 288 /* height */, 0 /* col offset 1 */, 20 /* row offset 1 */, 0 /* col offset 2 */, 12 /* row offset 2 */);

Button LeftButton(BUTTON1);
Button RightButton(BUTTON2);

void setup()
{
  disableCore0WDT();
  WiFi.mode(WIFI_OFF);

  Serial.setTxTimeoutMs(5); // set USB CDC Time TX
  Serial.begin(115200);

  esp_log_level_set("*", ESP_LOG_NONE);

  // Init Display
  gfx->begin(80000000);
  gfx->fillScreen(BLACK);
  // gfx->setTextSize(2);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  debugln("Init I2S");

  esp_err_t ret_val = i2s_init(I2S_NUM_0, 44100, I2S_MCLK /* MCLK */, I2S_SCLK /* SCLK */, I2S_LRCK /* LRCK */, I2S_DOUT /* DOUT */, -1 /* DIN */);

  if (ret_val != ESP_OK)
  {
    debugf("i2s_init failed: %d\n", ret_val);
  }
  i2s_zero_dma_buffer(I2S_NUM_0);

  debugln("Init FS");

  if (!SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0, SD_MMC_D1, SD_MMC_D2, SD_MMC_D3))
  {
    debugln("Pin change failed!");
    return;
  }

  if ((!SD_MMC.begin("/root", false)) && (!SD_MMC.begin("/root", false)) && (!SD_MMC.begin("/root", false)) && (!SD_MMC.begin("/root", false)))
  {
    debugln("ERROR: File system mount failed!");
  }
  else
  {
    listFilesByExtension(SD_MMC, "/", videoFiles, audioFiles);

    uint8_t cardType = SD_MMC.cardType();

    debug("SD Card Type: ");
    if (cardType == CARD_MMC)
    {
      debugln("MMC");
    }
    else if (cardType == CARD_SD)
    {
      debugln("SDSC");
    }
    else if (cardType == CARD_SDHC)
    {
      debugln("SDHC");
    }
    else
    {
      debugln("UNKNOWN");
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    debugf("SD Card Size: %lluMB\n", cardSize);

    debugln("\nVideo files:");
    for (const auto &file : videoFiles)
    {
      debugln(file);
    }
    delay(200);
    debugln("\nAudio files:");

    for (const auto &file : audioFiles)
    {
      debugln(file);
    }

    bool aac_file_available = false;
    debugln("\nOpen AAC file: " AAC_FILENAME);

    File aFile = SD_MMC.open(AAC_FILENAME);

    if (aFile)
    {
      aac_file_available = true;
    }
    else
    {
      debugln("Open MP3 file: " MP3_FILENAME);
      aFile = SD_MMC.open(MP3_FILENAME);
    }

    if (!aFile || aFile.isDirectory())
    {
      debugln("ERROR: Failed to open " AAC_FILENAME " or " MP3_FILENAME " file for reading");
    }
    else
    {
      debugln("Open MJPEG file: " MJPEG_FILENAME);

      File vFile = SD_MMC.open(MJPEG_FILENAME);

      if (!vFile || vFile.isDirectory())
      {
        debugln("ERROR: Failed to open " MJPEG_FILENAME " file for reading");
      }
      else
      {
        debugln("Init video");

        mjpeg_setup(&vFile, MJPEG_BUFFER_SIZE, drawMCU,
                    false /* useBigEndian */, DECODEASSIGNCORE, DRAWASSIGNCORE);

        debugln("Start play audio task");

        BaseType_t ret_val;
        if (aac_file_available)
        {
          ret_val = aac_player_task_start(&aFile, AUDIOASSIGNCORE);
        }
        else
        {
          ret_val = mp3_player_task_start(&aFile, AUDIOASSIGNCORE);
        }
        if (ret_val != pdPASS)
        {
          debugf("Audio player task start failed: %d\n", ret_val);
        }

        debugln("Start play video");

        start_ms = millis();
        curr_ms = millis();
        next_frame_ms = start_ms + (++next_frame * 1000 / FPS / 2);

        while (vFile.available() && mjpeg_read_frame()) // Read video
        {
          total_read_video_ms += millis() - curr_ms;
          curr_ms = millis();

          if (millis() < next_frame_ms) // check show frame or skip frame
          {
            // Play video
            mjpeg_draw_frame();
            total_decode_video_ms += millis() - curr_ms;
            curr_ms = millis();
          }
          else
          {
            ++skipped_frames;
            // debugln("Skip frame");
          }

          while (millis() < next_frame_ms)
          {
            vTaskDelay(pdMS_TO_TICKS(1));
          }

          curr_ms = millis();
          next_frame_ms = start_ms + (++next_frame * 1000 / FPS);
        }
        debugln("AV end");

        vFile.close();
        aFile.close();
      }
      delay(200);

      showStats();

      // delay(60000);
#ifdef GFX_BL
      // digitalWrite(GFX_BL, LOW);
#endif
      // gfx->displayOff();
      // esp_deep_sleep_start();
    }
  }
}

void loop()
{
  esp_task_wdt_reset(); // Reset watchdog in case of long operations
}

// pixel drawing callback
static int drawMCU(JPEGDRAW *pDraw)
{
  // debugf("Draw pos = (%d, %d), size = %d x %d\n", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
  unsigned long s = millis();
  gfx->draw16bitRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  total_show_video_ms += millis() - s;
  return 1;
} /* drawMCU() */

// shows stats
void showStats()
{

  gfx->fillScreen(BLACK);

  int time_used = millis() - start_ms;
  int total_frames = next_frame - 1;

  int played_frames = total_frames - skipped_frames;
  float fps = 1000.0 * played_frames / time_used;
  total_decode_audio_ms -= total_play_audio_ms;

  debugf("Played frames: %d\n", played_frames);
  debugf("Skipped frames: %d (%0.1f %%)\n", skipped_frames, 100.0 * skipped_frames / total_frames);
  debugf("Time used: %d ms\n", time_used);
  debugf("Expected FPS: %d\n", FPS);
  debugf("Actual FPS: %0.1f\n", fps);
  debugf("Read audio: %lu ms (%0.1f %%)\n", total_read_audio_ms, 100.0 * total_read_audio_ms / time_used);
  debugf("Decode audio: %lu ms (%0.1f %%)\n", total_decode_audio_ms, 100.0 * total_decode_audio_ms / time_used);
  debugf("Play audio: %lu ms (%0.1f %%)\n", total_play_audio_ms, 100.0 * total_play_audio_ms / time_used);
  debugf("Read video: %lu ms (%0.1f %%)\n", total_read_video_ms, 100.0 * total_read_video_ms / time_used);
  debugf("Decode video: %lu ms (%0.1f %%)\n", total_decode_video_ms, 100.0 * total_decode_video_ms / time_used);
  debugf("Show video: %lu ms (%0.1f %%)\n", total_show_video_ms, 100.0 * total_show_video_ms / time_used);

  gfx->setCursor(0, 50);
  gfx->setTextColor(WHITE);
  gfx->printf("Played frames: %d\n", played_frames);
  delay(50);
  gfx->printf("Skipped frames: %d (%0.1f %%)\n", skipped_frames, 100.0 * skipped_frames / total_frames);
  delay(50);
  gfx->printf("Time used: %d ms\n", time_used);
  delay(50);
  gfx->printf("Expected FPS: %d\n", FPS);
  delay(50);
  gfx->printf("Actual FPS: %0.1f\n\n", fps);
  delay(50);

  int16_t r1 = ((gfx->height() - CHART_MARGIN - CHART_MARGIN) / 2);
  int16_t r2 = r1 / 2;
  int16_t cx = gfx->width() - r1 - 10;
  int16_t cy = r1 + CHART_MARGIN;

  float arc_start1 = 0;
  float arc_end1 = arc_start1 + max(2.0, 360.0 * total_read_audio_ms / time_used);
  for (int i = arc_start1 + 1; i < arc_end1; i += 2)
  {
    gfx->fillArc(cx, cy, r1, r2, arc_start1 - 90.0, i - 90.0, LEGEND_A_COLOR);
    delay(5);
  }
  gfx->fillArc(cx, cy, r1, r2, arc_start1 - 90.0, arc_end1 - 90.0, LEGEND_A_COLOR);
  delay(50);
  gfx->setTextColor(LEGEND_A_COLOR);
  gfx->printf("Read audio: %lu ms (%0.1f %%)\n", total_read_audio_ms, 100.0 * total_read_audio_ms / time_used);
  delay(50);

  float arc_start2 = arc_end1;
  float arc_end2 = arc_start2 + max(2.0, 360.0 * total_decode_audio_ms / time_used);
  for (int i = arc_start2 + 1; i < arc_end2; i += 2)
  {
    gfx->fillArc(cx, cy, r1, r2, arc_start2 - 90.0, i - 90.0, LEGEND_B_COLOR);
    delay(5);
  }
  gfx->fillArc(cx, cy, r1, r2, arc_start2 - 90.0, arc_end2 - 90.0, LEGEND_B_COLOR);
  delay(50);
  gfx->setTextColor(LEGEND_B_COLOR);
  gfx->printf("Decode audio: %lu ms (%0.1f %%)\n", total_decode_audio_ms, 100.0 * total_decode_audio_ms / time_used);
  delay(50);
  gfx->setTextColor(LEGEND_J_COLOR);
  gfx->printf("Play audio: %lu ms (%0.1f %%)\n", total_play_audio_ms, 100.0 * total_play_audio_ms / time_used);
  delay(50);

  float arc_start3 = arc_end2;
  float arc_end3 = arc_start3 + max(2.0, 360.0 * total_read_video_ms / time_used);
  for (int i = arc_start3 + 1; i < arc_end3; i += 2)
  {
    gfx->fillArc(cx, cy, r1, r2, arc_start3 - 90.0, i - 90.0, LEGEND_C_COLOR);
    delay(5);
  }
  gfx->fillArc(cx, cy, r1, r2, arc_start3 - 90.0, arc_end3 - 90.0, LEGEND_C_COLOR);
  delay(50);
  gfx->setTextColor(LEGEND_C_COLOR);
  gfx->printf("Read video: %lu ms (%0.1f %%)\n", total_read_video_ms, 100.0 * total_read_video_ms / time_used);
  delay(50);

  float arc_start4 = arc_end3;
  float arc_end4 = arc_start4 + max(2.0, 360.0 * total_show_video_ms / time_used);
  for (int i = arc_start4 + 1; i < arc_end4; i += 2)
  {
    gfx->fillArc(cx, cy, r1, r2, arc_start4 - 90.0, i - 90.0, LEGEND_D_COLOR);
    delay(5);
  }
  gfx->fillArc(cx, cy, r1, r2, arc_start4 - 90.0, arc_end4 - 90.0, LEGEND_D_COLOR);
  delay(50);
  gfx->setTextColor(LEGEND_D_COLOR);
  gfx->printf("Show video: %lu ms (%0.1f %%)\n", total_show_video_ms, 100.0 * total_show_video_ms / time_used);
  delay(50);

  float arc_start5 = 0;
  float arc_end5 = arc_start5 + max(2.0, 360.0 * total_decode_video_ms / time_used);
  for (int i = arc_start5 + 1; i < arc_end5; i += 2)
  {
    gfx->fillArc(cx, cy, r2, 0, arc_start5 - 90.0, i - 90.0, LEGEND_E_COLOR);
    delay(5);
  }
  gfx->fillArc(cx, cy, r2, 0, arc_start5 - 90.0, arc_end5 - 90.0, LEGEND_E_COLOR);
  delay(50);
  gfx->setTextColor(LEGEND_E_COLOR);
  gfx->printf("Decode video: %lu ms (%0.1f %%)\n", total_decode_video_ms, 100.0 * total_decode_video_ms / time_used);
}

// lists files by extension
void listFilesByExtension(fs::FS &fs, const char *dirname, std::vector<String> &videoFiles, std::vector<String> &audioFiles)
{
  // debugf("Listing directory: %s\n", dirname);
  File root = fs.open(dirname);
  if (!root)
  {
    debugln("Failed to open directory");
    return;
  }
  if (!root.isDirectory())
  {
    debugln("Not a directory");
    return;
  }
  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      // debug(" DIR : ");
      // debugln(file.name());
      listFilesByExtension(fs, file.path(), videoFiles, audioFiles); // Recursively call the function for subdirectories
    }
    else
    {
      // debug(file.name());
      // debug("  ");
      // debugln(file.size());
      std::string filename = file.name();
      if (filename.find(".mjpeg") != std::string::npos)
      {
        videoFiles.push_back(String(filename.c_str())); // Convert to String and push to the vector
      }
      else if (filename.find(".aac") != std::string::npos)
      {
        audioFiles.push_back(String(filename.c_str())); // Convert to String and push to the vector
      }
    }
    file = root.openNextFile();
  }
  file.close();
}
