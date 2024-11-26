#include "player.h"
#include <Arduino.h>
#include <SD_MMC.h>
#include <esp_heap_caps.h>
#include <string>
#include <map>
#include <vector>
/* audio */
#include "esp32_audio_task.h"
/* MJPEG Video */
#include "mjpeg_decode_draw_task.h"
#include <Arduino_GFX_Library.h>

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

#define SD_MMC_CLK 3 // 7
#define SD_MMC_CMD 4 // 15
#define SD_MMC_D0 2  // 16
#define SD_MMC_D1 1  // 17
#define SD_MMC_D2 6  // 18
#define SD_MMC_D3 5  // 45

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

#define READ_BUFFER_SIZE 4096
#define MAXOUTPUTSIZE (288 / 3 / 16)
#define NUMBER_OF_DECODE_BUFFER 4
#define NUMBER_OF_DRAW_BUFFER 24

#define FPS 25
#define MJPEG_BUFFER_SIZE (288 * 250 * 2 / 8)

#define AUDIOASSIGNCORE 1
#define DECODEASSIGNCORE 0
#define DRAWASSIGNCORE 0

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

// Define the static member variables
libhelix::AACDecoderHelix Player::_aac;
TaskHandle_t Player::TaskHandle_0 = NULL;

std::vector<String> videoFiles;
std::vector<String> audioFiles;

int current_video=0;
int current_audio=0;

JPEGDEC Player::_jpegDec;
xQueueHandle Player::_xqh = NULL;
bool Player::_useBigEndian = false;
Stream *Player::_input = NULL;
int32_t Player::_mjpegBufSize = 0;
uint8_t *Player::_read_buf = NULL;
int32_t Player::_mjpeg_buf_offset = 0;
TaskHandle_t Player::_decodeTask = NULL;
TaskHandle_t Player::_draw_task = NULL;
paramDecodeTask Player::_pDecodeTask;
paramDrawTask Player::_pDrawTask;
uint8_t *Player::_mjpeg_buf = NULL;
uint8_t Player::_mBufIdx = 0;
int32_t Player::_inputindex = 0;
int32_t Player::_buf_read = 0;
int32_t Player::_remain = 0;
mjpegBuf Player::_mjpegBufs[NUMBER_OF_DECODE_BUFFER];
JPEGDRAW Player::jpegdraws[NUMBER_OF_DRAW_BUFFER];
int Player::_draw_queue_cnt = 0;

/* variables */
bool sdcard = false;
static int next_frame = 0;
static int skipped_frames = 0;
static unsigned long start_ms, curr_ms, next_frame_ms;

Arduino_DataBus *bus = NULL;
Arduino_GFX *gfx = NULL;

Player::Player() : vFileOpen(false), aFileOpen(false), start_ms(0), curr_ms(0), next_frame_ms(0), next_frame(0), total_read_video_ms(0), total_decode_video_ms(0), skipped_frames(0) {}

void Player::init()
{
  debug_memory_usage();

  // Initialize SD card if not already initialized
  debugln("Init FS");

  if (!SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0, SD_MMC_D1, SD_MMC_D2, SD_MMC_D3))
  {
    debugln("Pin change failed!");
    return;
  }

  // Init Display
  bus = new Arduino_ESP32SPIDMA(GFX_DC, GFX_CS, GFX_SCK, GFX_MOSI, GFX_NOT_DEFINED, HSPI, false);
  gfx = new Arduino_ST7789(bus, GFX_RST, 1 /* rotation */, true /* IPS */, 240 /* width */, 288 /* height */, 0 /* col offset 1 */, 20 /* row offset 1 */, 0 /* col offset 2 */, 12 /* row offset 2 */);
  gfx->begin(80000000);
  gfx->fillScreen(BLACK);

  debugln("Init I2S");

  esp_err_t ret_val = i2s_init(I2S_NUM_0, 44100, I2S_MCLK /* MCLK */, I2S_SCLK /* SCLK */, I2S_LRCK /* LRCK */, I2S_DOUT /* DOUT */, -1 /* DIN */);

  if (ret_val != ESP_OK)
  {
    debugf("i2s_init failed: %d\n", ret_val);
    return;
  }
  i2s_zero_dma_buffer(I2S_NUM_0);

  if (!SD_MMC.begin())
  {
    debugln("SD Card initialization failed!");
    return;
  }
  debugln("SD Card initialized.");

  getFiles();
}

void Player::start(const std::string &videoFile)
{
  debugln("Starting new video playback");
  debug_memory_usage();

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

  debugln("\nOpen AAC file: " + audioFiles[current_audio]);

  if (audioFiles[current_audio] != "X")
  {
    set_volume(0.5);
    aFile = SD_MMC.open(audioFiles[current_audio].c_str());
    aFileOpen = true;
  }
  else
  {
    debugln("No Sound");
  }

  debugln("Open MJPEG File: "+ videoFiles[current_video]);

  vFile = SD_MMC.open(videoFiles[current_video]);
  vFileOpen = true;

  if (!vFile || vFile.isDirectory())
  {
    debugln("ERROR: Failed to open file for reading");
  }
  else
  {
    debugln("Init video");

    mjpeg_setup(&vFile, MJPEG_BUFFER_SIZE, drawMCU, false /* useBigEndian */, DECODEASSIGNCORE, DRAWASSIGNCORE);

    debugln("Start play audio task");

    BaseType_t ret_val;
    if (audioFiles[current_audio] != "X")
    {
      ret_val = aac_player_task_start(&aFile, AUDIOASSIGNCORE);
      set_volume(0.5);

      if (ret_val != pdPASS)
      {
        debugf("Audio player task start failed: %d\n", ret_val);
      }
    }

    debugln("Start play video");

    start_ms = millis();
    curr_ms = millis();
    next_frame_ms = start_ms + (++next_frame * 1000 / FPS / 2);

    while (vFile.available() && mjpeg_read_frame())
    { // Read video
      total_read_video_ms += millis() - curr_ms;
      curr_ms = millis();

      if (millis() < next_frame_ms)
      { // check show frame or skip frame
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
  }
}

void Player::stop()
{
  debug_memory_usage();

  // Stop the AAC decoder
  _aac.end();
  delay(20);

  // Delete the AAC player task
  if (TaskHandle_0 != NULL)
  {
    vTaskDelete(TaskHandle_0);
    TaskHandle_0 = NULL;
  }

  // Close the audio file
  if (aFileOpen)
  {
    aFile.close();
    aFileOpen = false;
  }

  // Delete the decode task
  if (_decodeTask != NULL)
  {
    vTaskDelete(_decodeTask);
    _decodeTask = NULL;
  }

  // Delete the draw task
  if (_draw_task != NULL)
  {
    vTaskDelete(_draw_task);
    _draw_task = NULL;
  }

  // Close the video file
  if (vFileOpen)
  {
    vFile.close();
    vFileOpen = false;
  }

  // Free the read buffer
  if (_read_buf != NULL)
  {
    free(_read_buf);
    _read_buf = NULL;
  }

  // Free the decode buffers
  for (int i = 0; i < NUMBER_OF_DECODE_BUFFER; ++i)
  {
    if (_mjpegBufs[i].buf != NULL)
    {
      free(_mjpegBufs[i].buf);
      _mjpegBufs[i].buf = NULL;
    }
  }

  // Free the draw buffers
  for (int i = 0; i < NUMBER_OF_DRAW_BUFFER; ++i)
  {
    if (jpegdraws[i].pPixels != NULL)
    {
      free(jpegdraws[i].pPixels);
      jpegdraws[i].pPixels = NULL;
    }
  }

  // Delete the queues
  if (_xqh != NULL)
  {
    vQueueDelete(_xqh);
    _xqh = NULL;
  }

  if (_pDecodeTask.xqh != NULL)
  {
    vQueueDelete(_pDecodeTask.xqh);
    _pDecodeTask.xqh = NULL;
  }

  if (_pDrawTask.xqh != NULL)
  {
    vQueueDelete(_pDrawTask.xqh);
    _pDrawTask.xqh = NULL;
  }

  // Reset static variables
  _mjpeg_buf_offset = 0;
  _mBufIdx = 0;
  _inputindex = 0;
  _buf_read = 0;
  _remain = 0;
  _draw_queue_cnt = 0;

  debugln("Files closed");
  delay(100);
  debug_memory_usage();
}

void Player::debug_memory_usage()
{
  size_t free_heap = esp_get_free_heap_size();
  size_t min_free_heap = esp_get_minimum_free_heap_size();
  debugf("Free heap: %u bytes, Minimum free heap: %u bytes\n", free_heap, min_free_heap);
}

// Function to set the volume
void Player::set_volume(float volume)
{
  //volume_scale = volume;
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

void getFiles()
{
  if (!SD_MMC.begin("/root", false))
  {
    debugln("ERROR: File system mount failed!");
    return;
  }
  sdcard = true;
  debugln("SD Card found...");

  listFilesByExtension(SD_MMC, videoFiles, audioFiles);

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
}

void scanDirectory(fs::FS &fs, String dirname, std::map<std::string, std::string> &fileMap)
{
  File root = fs.open(dirname.c_str());
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
      String dir = dirname + "/" + file.name();
      // Recursively call the function for subdirectories
      scanDirectory(fs, dir, fileMap);
    }
    else
    {
      std::string filename = file.name();
      std::string baseName = filename.substr(0, filename.find_last_of("."));

      if (filename.find(".mjpeg") != std::string::npos)
      {
        // Check if the corresponding .aac file exists
        if (fileMap.find(baseName) == fileMap.end())
        {
          fileMap[baseName] = ""; // Mark the presence of .mjpeg file
        }
        // debugf("Found video file: %s\n", filename.c_str());
      }
      else if (filename.find(".aac") != std::string::npos)
      {
        fileMap[baseName] = filename; // Store the .aac file
                                      // debugf("Found audio file: %s\n", filename.c_str());
      }
    }
    file = root.openNextFile();
  }
  file.close();
}

void populateVectorsFromMap(const std::map<std::string, std::string> &fileMap, std::vector<String> &videoFiles, std::vector<String> &audioFiles)
{
  // Populate the vectors based on the map
  for (const auto &pair : fileMap)
  {
    videoFiles.push_back(("/" + pair.first + ".mjpeg").c_str());
    if (!pair.second.empty())
    {
      audioFiles.push_back(("/" + pair.second).c_str());
    }
    else
    {
      audioFiles.push_back("X");
    }
    debugf("Added video: %s, audio: %s\n", videoFiles.back().c_str(), audioFiles.back().c_str());
  }
}

void listFilesByExtension(fs::FS &fs, std::vector<String> &videoFiles, std::vector<String> &audioFiles)
{
  std::map<std::string, std::string> fileMap; // Map to store filenames without extensions and their corresponding .aac files
  scanDirectory(fs, "/", fileMap);
  populateVectorsFromMap(fileMap, videoFiles, audioFiles);
}

// shows stats
void showStats()
{
  gfx->fillScreen(BLACK);

  int time_used = millis() - start_ms;
  int total_frames = next_frame - 1;

  int played_frames = total_frames - skipped_frames;
  float fps = 1000.0 * played_frames / time_used;
  total_decode_audio_ms -= total_play_audio_ms;

  debugln("Show Stats");

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
