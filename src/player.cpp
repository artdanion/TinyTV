#include <Arduino.h>
#include <SD_MMC.h>
#include <esp_heap_caps.h>
#include <string>
#include <map>
#include <vector>
#include "driver/i2s.h"
#include "player.h"
#include "config.h"
#include <Audio.h>
#include "AACDecoderHelix.h"
#include <FS.h>
#include <JPEGDEC.h>
#include <Arduino_GFX_Library.h>

#define MAX_FRAME_SIZE 3200

std::vector<String> videoFiles;
std::vector<String> audioFiles;

int current_video = 0;
int current_audio = 0;

/* video task*/
int _draw_queue_cnt = 0;
JPEGDEC _jpegDec;
xQueueHandle _xqh;
bool _useBigEndian;

JPEGDRAW jpegdraws[NUMBER_OF_DRAW_BUFFER];
unsigned long total_read_video_ms = 0;
unsigned long total_decode_video_ms = 0;
unsigned long total_show_video_ms = 0;

Stream *_input;
int32_t _mjpegBufSize;
uint8_t *_read_buf;
int32_t _mjpeg_buf_offset = 0;

TaskHandle_t _decodeTask;
TaskHandle_t _drawTask;
paramDecodeTask _pDecodeTask;
paramDrawTask _pDrawTask;
uint8_t *_mjpeg_buf;
uint8_t _mBufIdx = 0;

int32_t _inputindex = 0;
int32_t _buf_read;
int32_t _remain = 0;
mjpegBuf _mjpegBufs[NUMBER_OF_DECODE_BUFFER];

/* audio task*/
QueueHandle_t audioSetQueue = NULL;
QueueHandle_t audioGetQueue = NULL;
struct audioMessage audioRxMessage;
struct audioMessage audioTxMessage;

static unsigned long total_read_audio_ms = 0;
static unsigned long total_decode_audio_ms = 0;
static unsigned long total_play_audio_ms = 0;

/* variables */
bool sdcard = false;
static int next_frame = 0;
static int skipped_frames = 0;
static unsigned long start_ms, curr_ms, next_frame_ms;

Arduino_DataBus *bus = NULL;
Arduino_GFX *gfx = NULL;
Audio audio;

//****************************************************************************************
//                                   PLAYER - CLASS                                      *
//****************************************************************************************

// Constructor
Player::Player() : vFileOpen(false),
                   aFileOpen(false),
                   start_ms(0),
                   curr_ms(0),
                   next_frame_ms(0),
                   next_frame(0),
                   total_read_video_ms(0),
                   total_decode_video_ms(0),
                   skipped_frames(0) {}

void Player::init()
{
  debug_memory_usage();
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

  if (!SD_MMC.begin())
  {
    debugln("SD Card initialization failed!");
    return;
  }
  debugln("SD Card initialized.");

  getFiles();

  // Initialize tasks and buffers using mjpeg_setup
  if (!mjpeg_setup(&vFile, MJPEG_BUFFER_SIZE, drawMCU, false /* useBigEndian */, DECODEASSIGNCORE, DRAWASSIGNCORE))
  {
    debugln("ERROR: Failed to initialize MJPEG setup");
    return;
  }

  audioInit();
}

void Player::start(const std::string &videoFile)
{
  debugln("Starting new video playback");
  debug_memory_usage();

  // Ensure the current playback is stopped
  stop();

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  debugf("SD Card Size: %lluMB\n", cardSize);

  debugln("Open MJPEG File: " + videoFiles[current_video]);

  vFile = SD_MMC.open(videoFiles[current_video]);
  vFileOpen = true;

  if (!vFile || vFile.isDirectory())
  {
    debugln("ERROR: Failed to open file for reading");
  }
  else
  {
    debugln("Init video");

    // Reinitialize the video setup with the new stream
    _input = &vFile; // Update the input stream
    _mjpeg_buf = _mjpegBufs[_mBufIdx].buf; // Reset the buffer pointer
    _mjpeg_buf_offset = 0; // Reset the buffer offset

    debugln("Start play audio task");

    if (audioFiles[current_audio] != "X")
    {
      audioConnecttoSD(audioFiles[current_audio].c_str());
    }
    else
    {
      debugln("No Sound");
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
    showStats();
  }
}

void Player::stop()
{
  debug_memory_usage();
  debugln("Stopping player");

  // Stop the AAC decoder
  delay(20);

  // Close the video file
  if (vFileOpen)
  {
    debugln("Closing video file");
    vFile.close();
    vFileOpen = false;
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

void Player::setVolume(int volume)
{
  audio.setVolume(volume);
}

//****************************************************************************************
//                                   A U D I O _ T A S K                                 *
//****************************************************************************************

void CreateQueues()
{
  audioSetQueue = xQueueCreate(10, sizeof(struct audioMessage));
  audioGetQueue = xQueueCreate(10, sizeof(struct audioMessage));
}

void audioInit()
{
  xTaskCreatePinnedToCore(
      audioTask,             /* Function to implement the task */
      "audioplay",           /* Name of the task */
      5000,                  /* Stack size in words */
      NULL,                  /* Task input parameter */
      2 | portPRIVILEGE_BIT, /* Priority of the task */
      NULL,                  /* Task handle. */
      AUDIOASSIGNCORE        /* Core where the task should run */
  );
}

void audioTask(void *parameter)
{
  CreateQueues();
  if (!audioSetQueue || !audioGetQueue)
  {
    log_e("queues are not initialized");
    while (true)
    {
      ;
    } // endless loop
  }

  struct audioMessage audioRxTaskMessage;
  struct audioMessage audioTxTaskMessage;

  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(15); // 0...21

  while (true)
  {
    if (xQueueReceive(audioSetQueue, &audioRxTaskMessage, 1) == pdPASS)
    {
      if (audioRxTaskMessage.cmd == SET_VOLUME)
      {
        audioTxTaskMessage.cmd = SET_VOLUME;
        audio.setVolume(audioRxTaskMessage.value);
        audioTxTaskMessage.ret = 1;
        xQueueSend(audioGetQueue, &audioTxTaskMessage, portMAX_DELAY);
      }
      else if (audioRxTaskMessage.cmd == CONNECTTOHOST)
      {
        audioTxTaskMessage.cmd = CONNECTTOHOST;
        audioTxTaskMessage.ret = audio.connecttohost(audioRxTaskMessage.txt);
        xQueueSend(audioGetQueue, &audioTxTaskMessage, portMAX_DELAY);
      }
      else if (audioRxTaskMessage.cmd == CONNECTTOSD)
      {
        audioTxTaskMessage.cmd = CONNECTTOSD;
        audioTxTaskMessage.ret = audio.connecttoFS(SD_MMC, audioRxTaskMessage.txt);
        xQueueSend(audioGetQueue, &audioTxTaskMessage, portMAX_DELAY);
      }
      else if (audioRxTaskMessage.cmd == GET_VOLUME)
      {
        audioTxTaskMessage.cmd = GET_VOLUME;
        audioTxTaskMessage.ret = audio.getVolume();
        xQueueSend(audioGetQueue, &audioTxTaskMessage, portMAX_DELAY);
      }
      else
      {
        log_i("error");
      }
    }
    audio.loop();
    if (!audio.isRunning())
    {
      sleep(1);
    }
  }
}

audioMessage transmitReceive(audioMessage msg)
{
  xQueueSend(audioSetQueue, &msg, portMAX_DELAY);
  if (xQueueReceive(audioGetQueue, &audioRxMessage, portMAX_DELAY) == pdPASS)
  {
    if (msg.cmd != audioRxMessage.cmd)
    {
      log_e("wrong reply from message queue");
    }
  }
  return audioRxMessage;
}

void audioSetVolume(uint8_t vol)
{
  audioTxMessage.cmd = SET_VOLUME;
  audioTxMessage.value = vol;
  audioMessage RX = transmitReceive(audioTxMessage);
}

uint8_t audioGetVolume()
{
  audioTxMessage.cmd = GET_VOLUME;
  audioMessage RX = transmitReceive(audioTxMessage);
  return RX.ret;
}

bool audioConnecttohost(const char *host)
{
  audioTxMessage.cmd = CONNECTTOHOST;
  audioTxMessage.txt = host;
  audioMessage RX = transmitReceive(audioTxMessage);
  return RX.ret;
}

bool audioConnecttoSD(const char *filename)
{
  audioTxMessage.cmd = CONNECTTOSD;
  audioTxMessage.txt = filename;
  audioMessage RX = transmitReceive(audioTxMessage);
  return RX.ret;
}

//****************************************************************************************
//                                   DECODE & DRAW _ T A S K                             *
//****************************************************************************************

int drawMCU(JPEGDRAW *pDraw)
{
  // debugf("Draw pos = (%d, %d), size = %d x %d\n", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
  unsigned long s = millis();
  gfx->draw16bitRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  total_show_video_ms += millis() - s;
  return 1;
} /* drawMCU() */

// ----------- Decode and Draw Task
int queueDrawMCU(JPEGDRAW *pDraw)
{
  int len = pDraw->iWidth * pDraw->iHeight * 2;
  JPEGDRAW *j = &jpegdraws[_draw_queue_cnt % NUMBER_OF_DRAW_BUFFER];
  j->x = pDraw->x;
  j->y = pDraw->y;
  j->iWidth = pDraw->iWidth;
  j->iHeight = pDraw->iHeight;
  memcpy(j->pPixels, pDraw->pPixels, len);

  // log_i("queueDrawMCU start.");
  ++_draw_queue_cnt;
  xQueueSend(_xqh, &j, portMAX_DELAY);
  // log_i("queueDrawMCU end.");

  return 1;
}

void decode_task(void *arg)
{
  paramDecodeTask *p = (paramDecodeTask *)arg;
  mjpegBuf *mBuf;
  log_i("decode_task start.");
  while (xQueueReceive(p->xqh, &mBuf, portMAX_DELAY))
  {
    // log_i("mBuf->size: %d", mBuf->size);
    // log_i("mBuf->buf start: %X %X, end: %X, %X.", mBuf->buf[0], mBuf->buf[1], mBuf->buf[mBuf->size - 2], mBuf->buf[mBuf->size - 1]);
    unsigned long s = millis();

    _jpegDec.openRAM(mBuf->buf, mBuf->size, p->drawFunc);

    // _jpegDec.setMaxOutputSize(MAXOUTPUTSIZE);
    if (_useBigEndian)
    {
      _jpegDec.setPixelType(RGB565_BIG_ENDIAN);
    }
    _jpegDec.setMaxOutputSize(MAXOUTPUTSIZE);
    _jpegDec.decode(0, 0, 0);
    _jpegDec.close();

    total_decode_video_ms += millis() - s;
  }
  vQueueDelete(p->xqh);
  log_i("decode_task end.");
  vTaskDelete(NULL);
}

void draw_task(void *arg)
{
  paramDrawTask *p = (paramDrawTask *)arg;
  JPEGDRAW *pDraw;
  log_i("draw_task start.");
  while (xQueueReceive(p->xqh, &pDraw, portMAX_DELAY))
  {
    // log_i("draw_task work start: x: %d, y: %d, iWidth: %d, iHeight: %d.", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
    p->drawFunc(pDraw);
    // log_i("draw_task work end.");
  }
  vQueueDelete(p->xqh);
  log_i("draw_task end.");
  vTaskDelete(NULL);
}

bool mjpeg_setup(Stream *input, int32_t mjpegBufSize, JPEG_DRAW_CALLBACK *pfnDraw,
                 bool useBigEndian, BaseType_t decodeAssignCore, BaseType_t drawAssignCore)
{
  _input = input;
  _mjpegBufSize = mjpegBufSize;
  _useBigEndian = useBigEndian;

  for (int i = 0; i < NUMBER_OF_DECODE_BUFFER; ++i)
  {
    _mjpegBufs[i].buf = (uint8_t *)malloc(mjpegBufSize);
    if (_mjpegBufs[i].buf)
    {
      log_i("#%d decode buffer allocated.", i);
    }
    else
    {
      log_e("#%d decode buffer allocat failed.", i);
    }
  }
  _mjpeg_buf = _mjpegBufs[_mBufIdx].buf;

  if (!_read_buf)
  {
    _read_buf = (uint8_t *)malloc(READ_BUFFER_SIZE);
  }
  if (_read_buf)
  {
    log_i("Read buffer allocated.");
  }

  _xqh = xQueueCreate(NUMBER_OF_DRAW_BUFFER, sizeof(JPEGDRAW));
  _pDrawTask.xqh = _xqh;
  _pDrawTask.drawFunc = pfnDraw;
  _pDecodeTask.xqh = xQueueCreate(NUMBER_OF_DECODE_BUFFER, sizeof(mjpegBuf));
  _pDecodeTask.drawFunc = queueDrawMCU;

  xTaskCreatePinnedToCore(
      (TaskFunction_t)decode_task,
      (const char *const)"MJPEG decode Task",
      (const uint32_t)2000,
      (void *const)&_pDecodeTask,
      (UBaseType_t)configMAX_PRIORITIES - 1,
      (TaskHandle_t *const)&_decodeTask,
      (const BaseType_t)decodeAssignCore);
  xTaskCreatePinnedToCore(
      (TaskFunction_t)draw_task,
      (const char *const)"MJPEG Draw Task",
      (const uint32_t)2000,
      (void *const)&_pDrawTask,
      (UBaseType_t)configMAX_PRIORITIES - 1,
      (TaskHandle_t *const)&_drawTask,
      (const BaseType_t)drawAssignCore);

  for (int i = 0; i < NUMBER_OF_DRAW_BUFFER; i++)
  {
    if (!jpegdraws[i].pPixels)
    {
      jpegdraws[i].pPixels = (uint16_t *)heap_caps_malloc(MAXOUTPUTSIZE * 16 * 16 * 2, MALLOC_CAP_DMA);
    }
    if (jpegdraws[i].pPixels)
    {
      log_i("#%d draw buffer allocated.", i);
    }
    else
    {
      log_e("#%d draw buffer allocat failed.", i);
    }
  }

  return true;
}

bool mjpeg_read_frame()
{
  if (_inputindex == 0)
  {
    _buf_read = _input->readBytes(_read_buf, READ_BUFFER_SIZE);
    _inputindex += _buf_read;
  }
  _mjpeg_buf_offset = 0;
  int i = 0;
  bool found_FFD8 = false;
  while ((_buf_read > 0) && (!found_FFD8))
  {
    i = 0;
    while ((i < _buf_read) && (!found_FFD8))
    {
      if ((_read_buf[i] == 0xFF) && (_read_buf[i + 1] == 0xD8)) // JPEG header
      {
        // log_i("Found FFD8 at: %d.", i);
        found_FFD8 = true;
      }
      ++i;
    }
    if (found_FFD8)
    {
      --i;
    }
    else
    {
      _buf_read = _input->readBytes(_read_buf, READ_BUFFER_SIZE);
    }
  }
  uint8_t *_p = _read_buf + i;
  _buf_read -= i;
  bool found_FFD9 = false;
  if (_buf_read > 0)
  {
    i = 3;
    while ((_buf_read > 0) && (!found_FFD9))
    {
      if ((_mjpeg_buf_offset > 0) && (_mjpeg_buf[_mjpeg_buf_offset - 1] == 0xFF) && (_p[0] == 0xD9)) // JPEG trailer
      {
        found_FFD9 = true;
      }
      else
      {
        while ((i < _buf_read) && (!found_FFD9))
        {
          if ((_p[i] == 0xFF) && (_p[i + 1] == 0xD9)) // JPEG trailer
          {
            found_FFD9 = true;
            ++i;
          }
          ++i;
        }
      }

      // log_i("i: %d", i);
      memcpy(_mjpeg_buf + _mjpeg_buf_offset, _p, i);
      _mjpeg_buf_offset += i;
      int32_t o = _buf_read - i;
      if (o > 0)
      {
        // log_i("o: %d", o);
        memcpy(_read_buf, _p + i, o);
        _buf_read = _input->readBytes(_read_buf + o, READ_BUFFER_SIZE - o);
        _p = _read_buf;
        _inputindex += _buf_read;
        _buf_read += o;
        // log_i("_buf_read: %d", _buf_read);
      }
      else
      {
        _buf_read = _input->readBytes(_read_buf, READ_BUFFER_SIZE);
        _p = _read_buf;
        _inputindex += _buf_read;
      }
      i = 0;
    }
    if (found_FFD9)
    {
      // log_i("Found FFD9 at: %d.", _mjpeg_buf_offset);
      if (_mjpeg_buf_offset > _mjpegBufSize)
      {
        log_e("_mjpeg_buf_offset(%d) > _mjpegBufSize (%d)", _mjpeg_buf_offset, _mjpegBufSize);
      }
      return true;
    }
  }

  return false;
}

bool mjpeg_draw_frame()
{
  mjpegBuf *mBuf = &_mjpegBufs[_mBufIdx];
  mBuf->size = _mjpeg_buf_offset;
  // log_i("_mjpegBufs[%d].size: %d.", _mBufIdx, _mjpegBufs[_mBufIdx].size);
  // log_i("_mjpegBufs[%d].buf start: %X %X, end: %X, %X.", _mjpegBufs, _mjpegBufs[_mBufId].buf[0], _mjpegBufs[_mBufIdx].buf[1], _mjpegBufs[_mBufIdx].buf[_mjpeg_buf_offset - 2], _mjpegBufs[_mBufIdx].buf[_mjpeg_buf_offset - 1]);
  xQueueSend(_pDecodeTask.xqh, &mBuf, portMAX_DELAY);
  ++_mBufIdx;
  if (_mBufIdx >= NUMBER_OF_DECODE_BUFFER)
  {
    _mBufIdx = 0;
  }
  _mjpeg_buf = _mjpegBufs[_mBufIdx].buf;
  // log_i("queue decode_task end");

  return true;
}

//****************************************************************************************
//                                  FILE _ FUNCTIONS                                     *
//****************************************************************************************

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

//****************************************************************************************
//                                  SHOW STATS FUNCTION                                  *
//****************************************************************************************

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
