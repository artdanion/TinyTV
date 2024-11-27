#include <Arduino.h>
#include <SD_MMC.h>
#include <esp_heap_caps.h>
#include <string>
#include <map>
#include <vector>
#include "driver/i2s.h"
#include "player.h"
#include "config.h"
#include "AACDecoderHelix.h"
#include <FS.h>
#include <JPEGDEC.h>
#include <Arduino_GFX_Library.h>

// Define the static member variables
//libhelix::AACDecoderHelix _aac;

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
TaskHandle_t _audioTask = NULL;
static unsigned long total_read_audio_ms = 0;
static unsigned long total_decode_audio_ms = 0;
static unsigned long total_play_audio_ms = 0;

static i2s_port_t _i2s_num;
float volume_scale = 0.8f; // Volume scaling factor (1.0f means no change)

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

  // debugln("Init I2S");

  // esp_err_t ret_val = i2s_init(I2S_NUM_0, 44100, I2S_MCLK /* MCLK */, I2S_SCLK /* SCLK */, I2S_LRCK /* LRCK */, I2S_DOUT /* DOUT */, I2S_DIN /* DIN */);

  // if (ret_val != ESP_OK)
  // {
  //   debugf("i2s_init failed: %d\n", ret_val);
  //   return;
  // }
  // i2s_zero_dma_buffer(I2S_NUM_0);

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
    aFile = SD_MMC.open(audioFiles[current_audio].c_str());
    aFileOpen = true;
  }
  else
  {
    debugln("No Sound");
  }

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

    mjpeg_setup(&vFile, MJPEG_BUFFER_SIZE, drawMCU, false /* useBigEndian */, DECODEASSIGNCORE, DRAWASSIGNCORE);

    debugln("Start play audio task");

    BaseType_t ret_val;
    if (aFileOpen)
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
    showStats();
  }
}

void Player::stop()
{
  debug_memory_usage();

  // Stop the AAC decoder
  _aac.end();
  delay(20);

  // Delete the AAC player task
  if (_audioTask != NULL)
  {
    vTaskDelete(_audioTask);
    _audioTask = NULL;
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
  if (_drawTask != NULL)
  {
    vTaskDelete(_drawTask);
    _drawTask = NULL;
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
  // volume_scale = volume;
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

// ----------- Decode and Draw Task
static int queueDrawMCU(JPEGDRAW *pDraw)
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

static void decode_task(void *arg)
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

static void draw_task(void *arg)
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

// ------------audio task
esp_err_t i2s_init(i2s_port_t i2s_num, uint32_t sample_rate,
                          int mck_io_num,   /*!< MCK in out pin. Note that ESP32 supports setting MCK on GPIO0/GPIO1/GPIO3 only*/
                          int bck_io_num,   /*!< BCK in out pin*/
                          int ws_io_num,    /*!< WS in out pin*/
                          int data_out_num, /*!< DATA out pin*/
                          int data_in_num   /*!< DATA in pin*/
)
{
  _i2s_num = i2s_num;

  esp_err_t ret_val = ESP_OK;

  i2s_config_t i2s_config;
  i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2s_config.sample_rate = sample_rate;
  i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2s_config.dma_buf_count = 8;
  i2s_config.dma_buf_len = 160;
  i2s_config.use_apll = false;
  i2s_config.tx_desc_auto_clear = true;
  i2s_config.fixed_mclk = 0;
  i2s_config.bits_per_chan = I2S_BITS_PER_CHAN_16BIT;

  i2s_pin_config_t pin_config;
  pin_config.mck_io_num = mck_io_num;
  pin_config.bck_io_num = bck_io_num;
  pin_config.ws_io_num = ws_io_num;
  pin_config.data_out_num = data_out_num;
  pin_config.data_in_num = data_in_num;

  ret_val |= i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
  if (ret_val != ESP_OK)
  {
    debugf("i2s_driver_install failed: %d\n", ret_val);
    return ret_val;
  }

  ret_val |= i2s_set_pin(i2s_num, &pin_config);
  if (ret_val != ESP_OK)
  {
    debugf("i2s_set_pin failed: %d\n", ret_val);
    return ret_val;
  }

  debugln("I2S initialized successfully");
  return ESP_OK;
}

static int _samprate = 0;
void aacAudioDataCallback(AACFrameInfo &info, int16_t *pwm_buffer, size_t len)
{
  unsigned long s = millis();
  if (_samprate != info.sampRateOut)
  {
    i2s_set_clk(_i2s_num, info.sampRateOut /* sample_rate */, info.bitsPerSample /* bits_cfg */, (info.nChans == 2) ? I2S_CHANNEL_STEREO : I2S_CHANNEL_MONO /* channel */);
    _samprate = info.sampRateOut;
  }

  // Apply volume scaling
  for (size_t i = 0; i < len; i++)
  {
    pwm_buffer[i] = static_cast<int16_t>(pwm_buffer[i] * volume_scale);
  }

  size_t i2s_bytes_written = 0;
  i2s_write(_i2s_num, pwm_buffer, len * 2, &i2s_bytes_written, portMAX_DELAY);
  total_play_audio_ms += millis() - s;
}

static uint8_t _frame[3200]; // MP3_MAX_FRAME_SIZE is smaller, so always use MP3_MAX_FRAME_SIZE

static void aac_player_task(void *pvParam)
{
  Stream *input = (Stream *)pvParam;

  int r, w;
  unsigned long ms = millis();
  while (r = input->readBytes(_frame, 3200))
  {
    total_read_audio_ms += millis() - ms;
    ms = millis();

    while (r > 0)
    {
      w = _aac.write(_frame, r);
      r -= w;
    }
    total_decode_audio_ms += millis() - ms;
    ms = millis();
  }
  debugln("AAC stop.");

  vTaskDelete(NULL);
}

static BaseType_t aac_player_task_start(Stream *input, BaseType_t audioAssignCore)
{
  _aac.begin();

  return xTaskCreatePinnedToCore(
      (TaskFunction_t)aac_player_task,
      (const char *const)"AAC Player Task",
      (const uint32_t)2000,
      (void *const)input,
      (UBaseType_t)configMAX_PRIORITIES - 1,
      (TaskHandle_t *const)&_audioTask,
      (const BaseType_t)audioAssignCore);
}

// scan SD Card and fill file vectors
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
