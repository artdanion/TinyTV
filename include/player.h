#ifndef PLAYER_H
#define PLAYER_H

#include <FS.h>
#include <JPEGDEC.h>
#include <AACDecoderHelix.h>
#include <driver/i2s.h>
#include <string>
#include <map>
#include <vector>

#define READ_BUFFER_SIZE 4096
#define MAXOUTPUTSIZE (288 / 3 / 16)
#define NUMBER_OF_DECODE_BUFFER 4
#define NUMBER_OF_DRAW_BUFFER 24

void getFiles();
void scanDirectory(fs::FS &fs, String dirname, std::map<std::string, std::string> &fileMap);
void populateVectorsFromMap(const std::map<std::string, std::string> &fileMap, std::vector<String> &videoFiles, std::vector<String> &audioFiles);
void listFilesByExtension(fs::FS &fs, std::vector<String> &videoFiles, std::vector<String> &audioFiles);
static int drawMCU(JPEGDRAW *pDraw);
void showStats();

// Declare the vectors as extern
extern std::vector<String> videoFiles;
extern std::vector<String> audioFiles;

extern int current_video;
extern int current_audio;


typedef struct
{
  int32_t size;
  uint8_t *buf;
} mjpegBuf;

typedef struct
{
  xQueueHandle xqh;
  JPEG_DRAW_CALLBACK *drawFunc;
} paramDrawTask;

typedef struct
{
  xQueueHandle xqh;
  mjpegBuf *mBuf;
  JPEG_DRAW_CALLBACK *drawFunc;
} paramDecodeTask;

class Player
{
public:
  Player();
  void init();
  void start(const std::string &videoFile);
  void stop();
  void set_volume(float volume);

private:
  File vFile;
  File aFile;
  bool vFileOpen;
  bool aFileOpen;
  unsigned long start_ms;
  unsigned long curr_ms;
  unsigned long next_frame_ms;
  unsigned long next_frame;
  unsigned long total_read_video_ms;
  unsigned long total_decode_video_ms;
  unsigned long skipped_frames;

  void debug_memory_usage();

  static libhelix::AACDecoderHelix _aac;
};

// decode and draw task
static int queueDrawMCU(JPEGDRAW *pDraw);
static void decode_task(void *arg);
static void draw_task(void *arg);
bool mjpeg_setup(Stream *input, int32_t mjpegBufSize, JPEG_DRAW_CALLBACK *pfnDraw,
                 bool useBigEndian, BaseType_t decodeAssignCore, BaseType_t drawAssignCore);
bool mjpeg_read_frame();
bool mjpeg_draw_frame();

// audio task
static esp_err_t i2s_init(i2s_port_t i2s_num, uint32_t sample_rate,
                          int mck_io_num,   /*!< MCK in out pin. Note that ESP32 supports setting MCK on GPIO0/GPIO1/GPIO3 only*/
                          int bck_io_num,   /*!< BCK in out pin*/
                          int ws_io_num,    /*!< WS in out pin*/
                          int data_out_num, /*!< DATA out pin*/
                          int data_in_num   /*!< DATA in pin*/
);

void aacAudioDataCallback(AACFrameInfo &info, int16_t *pwm_buffer, size_t len);
static libhelix::AACDecoderHelix _aac(aacAudioDataCallback);
static void aac_player_task(void *pvParam);
static BaseType_t aac_player_task_start(Stream *input, BaseType_t audioAssignCore);

#endif
