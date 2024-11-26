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
  static i2s_port_t _i2s_num;
  static TaskHandle_t TaskHandle_0;

  static JPEGDEC _jpegDec;
  static xQueueHandle _xqh;
  static bool _useBigEndian;
  static Stream *_input;
  static int32_t _mjpegBufSize;
  static uint8_t *_read_buf;
  static int32_t _mjpeg_buf_offset;
  static TaskHandle_t _decodeTask;
  static TaskHandle_t _draw_task;
  static paramDecodeTask _pDecodeTask;
  static paramDrawTask _pDrawTask;
  static uint8_t *_mjpeg_buf;
  static uint8_t _mBufIdx;
  static int32_t _inputindex;
  static int32_t _buf_read;
  static int32_t _remain;
  static mjpegBuf _mjpegBufs[NUMBER_OF_DECODE_BUFFER];
  static JPEGDRAW jpegdraws[NUMBER_OF_DRAW_BUFFER];
  static int _draw_queue_cnt;
};

#endif
