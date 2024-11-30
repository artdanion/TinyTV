#ifndef PLAYER_H
#define PLAYER_H

#include <FS.h>
#include <Audio.h>
#include <JPEGDEC.h>
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
int drawMCU(JPEGDRAW *pDraw);
void showStats();

// Declare the vectors as extern
extern std::vector<String> videoFiles;
extern std::vector<String> audioFiles;

extern int current_video;
extern int current_audio;

extern bool task_response;

extern unsigned long  total_read_video_ms;
extern unsigned long  total_decode_video_ms;
extern unsigned long  total_show_video_ms;
extern unsigned long  total_read_audio_ms;
extern unsigned long  total_play_audio_ms;

extern struct audioMessage
{
  uint8_t cmd;
  const char *txt;
  uint32_t value;
  uint32_t ret;
} audioTxMessage, audioRxMessage;

enum : uint8_t
{
  SET_VOLUME,
  GET_VOLUME,
  CONNECTTOHOST,
  CONNECTTOSD
};
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
  void setVolume(int volume);
  void showStats();

private:
  File vFile;
  File aFile;
  bool vFileOpen;
  bool aFileOpen;
  uint64_t start_ms;
  uint64_t curr_ms;
  uint64_t next_frame_ms;
  int next_frame;
  int skipped_frames;
  void debug_memory_usage();
};

// audio functions
void CreateQueues();
void audioTask(void *parameter);
void audioInit();
audioMessage transmitReceive(audioMessage msg);
void audioSetVolume(uint8_t vol);
uint8_t audioGetVolume();
bool audioConnecttohost(const char *host);
bool audioConnecttoSD(const char *filename);

// decode and draw task
void CreateVideoQueues();
struct videoMessage transmitReceiveVideo(struct videoMessage msg);
bool videoSwitchFile(Stream *newFile);
int queueDrawMCU(JPEGDRAW *pDraw);
void decode_task(void *arg);
void draw_task(void *arg);
bool mjpeg_setup(Stream *input, int32_t mjpegBufSize, JPEG_DRAW_CALLBACK *pfnDraw,
                 bool useBigEndian, BaseType_t decodeAssignCore, BaseType_t drawAssignCore);
bool mjpeg_read_frame();
bool mjpeg_draw_frame();

#endif
