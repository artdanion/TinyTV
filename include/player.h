#ifndef VIDEO_PLAYER_H
#define VIDEO_PLAYER_H

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <Audio.h>
#include <JPEGDEC.h>
#include <driver/i2s.h>
#include <string>
#include <map>
#include <vector>

void getFiles();
void scanDirectory(fs::FS &fs, String dirname, std::map<std::string, std::string> &fileMap);
void populateVectorsFromMap(const std::map<std::string, std::string> &fileMap, std::vector<String> &videoFiles, std::vector<String> &audioFiles);
void listFilesByExtension(fs::FS &fs, std::vector<String> &videoFiles, std::vector<String> &audioFiles);
int drawMCU(JPEGDRAW *pDraw);
void clearQueues();

// Declare the vectors as extern
extern std::vector<String> videoFiles;
extern std::vector<String> audioFiles;

extern int current_video;
extern int current_audio;
extern bool isPlaying;
extern bool isStopping;

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
    std::unique_ptr<uint8_t[]> buf;
} mjpegBuf;

typedef struct
{
  xQueueHandle xqh;
  JPEG_DRAW_CALLBACK *drawFunc;
  int data;
} paramDrawTask;

typedef struct
{
  xQueueHandle xqh;
  mjpegBuf *mBuf;
  JPEG_DRAW_CALLBACK *drawFunc;
  int data;
} paramDecodeTask;

class VideoPlayer
{
public:
  Player();
  ~Player();
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
  void stopTasks();
  void clearQueues();
  void signalStopToQueues();
  void resetPlaybackState();
  void freeBuffers();
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
int queueDrawMCU(JPEGDRAW *pDraw);
void decode_task(void *arg);
void draw_task(void *arg);
bool mjpeg_setup(Stream *input, int32_t mjpegBufSize, JPEG_DRAW_CALLBACK *pfnDraw,
                 bool useBigEndian, BaseType_t decodeAssignCore, BaseType_t drawAssignCore);
bool mjpeg_read_frame();
bool mjpeg_draw_frame();

public:
    // Konstruktor und Destruktor
    VideoPlayer(Arduino_GFX *gfxInstance, size_t mjpegBufferSize, size_t readBufferSize);
    ~VideoPlayer();

    // Öffentliche Methoden
    void init();
    void play(const char *filename);
    void stop();
    void task();
    int drawMCU(JPEGDRAW *pDraw);
    int queueDrawMCU(JPEGDRAW *pDraw);
    static VideoPlayer *instance;

    // Statische Wrapper-Funktionen
    static void decode_task(void *arg);
    static void draw_task(void *arg);
};

int drawMCU_wrapper(JPEGDRAW *pDraw);
int queueDrawMCU_wrapper(JPEGDRAW *pDraw);

#endif // VIDEO_PLAYER_H
