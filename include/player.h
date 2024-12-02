#ifndef VIDEO_PLAYER_H
#define VIDEO_PLAYER_H

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <Audio.h>
#include <JPEGDEC.h>
#include <Arduino_GFX_Library.h>
#include <memory>

#define NUMBER_OF_DECODE_BUFFER 4
#define NUMBER_OF_DRAW_BUFFER 24
#define MAXOUTPUTSIZE (288 / 3 / 16)

typedef struct
{
    int32_t size;
    std::unique_ptr<uint8_t[]> buf;
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

class VideoPlayer
{
private:
    // Membervariablen für Video-Handling
    std::unique_ptr<File> _input;
    size_t _mjpegBufSize;
    size_t _readBufferSize;
    std::unique_ptr<uint8_t[]> _read_buf; // Automatische Speicherverwaltung
    std::unique_ptr<uint8_t[]> _mjpeg_buf;
    Arduino_GFX *gfx;
    QueueHandle_t _videoQueue;
    int _frameIndex;
    unsigned long _startTime;
    bool _useBigEndian;
    JPEGDEC _jpegDec;
    int32_t _mjpeg_buf_offset;
    int32_t _inputindex;
    int32_t _buf_read;
    int32_t _remain;
    TaskHandle_t _decodeTask;
    TaskHandle_t _draw_task;
    paramDecodeTask _pDecodeTask;
    paramDrawTask _pDrawTask;
    uint8_t _mBufIdx;
    mjpegBuf _mjpegBufs[NUMBER_OF_DECODE_BUFFER];
    JPEGDRAW jpegdraws[NUMBER_OF_DRAW_BUFFER];
    static int _draw_queue_cnt;
    xQueueHandle _xqh;

    // Private Hilfsfunktionen
    bool readFrame();
    void drawFrame();
    void decode_task_internal();
    void draw_task_internal();
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
