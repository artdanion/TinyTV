#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <JPEGDEC.h>
#include <Arduino_GFX_Library.h>
#include <memory>
#include "player.h"

int VideoPlayer::_draw_queue_cnt = 0;
VideoPlayer *VideoPlayer::instance = nullptr;

VideoPlayer::VideoPlayer(Arduino_GFX *gfxInstance, size_t mjpegBufferSize, size_t readBufferSize)
    : gfx(gfxInstance), _mjpegBufSize(mjpegBufferSize), _readBufferSize(readBufferSize)
{
    _read_buf = std::make_unique<uint8_t[]>(readBufferSize);
    _mjpeg_buf = std::make_unique<uint8_t[]>(mjpegBufferSize);
    _input = nullptr;
    _videoQueue = nullptr;
    _frameIndex = 0;
    _startTime = 0;
    _useBigEndian = false;
    _mjpeg_buf_offset = 0;
    _inputindex = 0;
    _buf_read = 0;
    _remain = 0;
    _mBufIdx = 0;
    _decodeTask = nullptr;
    _draw_task = nullptr;
    _xqh = nullptr;

    // Setze die statische Instanzvariable
    instance = this;
}

VideoPlayer::~VideoPlayer()
{
    stop();
}

void VideoPlayer::init()
{
    _videoQueue = xQueueCreate(1, sizeof(File *));
    if (!_videoQueue)
    {
        Serial.println("Failed to create video queue");
        return;
    }

    gfx->begin(80000000);
    gfx->fillScreen(BLACK);

    Serial.println("VideoPlayer initialized.");
}

void VideoPlayer::play(const char *filename)
{
    _input = std::make_unique<File>(SD_MMC.open(filename, "r"));
    if (!_input || !_input->available())
    {
        Serial.println("Failed to open video file");
        return;
    }

    _frameIndex = 0;
    _startTime = millis();

    xQueueSend(_videoQueue, &_input, portMAX_DELAY);
    Serial.printf("Playing video: %s\n", filename);
}

void VideoPlayer::stop()
{
    if (_input)
    {
        _input->close();
        _input.reset();
    }
    Serial.println("Video stopped");
}

bool VideoPlayer::readFrame()
{
    if (_inputindex == 0)
    {
        _buf_read = _input->readBytes(reinterpret_cast<char *>(_read_buf.get()), _readBufferSize);
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
            _buf_read = _input->readBytes(reinterpret_cast<char *>(_read_buf.get()), _readBufferSize);
        }
    }
    uint8_t *_p = _read_buf.get() + i;
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

            memcpy(_mjpeg_buf.get() + _mjpeg_buf_offset, _p, i);
            _mjpeg_buf_offset += i;
            int32_t o = _buf_read - i;
            if (o > 0)
            {
                memcpy(_read_buf.get(), _p + i, o);
                _buf_read = _input->readBytes(reinterpret_cast<char *>(_read_buf.get()) + o, _readBufferSize - o);
                _p = _read_buf.get();
                _inputindex += _buf_read;
                _buf_read += o;
            }
            else
            {
                _buf_read = _input->readBytes(reinterpret_cast<char *>(_read_buf.get()), _readBufferSize);
                _p = _read_buf.get();
                _inputindex += _buf_read;
            }
            i = 0;
        }
        if (found_FFD9)
        {
            if (_mjpeg_buf_offset > _mjpegBufSize)
            {
                Serial.printf("_mjpeg_buf_offset(%d) > _mjpegBufSize (%d)\n", _mjpeg_buf_offset, _mjpegBufSize);
            }
            return true;
        }
    }

    return false;
}

void VideoPlayer::drawFrame()
{
    mjpegBuf *mBuf = &_mjpegBufs[_mBufIdx];
    mBuf->size = _mjpeg_buf_offset;

    xQueueSend(_pDecodeTask.xqh, &mBuf, portMAX_DELAY);
    ++_mBufIdx;
    if (_mBufIdx >= NUMBER_OF_DECODE_BUFFER)
    {
        _mBufIdx = 0;
    }
    _mjpeg_buf = std::move(_mjpegBufs[_mBufIdx].buf);
}

int VideoPlayer::drawMCU(JPEGDRAW *pDraw)
{
    unsigned long s = millis();
    gfx->draw16bitRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
    return 1;
}

int VideoPlayer::queueDrawMCU(JPEGDRAW *pDraw)
{
    int len = pDraw->iWidth * pDraw->iHeight * 2;
    JPEGDRAW *j = &jpegdraws[_draw_queue_cnt % NUMBER_OF_DRAW_BUFFER];
    j->x = pDraw->x;
    j->y = pDraw->y;
    j->iWidth = pDraw->iWidth;
    j->iHeight = pDraw->iHeight;
    memcpy(j->pPixels, pDraw->pPixels, len);

    ++_draw_queue_cnt;
    xQueueSend(_xqh, &j, portMAX_DELAY);

    return 1;
}

void VideoPlayer::decode_task_internal()
{
    mjpegBuf *mBuf;
    while (xQueueReceive(_pDecodeTask.xqh, &mBuf, portMAX_DELAY))
    {
        unsigned long s = millis();

        _jpegDec.openRAM(mBuf->buf.get(), mBuf->size, _pDecodeTask.drawFunc);

        if (_useBigEndian)
        {
            _jpegDec.setPixelType(RGB565_BIG_ENDIAN);
        }
        _jpegDec.setMaxOutputSize(MAXOUTPUTSIZE);
        _jpegDec.decode(0, 0, 0);
        _jpegDec.close();
    }
    vQueueDelete(_pDecodeTask.xqh);
    vTaskDelete(NULL);
}

void VideoPlayer::draw_task_internal()
{
    JPEGDRAW *pDraw;
    while (xQueueReceive(_pDrawTask.xqh, &pDraw, portMAX_DELAY))
    {
        _pDrawTask.drawFunc(pDraw);
    }
    vQueueDelete(_pDrawTask.xqh);
    vTaskDelete(NULL);
}

void VideoPlayer::decode_task(void *arg)
{
    VideoPlayer *instance = static_cast<VideoPlayer *>(arg);
    instance->decode_task_internal();
}

void VideoPlayer::draw_task(void *arg)
{
    VideoPlayer *instance = static_cast<VideoPlayer *>(arg);
    instance->draw_task_internal();
}

bool VideoPlayer::mjpeg_setup(Stream *input, int32_t mjpegBufSize, JPEG_DRAW_CALLBACK *pfnDraw,
                              bool useBigEndian, BaseType_t decodeAssignCore, BaseType_t drawAssignCore)
{
    _input = std::unique_ptr<File>(static_cast<File *>(input));
    _mjpegBufSize = mjpegBufSize;
    _useBigEndian = useBigEndian;

    for (int i = 0; i < NUMBER_OF_DECODE_BUFFER; ++i)
    {
        _mjpegBufs[i].buf = std::make_unique<uint8_t[]>(mjpegBufSize);
        if (_mjpegBufs[i].buf)
        {
            Serial.printf("#%d decode buffer allocated.\n", i);
        }
        else
        {
            Serial.printf("#%d decode buffer allocat failed.\n", i);
        }
    }
    _mjpeg_buf = std::move(_mjpegBufs[_mBufIdx].buf);

    if (!_read_buf)
    {
        _read_buf = std::make_unique<uint8_t[]>(_readBufferSize);
    }
    if (_read_buf)
    {
        Serial.println("Read buffer allocated.");
    }

    _xqh = xQueueCreate(NUMBER_OF_DRAW_BUFFER, sizeof(JPEGDRAW));
    _pDrawTask.xqh = _xqh;
    _pDrawTask.drawFunc = pfnDraw;
    _pDecodeTask.xqh = xQueueCreate(NUMBER_OF_DECODE_BUFFER, sizeof(mjpegBuf));
    _pDecodeTask.drawFunc = queueDrawMCU_wrapper;

    xTaskCreatePinnedToCore(
        (TaskFunction_t)decode_task,
        (const char *const)"MJPEG decode Task",
        (const uint32_t)4096,
        (void *const)this,
        (UBaseType_t)configMAX_PRIORITIES - 5,
        (TaskHandle_t *const)&_decodeTask,
        (const BaseType_t)decodeAssignCore);
    xTaskCreatePinnedToCore(
        (TaskFunction_t)draw_task,
        (const char *const)"MJPEG Draw Task",
        (const uint32_t)4096,
        (void *const)this,
        (UBaseType_t)configMAX_PRIORITIES - 1,
        (TaskHandle_t *const)&_draw_task,
        (const BaseType_t)drawAssignCore);

    for (int i = 0; i < NUMBER_OF_DRAW_BUFFER; i++)
    {
        if (!jpegdraws[i].pPixels)
        {
            jpegdraws[i].pPixels = (uint16_t *)heap_caps_malloc(MAXOUTPUTSIZE * 16 * 16 * 2, MALLOC_CAP_DMA);
        }
        if (jpegdraws[i].pPixels)
        {
            Serial.printf("#%d draw buffer allocated.\n", i);
        }
        else
        {
            Serial.printf("#%d draw buffer allocat failed.\n", i);
        }
    }

    return true;
}

bool VideoPlayer::mjpeg_read_frame()
{
    if (_inputindex == 0)
    {
        _buf_read = _input->readBytes(reinterpret_cast<char *>(_read_buf.get()), _readBufferSize);
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
            _buf_read = _input->readBytes(reinterpret_cast<char *>(_read_buf.get()), _readBufferSize);
        }
    }
    uint8_t *_p = _read_buf.get() + i;
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

            memcpy(_mjpeg_buf.get() + _mjpeg_buf_offset, _p, i);
            _mjpeg_buf_offset += i;
            int32_t o = _buf_read - i;
            if (o > 0)
            {
                memcpy(_read_buf.get(), _p + i, o);
                _buf_read = _input->readBytes(reinterpret_cast<char *>(_read_buf.get()) + o, _readBufferSize - o);
                _p = _read_buf.get();
                _inputindex += _buf_read;
                _buf_read += o;
            }
            else
            {
                _buf_read = _input->readBytes(reinterpret_cast<char *>(_read_buf.get()), _readBufferSize);
                _p = _read_buf.get();
                _inputindex += _buf_read;
            }
            i = 0;
        }
        if (found_FFD9)
        {
            if (_mjpeg_buf_offset > _mjpegBufSize)
            {
                Serial.printf("_mjpeg_buf_offset(%d) > _mjpegBufSize (%d)\n", _mjpeg_buf_offset, _mjpegBufSize);
            }
            return true;
        }
    }

    return false;
}

bool VideoPlayer::mjpeg_draw_frame()
{
    mjpegBuf *mBuf = &_mjpegBufs[_mBufIdx];
    mBuf->size = _mjpeg_buf_offset;

    xQueueSend(_pDecodeTask.xqh, &mBuf, portMAX_DELAY);
    ++_mBufIdx;
    if (_mBufIdx >= NUMBER_OF_DECODE_BUFFER)
    {
        _mBufIdx = 0;
    }
    _mjpeg_buf = std::move(_mjpegBufs[_mBufIdx].buf);

    return true;
}

void VideoPlayer::task()
{
    while (true)
    {
        File *file = nullptr;
        if (xQueueReceive(_videoQueue, &file, portMAX_DELAY))
        {
            while (file && readFrame())
            {
                drawFrame();
                vTaskDelay(pdMS_TO_TICKS(1000 / 25)); // 25 FPS
            }
            stop();
        }
    }
}

// Statische Wrapper-Funktionen für die Memberfunktionen
int drawMCU_wrapper(JPEGDRAW *pDraw) {
    return VideoPlayer::instance->drawMCU(pDraw);
}

int queueDrawMCU_wrapper(JPEGDRAW *pDraw) {
    return VideoPlayer::instance->queueDrawMCU(pDraw);
}
