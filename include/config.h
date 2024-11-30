/* Config File
*   
*       ASSIGN Core to Task
*
*       Debug msgs true/false
*
*       Pin definitions
*
*       Color definitions
*
*/

#define AUDIOASSIGNCORE 1
#define DECODEASSIGNCORE 0
#define DRAWASSIGNCORE 0
#define INPUTASSIGNCORE 1

#define AUDIO_PRIO 1
#define DECODE_PRIO 1
#define DRAW_PRIO 2
#define INPUT_PRIO 4

#define DEBUG true

#define READ_BUFFER_SIZE 4096
#define MAXOUTPUTSIZE (288 / 3 / 16)
#define NUMBER_OF_DECODE_BUFFER 4
#define NUMBER_OF_DRAW_BUFFER 24

#define FPS 25
#define MJPEG_BUFFER_SIZE (288 * 250 * 2 / 8)

#if DEBUG == true
#define debug(x) Serial.print(x)
#define debugf(...) Serial.printf(__VA_ARGS__)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugf(...)
#define debugln(x)
#endif

#define AAC_FILENAME "/kirk.aac"
#define MJPEG_FILENAME "/kirk.mjpeg"

#define BATSENS 11
#define BUTTON1 10
#define BUTTON2 12

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
#define I2S_BCLK 8 // 5
#define I2S_LRC 9 // 6
#define I2S_DOUT 7 // 4
#define I2S_DIN -1

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