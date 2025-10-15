// st7789.h

#ifndef ST7789_H
#define ST7789_H

#include "stm32f4xx_hal.h"

// Defina aqui as dimensões do seu display
#define ST7789_WIDTH  240
#define ST7789_HEIGHT 240
#define ST7789_SIZE   2

// Definição de cores (formato RGB565)
#define ST7789_BLACK   0x0000
#define ST7789_BLUE    0x001F
#define ST7789_RED     0xF800
#define ST7789_GREEN   0x07E0
#define ST7789_CYAN    0x07FF
#define ST7789_MAGENTA 0xF81F
#define ST7789_YELLOW  0xFFE0
#define ST7789_WHITE   0xFFFF

// Funções públicas existentes
void ST7789_Init(void);
void ST7789_FillScreen(uint16_t color);
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7789_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7789_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t foreground, uint16_t background, uint8_t size);
void ST7789_DrawText(uint16_t x, uint16_t y, const char* str, uint16_t foreground, uint16_t background, uint8_t size);

// Linhas
void ST7789_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void ST7789_DrawHorizontalLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void ST7789_DrawVerticalLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);

// Retângulos (a função para preencher já existe, esta é para o contorno)
void ST7789_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

// Triângulos
void ST7789_DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ST7789_FillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

// Círculos
void ST7789_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
void ST7789_FillCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);

// Extra: Retângulos com cantos arredondados (muito útil para interfaces)
void ST7789_DrawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint16_t color);
void ST7789_FillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint16_t color);

#endif // ST7789_H