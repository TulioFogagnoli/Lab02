#include "st7789.h"
#include "main.h" // Para acessar os defines dos pinos e a instância do SPI
#include "font.h" 

// Associe a instância do SPI gerada pelo CubeIDE
extern SPI_HandleTypeDef hspi1;
#define ST7789_SPI_PORT hspi1

// Mapeamento dos pinos (usando os User Labels do CubeIDE)
#define LCD_CS_PORT   LCD_CS_GPIO_Port
#define LCD_CS_PIN    LCD_CS_Pin
#define LCD_DC_PORT   LCD_DC_GPIO_Port
#define LCD_DC_PIN    LCD_DC_Pin
#define LCD_RST_PORT  LCD_RST_GPIO_Port
#define LCD_RST_PIN   LCD_RST_Pin

// Funções privadas (auxiliares)
static void ST7789_Select(void) {
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
}

static void ST7789_Unselect(void) {
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

static void ST7789_Reset(void) {
    HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
}

static void ST7789_WriteCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET); // Modo Comando
    ST7789_Select();
    HAL_SPI_Transmit(&ST7789_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
    ST7789_Unselect();
}

static void ST7789_WriteData(uint8_t* buff, size_t buff_size) {
    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET); // Modo Dado
    ST7789_Select();
    HAL_SPI_Transmit(&ST7789_SPI_PORT, buff, buff_size, HAL_MAX_DELAY);
    ST7789_Unselect();
}

// Define a "janela" de memória onde os pixels serão escritos
static void ST7789_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];

    // Column Address Set
    ST7789_WriteCommand(0x2A);
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    ST7789_WriteData(data, sizeof(data));

    // Row Address Set
    ST7789_WriteCommand(0x2B);
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    ST7789_WriteData(data, sizeof(data));

    // Write to RAM
    ST7789_WriteCommand(0x2C);
}

// Função de inicialização com a sequência de comandos para o ST7789
void ST7789_Init(void) {
    ST7789_Reset();

    ST7789_WriteCommand(0x11); // Sleep Out
    HAL_Delay(120);

    ST7789_WriteCommand(0x36); // Memory Data Access Control
    uint8_t madctl = 0x00;
    ST7789_WriteData(&madctl, 1);

    ST7789_WriteCommand(0x3A); // Interface Pixel Format
    uint8_t pixfmt = 0x55; // 16 bits/pixel
    ST7789_WriteData(&pixfmt, 1);

    ST7789_WriteCommand(0x21); // Display Inversion On

    ST7789_WriteCommand(0x13); // Normal Display Mode On

    ST7789_WriteCommand(0x29); // Display On
    HAL_Delay(120);
}

// Funções públicas (implementação)
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT)) return;

    ST7789_SetAddressWindow(x, y, x, y);
    uint8_t data[] = { (color >> 8) & 0xFF, color & 0xFF };
    ST7789_WriteData(data, sizeof(data));
}

void ST7789_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT)) return;
    if ((x + w - 1) >= ST7789_WIDTH) w = ST7789_WIDTH - x;
    if ((y + h - 1) >= ST7789_HEIGHT) h = ST7789_HEIGHT - y;

    ST7789_SetAddressWindow(x, y, x + w - 1, y + h - 1);

    uint8_t data[] = { (color >> 8) & 0xFF, color & 0xFF };
    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET); // Data mode
    ST7789_Select();

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            HAL_SPI_Transmit(&ST7789_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
        }
    }
    ST7789_Unselect();
}

void ST7789_FillScreen(uint16_t color) {
    ST7789_FillRectangle(0, 0, ST7789_WIDTH, ST7789_HEIGHT, color);
}

/**
 * @brief Desenha um único caractere no display.
 * @param x Coordenada X do canto superior esquerdo.
 * @param y Coordenada Y do canto superior esquerdo.
 * @param ch O caractere a ser desenhado.
 * @param foreground Cor do caractere.
 * @param background Cor do fundo.
 */
void ST7789_DrawChar(uint16_t x, uint16_t y, char ch, uint16_t foreground, uint16_t background, uint8_t size) {
    if (x + (FONT_WIDTH * size) > ST7789_WIDTH || y + (FONT_HEIGHT * size) > ST7789_HEIGHT) {
        return; // Garante que o caractere ampliado ainda caiba na tela
    }

    uint32_t font_idx = (ch - ' ') * FONT_HEIGHT;

    for (int i = 0; i < FONT_HEIGHT; i++) {
        uint8_t line_data = font[font_idx + i];
        for (int j = 0; j < FONT_WIDTH; j++) {
            if ((line_data >> j) & 1) {
                // Em vez de desenhar um pixel, desenha um retângulo do tamanho do scale
                if (size == 1) {
                    ST7789_DrawPixel(x + j, y + i, foreground);
                } else {
                    ST7789_FillRectangle(x + (i * size), y + (j * size), size, size, foreground);
                }
            } else {
                 // Faz o mesmo para o fundo, para não deixar "buracos"
                 ST7789_FillRectangle(x + (i * size), y + (j * size), size, size, background);
            }
        }
    }
}

/**
 * @brief Desenha uma string (texto) no display.
 * @param x Coordenada X do início do texto.
 * @param y Coordenada Y do início do texto.
 * @param str Ponteiro para a string a ser desenhada.
 * @param foreground Cor do texto.
 * @param background Cor do fundo.
 */
void ST7789_DrawText(uint16_t x, uint16_t y, const char* str, uint16_t foreground, uint16_t background, uint8_t size) {
    uint16_t current_x = x;

    while (*str) {
        // A largura de cada caractere agora é multiplicada pelo tamanho
        if (current_x + (FONT_WIDTH * size) > ST7789_WIDTH) {
            break; 
        }

        ST7789_DrawChar(current_x, y, *str, foreground, background, size);
        current_x += (FONT_WIDTH * size); // Avança o cursor pela largura correta
        str++;
    }
}

// Funções auxiliares para trocar valores, usadas nos algoritmos
static void swap_uint16(uint16_t* a, uint16_t* b) {
    uint16_t t = *a;
    *a = *b;
    *b = t;
}

/**
 * @brief Desenha uma linha horizontal otimizada.
 */
void ST7789_DrawHorizontalLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
    // Usa a função de preencher retângulo com altura 1, que é muito mais rápida
    ST7789_FillRectangle(x, y, w, 1, color);
}

/**
 * @brief Desenha uma linha vertical otimizada.
 */
void ST7789_DrawVerticalLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
    // Usa a função de preencher retângulo com largura 1, que é muito mais rápida
    ST7789_FillRectangle(x, y, 1, h, color);
}

/**
 * @brief Desenha uma linha entre dois pontos usando o algoritmo de Bresenham.
 */
void ST7789_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        swap_uint16(&x0, &y0);
        swap_uint16(&x1, &y1);
    }
    if (x0 > x1) {
        swap_uint16(&x0, &x1);
        swap_uint16(&y0, &y1);
    }

    int16_t dx = x1 - x0;
    int16_t dy = abs(y1 - y0);
    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0 <= x1; x0++) {
        if (steep) {
            ST7789_DrawPixel(y0, x0, color);
        } else {
            ST7789_DrawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

/**
 * @brief Desenha o contorno de um retângulo.
 */
void ST7789_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    ST7789_DrawHorizontalLine(x, y, w, color);         // Top
    ST7789_DrawHorizontalLine(x, y + h - 1, w, color); // Bottom
    ST7789_DrawVerticalLine(x, y, h, color);           // Left
    ST7789_DrawVerticalLine(x + w - 1, y, h, color);   // Right
}

/**
 * @brief Desenha o contorno de um triângulo.
 */
void ST7789_DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    ST7789_DrawLine(x0, y0, x1, y1, color);
    ST7789_DrawLine(x1, y1, x2, y2, color);
    ST7789_DrawLine(x2, y2, x0, y0, color);
}

/**
 * @brief Preenche um triângulo usando a técnica de scanline.
 */
void ST7789_FillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    int16_t a, b, y, last;
    
    // Ordena os vértices por y (y0 <= y1 <= y2)
    if (y0 > y1) { swap_uint16(&y0, &y1); swap_uint16(&x0, &x1); }
    if (y1 > y2) { swap_uint16(&y2, &y1); swap_uint16(&x2, &x1); }
    if (y0 > y1) { swap_uint16(&y0, &y1); swap_uint16(&x0, &x1); }

    if (y0 == y2) { // Triângulo completamente horizontal
        a = b = x0;
        if (x1 < a) a = x1;
        else if (x1 > b) b = x1;
        if (x2 < a) a = x2;
        else if (x2 > b) b = x2;
        ST7789_DrawHorizontalLine(a, y0, b - a + 1, color);
        return;
    }

    int16_t dx01 = x1 - x0, dy01 = y1 - y0,
            dx02 = x2 - x0, dy02 = y2 - y0,
            dx12 = x2 - x1, dy12 = y2 - y1;
    int32_t sa = 0, sb = 0;

    // Parte de cima do triângulo
    if (y1 == y2) last = y1;
    else last = y1 - 1;

    for (y = y0; y <= last; y++) {
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        if (a > b) swap_uint16((uint16_t*)&a, (uint16_t*)&b);
        ST7789_DrawHorizontalLine(a, y, b - a + 1, color);
    }

    // Parte de baixo do triângulo
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (; y <= y2; y++) {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        if (a > b) swap_uint16((uint16_t*)&a, (uint16_t*)&b);
        ST7789_DrawHorizontalLine(a, y, b - a + 1, color);
    }
}

/**
 * @brief Desenha o contorno de um círculo usando o algoritmo de ponto médio.
 */
void ST7789_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color) {
    int16_t x = -r, y = 0, err = 2 - 2 * r;
    do {
        ST7789_DrawPixel(x0 - x, y0 + y, color);
        ST7789_DrawPixel(x0 + x, y0 + y, color);
        ST7789_DrawPixel(x0 + x, y0 - y, color);
        ST7789_DrawPixel(x0 - x, y0 - y, color);
        int16_t e2 = err;
        if (e2 <= y) err += ++y * 2 + 1;
        if (e2 > x || err > y) err += ++x * 2 + 1;
    } while (x < 0);
}

/**
 * @brief Preenche um círculo desenhando linhas horizontais.
 */
void ST7789_FillCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color) {
    int16_t x = -r, y = 0, err = 2 - 2 * r;
    do {
        ST7789_DrawHorizontalLine(x0 + x, y0 - y, 2 * (-x) + 1, color);
        ST7789_DrawHorizontalLine(x0 + x, y0 + y, 2 * (-x) + 1, color);
        int16_t e2 = err;
        if (e2 <= y) err += ++y * 2 + 1;
        if (e2 > x || err > y) err += ++x * 2 + 1;
    } while (x < 0);
}

// Função auxiliar para desenhar os cantos de um retângulo arredondado
static void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        if (cornername & 0x4) { // Canto superior direito
            ST7789_DrawPixel(x0 + x, y0 - y, color);
            ST7789_DrawPixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x2) { // Canto superior esquerdo
            ST7789_DrawPixel(x0 - x, y0 - y, color);
            ST7789_DrawPixel(x0 - y, y0 - x, color);
        }
        if (cornername & 0x8) { // Canto inferior direito
            ST7789_DrawPixel(x0 + x, y0 + y, color);
            ST7789_DrawPixel(x0 + y, y0 + x, color);
        }
        if (cornername & 0x1) { // Canto inferior esquerdo
            ST7789_DrawPixel(x0 - x, y0 + y, color);
            ST7789_DrawPixel(x0 - y, y0 + x, color);
        }
    }
}

// Função auxiliar para preencher os cantos de um retângulo arredondado
static void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        if (cornername & 0x1) { // Superior esquerdo
            ST7789_DrawVerticalLine(x0 - y, y0 - x, 2 * x + 1 + delta, color);
            ST7789_DrawVerticalLine(x0 - x, y0 - y, 2 * y + 1 + delta, color);
        }
        if (cornername & 0x2) { // Superior direito
            ST7789_DrawVerticalLine(x0 + x, y0 - y, 2 * y + 1 + delta, color);
            ST7789_DrawVerticalLine(x0 + y, y0 - x, 2 * x + 1 + delta, color);
        }
    }
}


/**
 * @brief Desenha o contorno de um retângulo com cantos arredondados.
 */
void ST7789_DrawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint16_t color) {
    // Desenha as linhas retas
    ST7789_DrawHorizontalLine(x + r, y, w - 2 * r, color);
    ST7789_DrawHorizontalLine(x + r, y + h - 1, w - 2 * r, color);
    ST7789_DrawVerticalLine(x, y + r, h - 2 * r, color);
    ST7789_DrawVerticalLine(x + w - 1, y + r, h - 2 * r, color);
    // Desenha os quatro cantos
    drawCircleHelper(x + r, y + r, r, 1, color);
    drawCircleHelper(x + w - r - 1, y + r, r, 2, color);
    drawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
    drawCircleHelper(x + r, y + h - r - 1, r, 8, color);
}

/**
 * @brief Preenche um retângulo com cantos arredondados.
 */
void ST7789_FillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint16_t color) {
    // Preenche o retângulo central
    ST7789_FillRectangle(x + r, y, w - 2 * r, h, color);
    // Preenche as áreas laterais restantes
    fillCircleHelper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1, color);
    fillCircleHelper(x + r, y + r, r, 2, h - 2 * r - 1, color);
}