                            #ifndef FONT_H
#define FONT_H

#include <stdint.h>

// Definições da fonte
#define FONT_WIDTH 		7
#define FONT_HEIGHT 	10

// Array da fonte. Contém os dados de bitmap para os caracteres ASCII de ' ' (espaço) até '~'.
extern const uint8_t font[];

#endif // FONT_H