

/*
 *
 * new Font
 *
 * created with GLCDFontCreator
 * original framework by F. Maximilian Thiele
 * Modified By Siddharth Kaul
 *
 *
 * File Name           : callibri_size14.h
 * Date                : 10.11.2012
 * Font size in bytes  : 7102
 * Font width          : 10
 * Font height         : 14
 * Font first char     : 32
 * Font last char      : 128
 * Font used chars     : 96
 *
 * The font data are defined as
 *
 * struct _FONT_ {
 *     uint16_t   font_Size_in_Bytes_over_all_included_Size_it_self;
 *     uint8_t    font_Width_in_Pixel_for_fixed_drawing;
 *     uint8_t    font_Height_in_Pixel_for_all_characters;
 *     unit8_t    font_First_Char;
 *     uint8_t    font_Char_Count;
 *
 *     uint8_t    font_Char_Widths[font_Last_Char - font_First_Char +1];
 *                  // for each character the separate width in pixels,
 *                  // characters < 128 have an implicit virtual right empty row
 *
 *     uint8_t    font_data[];
 *                  // bit field of all characters
 */

#include <inttypes.h>
#include <avr/pgmspace.h>

#ifndef callibri_size14_H
#define callibri_size14_H

#define callibri_size14_WIDTH 10
#define callibri_size14_HEIGHT 14

static uint8_t callibri_size14[] PROGMEM = {
    0x1B, 0xBE, // size
    0x0A, // width
    0x0E, // height
    0x20, // first char
    0x60, // char count
    
    // char widths
    0x00, 0x01, 0x04, 0x07, 0x05, 0x09, 0x08, 0x01, 0x02, 0x02, 
    0x05, 0x07, 0x03, 0x03, 0x01, 0x05, 0x07, 0x05, 0x06, 0x06, 
    0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x01, 0x02, 0x06, 0x06, 
    0x05, 0x05, 0x0A, 0x08, 0x06, 0x06, 0x07, 0x05, 0x05, 0x07, 
    0x06, 0x01, 0x03, 0x05, 0x05, 0x0A, 0x07, 0x08, 0x05, 0x08, 
    0x06, 0x05, 0x07, 0x07, 0x08, 0x0C, 0x07, 0x05, 0x06, 0x03, 
    0x05, 0x02, 0x05, 0x07, 0x02, 0x05, 0x06, 0x05, 0x06, 0x06, 
    0x04, 0x06, 0x05, 0x01, 0x02, 0x05, 0x01, 0x09, 0x05, 0x06, 
    0x06, 0x06, 0x04, 0x04, 0x04, 0x05, 0x06, 0x0A, 0x06, 0x06, 
    0x04, 0x03, 0x01, 0x03, 0x06, 0x07, 
    
    // font data
    0xFE, 0x18, // 33
    0x1E, 0x00, 0x0E, 0x02, 0x00, 0x00, 0x00, 0x00, // 34
    0x80, 0xD0, 0xBC, 0x90, 0xD0, 0xBC, 0x10, 0x00, 0x1C, 0x00, 0x00, 0x1C, 0x00, 0x00, // 35
    0x38, 0x64, 0x47, 0x44, 0x88, 0x18, 0x70, 0x10, 0x10, 0x0C, // 36
    0x38, 0x44, 0x44, 0x38, 0x80, 0xC0, 0x70, 0x48, 0x84, 0x00, 0x00, 0x10, 0x08, 0x04, 0x0C, 0x10, 0x10, 0x0C, // 37
    0xC0, 0x7E, 0x62, 0xF2, 0x9E, 0x04, 0xC0, 0x00, 0x0C, 0x10, 0x10, 0x10, 0x1C, 0x0C, 0x1C, 0x10, // 38
    0x1E, 0x00, // 39
    0xF0, 0x0E, 0x1C, 0xE0, // 40
    0x0E, 0xF0, 0xE0, 0x1C, // 41
    0x14, 0x14, 0x3E, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, // 42
    0x80, 0x80, 0x80, 0xF0, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, // 43
    0x00, 0x00, 0x00, 0x40, 0x38, 0x08, // 44
    0x80, 0x80, 0x80, 0x00, 0x00, 0x00, // 45
    0x00, 0x18, // 46
    0x00, 0x00, 0xC0, 0x38, 0x0E, 0x60, 0x38, 0x04, 0x00, 0x00, // 47
    0xF0, 0x08, 0x04, 0x04, 0x04, 0x0C, 0xF0, 0x04, 0x18, 0x10, 0x10, 0x10, 0x08, 0x04, // 48
    0x08, 0x04, 0xFC, 0x00, 0x00, 0x10, 0x10, 0x1C, 0x10, 0x10, // 49
    0x08, 0x04, 0x04, 0x84, 0x44, 0x38, 0x10, 0x18, 0x14, 0x10, 0x10, 0x10, // 50
    0x08, 0x44, 0x44, 0x44, 0xE4, 0xB8, 0x08, 0x10, 0x10, 0x10, 0x10, 0x0C, // 51
    0x80, 0x60, 0x30, 0x0C, 0xFC, 0x00, 0x04, 0x04, 0x04, 0x04, 0x1C, 0x04, // 52
    0x3C, 0x24, 0x24, 0x24, 0x64, 0xC0, 0x18, 0x10, 0x10, 0x10, 0x18, 0x0C, // 53
    0xF0, 0x28, 0x24, 0x24, 0x24, 0xC4, 0x0C, 0x18, 0x10, 0x10, 0x18, 0x0C, // 54
    0x04, 0x04, 0x04, 0xC4, 0x3C, 0x0C, 0x00, 0x10, 0x1C, 0x04, 0x00, 0x00, // 55
    0x98, 0xA4, 0x44, 0x44, 0xA4, 0x98, 0x0C, 0x10, 0x10, 0x10, 0x10, 0x0C, // 56
    0x38, 0x44, 0x44, 0x44, 0x4C, 0xF8, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, // 57
    0x30, 0x18, // 58
    0x00, 0x60, 0x40, 0x38, // 59
    0x40, 0xE0, 0xA0, 0x10, 0x10, 0x08, 0x00, 0x00, 0x00, 0x04, 0x04, 0x08, // 60
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, // 61
    0x10, 0x20, 0x20, 0xC0, 0xC0, 0x08, 0x04, 0x04, 0x00, 0x00, // 62
    0x06, 0xE2, 0x22, 0x36, 0x1C, 0x00, 0x18, 0x18, 0x00, 0x00, // 63
    0xC0, 0x30, 0xC8, 0x64, 0x24, 0xE4, 0x64, 0x04, 0x08, 0xF0, 0x1C, 0x20, 0x4C, 0x48, 0x48, 0x44, 0x48, 0x08, 0x0C, 0x00, // 64
    0x00, 0x80, 0xF0, 0x1C, 0x1C, 0xF0, 0x80, 0x00, 0x10, 0x1C, 0x04, 0x04, 0x04, 0x04, 0x1C, 0x10, // 65
    0xFC, 0x44, 0x44, 0x44, 0x78, 0x80, 0x1C, 0x10, 0x10, 0x10, 0x10, 0x0C, // 66
    0xF0, 0x08, 0x04, 0x04, 0x04, 0x08, 0x04, 0x08, 0x10, 0x10, 0x10, 0x08, // 67
    0xFC, 0x04, 0x04, 0x04, 0x04, 0x08, 0xF0, 0x1C, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, // 68
    0xFC, 0x44, 0x44, 0x44, 0x04, 0x1C, 0x10, 0x10, 0x10, 0x10, // 69
    0xFC, 0x44, 0x44, 0x44, 0x44, 0x1C, 0x00, 0x00, 0x00, 0x00, // 70
    0xF0, 0x08, 0x04, 0x04, 0x44, 0x44, 0xC8, 0x04, 0x0C, 0x10, 0x10, 0x10, 0x10, 0x1C, // 71
    0xFC, 0x40, 0x40, 0x40, 0x40, 0xFC, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x1C, // 72
    0xFC, 0x1C, // 73
    0x00, 0x00, 0xFC, 0x10, 0x10, 0x0C, // 74
    0xFC, 0x40, 0xB0, 0x18, 0x04, 0x1C, 0x00, 0x00, 0x0C, 0x18, // 75
    0xFC, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x10, 0x10, 0x10, 0x10, // 76
    0xFC, 0x0C, 0x30, 0xC0, 0x00, 0x00, 0xC0, 0x30, 0x0C, 0xFC, 0x1C, 0x00, 0x00, 0x04, 0x1C, 0x1C, 0x04, 0x00, 0x00, 0x1C, // 77
    0xFC, 0x0C, 0x30, 0x60, 0x80, 0x00, 0xFC, 0x1C, 0x00, 0x00, 0x00, 0x04, 0x1C, 0x1C, // 78
    0xF0, 0x08, 0x04, 0x04, 0x04, 0x04, 0x08, 0xF0, 0x04, 0x08, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, // 79
    0xFC, 0x84, 0x84, 0xC4, 0x78, 0x1C, 0x00, 0x00, 0x00, 0x00, // 80
    0xF0, 0x08, 0x04, 0x04, 0x04, 0x04, 0x08, 0xF0, 0x04, 0x08, 0x10, 0x10, 0x10, 0x10, 0x18, 0x34, // 81
    0xFC, 0x44, 0x44, 0xC4, 0x38, 0x00, 0x1C, 0x00, 0x00, 0x04, 0x1C, 0x10, // 82
    0x38, 0x24, 0x44, 0xC4, 0x88, 0x08, 0x10, 0x10, 0x10, 0x0C, // 83
    0x04, 0x04, 0x04, 0xFC, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, // 84
    0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x0C, 0x18, 0x10, 0x10, 0x10, 0x08, 0x0C, // 85
    0x04, 0x3C, 0xE0, 0x00, 0x00, 0xE0, 0x3C, 0x04, 0x00, 0x00, 0x04, 0x1C, 0x1C, 0x04, 0x00, 0x00, // 86
    0x04, 0x3C, 0xE0, 0x00, 0xC0, 0x7C, 0x1C, 0xE0, 0x00, 0x80, 0xF0, 0x0C, 0x00, 0x00, 0x0C, 0x18, 0x0C, 0x00, 0x00, 0x04, 0x1C, 0x1C, 0x00, 0x00, // 87
    0x00, 0x0C, 0xB8, 0x60, 0xB0, 0x1C, 0x04, 0x10, 0x18, 0x04, 0x00, 0x04, 0x1C, 0x10, // 88
    0x0C, 0x30, 0xC0, 0x30, 0x0C, 0x00, 0x00, 0x1C, 0x00, 0x00, // 89
    0x04, 0x04, 0x84, 0x64, 0x3C, 0x0C, 0x10, 0x1C, 0x14, 0x10, 0x10, 0x10, // 90
    0xFE, 0x02, 0x02, 0x7C, 0x40, 0x40, // 91
    0x02, 0x1C, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x70, // 92
    0x02, 0xFE, 0x40, 0x7C, // 93
    0x60, 0x18, 0x04, 0x38, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, // 94
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, // 95
    0x02, 0x04, 0x00, 0x00, // 96
    0x20, 0x90, 0x90, 0x90, 0xE0, 0x0C, 0x10, 0x10, 0x10, 0x1C, // 97
    0xFE, 0x20, 0x10, 0x10, 0x30, 0xE0, 0x1C, 0x08, 0x10, 0x10, 0x18, 0x0C, // 98
    0xE0, 0x30, 0x10, 0x10, 0x20, 0x0C, 0x18, 0x10, 0x10, 0x08, // 99
    0xE0, 0x30, 0x10, 0x10, 0x20, 0xFE, 0x0C, 0x18, 0x10, 0x10, 0x08, 0x1C, // 100
    0xE0, 0xB0, 0x90, 0x90, 0xB0, 0xE0, 0x0C, 0x18, 0x10, 0x10, 0x10, 0x10, // 101
    0x10, 0xFC, 0x12, 0x12, 0x00, 0x1C, 0x00, 0x00, // 102
    0xE0, 0x10, 0x10, 0x10, 0xF0, 0x10, 0x6C, 0x94, 0x94, 0x94, 0x90, 0x60, // 103
    0xFE, 0x20, 0x10, 0x10, 0xE0, 0x1C, 0x00, 0x00, 0x00, 0x1C, // 104
    0xF4, 0x1C, // 105
    0x00, 0xF4, 0x80, 0xFC, // 106
    0xFE, 0xC0, 0x60, 0x30, 0x10, 0x1C, 0x00, 0x04, 0x18, 0x10, // 107
    0xFE, 0x1C, // 108
    0xF0, 0x20, 0x10, 0x10, 0xE0, 0x20, 0x10, 0x10, 0xE0, 0x1C, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x1C, // 109
    0xF0, 0x20, 0x10, 0x10, 0xE0, 0x1C, 0x00, 0x00, 0x00, 0x1C, // 110
    0xE0, 0x30, 0x10, 0x10, 0x30, 0xE0, 0x0C, 0x18, 0x10, 0x10, 0x18, 0x0C, // 111
    0xF0, 0x20, 0x10, 0x10, 0x30, 0xE0, 0xFC, 0x08, 0x10, 0x10, 0x18, 0x0C, // 112
    0xE0, 0x30, 0x10, 0x10, 0x20, 0xF0, 0x0C, 0x18, 0x10, 0x10, 0x08, 0xFC, // 113
    0xF0, 0x20, 0x10, 0x10, 0x1C, 0x00, 0x00, 0x00, // 114
    0x60, 0x90, 0x90, 0x10, 0x10, 0x10, 0x10, 0x0C, // 115
    0x10, 0xFC, 0x10, 0x10, 0x00, 0x1C, 0x10, 0x10, // 116
    0xF0, 0x00, 0x00, 0x00, 0xF0, 0x0C, 0x10, 0x10, 0x08, 0x1C, // 117
    0x30, 0xE0, 0x00, 0x00, 0xE0, 0x10, 0x00, 0x04, 0x1C, 0x1C, 0x00, 0x00, // 118
    0x10, 0xF0, 0x00, 0x00, 0xF0, 0xF0, 0x00, 0x00, 0xF0, 0x10, 0x00, 0x00, 0x1C, 0x1C, 0x00, 0x00, 0x1C, 0x1C, 0x00, 0x00, // 119
    0x00, 0x30, 0xC0, 0xC0, 0x30, 0x10, 0x10, 0x18, 0x04, 0x04, 0x18, 0x10, // 120
    0x10, 0xF0, 0x80, 0x00, 0xC0, 0x30, 0x00, 0x00, 0xDC, 0x38, 0x04, 0x00, // 121
    0x10, 0x90, 0x70, 0x30, 0x18, 0x14, 0x10, 0x10, // 122
    0x40, 0xBE, 0x02, 0x00, 0x7C, 0x40, // 123
    0xFE, 0xFC, // 124
    0x02, 0xBE, 0x40, 0x40, 0x7C, 0x00, // 125
    0x30, 0x08, 0x18, 0x20, 0x20, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 126
    0xFC, 0x04, 0x14, 0x54, 0x74, 0x04, 0xFC, 0x1C, 0x10, 0x10, 0x14, 0x10, 0x10, 0x1C // 127
    
};

#endif
