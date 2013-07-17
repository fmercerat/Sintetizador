#ifndef TABLAS_H_
#define TABLAS_H_

#include <avr/pgmspace.h>

// Sine Wave
const uint8_t seno[256] PROGMEM = { 127, 130, 133, 136, 139, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173, 176, 179, 182, 184, 187, 190, 193, 195, 198, 200, 203, 205, 208, 210, 213, 215, 217, 219, 221, 224, 226, 228, 229, 231, 233, 235, 236, 238, 239, 241, 242, 244, 245, 246, 247, 248, 249, 250, 251, 251, 252, 253, 253, 254, 254, 254, 254, 254, 255, 254, 254, 254, 254, 254, 253, 253, 252, 251, 251, 250, 249, 248, 247, 246, 245, 244, 242, 241, 239, 238, 236, 235, 233, 231, 229, 228, 226, 224, 221, 219, 217, 215, 213, 210, 208, 205, 203, 200, 198, 195, 193, 190, 187, 184, 182, 179, 176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 139, 136, 133, 130, 127, 124, 121, 118, 115, 111, 108, 105, 102, 99, 96, 93, 90, 87, 84, 81, 78, 75, 72, 70, 67, 64, 61, 59, 56, 54, 51, 49, 46, 44, 41, 39, 37, 35, 33, 30, 28, 26, 25, 23, 21, 19, 18, 16, 15, 13, 12, 10, 9, 8, 7, 6, 5, 4, 3, 3, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 15, 16, 18, 19, 21, 23, 25, 26, 28, 30, 33, 35, 37, 39, 41, 44, 46, 49, 51, 54, 56, 59, 61, 64, 67, 70, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 115, 118, 121, 124 };

const uint8_t super[256] PROGMEM =
{
	0x0, 0xa, 0x14, 0x1e, 0x28, 0x33, 0x3d, 0x47,
	0x51, 0x5c, 0x66, 0x70, 0x7a, 0x85, 0x8f, 0x99,
	0xa3, 0xae, 0xb8, 0xc2, 0xcc, 0xd7, 0xe1, 0xeb,
	0xf5, 0xf8, 0xf9, 0xfa, 0xfb, 0xfb, 0xfc, 0xfc,
	0xfd, 0xfd, 0xfd, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe,
	0xff, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfd, 0xfd,
	0xfd, 0xfc, 0xfc, 0xfb, 0xfb, 0xfa, 0xf9, 0xf8,
	0xf7, 0xf7, 0xf6, 0xf5, 0xf4, 0xf2, 0xf1, 0xf0,
	0xef, 0xed, 0xec, 0xeb, 0xe9, 0xe8, 0xe6, 0xe5,
	0xe3, 0xe1, 0xe0, 0xde, 0xdc, 0xda, 0xd8, 0xd6,
	0xd4, 0xd2, 0xd0, 0xce, 0xcc, 0xca, 0xc8, 0xc6,
	0xc4, 0xc1, 0xbf, 0xbd, 0xba, 0xb8, 0xb6, 0xb3,
	0xb1, 0xae, 0xac, 0xa9, 0xa7, 0xa4, 0xa2, 0x9f,
	0x9d, 0x9a, 0x97, 0x95, 0x92, 0x8f, 0x8d, 0x8a,
	0x87, 0x85, 0x82, 0x80, 0x7d, 0x7a, 0x78, 0x75,
	0x72, 0x70, 0x6d, 0x6a, 0x68, 0x65, 0x62, 0x60,
	0x5d, 0x5b, 0x58, 0x56, 0x53, 0x51, 0x4e, 0x4c,
	0x49, 0x47, 0x45, 0x42, 0x40, 0x3e, 0x3b, 0x39,
	0x37, 0x35, 0x33, 0x31, 0x2f, 0x2d, 0x2b, 0x29,
	0x27, 0x25, 0x23, 0x21, 0x1f, 0x1e, 0x1c, 0x1a,
	0x19, 0x17, 0x16, 0x14, 0x13, 0x12, 0x10, 0xf,
	0xe, 0xd, 0xb, 0xa, 0x9, 0x8, 0x8, 0x7,
	0x6, 0x5, 0x4, 0x4, 0x3, 0x3, 0x2, 0x2,
	0x2, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x2, 0x2, 0x2, 0x3,
	0x3, 0x4, 0x4, 0x5, 0x6, 0x7, 0x8, 0x8,
	0x9, 0xa, 0xb, 0xd, 0xe, 0xf, 0x10, 0x12,
	0x13, 0x14, 0x16, 0x17, 0x19, 0x1a, 0x1c, 0x1e,
	0x1f, 0x21, 0x23, 0x25, 0x27, 0x29, 0x2b, 0x2d,
	0x2f, 0x31, 0x33, 0x35, 0x37, 0x39, 0x3b, 0x3e,
	0x40, 0x42, 0x45, 0x47, 0x49, 0x4c, 0x4e, 0x51,
	0x53, 0x56, 0x58, 0x5b, 0x5d, 0x60, 0x62, 0x65
};

const uint8_t super2[] PROGMEM = {
		0x0, 0xa, 0x14, 0x1e, 0x28, 0x33, 0x3d, 0x47,
		0x51, 0x5c, 0x66, 0x70, 0x7a, 0x85, 0x8f, 0x99,
		0xa3, 0xae, 0xb8, 0xc2, 0xcc, 0xd7, 0xe1, 0xeb,
		0xf5, 0xf6, 0xf7, 0xf8, 0xf8, 0xf9, 0xfa, 0xfa,
		0xfb, 0xfb, 0xfb, 0xfc, 0xfc, 0xfd, 0xfd, 0xfd,
		0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfd, 0xfd, 0xfd,
		0xfc, 0xfc, 0xfb, 0xfb, 0xfb, 0xfa, 0xfa, 0xf9,
		0xf8, 0xf8, 0xf7, 0xf6, 0xf6, 0xf5, 0xf4, 0xf4,
		0xf3, 0xf2, 0xf1, 0xf0, 0xf0, 0xef, 0xee, 0xed,
		0xec, 0xeb, 0xea, 0xe9, 0xe8, 0xe7, 0xe6, 0xe5,
		0xe4, 0xe3, 0xe2, 0xe0, 0xdf, 0xde, 0xdd, 0xdc,
		0xdb, 0xd9, 0xd8, 0xd7, 0xd6, 0xd4, 0xd3, 0xd2,
		0xd1, 0xcf, 0xce, 0xcd, 0xcb, 0xca, 0xc9, 0xc7,
		0xc6, 0xc5, 0xc3, 0xc2, 0xc1, 0xc0, 0xbe, 0xbd,
		0xbc, 0xba, 0xb9, 0xb8, 0xb6, 0xb5, 0xb4, 0xb2,
		0xb1, 0xb0, 0xae, 0xad, 0xac, 0xab, 0xa9, 0xa8,
		0xa7, 0xa6, 0xa4, 0xa3, 0xa2, 0xa1, 0xa0, 0x9f,
		0x9d, 0x9c, 0x9b, 0x9a, 0x99, 0x98, 0x97, 0x96,
		0x95, 0x94, 0x93, 0x92, 0x91, 0x90, 0x8f, 0x8f,
		0x8e, 0x8d, 0x8c, 0x8b, 0x8b, 0x8a, 0x89, 0x89,
		0x88, 0x87, 0x87, 0x86, 0x85, 0x85, 0x84, 0x84,
		0x84, 0x83, 0x83, 0x82, 0x82, 0x82, 0x81, 0x81,
		0x81, 0x81, 0x81, 0x80, 0x80, 0x80, 0x80, 0x80,
		0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x81, 0x81,
		0x81, 0x81, 0x81, 0x82, 0x82, 0x82, 0x83, 0x83,
		0x84, 0x84, 0x84, 0x85, 0x85, 0x86, 0x87, 0x87,
		0x88, 0x89, 0x89, 0x8a, 0x8b, 0x8b, 0x8c, 0x8d,
		0x8e, 0x8f, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94,
		0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c,
		0x9d, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa6
};
/*
const uint8_t exponencial[256] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02,
	0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,	0x02, 0x02, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
	0x03, 0x03, 0x03, 0x04, 0x04, 0x04, 0x04, 0x04,	0x04, 0x04, 0x04, 0x05, 0x05, 0x05, 0x05, 0x05,
	0x05, 0x05, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06,	0x07, 0x07, 0x07, 0x07, 0x07, 0x08, 0x08, 0x08,
	0x08, 0x08, 0x09, 0x09, 0x09, 0x09, 0x09, 0x0A,	0x0A, 0x0A, 0x0A, 0x0B, 0x0B, 0x0B, 0x0B, 0x0C,
	0x0C, 0x0C, 0x0C, 0x0D, 0x0D, 0x0D, 0x0E, 0x0E,	0x0E, 0x0F, 0x0F, 0x0F, 0x0F, 0x10, 0x10, 0x11,
	0x11, 0x11, 0x12, 0x12, 0x12, 0x13, 0x13, 0x14,	0x14, 0x14, 0x15, 0x15, 0x16, 0x16, 0x17, 0x17,
	0x18, 0x18, 0x18, 0x19, 0x19, 0x1A, 0x1B, 0x1B,	0x1C, 0x1C, 0x1D, 0x1D, 0x1E, 0x1E, 0x1F, 0x20,
	0x20, 0x21, 0x22, 0x22, 0x23, 0x24, 0x24, 0x25,	0x26, 0x27, 0x27, 0x28, 0x29, 0x2A, 0x2A, 0x2B,
	0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x31, 0x32,	0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x3A, 0x3B,
	0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x42, 0x43, 0x44,	0x45, 0x47, 0x48, 0x49, 0x4B, 0x4C, 0x4E, 0x4F,
	0x51, 0x52, 0x54, 0x55, 0x57, 0x58, 0x5A, 0x5C,	0x5D, 0x5F, 0x61, 0x63, 0x65, 0x66, 0x68, 0x6A,
	0x6C, 0x6E, 0x70, 0x72, 0x75, 0x77, 0x79, 0x7B,	0x7D, 0x80, 0x82, 0x85, 0x87, 0x89, 0x8C, 0x8F,
	0x91, 0x94, 0x97, 0x99, 0x9C, 0x9F, 0xA2, 0xA5,	0xA8, 0xAB, 0xAE, 0xB1, 0xB5, 0xB8, 0xBB, 0xBF,
	0xC2, 0xC6, 0xC9, 0xCD, 0xD1, 0xD5, 0xD9, 0xDD,	0xE1, 0xE5, 0xE9, 0xED, 0xF1, 0xF6, 0xFA, 0xFF
};
*/
#endif
