#ifndef _NEOPIXEL_TEST2_H_
#define _NEOPIXEL_TEST2_H_

#define		NEOPIXEL_PIN_STRIP8			4
#define		NEOPIXEL_MAX_STRIP8			8

#define		NEOPIXEL_PIN_STRIP12		4
#define		NEOPIXEL_MAX_STRIP12		12

#define		NEOPIXEL_PIN_STRIP16		5
#define		NEOPIXEL_MAX_STRIP16		16

#define		NEOPIXEL_PIN_STRIP30		6
#define		NEOPIXEL_MAX_STRIP30		30

#define		NEOPIXEL_PIN_STRIP60		7
#define		NEOPIXEL_MAX_STRIP60		60

#define		NEOPIXEL_PIN_STRIP144		8
#define		NEOPIXEL_MAX_STRIP144		144

#define		NEOPIXEL_PIN_RING12			9
#define		NEOPIXEL_MAX_RING12			12

#define		NEOPIXEL_PIN_RING16			10
#define		NEOPIXEL_MAX_RING16			16

#define		NEOPIXEL_PIN_RING24			11
#define		NEOPIXEL_MAX_RING24			24

struct NeoPixel {
	uint16_t pixel;
	Adafruit_NeoPixel *strip;
	boolean currState;
	uint32_t currColor;

	boolean isBlinking;
	uint16_t waitMS;
};

struct NeoPixelStrip {
	Adafruit_NeoPixel *strip;
	NeoPixel pixels[];
};

#endif
