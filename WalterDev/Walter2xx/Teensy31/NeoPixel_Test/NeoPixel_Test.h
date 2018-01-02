#ifndef _NEOPIXEL_TEST_H_
#define _NEOPIXEL_TEST_H_

#define		NEOPIXEL_PIN_STRIP1			5
#define		NEOPIXEL_MAX_STRIP1			8

#define		NEOPIXEL_PIN_STRIP2			6
#define		NEOPIXEL_MAX_STRIP2			30

#define		NEOPIXEL_PIN_STRIP3			7
#define		NEOPIXEL_MAX_STRIP3			60

struct NeoPixel {
	boolean state;
	boolean blinking;
	uint16_t blinkTime;
	uint32_t color;
};

struct NeoPixelStrip {
	Adafruit_NeoPixel *strip;
	NeoPixel *pixels[];
};

#endif
