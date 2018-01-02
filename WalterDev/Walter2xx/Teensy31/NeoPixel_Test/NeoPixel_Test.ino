#include <Adafruit_NeoPixel.h>

#include "NeoPixel_Test.h"

/*
	Parameter 1 = number of pixels in strip1
	Parameter 2 = pin number (most are valid)
	Parameter 3 = pixel type flags, add together as needed:

		NEO_KHZ800	800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
		NEO_KHZ400	400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
		NEO_GRB		Pixels are wired for GRB bitstream (most NeoPixel products)
		NEO_RGB		Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
*/

//	Eight individual NeoPixel LEDs on a breadboard
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(NEOPIXEL_MAX_STRIP1, NEOPIXEL_PIN_STRIP1, NEO_GRB + NEO_KHZ800);

//	A strip of 30 NeoPixel LEDs
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NEOPIXEL_MAX_STRIP2, NEOPIXEL_PIN_STRIP2, NEO_GRB + NEO_KHZ800);

//	A strip of 60 NeoPixel LEDs
Adafruit_NeoPixel strip3 = Adafruit_NeoPixel(NEOPIXEL_MAX_STRIP3, NEOPIXEL_PIN_STRIP3, NEO_GRB + NEO_KHZ800);

/*
	Demonstrate the addressability of each NeoPixel in a strip1
*/
void addressablePixels (Adafruit_NeoPixel *strip, uint16_t nrPixels, uint32_t firstC, uint32_t secondC, uint32_t thirdC, uint16_t waitMS) {
	uint8_t pixel;
	uint32_t pixelColor;

	Serial.println("Addressable Pixels");

    for (pixel = 0; pixel < nrPixels; pixel++) {
		if ((pixel == 0) || (pixel == 3) || (pixel == 6) || (pixel == 9)) {
			pixelColor = firstC;
		} else if ((pixel == 1) || (pixel == 4) || (pixel == 7)) {
			pixelColor = secondC;
		} else {
			pixelColor = thirdC;
		}

		strip->setPixelColor(pixel, pixelColor);
	}

	strip->show();
	delay(waitMS);
}

/*
	Blink a single pixel in a strip1
*/
void blinkPixel (uint16_t nrPixels, uint16_t pixelNr, uint32_t pixelColor, uint16_t nrCycles, uint16_t waitMS) {
	uint16_t cycles;

	Serial.println("Blinking a single pixel");

	clearPixels(NEOPIXEL_MAX_STRIP1, 0, NEOPIXEL_MAX_STRIP1);

	for (cycles = 0; cycles < nrCycles; cycles++) {
		//	Turn the pixel ON
		strip1.setPixelColor(pixelNr, pixelColor);
		strip1.show();
		delay(waitMS);

		//	Turn the pixel OFF
		strip1.setPixelColor(pixelNr, 0);
		strip1.show();
		delay(waitMS);
	}
}

/*
	Blink an entire strip1 of pixels
*/
void blinkStrip (uint16_t nrPixels, uint16_t nrCycles, uint32_t strip1Color, uint16_t waitMS) {
	uint16_t cycles;

	Serial.println("Blinking the entire strip1");

	for (cycles = 0; cycles < nrCycles; cycles++) {
		colorWipe(nrPixels, strip1Color, waitMS);

		clearPixels(NEOPIXEL_MAX_STRIP1, 0, NEOPIXEL_MAX_STRIP1);
		delay(waitMS);
	}
}

/*
	Clear all the pixels to OFF
*/
void clearPixels (uint16_t nrPixels, uint16_t startPixel, uint16_t endPixel) {
	uint8_t pixel;

	for (pixel = startPixel; ((pixel < endPixel) && (pixel < nrPixels)); pixel++) {
		strip1.setPixelColor(pixel, 0);
	}

	strip1.show();
}

/*
	I got the code for colorWipe(), heartThrob(), rainbow(), and wheel()
		from: https://gist.github.com/antitree/7188144

	I've modified the code to work with this sketch.
*/

/*
	Fill the pixels, one after the other, with a color
*/
void colorWipe (uint16_t nrPixels, uint32_t pixelColor, uint16_t waitMS) {
	uint16_t pixel = 0;

	for(pixel = 0; pixel < nrPixels; pixel++) {
		strip1.setPixelColor(pixel, pixelColor);
	}

	strip1.show();

	delay(waitMS);
}
 
void heartThrob (uint16_t nrPixels, uint32_t startColor, uint32_t endColor, uint16_t waitMS) {
	uint16_t inner, outer;
 
	Serial.println("Heart Throb");

	for (outer = startColor; outer < endColor; outer++) {
		for (inner = 0; inner < nrPixels; inner++) {
			strip1.setPixelColor(inner, wheel((inner + outer) & 255));
		}

		strip1.show();

		delay(waitMS);
	}
}

/*
	Knight Rider - Kitt's front scanner
*/
void knightRider (uint16_t nrPixels, uint32_t ledColor, uint8_t nrCycles, uint16_t waitMS) {
	int pixel;
	uint8_t cycles = 0;

	Serial.println("Knight Rider");

	clearPixels(NEOPIXEL_MAX_STRIP1, 0, NEOPIXEL_MAX_STRIP1);

	while (cycles < nrCycles) {
		pixel = 0;

		//	Forward sequence
		while (pixel < nrPixels) {
			strip1.setPixelColor(pixel, ledColor);
			strip1.show();
			delay(waitMS);

			strip1.setPixelColor(pixel, 0);
			strip1.show();

			pixel += 1;
		}

		pixel -= 2;

		//	Reverse sequence
		while (pixel > 0) {
			strip1.setPixelColor(pixel, ledColor);
			strip1.show();
			delay(waitMS);

			strip1.setPixelColor(pixel, 0);
			strip1.show();

			pixel -= 1;
		}

		cycles += 1;
	}
}

/*
	Secret rainbow mode
*/ 
void rainbow (uint16_t nrPixels, uint8_t waitMS) {
	uint16_t inner, outer;

	Serial.println("Rainbow");
 
	for(outer = 0; outer < 256; outer++) {
		for(inner = 0; inner < nrPixels; inner++) {
			strip1.setPixelColor(inner, wheel((inner + outer) & 255));
		}

		strip1.show();
		delay(waitMS);
	}
}

/*
	Input a value 0 to 255 to get a color value.

	The colours are a transition r - g - b - back to r.
*/
uint32_t wheel (uint8_t position) {
	uint8_t wheelPos = position;
	uint32_t resultColor;

	if (wheelPos < 85) {
		resultColor = strip1.Color(wheelPos * 3, 255 - wheelPos * 3, 0);
	} else if (wheelPos < 170) {
		wheelPos -= 85;
		resultColor = strip1.Color(255 - wheelPos * 3, 0, wheelPos * 3);
	} else {
		wheelPos -= 170;
		resultColor = strip1.Color(0, wheelPos * 3, 255 - wheelPos * 3);
	}

	return resultColor;
}

void setup(void) {
	Serial.begin(115200);
	Serial.println("NeoPixel Test");

	strip1.begin();
	strip1.setBrightness(15);

	//	Initialize all pixels to 'off'
	strip1.show();
}

void loop (void) {
	uint32_t shadePink = strip1.Color(255, 55, 85);
	uint32_t shadeLime = strip1.Color(150, 225, 0);
	uint32_t shadePurple = strip1.Color(50, 0, 50);
	uint32_t shadeOrange = strip1.Color(180, 135, 0);
	uint32_t shadeMagenta = strip1.Color(195, 0, 125);
	uint32_t shadeYellow = strip1.Color(255, 220, 0);

	uint32_t shadeRed = strip1.Color(255, 0, 0);
	uint32_t shadeGreen = strip1.Color(0, 255, 0);
	uint32_t shadeBlue = strip1.Color(0, 0, 255);

	//	The basic color components
	uint8_t red = 0, green = 0, blue = 0;
	uint8_t cycles = 0;

	uint16_t nrPixels = strip1.numPixels();

	/*
		Code starts here
	*/


	//	Display the color values
	Serial.println("Colors:");
	Serial.print("     Pink = ");
	Serial.println(shadePink);

	Serial.print("     Lime = ");
	Serial.println(shadeLime);

	Serial.print("     Purple = ");
	Serial.println(shadePurple);

	Serial.print("     Orange = ");
	Serial.println(shadeOrange);

	Serial.print("     Magenta = ");
	Serial.println(shadeMagenta);

	Serial.print("     Yellow = ");
	Serial.println(shadeYellow);

	Serial.print("     Red = ");
	Serial.println(shadeRed);

	Serial.print("     Green = ");
	Serial.println(shadeGreen);

	Serial.print("     Blue = ");
	Serial.println(shadeBlue);

	//	Do some complete strip1 color wipes
	Serial.println("Color Wipes:");

	Serial.println("     Pink");
	colorWipe(nrPixels, shadePink, 3500);

	Serial.println("     Lime");
	colorWipe(nrPixels, shadeLime, 3500);

	Serial.println("     Purple");
	colorWipe(nrPixels, shadePurple, 3500);

	Serial.println("     Orange");
	colorWipe(nrPixels, shadeOrange, 3500);

	Serial.println("     Magenta");
	colorWipe(nrPixels, shadeMagenta, 3500);

	Serial.println("     Yellow");
	colorWipe(nrPixels, shadeYellow, 3500);

	Serial.println("     Red");
	colorWipe(nrPixels, shadeRed, 3500);

	Serial.println("     Green");
	colorWipe(nrPixels, shadeGreen, 3500);

	Serial.println("     Blue");
	colorWipe(nrPixels, shadeBlue, 3500);

	delay(500);

	//	Show how each NeoPixel LED is individually addressable
	addressablePixels(&strip1, NEOPIXEL_MAX_STRIP1, shadeRed, shadeYellow, shadeGreen, 5000);
	addressablePixels(&strip1, NEOPIXEL_MAX_STRIP1, shadeMagenta, shadeOrange, shadeBlue, 5000);
	addressablePixels(&strip1, NEOPIXEL_MAX_STRIP1, shadeBlue, shadeLime, shadePurple, 5000);

	heartThrob(nrPixels, 150, 375, 100);

	//	That good old Kitt car on Knight Rider
	knightRider(nrPixels, shadeRed, 10, 75);
	knightRider(nrPixels, shadeLime, 10, 75);
	knightRider(nrPixels, shadeBlue, 10, 75);
	knightRider(nrPixels, shadeOrange, 10, 75);

	//	Rainbow
	rainbow(nrPixels, 250);
	delay(5000);

	//	Blink the entire strip1
	blinkStrip(nrPixels, 10, shadeBlue, 500);
	blinkStrip(nrPixels, 10, shadeMagenta, 500);
	blinkStrip(nrPixels, 10, shadeLime, 500);

	//	Blink some individual pixels
	blinkPixel(nrPixels, 3, shadeBlue, 10, 250);
	blinkPixel(nrPixels, 6, shadeOrange, 10, 250);
	blinkPixel(nrPixels, 1, shadeLime, 10, 250);
	blinkPixel(nrPixels, 4, shadeMagenta, 10, 250);

	//	The End
	clearPixels(NEOPIXEL_MAX_STRIP1, 0, NEOPIXEL_MAX_STRIP1);
}
