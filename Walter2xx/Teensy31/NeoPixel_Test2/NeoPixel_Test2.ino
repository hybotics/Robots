/*
	Program:		NeoPixel_Test2 - Based on NeoPixel_Test - Converted to using pointers
	Date:			30-Jan-2014
	Version:		0.1.4 ALPHA (Based on v0.1.2 of NeoPixel_Test)

	Purpose:		Converted NeoPixel_Test to pass a pointer to the NeoPixel strip to
						operate on. This *should* make all my routines usable with any defined
						"strip" of NeoPixels, regardless of whether they are individual, a string,
						a disk, stick, etc. So far, this is working great, but more testing with
						different types of NeoPixel arrangements is needed. I have this working with
						a "strip" of eight individual NeoPixels (http://www.adafruit.com/products/1312).
					-------------------------------------------------------------------------------------
					v0.1.3 28-Jan-2014
					Added range error checking to those routines that have a pixel number or startPixel and
						endPixel ranges.

					Experimenting with adding start and end pixel ranges to my routines - added to a version
						of colorWipe() and knightRider(), as the last two parameters. Two versions (duplicated
						and modified code) of colorWipe(), one version of knightRider().
					-------------------------------------------------------------------------------------
					v1.1.4 30-Jan-2014
					Added the setNeoPixel() routine, and modifed all routines to use it. It sets an idividual
						NeoPixel to the specified color.

					Most NeoPixel display routines can also do ranges of NeoPixels now. The colorWipe() and
						knightRider() routines have this capability now. If they are called with the two optional
						parameters, one of which must be greater than zero, a range of NeoPixels will be used,
						rather than the entire strip. If the startPixel and endPixel parameters are omitted, the
						entire strip will be used and/or set to the specified color.

					Added definitions to the header file for all the different configurations of NeoPixels,
						sold by Adafruit.

	Dependencies:	Adafruit libraries:
						NeoPixel

	Comments:		Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2Y0A21YK0F IR and PING sensors from the Arduino
						Playground, which I have modified to suit my needs.

					Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/
#include <Adafruit_NeoPixel.h>

#include "NeoPixel_Test2.h"

/*
	I got the code for colorWipe(), heartThrob() [now called
		colorFades()], and wheel()
		from: https://gist.github.com/antitree/7188144

	I've modified that code to work best with this sketch.
*/

/*
	Parameter 1 = number of pixels in strip8
	Parameter 2 = pin number (most are valid)
	Parameter 3 = pixel type flags, add together as needed:

		NEO_KHZ800	800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
		NEO_KHZ400	400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
		NEO_GRB		Pixels are wired for GRB bitstream (most NeoPixel products)
		NEO_RGB		Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
*/

//	Eight individual NeoPixel LEDs on a breadboard
Adafruit_NeoPixel strip8 = Adafruit_NeoPixel(NEOPIXEL_MAX_STRIP8, NEOPIXEL_PIN_STRIP8, NEO_GRB + NEO_KHZ800);

//	Eight individual NeoPixel LEDs on a breadboard
Adafruit_NeoPixel strip16 = Adafruit_NeoPixel(NEOPIXEL_MAX_STRIP16, NEOPIXEL_PIN_STRIP16, NEO_GRB + NEO_KHZ800);

//	A strip of 30 NeoPixel LEDs
Adafruit_NeoPixel strip30 = Adafruit_NeoPixel(NEOPIXEL_MAX_STRIP30, NEOPIXEL_PIN_STRIP30, NEO_GRB + NEO_KHZ800);

//	A strip of 60 NeoPixel LEDs
Adafruit_NeoPixel strip60 = Adafruit_NeoPixel(NEOPIXEL_MAX_STRIP60, NEOPIXEL_PIN_STRIP60, NEO_GRB + NEO_KHZ800);

/*
	Check a parameter for range errors - if it's less than zero,
		or greater than the numbers of NeoPixels in a strip, it's
		out of range.
*/
uint16_t checkParameter (Adafruit_NeoPixel *strip, uint16_t parameter) {
	uint16_t error = 0;

	//	Check parameters for range errors
	if ((parameter < 0) || (parameter > (strip->numPixels() - 1))) {
		error = 1000;
	}

	return error;
}

uint16_t checkRange (Adafruit_NeoPixel *strip, uint16_t startRange, uint16_t endRange, bool zeroBase=false) {
	uint16_t error = 0;

	//	Check parameters for range errors
	if (startRange > endRange) {
		error = 1003;
	} else if (checkParameter(strip, startRange) != 0) {
		error = 1001;
	} else if (zeroBase) {
		if (checkParameter(strip, endRange) != 0) {
			error = 1002;
		} else if (checkParameter(strip, (endRange - 1)) != 0) {
			error = 1002;
		}
	}

	return error;
}

/*
	Return a string for true/false
*/
String trueFalse (boolean state) {
	String result;

	if (state) {
		result = "True";
	} else {
		result = "False";
	}

	return result;
}

/*
	Set a single NeoPixel to the specified color
*/
uint16_t setNeoPixel (Adafruit_NeoPixel *strip, uint16_t pixelNr, uint32_t pixelColor, boolean showNow=true) {
	uint16_t error = 0;

	error = checkParameter(strip, pixelNr);

	if (error != 0) {
		displayError("(setNeoPixel) Error: ", error);
	} else {
		strip->setPixelColor(pixelNr, pixelColor);

		if (showNow) {
			strip->show();
		}
	}

	return error;
}

/*
	Clear all the pixels in a strip, or a range of pixels in a strip, to OFF (color = 0)
*/
uint16_t clearPixels (Adafruit_NeoPixel *strip, uint16_t startPixel=0, uint16_t endPixel=0) {
	uint16_t error = 0;
	uint16_t pixelStart = startPixel, pixelEnd = endPixel;
	uint8_t pixel;

	//	Default to all pixels on the strip
	if ((pixelStart == 0) && (pixelEnd == 0)) {
		pixelEnd = strip->numPixels();
	} else {
		//	Check parameter for range errors
		error = checkRange(strip, pixelStart, pixelEnd, true);
		pixelEnd += 1;
	}

	if (error != 0) {
		displayError("(clearPixels) Error: ", error);
	} else {
		//	Clear the range of pixels
		for (pixel = pixelStart; pixel < pixelEnd; pixel++) {
			setNeoPixel(strip, pixel, 0, false);
		}

		strip->show();
	}

	return error;
}

/*
	Demonstrate the addressability of each NeoPixel in a strip of up to 16 (0 - 15)
		NeoPixels.
*/
uint16_t addressablePixels (Adafruit_NeoPixel *strip, uint32_t firstColor, uint32_t secondColor, uint32_t thirdColor, uint16_t waitMS) {
	uint16_t error = 0;
	uint8_t pixel;
	uint32_t pixelColor;

	Serial.println("Addressable Pixels");

    for (pixel = 0; pixel < strip->numPixels(); pixel++) {
		if ((pixel == 0) || (pixel == 3) || (pixel == 6) || (pixel == 9) || (pixel == 12) || (pixel == 15)) {
			pixelColor = firstColor;
		} else if ((pixel == 1) || (pixel == 4) || (pixel == 7) || (pixel == 10) || (pixel == 13)) {
			pixelColor = secondColor;
		} else {
			pixelColor = thirdColor;
		}

		setNeoPixel(strip, pixel, pixelColor, false);
	}

	strip->show();
	delay(waitMS);

	return error;
}

/*
	Fill all, or a range of, pixels in a strip with a color
*/
uint16_t colorWipe (Adafruit_NeoPixel *strip, uint32_t pixelColor, uint16_t waitMS, uint16_t startPixel=0, uint16_t endPixel=0) {
	uint16_t error = 0;
	uint16_t pixel = 0;
	uint16_t pixelStart = startPixel, pixelEnd = endPixel;

	if ((pixelStart == 0) && (pixelEnd == 0)) {
		//	Fill the entire strip
		pixelEnd = strip->numPixels() - 1;
	} else {
		//	Fill a range of pixels in the strip
		//	Check parameters for range errors
		error = checkRange(strip, pixelStart, pixelEnd, true);

		if (error != 0) {
			displayError("(colorWipe[range]) Error: ", error);
			return error;
		} else {
			pixelEnd -= 1;
		}
	}

	for (pixel = pixelStart; pixel <= pixelEnd; pixel++) {
		setNeoPixel(strip, pixel, pixelColor, false);
	}

	strip->show();
	delay(waitMS);

	return error;
}

/*
	Blink a single pixel in a strip8
*/
uint16_t blinkNeoPixel (Adafruit_NeoPixel *strip, uint16_t pixelNr, uint32_t pixelColor, uint16_t nrCycles, uint16_t waitMS, boolean restoreState=false) {
	uint16_t error = 0;
	uint16_t cycles;
	uint32_t savedColor;

	NeoPixel backup = {
		0,
		false,
		0,
		false,
		0
	};

	if (pixelNr > (strip->numPixels() - 1)) {
		error = 1005;
	} else {
		Serial.print("Blinking pixel #");
		Serial.println(pixelNr);

		if (restoreState) {
			//	Save the current state of the pixel
			Serial.println("(blinkNeoPixel) Saving pixel state..");

			savedColor = strip->getPixelColor(pixelNr);

			Serial.print("(blinkNeoPixel) Saved color = ");
			Serial.print(savedColor);
			Serial.print(", pixelColor = ");
			Serial.println(pixelColor);

			backup.strip = strip;
			backup.pixel = pixelNr;
			backup.currState = (savedColor > 0);
			backup.currColor = savedColor;
		}

		//	Start blinking!
		for (cycles = 0; cycles < nrCycles; cycles++) {
			//	Turn the pixel ON
			setNeoPixel(strip, pixelNr, pixelColor);
			delay(waitMS);

			//	Turn the pixel OFF
			setNeoPixel(strip, pixelNr, 0);
			delay(waitMS);
		}

		//	Restore the previous state of the pixel
		if (restoreState && backup.currState) {
			Serial.println("(blinkNeoPixel) Restoring previous pixel state..");

			Serial.print("(blinkNeoPixel) Saved color = ");
			Serial.print(savedColor);
			Serial.print(", pixelColor = ");
			Serial.print(pixelColor);
			Serial.print(", backup.currColor = ");
			Serial.println(backup.currColor);

			setNeoPixel(strip, pixelNr, savedColor);
		}
	}

	return error;
}

/*
	Blink an entire strip8 of pixels
*/
uint16_t blinkNeoPixelStrip (Adafruit_NeoPixel *strip, uint16_t nrCycles, uint32_t stripColor, uint16_t waitMS, uint16_t startPixel=0, uint16_t endPixel=0) {
	uint16_t error = 0;
	uint16_t cycles;
	uint16_t pixelStart = startPixel, pixelEnd = endPixel;

	Serial.println("Blinking the entire strip");

	if ((pixelStart == 0) && (pixelEnd == 0)) {
		pixelEnd = strip->numPixels() - 1;
	} else {
		error = checkRange(strip, pixelStart, pixelEnd, true);
	}

	if (error != 0) {
		displayError("(blinkNeoPixelStrip) checkRange Error: ", error);
	} else {
		for (cycles = 0; cycles < nrCycles; cycles++) {
			error = colorWipe(strip, stripColor, waitMS);

			if (error != 0) {
				displayError("(blinkNeoPixelStrip) colorWipe Error: ", error);
				break;
			} else {
				error = clearPixels(strip, pixelStart, pixelEnd);

				if (error != 0) {
					displayError("(blinkNeoPixelStrip) clearPixels Error: ", error);
					break;
				} else {
					delay(waitMS);
				}
			}
		}
	}

	return error;
}

/*
	Display an error message on the console terminal
*/
void displayError (String message, uint16_t error) {
	Serial.print(message);
	Serial.print(" Error: ");
	Serial.println(error);
}
 
uint16_t colorFades (Adafruit_NeoPixel *strip, uint32_t startColor, uint32_t endColor, uint16_t waitMS) {
	uint16_t error = 0;
	uint16_t inner, outer;

	if (startColor > endColor) {
		error = 1004;
	} else {
		Serial.println("Color Fades");

		for (outer = startColor; outer < endColor; outer++) {
			for (inner = 0; inner < strip->numPixels(); inner++) {
				strip->setPixelColor(inner, wheel(strip, (inner + outer) & 255));
			}

			strip->show();
			delay(waitMS);
		}
	}

	return error;
}

/*
	Knight Rider - Kitt's front scanner
*/
uint16_t knightRider (Adafruit_NeoPixel *strip, uint32_t pixelColor, uint8_t nrCycles, uint16_t waitMS, uint16_t startPixel=0, uint16_t endPixel=0) {
	uint16_t error = 0;
	uint16_t pixelDiff = 2;
	int pixel;
	uint8_t cycles = 0;

	uint16_t pixelStart = startPixel, pixelEnd = endPixel;

	//	Default to all pixels in the strip
	if ((pixelStart == 0) && (pixelEnd == 0)) {
		pixelEnd = strip->numPixels();

		Serial.println("Knight Rider");
		error = clearPixels(strip);

		if (error != 0) {
			displayError("(knightRider) clearPixels", error);
		}
	} else {
		Serial.println("Knight Rider[range]");

		//	Check parameters for range errors
		error = checkRange(strip, pixelStart, pixelEnd, true);

		if (error != 0) {
			displayError("(knightRider[range]) checkRange", error);
		} else {
			error = clearPixels(strip, pixelStart, pixelEnd);

			if (error != 0) {
				displayError("(knightRider[range]) clearPixels", error);
			} else {
				pixelEnd += 1;
			}
		}
	}

	if (error != 0) {
		return error;
	}

	pixel = pixelStart;

	while (cycles < nrCycles) {
		//	Forward sequence
		while (pixel < pixelEnd) {
			//	Turn the pixel ON
			setNeoPixel(strip, pixel, pixelColor);
			delay(waitMS);

			//	Turn the pixel OFF
			setNeoPixel(strip, pixel, 0);
			delay(waitMS);

			pixel += 1;
		}

		pixel -= pixelDiff;

		//	Reverse sequence
		while ((pixel >= 0) && (pixel >= pixelStart)) {
			setNeoPixel(strip, pixel, pixelColor);
			delay(waitMS);

			setNeoPixel(strip, pixel, 0);
			delay(waitMS);

			pixel -= 1;
		}

		pixel += pixelDiff;

		cycles += 1;
	}

	return error;
}

/*
	Input a value 0 to 255 to get a color value.

	The colours are a transition r - g - b - back to r.
*/
uint32_t wheel (Adafruit_NeoPixel *strip, uint8_t position) {
	uint8_t wheelPos = position;
	uint32_t resultColor;

	if (wheelPos < 85) {
		resultColor = strip->Color(wheelPos * 3, 255 - wheelPos * 3, 0);
	} else if (wheelPos < 170) {
		wheelPos -= 85;
		resultColor = strip->Color(255 - wheelPos * 3, 0, wheelPos * 3);
	} else {
		wheelPos -= 170;
		resultColor = strip->Color(0, wheelPos * 3, 255 - wheelPos * 3);
	}

	return resultColor;
}

void setup (void) {
	Serial.begin(115200);
	delay(950);
	Serial.println("NeoPixel Test 2");

	/*
		Eight individual NeoPixels on a breadboard
	*/

	strip8.begin();
	strip8.setBrightness(10);

	//	Initialize all pixels to 'off'
	strip8.show();

	/*
		60 NeoPixels in one flexible strip
	*/

//	strip60.begin();
//	strip60.setBrightness(10);

	//	Initialize all pixels to 'off'
//	strip60.show();
}

void loop (void) {
	uint16_t error = 0;

	uint32_t shadePink = strip8.Color(255, 55, 85);
	uint32_t shadeLime = strip8.Color(125, 200, 0);
	uint32_t shadePurple = strip8.Color(50, 0, 50);
	uint32_t shadeOrange = strip8.Color(220, 135, 0);
	uint32_t shadeMagenta = strip8.Color(195, 0, 125);
	uint32_t shadeYellow = strip8.Color(220, 220, 0);

	uint32_t shadeRed = strip8.Color(255, 0, 0);
	uint32_t shadeGreen = strip8.Color(0, 255, 0);
	uint32_t shadeBlue = strip8.Color(0, 0, 255);

	//	The basic color components
	uint8_t red = 0, green = 0, blue = 0;

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

	//	Do some complete strip8 color wipes
	Serial.println("Color Wipes:");

	Serial.println("     Pink");
	colorWipe(&strip8, shadePink, 2500);

	Serial.println("     Lime");
	colorWipe(&strip8, shadeLime, 2500);

	Serial.println("     Purple");
	colorWipe(&strip8, shadePurple, 2500);

	Serial.println("     Orange");
	colorWipe(&strip8, shadeOrange, 2500);

	Serial.println("     Magenta");
	colorWipe(&strip8, shadeMagenta, 2500);

	Serial.println("     Yellow");
	colorWipe(&strip8, shadeYellow, 2500);

	Serial.println("     Red");
	colorWipe(&strip8, shadeRed, 2500);

	Serial.println("     Green");
	colorWipe(&strip8, shadeGreen, 2500);

	Serial.println("     Blue");
	colorWipe(&strip8, shadeBlue, 2500);
//	colorWipe(&strip60, shadeBlue, 2500);

	delay(500);

	//	Color wipe range - zero based start pixel and end pixel
	error = colorWipe(&strip8, shadeOrange, 2500, 2, 6);

	if (error != 0) {
		displayError("(main) colorWipe[range]", error);
	} else {
		Serial.println("Color Wipe Range");
		Serial.println("     Orange");
	}

	//	Show how each NeoPixel LED is individually addressable
	addressablePixels(&strip8, shadeRed, shadeYellow, shadeGreen, 3500);
	addressablePixels(&strip8, shadeMagenta, shadeOrange, shadeBlue, 3500);
	addressablePixels(&strip8, shadeGreen, shadePurple, shadeYellow, 3500);

	error = clearPixels(&strip8);

	//	That good old Knight Rider Kitt car's scanner
	knightRider(&strip8, shadeRed, 3, 75);
	knightRider(&strip8, shadeGreen, 3, 75);
	knightRider(&strip8, shadeBlue, 3, 75);

	//	Knight Rider - range of pixels (zero based)
	colorWipe(&strip8, shadeMagenta, 1000);
	error = knightRider(&strip8, shadeOrange, 10, 75, 2, 5);
//	colorWipe(&strip60, shadeMagenta, 1000);
//	error = knightRider(&strip60, shadeOrange, 10, 75, 10, 50);

	if (error != 0) {
		displayError("Knight Rider[range]", error);
	}

	//	Blink the entire strip8
	error = blinkNeoPixelStrip(&strip8, 3, shadeBlue, 500);
//	error = blinkNeoPixelStrip(&strip60, 3, shadeBlue, 500);

	if (error != 0) {
		displayError("Blink Strip #1", error);
	}

	error = blinkNeoPixelStrip(&strip8, 3, shadeMagenta, 500);
//	error = blinkNeoPixelStrip(&strip60, 3, shadeMagenta, 500);

	if (error != 0) {
		displayError("Blink Strip #2", error);
	}

	error = blinkNeoPixelStrip(&strip8, 3, shadeLime, 500);
//	error = blinkNeoPixelStrip(&strip60, 3, shadeLime, 500);

	if (error != 0) {
		displayError("Blink Strip #3", error);
	}

	colorFades(&strip8, 150, 375, 100);
//	colorFades(&strip60, 150, 375, 100);

	//	Blink some individual pixels
	error = blinkNeoPixel(&strip8, 0, shadeBlue, 5, 250, true);

	if (error != 0) {
		displayError("blinkNeoPixel #1", error);
	}

	error = blinkNeoPixel(&strip8, 3, shadeOrange, 5, 250);

	if (error != 0) {
		displayError("blinkNeoPixel #2", error);
	}

	error = blinkNeoPixel(&strip8, 4, shadeMagenta, 5, 250);

	if (error != 0) {
		displayError("blinkNeoPixel #3", error);
	}

	error = blinkNeoPixel(&strip8, 7, shadeGreen, 5, 250, true);

	if (error != 0) {
		displayError("blinkNeoPixel #4", error);
	}

	delay(2000);

	//	The End
	clearPixels(&strip8);
//	clearPixels(&strip60);
}
