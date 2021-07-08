/*
	Program:   SES Rover, PiezoSound sketch
	Date:      26-Apr-2014
	Version:   0.0.2 ALPHA

	Platform:	Arduino Duemilanove.

	Purpose:	To experiment with sound generation and configurations,
					using piezo buzzers.

	Comments:	Credit is given, where applicable, for code I did not originate.

				Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/

#include <toneAC.h>

#include <BMSerial.h>

#include "PiezoSound.h"
#include "Pitches.h"

/*
	NOTE: We MUST use pins 11 and 12 for the Piezo buzzer,
		because we are using the toneAC library to generate
		tones.
*/

/************************************************************/
/*	Initialize global variables								*/
/************************************************************/

//	This will always have the name of the last routine executed before an error
String lastRoutine;

/*
	Setup all our serial devices
*/

//	Hardware Serial0: Console and debug (replaces Serial.* routines)
BMSerial console(SERIAL_CONSOLE_RX_PIN, SERIAL_CONSOLE_TX_PIN);

//	Hardware Serial1: Lynxmotion SSC-32 Servo Controller
BMSerial ssc32(SERIAL_SSC32_RX_PIN, SERIAL_SSC32_TX_PIN);

//	Hardware Serial2: XBee Mesh Wireless
BMSerial xbee(SERIAL_XBEE_RX_PIN, SERIAL_XBEE_TX_PIN);

/************************************************************/
/*	Utility routines										*/
/************************************************************/

/*
    Process error conditions
*/
void processError (byte errCode, String errMsg) {
	console.print(F("Error in routine '"));
	console.print(lastRoutine);
	console.print(F("', Code: "));
	console.print(errCode);
	console.print(F(", Message: "));
	console.print(errMsg);
	console.println(F("!"));
}

/************************************************************/
/*	Sound Generation routines								*/
/************************************************************/

/*
    Play melodies stored in an array, it requires you to know
      about timing issues and about how to play tones.

    The calculation of the tones is made following the mathematical
      operation:

      timeHigh = 1/(2 * toneFrequency) = period / 2

      where the different tones are described as in the table:

      note          frequency       period  PW (timeHigh)  
      ----          ---------       ------  -------------
      c             261 Hz          3830    1915    
      d             294 Hz          3400    1700    
      e             329 Hz          3038    1519    
      f             349 Hz          2864    1432    
      g             392 Hz          2550    1275    
      a             440 Hz          2272    1136    
      b             493 Hz          2028    1014   
      C             523 Hz          1912    956

      (cleft) 2005 D. Cuartielles for K3

	The original code, this routine is based on, was found at:
		http://www.arduino.cc/en/Tutorial/PlayMelody
*/
void playMelody(byte melody[]) {
  byte toneName[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };  
  int toneHigh[] = { 1915, 1700, 1519, 1432, 1275, 1136, 1014, 956 };
  int toneFreq[] = { 261, 294, 329, 349, 392, 440, 493, 523 };

  int count = 0;
  int count2 = 0;
  int count3 = 0;

  int statePin = LOW;
  int index = 0;

  analogWrite(SPEAKER_OUT, 0);    

  for (count = 0; count < MAX_COUNT; count++) {
    statePin = !statePin;

    digitalWrite(HEARTBEAT_LED, statePin);

    index = count * 2 + 1;

    for (count3 = 0; count3 <= (melody[index] - 48) * 30; count3++) {
      for (count2 = 0; count2 < 8; count2++) {

        if (toneName[index] == melody[index]) {      
          playTone(toneFreq[index], 100);
//          analogWrite(SPEAKER_OUT, 500);
//          delayMicroseconds(toneHigh[index]);
//          analogWrite(SPEAKER_OUT, 0);
//          delayMicroseconds(toneHigh[index]);
        } else if (melody[index] == 'p') {
          //  Make a pause of a certain size
          analogWrite(SPEAKER_OUT, 0);
          delayMicroseconds(500);
        }
      }
    }
  }
}

/*
	Play a single tone on a Piezo buzzer
*/
void playTone (unsigned long freqHz, unsigned long durationMS) {
	toneAC(freqHz, 10, durationMS);
	noToneAC();
}

/*
	Play a sound - a sequence of pitches (or notes)
*/
uint16_t makeSound (uint8_t soundNr, uint8_t nrTimes, uint16_t durationMS) {
	uint16_t errorStatus = 0;
	uint8_t pitch = NOTE_C7;
	uint8_t volume = 100;
	uint8_t lengthMS;
	uint8_t count = 0;

	for (count = 0; count < nrTimes; count++) {
		switch (soundNr) {
			case 1:
				playTone(pitch, lengthMS);
				delay(150);
				playTone(pitch, lengthMS);
				break;

			default:
				errorStatus = 901;
				processError(errorStatus, F("Invalid sound number"));
				break;
		}

		delay(durationMS);
	}

	return errorStatus;
}

/*
	Sound an alarm when we need assistance
*/
uint16_t soundAlarm (uint8_t count) {
	uint16_t errorStatus = 0;
	uint8_t alarmCount;

	for (alarmCount = 0; alarmCount < count; alarmCount++) {
		makeSound(1, count, 250);
	}

	return errorStatus;
}

/*
	Call for help!

	We're in a situation we can't get out of on our own.
*/
uint16_t callForHelp (void) {
	uint16_t errorStatus = 0;
	uint8_t count;

	//	Send out a call for help every 20 seconds
	for (count = 0; count < 20; count++) {
		console.println(F("Help, help, help! I am stuck!"));
		xbee.println(F("Help, help, help! I am stuck!"));

		soundAlarm(5);

		//	20 second delay between calls for help
		delay(20000);
	}

	return errorStatus;
}

void setup (void) {
	byte song[] = "2d2a1f2c2d2a2d2c2f2d2a2c2d2a1f2c2d2a2a2g2p8p8p8p";

	console.begin(115200);

	console.print(F("PiezoSound, version "));
	console.print(BUILD_VERSION);
	console.print(F(" on "));
	console.println(BUILD_DATE);
	console.print(F("     for the "));
	console.print(BUILD_BOARD);
	console.println(F("."));
	console.print("");

	pinMode(HEARTBEAT_LED, OUTPUT);
	digitalWrite(HEARTBEAT_LED, LOW);

	pinMode(SPEAKER_OUT, OUTPUT);
	digitalWrite(SPEAKER_OUT, LOW);

	playMelody(song);
}

void loop (void) {

}
