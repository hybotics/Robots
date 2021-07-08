/*
	Program:      	SES Rover, Pitches header file, for sound generation routines
	Date:         	17-Feb-2014
	Version:      	1.0.0

	Platform:		Lynxmotion's BotBoarduino (Arduino),
						with Lynxmotion's SSC-32 Servo Controller,
						and a 3DOF (Raise/Lower, Wrist Rotate, Open/Close) Little Grip gripper

	Purpose:		To experiment with various sensor configurations, tracking objects (heat or
						color based), course following, manipulation of the environment, and to
						test code that will later be used on W.A.L.T.E.R. 2.0.

	Dependencies:	Adafruit libraries:
						LSM303DLHC, L3GD20, TMP006, TCS34727, RTClib for the DS1307

					Hybotics libraries:
						BMP180 (modified from Adafruit's BMP085 library)

	Comments:		Credit is given, where applicable, for code I did not originate.

					Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/
#ifndef _PITCHES_H_
#define _PITCHES_H_

/************************************************************/
/*	Public Constants										*/
/************************************************************/

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

#define NAME_B0  "B0"
#define NAME_C1  "C1"
#define NAME_CS1 "C#1"
#define NAME_D1  "D1"
#define NAME_DS1 "D#1"
#define NAME_E1  "E1"
#define NAME_F1  "F1"
#define NAME_FS1 "F#1"
#define NAME_G1  "G1"
#define NAME_GS1 "G#1"
#define NAME_A1  "A1"
#define NAME_AS1 "A#1"
#define NAME_B1  "B1"
#define NAME_C2  "C2"
#define NAME_CS2 "C#2"
#define NAME_D2  "D2"
#define NAME_DS2 "D#2"
#define NAME_E2  "E2"
#define NAME_F2  "F2"
#define NAME_FS2 "F#2"
#define NAME_G2  "G2"
#define NAME_GS2 "G#2"
#define NAME_A2  "A2"
#define NAME_AS2 "A#2"
#define NAME_B2  "B2"
#define NAME_C3  "C3"
#define NAME_CS3 "C#3"
#define NAME_D3  "D3"
#define NAME_DS3 "D#3"
#define NAME_E3  "E3"
#define NAME_F3  "F3"
#define NAME_FS3 "F#3"
#define NAME_G3  "G3"
#define NAME_GS3 "G#3"
#define NAME_A3  "A3"
#define NAME_AS3 "A#3"
#define NAME_B3  "B3"
#define NAME_C4  "C4"
#define NAME_CS4 "C#4"
#define NAME_D4  "D4"
#define NAME_DS4 "D#4"
#define NAME_E4  "E4"
#define NAME_F4  "F4"
#define NAME_FS4 "F#4"
#define NAME_G4  "G4"
#define NAME_GS4 "G#4"
#define NAME_A4  "A4"
#define NAME_AS4 "A#4"
#define NAME_B4  "B4"
#define NAME_C5  "C5"
#define NAME_CS5 "C#5"
#define NAME_D5  "D5"
#define NAME_DS5 "D#5"
#define NAME_E5  "E5"
#define NAME_F5  "F5"
#define NAME_FS5 "F#5"
#define NAME_G5  "G5"
#define NAME_GS5 "G#5"
#define NAME_A5  "A5"
#define NAME_AS5 "A#5"
#define NAME_B5  "B5"
#define NAME_C6  "C6"
#define NAME_CS6 "C#6"
#define NAME_D6  "D6"
#define NAME_DS6 "D#6"
#define NAME_E6  "E6"
#define NAME_F6  "F6"
#define NAME_FS6 "F#6"
#define NAME_G6  "G6"
#define NAME_GS6 "G#6"
#define NAME_A6  "A6"
#define NAME_AS6 "A#6"
#define NAME_B6  "B6"
#define NAME_C7  "C7"
#define NAME_CS7 "C#7"
#define NAME_D7  "D7"
#define NAME_DS7 "D#7"
#define NAME_E7  "E7"
#define NAME_F7  "F7"
#define NAME_FS7 "F#7"
#define NAME_G7  "G7"
#define NAME_GS7 "G#7"
#define NAME_A7  "A7"
#define NAME_AS7 "A#7"
#define NAME_B7  "B7"
#define NAME_C8  "C8"
#define NAME_CS8 "C#8"
#define NAME_D8  "D8"
#define NAME_DS8 "D#8"

#endif