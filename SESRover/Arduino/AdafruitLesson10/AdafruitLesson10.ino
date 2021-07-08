/*
  Adafruit Arduino - Lesson 10. Simple Sounds
*/

#include <toneAC.h>
     
#include "AdafruitLesson10.h"
#include "Pitches.h"

/*

R2D2

beep(speakerPin, note_A7,100); //A
beep(speakerPin, note_G7,100); //G
beep(speakerPin, note_E7,100); //E
beep(speakerPin, note_C7,100); //C
beep(speakerPin, note_D7,100); //D
beep(speakerPin, note_B7,100); //B
beep(speakerPin, note_F7,100); //F
beep(speakerPin, note_C8,100); //C
beep(speakerPin, note_A7,100); //A
beep(speakerPin, note_G7,100); //G
beep(speakerPin, note_E7,100); //E
beep(speakerPin, note_C7,100); //C
beep(speakerPin, note_D7,100); //D
beep(speakerPin, note_B7,100); //B
beep(speakerPin, note_F7,100); //F
beep(speakerPin, note_C8,100); //C
*/

int r2d2[] = { 16, NOTE_A7, NOTE_G7, NOTE_E7, NOTE_C7, NOTE_D7, NOTE_B7, NOTE_F7, NOTE_C8, NOTE_A7, NOTE_G7, NOTE_E7, NOTE_C7, NOTE_D7, NOTE_B7, NOTE_F7, NOTE_C8 };
int tones[] = { 20, NOTE_C4, NOTE_CS4, NOTE_D4, NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_GS4, NOTE_A4, NOTE_C5, NOTE_CS5, NOTE_D5, NOTE_DS5, NOTE_F5, NOTE_FS5, NOTE_G5, NOTE_A5, NOTE_C6, NOTE_CS6, NOTE_D6, NOTE_DS6, NOTE_F6, NOTE_FS6, NOTE_G6, NOTE_A6, NOTE_C7, NOTE_CS7, NOTE_D7, NOTE_DS7, NOTE_F7, NOTE_FS7, NOTE_G7, NOTE_A7 };
          // mid C C# D D# E F F# G G# A

void playTone (unsigned long freqHz, uint8_t volume, unsigned long durationMS) {
	toneAC(freqHz, volume, durationMS);
	noToneAC();
}

void playSequence (int song[], unsigned long durationMS) {
  int numTones = song[0];
  int toneNr;

  for (toneNr = 1; toneNr < numTones; toneNr++) {
    playTone(song[toneNr], 10, durationMS);
  }
}

void setup() {
  playSequence(r2d2, 100);
  delay(2000);
  playSequence(tones, 100);
}
     
void loop() {
}
