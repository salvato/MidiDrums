/*
 * Author: Gabriele Salvato
 * derived from the original work by:
 * 
 * Copyright (c) 2015 Evan Kale
 * Email: EvanKale91@gmail.com
 * Website: www.ISeeDeadPixel.com
 *          www.evankale.blogspot.ca
 *
 *
 * Modifyed following 
 * MIDIUSB_test.ino
 *
 * Created: 4/6/2015 10:47:08 AM
 * Author: gurbrinder grewal
 * Modified by Arduino LLC (2015)
 * This file is part of ArduinoMidiDrums.
 *
 * ArduinoMidiDrums is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "MIDIUSB.h"

byte note;
byte chan;
byte patch;
midiEventPacket_t event;

//Piezo defines
#define NUM_PIEZOS 6
#define SNARE_THRESHOLD 200    //anything < TRIGGER_THRESHOLD is treated as 0
#define LTOM_THRESHOLD  200
#define RTOM_THRESHOLD  200
#define LCYM_THRESHOLD  40
#define RCYM_THRESHOLD  60
#define KICK_THRESHOLD  500
#define START_ADC_SLOT  0     //first analog slot of piezos
#define START_DIG_SLOT  22    //first digital slot of buttons


//MIDI note defines for each trigger
#define SNARE_NOTE 38 // Rullante
#define LTOM_NOTE  43 // Tom
#define RTOM_NOTE  45 // Floor Tom
#define LCYM_NOTE  42 // Charlie Chiuso
#define RCYM_NOTE  51 // Piatto
#define KICK_NOTE  36 // Cassa


//MIDI defines
#define DRUM_CHAN 9
#define MAX_MIDI_VELOCITY 127


//Program defines 
#define SIGNAL_BUFFER_SIZE 100
#define PEAK_BUFFER_SIZE 30

//ALL TIME MEASURED IN MILLISECONDS
#define MAX_TIME_BETWEEN_PEAKS 20
#define MIN_TIME_BETWEEN_NOTES 50


//map that holds the mux slots of the piezos
unsigned short slotMap[NUM_PIEZOS];
//map that holds the respective note to each piezo
unsigned short noteMap[NUM_PIEZOS];
//map that holds the respective threshold to each piezo
unsigned short thresholdMap[NUM_PIEZOS];
//map that holds the status of the digital inputs
unsigned short digitalPinMap[NUM_PIEZOS];


//Ring buffers to store analog signal and peaks
short currentSignalIndex[NUM_PIEZOS];
short currentPeakIndex[NUM_PIEZOS];
unsigned short signalBuffer[NUM_PIEZOS][SIGNAL_BUFFER_SIZE];
unsigned short peakBuffer[NUM_PIEZOS][PEAK_BUFFER_SIZE];


boolean noteReady[NUM_PIEZOS];
unsigned short noteReadyVelocity[NUM_PIEZOS];
boolean isLastPeakZeroed[NUM_PIEZOS];


unsigned long lastPeakTime[NUM_PIEZOS];
unsigned long lastNoteTime[NUM_PIEZOS];


// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).
void 
noteOn(byte channel, byte pitch, byte velocity) {
  event = {0x09, byte(0x90 | channel), pitch, velocity};
  MidiUSB.sendMIDI(event);
}


void 
noteOff(byte channel, byte pitch, byte velocity) {
  event = {0x08, byte(0x80 | channel), pitch, velocity};
  MidiUSB.sendMIDI(event);
}


// First parameter is the event type (0x0B = control change).
// Second parameter is the event type, combined with the channel.
// Third parameter is the control number number (0-119).
// Fourth parameter is the control value (0-127).
void 
controlChange(byte channel, byte control, byte value) {
  event = {0x0B, byte(0xB0 | channel), control, value};
  MidiUSB.sendMIDI(event);
}


void 
programChange(byte channel, byte program) {
  event = {0x0C, byte(0xC0 | channel), program, 0};
  MidiUSB.sendMIDI(event);
}


void 
noteFire(byte channel, unsigned short note, unsigned short velocity) {
  if(velocity > MAX_MIDI_VELOCITY)
    velocity = MAX_MIDI_VELOCITY;
  
  noteOn(channel, note, velocity);
  MidiUSB.flush();
  noteOff(channel, note, velocity);
  MidiUSB.flush();
}


void 
recordNewPeak(short slot, short newPeak) {
  isLastPeakZeroed[slot] = (newPeak == 0);
  
  unsigned long currentTime = millis();
  lastPeakTime[slot] = currentTime;
  
  //new peak recorded (newPeak)
  peakBuffer[slot][currentPeakIndex[slot]] = newPeak;
  
  //1 of 3 cases can happen:
  // 1) note ready - if new peak >= previous peak
  // 2) note fire - if new peak < previous peak and previous peak was a note ready
  // 3) no note - if new peak < previous peak and previous peak was NOT note ready
  
  //get previous peak
  short prevPeakIndex = currentPeakIndex[slot]-1;
  if(prevPeakIndex < 0) prevPeakIndex = PEAK_BUFFER_SIZE-1;        
  unsigned short prevPeak = peakBuffer[slot][prevPeakIndex];
   
  if(newPeak > prevPeak && (currentTime - lastNoteTime[slot])>MIN_TIME_BETWEEN_NOTES) {
    noteReady[slot] = true;
    if(newPeak > noteReadyVelocity[slot])
      noteReadyVelocity[slot] = newPeak;
  }
  else if(newPeak < prevPeak && noteReady[slot]) {
    noteFire(chan, noteMap[slot], noteReadyVelocity[slot]);
    noteReady[slot] = false;
    noteReadyVelocity[slot] = 0;
    lastNoteTime[slot] = currentTime;
  }
  
  currentPeakIndex[slot]++;
  if(currentPeakIndex[slot] == PEAK_BUFFER_SIZE) currentPeakIndex[slot] = 0;  
}


void 
setup() {
  //initialize globals
  chan = DRUM_CHAN;
  patch = 0;
  programChange(chan, patch);
  for(short i=0; i<NUM_PIEZOS; ++i) {
    pinMode(START_DIG_SLOT+i, INPUT_PULLUP);
    digitalPinMap[i] = digitalRead(START_DIG_SLOT+i);
    currentSignalIndex[i] = 0;
    currentPeakIndex[i] = 0;
    memset(signalBuffer[i],0,sizeof(signalBuffer[i]));
    memset(peakBuffer[i],0,sizeof(peakBuffer[i]));
    noteReady[i] = false;
    noteReadyVelocity[i] = 0;
    isLastPeakZeroed[i] = true;
    lastPeakTime[i] = 0;
    lastNoteTime[i] = 0;    
    slotMap[i] = START_ADC_SLOT + i;
  }
  
  thresholdMap[0] = KICK_THRESHOLD;
  thresholdMap[1] = RTOM_THRESHOLD;
  thresholdMap[2] = RCYM_THRESHOLD;
  thresholdMap[3] = LCYM_THRESHOLD;
  thresholdMap[4] = SNARE_THRESHOLD;
  thresholdMap[5] = LTOM_THRESHOLD;  
  /*
     #define SNARE_NOTE 38 // Rullante
     #define LTOM_NOTE  43 // Tom
     #define RTOM_NOTE  45 // Floor Tom
     #define LCYM_NOTE  42 // Charlie Chiuso
     #define RCYM_NOTE  51 // Piatto
     #define KICK_NOTE  36 // Cassa
  */
  noteMap[0] = KICK_NOTE;
  noteMap[1] = RTOM_NOTE;
  noteMap[2] = RCYM_NOTE;
  noteMap[3] = LCYM_NOTE;
  noteMap[4] = SNARE_NOTE;
  noteMap[5] = LTOM_NOTE;  
}


void 
loop() {
  unsigned long currentTime = millis();
  
  for(short i=0; i<NUM_PIEZOS; ++i) {
    //get a new signal from analog read
    unsigned short newSignal = analogRead(slotMap[i]);
    signalBuffer[i][currentSignalIndex[i]] = newSignal;
    
    //if new signal is 0
    if(newSignal < thresholdMap[i]) {
      if(!isLastPeakZeroed[i] && (currentTime - lastPeakTime[i]) > MAX_TIME_BETWEEN_PEAKS) {
        recordNewPeak(i, 0);
      }
      else {
        //get previous signal
        short prevSignalIndex = currentSignalIndex[i]-1;
        if(prevSignalIndex < 0) prevSignalIndex = SIGNAL_BUFFER_SIZE-1;        
        unsigned short prevSignal = signalBuffer[i][prevSignalIndex];
        unsigned short newPeak = 0;
        //find the wave peak if previous signal was not 0 by going
        //through previous signal values until another 0 is reached
        while(prevSignal >= thresholdMap[i]) {
          if(signalBuffer[i][prevSignalIndex] > newPeak) {
            newPeak = signalBuffer[i][prevSignalIndex];        
          }
          //decrement previous signal index, and get previous signal
          prevSignalIndex--;
          if(prevSignalIndex < 0) prevSignalIndex = SIGNAL_BUFFER_SIZE-1;
          prevSignal = signalBuffer[i][prevSignalIndex];
        }
        if(newPeak > 0) {
          recordNewPeak(i, newPeak);
        }
      }
    }
    currentSignalIndex[i]++;
    if(currentSignalIndex[i] == SIGNAL_BUFFER_SIZE) currentSignalIndex[i] = 0;
    
    if(digitalRead(START_DIG_SLOT+i) != digitalPinMap[i]) {
      if(digitalPinMap[i] == LOW) {
        digitalPinMap[i] = HIGH;
        noteOff(chan, noteMap[i], 127);
        MidiUSB.flush();
      }
      else {
        digitalPinMap[i] = LOW;
        noteOn(chan, noteMap[i], 127);
        MidiUSB.flush();
      }
    }
  }
}


