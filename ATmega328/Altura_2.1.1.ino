/*
 ALTURA MIDI Theremin by Zeppelin Design Labs LLC, 2017
  by Thomas K Wray & Glen van Alkemade. Code inspirations included:
  thereThing
  MiniWI
*/

#include <MIDI.h>
MIDI_CREATE_DEFAULT_INSTANCE();

// Potentiometer layout with their multiplex
// channel number.
//
//                          7
//                        ++o
//
//  3   0   1   2   4   6   5 
// +-+ +-+ +-+ +-+ +-+ +-+ +-+
// |o| |o| |o| |o| |o| |o| |o|
// +-+ +-+ +-+ +-+ +-+ +-+ +-+  

const byte dataFarPot      = 3;
const byte dataNearPot      = 0;
const byte functionSelectPot  = 1;
const byte keyPot       = 2;
const byte scalePot       = 4;
const byte octaveNearPot    = 6;
const byte octaveFarPot     = 5;
const byte articulationPot    = 7; //This channel is a special case and should be left alone.

// Sensor pin definitions -------------------------
const byte rightTriggerPin = 7;
const byte rightEchoPin = 8;
const byte leftTriggerPin = 5;
const byte leftEchoPin = 6;

//Multiplexer -------------------------------------
const byte multiplexChannelSelectMSBPin = 4;
const byte multiplexChannelSelectMiddleBitPin = 3;
const byte multiplexChannelSelectLSBPin = 2;

#define multiplexDataPin A5

const byte totalChannels = 8;

int Pot[totalChannels];

//8-Segment LED Display --------------------------
const byte ledLatch = 14;
const byte ledClock = 15;
const byte ledData = 9;
const byte ledLeftDigitPin = 18;
const byte ledMiddleDigitPin = 17;
const byte ledRightDigitPin = 16;

byte ledLeftDigit = 0;  
byte ledMiddleDigit = 0;
byte ledRightDigit = 0;

int displayTimeout = 2000;
unsigned long currentMillis;
unsigned long shortTimeout;
unsigned long longTimeout;
byte displayPriority = 0;
 
//This defines the appearance of each character that can be displayed on the 8-segment LED displays.
//A generalized solution is under development with more detailed documentation.
const byte displayableCharacters[50] = { 
                                         0b10101111, 0b00101000, 0b10011011, 0b10111010, 0b00111100,
                                         0b10110110, 0b10110111, 0b00101010, 0b10111111, 0b10111110,
                                         0b10000111, 0b11000111, 0b10111001, 0b11111001, 0b10010111,
                                         0b00010111, 0b01010111, 0b10110111, 0b11110111, 0b00111111,
                                         0b01111111, 0b10110101, 0b00000000, 0b11101111, 0b10101111,
                                         0b10101110, 0b10101010, 0b10101000, 0b10100000, 0b10000000,
                                         0b10000001, 0b10000101, 0b10000111, 0b10001111, 0b10101111,
                                         0b11101111, 0b11111110, 0b11111101, 0b11111011, 0b11110111,
                                         0b11101111, 0b11011111, 0b10111111, 0b01111111, 0b11111111,
                                         0b11011011, 0b01101000, 0b00010000, 0b00010101, 0b00111000
                                       };

//Sensor Setup -----------------------------------------                     
const byte leftHandBufferAmmount = 4; //try to keep as a power of 2

const int minimumDistance = 350;
const int maximumDistance = 3000;
const int sensorTimeOut = 4000;
const int noteBuffer = 150; // later divided by numberOfOctavesCurrent

unsigned long leftSensorProcessed=0;
unsigned long rightSensorProcessed=0; 

//Notes Setup -----------------------------------------
byte notesInCurrentScale = 15; 
byte scaleCurrent;
byte keyCurrent;


const byte octaveMax = 8;
int octaveNearCurrent = 5;
int octaveFarCurrent = 4;
int numberOfOctavesCurrent = 2;
bool descending = true;

const byte scales[12][13] = {       //The value in the last column indicates the number of notes in the scale.
                  
  { 2, 2, 1, 2, 2, 2, 1, 0, 0, 0, 0, 0, 7 }, // Ionian Mode (Major)
  { 2, 1, 2, 2, 2, 1, 2, 0, 0, 0, 0, 0, 7 }, // Dorian Mode
  { 1, 2, 2, 2, 1, 2, 2, 0, 0, 0, 0, 0, 7 }, // Phrygian Mode
  { 2, 2, 2, 1, 2, 2, 1, 0, 0, 0, 0, 0, 7 }, // Lydian Mode
  { 2, 2, 1, 2, 2, 1, 2, 0, 0, 0, 0, 0, 7 }, // Mixolydian
  { 2, 1, 2, 2, 1, 2, 2, 0, 0, 0, 0, 0, 7 }, // Aeolian Mode (Natural Minor)
  { 1, 2, 2, 1, 2, 2, 2, 0, 0, 0, 0, 0, 7 }, // Locrian Mode
  { 2, 1, 2, 2, 1, 3, 1, 0, 0, 0, 0, 0, 7 }, // Harmonic Minor
  { 2, 2, 3, 2, 3, 0, 0, 0, 0, 0, 0, 0, 5 }, // Major Pentatonic
  { 3, 2, 2, 3, 2, 0, 0, 0, 0, 0, 0, 0, 5 }, // Minor Pentatonic
  { 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 6 }, // Whole Tone
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 12}, // Chromatic
  };

//Pot Functionality Setup -----------------------------
byte functionSelectCurrent;

int dataFar = 0;
int dataNear = 0;

int dataFarOld  = -1;
int dataNearOld  = -1;

int xyDataFarRight = 0;
int xyDataNearRight = 0;

int xyLeftControlChange = 85;
int xyRightControlChange = 86;

bool xyMode = false;
bool xyLeftInRange;

bool articulationMode = true;

byte fastActionRatio = 1; 

//MIDI Packet Data -------------------------------------
int pitchBendNeutralZone = 10;
int  pitchBendUp = 1700;
int  pitchBendDown = 1700;

bool portamentoOn = false;
byte portamentoTime = 0;

byte noteVelocity = 127;

byte midiChannel = 1;


//volitile handle with care------------------------------------
byte midiNotes[109]; 
///////////////////////////////////////////////
// FUNCTIONS                        //
///////////////////////////////////////////////

void readMultiplex(){
   for (int i = 0; i < totalChannels; i++)
  {
    digitalWrite(multiplexChannelSelectMSBPin, bitRead(i, 0));
    digitalWrite(multiplexChannelSelectMiddleBitPin, bitRead(i, 1));
    digitalWrite(multiplexChannelSelectLSBPin, bitRead(i, 2));
    Pot[i] = analogRead(multiplexDataPin);
  }
}

void SetScale() {
  if(descending){
   
    midiNotes[notesInCurrentScale] = keyCurrent + (octaveFarCurrent * 12);  
    for (int note = notesInCurrentScale; note >= 0; note--) {
    midiNotes[note - 1] = midiNotes[note] + scales[scaleCurrent][(notesInCurrentScale - note) % scales[scaleCurrent][12]]; 
    }
  }
  
  if(!descending){  
  midiNotes[0] = keyCurrent + (octaveNearCurrent * 12);
  for (int note = 0; note <= notesInCurrentScale; note++) {
    midiNotes[note + 1] = midiNotes[note] + scales[scaleCurrent][note % scales[scaleCurrent][12]];
  }
  }
}

void cycleDisplay(int t) {
  for (int i = t; i > 0; i--) {
    lightDigit(displayableCharacters[ledLeftDigit], ledRightDigitPin, ledLeftDigitPin);
    delayMicroseconds(500);
    lightDigit(displayableCharacters[ledMiddleDigit], ledLeftDigitPin, ledMiddleDigitPin);
    delayMicroseconds(500);
    lightDigit(displayableCharacters[ledRightDigit], ledMiddleDigitPin, ledRightDigitPin);
    delayMicroseconds(500);
  }
}

void lightDigit(byte displayNumber, byte digitOff, byte digitOn) {
  digitalWrite(digitOff, LOW);
  digitalWrite(ledLatch, LOW);
  shiftOut(ledData, ledClock, MSBFIRST, displayNumber);
  digitalWrite(ledLatch, HIGH);
  digitalWrite(digitOn, HIGH);
}

void digitSplit2(int number) {
ledMiddleDigit = (number < 10 ? 22 : (number/10) % 10);
ledRightDigit = number % 10;
   }

void digitSplit(int number) {
ledLeftDigit = (number < 100 ? 22 : (number / 100) % 10 );  
digitSplit2(number);
  }  
  
void startTimerWithPriority(byte priority) {
  currentMillis = millis();
  shortTimeout = currentMillis + displayTimeout;
  longTimeout = currentMillis + 60000;
  displayPriority = priority;  //used to prevent certain sections from running until reverting to the default display
 }

bool outsidePotBuffer(int oldValue, int newValue) {
  return (oldValue >= newValue + 3 || oldValue <= newValue - 3 && oldValue >= 0);
}

void startupDisplay(){
   for (int k = 0; k < 3; k++) {
      for (int j = 36; j < 45; j++) {
        ledLeftDigit = j;
        ledMiddleDigit = j;
        ledRightDigit = j;
        cycleDisplay(30);
      }
      rightSensorProcessed = readRightSensor();
    }
}

void displayKeyAndMode(){
  digitSplit2(scaleCurrent + 1);
  ledLeftDigit = keyCurrent + 10;  
}

void checkScalePot(int scalePot){
  static int scaleOld = -1;
    if (Pot[scalePot] != scaleOld) {
    scaleCurrent = map(Pot[scalePot], 0, 1023, 0, 12);
    if (scaleCurrent > 11) {
      scaleCurrent = 11;
    }
    SetScale();               
    if (outsidePotBuffer(scaleOld, Pot[scalePot])&& displayPriority<3) {
      startTimerWithPriority(2);
      displayKeyAndMode();
    }
    scaleOld = Pot[scalePot];
  }
}

void checkKeyPot(int keyPot){
  static int keyOld = -1;
    if (Pot[keyPot] != keyOld) {
    keyCurrent = map(Pot[keyPot], 0, 1023, 0, 12);
    if (keyCurrent > 11) {
      keyCurrent = 11;
    }
    SetScale();   

    if (outsidePotBuffer(keyOld, Pot[keyPot])&& displayPriority<3) {
      startTimerWithPriority(2);
      displayKeyAndMode();
    }
    keyOld = Pot[keyPot];
  }
}

void checkOctavePots (int octaveNearPot, int octaveFarPot){
  static int octaveNearOld = -1;
    static int octaveFarOld = -1;
  static int slopeCurrent = 1;
    static int slopeOld = 1;
  if (Pot[octaveNearPot] != octaveNearOld || Pot[octaveFarPot] != octaveFarOld) {
  if(Pot[octaveNearPot] != octaveNearOld){
    octaveNearCurrent = map(Pot[octaveNearPot], 0, 1023, 1, octaveMax+2);
        if (octaveNearCurrent > octaveMax) {
      octaveNearCurrent = octaveMax;
        }
  }
  if (Pot[octaveFarPot] != octaveFarOld){
    octaveFarCurrent = map(Pot[octaveFarPot], 0, 1023, 1, octaveMax+2);
        if (octaveFarCurrent > octaveMax) {
      octaveFarCurrent = octaveMax;
        }
  }

  slopeCurrent = (octaveFarCurrent + octaveMax) - octaveNearCurrent;
  if (slopeCurrent == octaveMax) {slopeCurrent = slopeOld;}
  if (slopeCurrent < octaveMax) {descending = true;}
  if (slopeCurrent > octaveMax) {descending = false;}

  numberOfOctavesCurrent = max(octaveFarCurrent, octaveNearCurrent) - min(octaveFarCurrent, octaveNearCurrent) + 1;

  notesInCurrentScale = numberOfOctavesCurrent * scales[scaleCurrent][12] ; 
  SetScale();   
    if (outsidePotBuffer(octaveFarOld, Pot[octaveFarPot]) || outsidePotBuffer(octaveNearOld, Pot[octaveNearPot])&& displayPriority<3 ) {
        startTimerWithPriority(2);
          ledLeftDigit = octaveNearCurrent;
          ledRightDigit = octaveFarCurrent;
          ledMiddleDigit = 22;
    }
    octaveNearOld = Pot[octaveNearPot];
    octaveFarOld = Pot[octaveFarPot];
    slopeOld = slopeCurrent;
  }
}

void checkFunctionPot(int functionSelectPot){
  static int functionSelectOld = -11;
   if (Pot[functionSelectPot] != functionSelectOld) {
    functionSelectCurrent = map(Pot[functionSelectPot], 0, 1023, 1, 8);
    if (functionSelectCurrent > 7) {
      functionSelectCurrent = 7;
    }
    if (outsidePotBuffer(functionSelectOld, Pot[functionSelectPot])) {
      startTimerWithPriority(3);
      ledRightDigit = 22;
      ledMiddleDigit = 22;
      ledLeftDigit = functionSelectCurrent;
      
      //reset data to force it to adjust to the new setting
      dataNearOld  = -1;    
      dataFarOld  = -1;
    }
  if (functionSelectCurrent == 6 && !xyMode){
    xyModeStart();
  }
  if (functionSelectCurrent != 6 && xyMode){
    xyModeStop();
  }
    functionSelectOld = Pot[functionSelectPot];
  }
}

void checkDataPots(int dataNearPot, int dataFarPot){
   if (Pot[dataNearPot] != dataNearOld ) {
    switch (functionSelectCurrent)
    {
      case 1:
        pitchBendNeutralZone = map(Pot[dataNearPot], 0, 1023, 0, 127);
        if (outsidePotBuffer(dataNearOld , Pot[dataNearPot])&& displayPriority<3) {
          startTimerWithPriority(2);
          digitSplit(pitchBendNeutralZone);
        }
        pitchBendUp = 1700 + pitchBendNeutralZone * 4;
        pitchBendDown = 1700 - pitchBendNeutralZone * 4;
        break;
    case 7:
      break;
      default:
        dataNear = map(Pot[dataNearPot], 0, 1023, 0, 127);
        if (outsidePotBuffer(dataNearOld , Pot[dataNearPot])&& displayPriority<3) {
          startTimerWithPriority(2);
          digitSplit(dataNear);

        }
    }
    dataNearOld = Pot[dataNearPot];

  }

  if (Pot[dataFarPot] != dataFarOld ) {
    switch (functionSelectCurrent)
    {
      case 1: 
        dataFar = map(Pot[dataFarPot], 0, 1023, 0, 13);
        if (dataFar > 12) {
          dataFar = 12;
        }
        if (outsidePotBuffer(dataFarOld , Pot[dataFarPot])&& displayPriority<3) {
          startTimerWithPriority(2);
          digitSplit(dataFar);
        }
        MIDI.sendControlChange(20, dataFar, midiChannel);
        break;
    case 7:
    if (displayPriority <= 1){
      midiChannel = map(Pot[dataFarPot],0,1023,1,17);
    if (midiChannel > 16) {
          midiChannel = 16;
    }
    
    startTimerWithPriority(1);
          digitSplit(midiChannel);
    }
    break;
      default:
        dataFar = map(Pot[dataFarPot], 0, 1023, 0, 127);
        if (outsidePotBuffer(dataFarOld , Pot[dataFarPot])&& displayPriority<3) {
          startTimerWithPriority(2);
          digitSplit(dataFar);

        }
    }

    dataFarOld  = Pot[dataFarPot];
  }
}

void xyModeStart(){
  xyMode = true;
  displayTimeout = 800;
}

void xyModeStop(){
  xyMode = false;
  displayTimeout = 2000;
}

void xyCheckControlPots(int leftControlPot, int rightControlPot){
  static int leftControlOld;
  static int rightControlOld;
  if (leftControlOld != Pot[leftControlPot]){
    xyLeftControlChange = map(Pot[leftControlPot], 0, 1023, 0, 127);
  if (outsidePotBuffer(leftControlOld, Pot[leftControlPot]) && displayPriority <3){
    startTimerWithPriority(2);
    digitSplit(xyLeftControlChange);
  }
  }
  if (rightControlOld != Pot[rightControlPot]){   
    xyRightControlChange = map(Pot[rightControlPot], 0, 1023, 0, 127);
  if (outsidePotBuffer(rightControlOld, Pot[rightControlPot]) && displayPriority <3){
    startTimerWithPriority(2);
    digitSplit(xyRightControlChange);
  }
  }
  leftControlOld = Pot[leftControlPot];
  rightControlOld = Pot[rightControlPot];
}

void xyCheckRightDataPots(int rightDataNearPot, int rightDataFarPot){
  static int rightDataFarOld = -1;
  static int rightDataNearOld = -1;
  if (rightDataFarOld != Pot[rightDataFarPot]){ 
  xyDataFarRight = map(Pot[rightDataFarPot], 0, 1023, 0, 127);
        if (outsidePotBuffer(rightDataFarOld , Pot[rightDataFarPot])&& displayPriority<3) {
          startTimerWithPriority(2);
          digitSplit(xyDataFarRight);
        }
    }
  if (rightDataNearOld != Pot[rightDataNearPot]){ 
  xyDataNearRight = map(Pot[rightDataNearPot], 0, 1023, 0, 127);
        if (outsidePotBuffer(rightDataNearOld , Pot[rightDataNearPot])&& displayPriority<3) {
          startTimerWithPriority(2);
          digitSplit(xyDataNearRight);
        }
    }
    rightDataFarOld  = Pot[rightDataFarPot];
  rightDataNearOld = Pot[rightDataNearPot];
  }

void checkPots(){
  checkScalePot(scalePot);
  checkKeyPot(keyPot);
  checkOctavePots(octaveNearPot, octaveFarPot);
  checkFunctionPot(functionSelectPot);
  checkDataPots(dataNearPot,dataFarPot);
}

void xyCheckPots(){
  xyCheckControlPots(keyPot, scalePot);
  checkDataPots(dataNearPot, dataFarPot);
  xyCheckRightDataPots(octaveNearPot, octaveFarPot);  
  checkFunctionPot(functionSelectPot);  
}

void defaultDisplay(){
  digitSplit2(scaleCurrent + 1);    
  ledLeftDigit = keyCurrent + 10;  
  displayPriority = 0;
}

void wipeDisplay(){
    ledLeftDigit=22;
    ledMiddleDigit=22;
    ledRightDigit=22;
}

void checkTimeouts(){
    currentMillis = millis();
  if (longTimeout < currentMillis){
    wipeDisplay();
  }
    else if (shortTimeout < currentMillis) {
    defaultDisplay();
  if (xyMode){
    ledLeftDigit = 47;
    ledMiddleDigit = 47;
    ledRightDigit = 47;
  }
 }
}

void checkArticulation(){
  static int fastActionRatioOld;
  if (articulationMode) {
      digitalWrite(multiplexChannelSelectMSBPin, 1);
      digitalWrite(multiplexChannelSelectMiddleBitPin, 1);
      digitalWrite(multiplexChannelSelectLSBPin, 1);

      Pot[articulationPot] = analogRead(multiplexDataPin);

      if (Pot[articulationPot] != fastActionRatioOld) {
        fastActionRatio = map(Pot[articulationPot], 0, 1023, 1, 17); 
        fastActionRatioOld = Pot[articulationPot];        
        digitSplit(fastActionRatio * 15);     
        startTimerWithPriority(3);
      }
    }
}

long sensorConstrain(long reading){
  if (reading == 0){return 0;}
  if (reading <= minimumDistance){return minimumDistance;}
  if (reading >= maximumDistance){return maximumDistance;}
  return reading;
}

long pingSensor(byte trigger, byte echo){
    digitalWrite(trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
    return pulseIn(echo, HIGH, sensorTimeOut);
  
}

long stabilizeLeftReadings(long reading){
  static byte pointer = 0;
  static int leftReadings[leftHandBufferAmmount];
 
 if (reading == 0){
   return reading;
   }
  
  leftReadings[pointer] = reading;
      pointer++;
      if (pointer >= leftHandBufferAmmount) {
        pointer = 0;
      }
      int readingsTotal = 0;
      for (int j = 0; j < leftHandBufferAmmount; j++) {
        readingsTotal = readingsTotal + leftReadings[j];
      }
      return readingsTotal / leftHandBufferAmmount;
}

long stabilizeRightReadings(long reading){
  static byte pointer = 0;
  static int rightReadings[leftHandBufferAmmount];
 
 if (reading == 0){
   return reading;
   }
  
  rightReadings[pointer] = reading;
      pointer++;
      if (pointer >= leftHandBufferAmmount) {
        pointer = 0;
      }
      int readingsTotal = 0;
      for (int j = 0; j < leftHandBufferAmmount; j++) {
        readingsTotal = readingsTotal + rightReadings[j];
      }
      return readingsTotal / leftHandBufferAmmount;
}

void handleVelocity(){
  if (leftSensorProcessed != 0) {
            noteVelocity = map(leftSensorProcessed, minimumDistance, maximumDistance, dataNear, dataFar);

            digitSplit(noteVelocity);
            startTimerWithPriority(1);
          }
}

void handlePitchBend(){
  static int  pitchBendOld = 0;
  static byte OutOfRangeL = 0;
  static byte spinDial = 29;
  int  pitchBend = 0;
  if (portamentoTime != 0) {
            portamentoOn = false;
            portamentoTime = 0;
            MIDI.sendControlChange(5, portamentoTime, midiChannel);
            MIDI.sendControlChange (65, 0, midiChannel);
          }
            if (leftSensorProcessed > pitchBendUp) {
              pitchBend = map(leftSensorProcessed, pitchBendUp, maximumDistance, 0, -1023);
            }
            else if (leftSensorProcessed < pitchBendDown) {
              pitchBend = map(leftSensorProcessed, minimumDistance, pitchBendDown, 1023, 0);
            }
            else
              pitchBend = 0;

            if (leftSensorProcessed == 0) {
              if (OutOfRangeL < 16) {
                OutOfRangeL++;
              }
              if (OutOfRangeL == 15)
              {

                ledRightDigit = 29;
                ledMiddleDigit = 22;
                ledLeftDigit = 22;
                startTimerWithPriority(1);

                MIDI.sendPitchBend( 0, midiChannel);
              }
            }
            else {
              if (pitchBend != pitchBendOld)
              {
                if (pitchBend > 0) {
                  spinDial = map(constrain(pitchBend, 0, 1023), 1023, 0 , 34, 29);
                  if (pitchBend == 1023){
                    spinDial++;
                  }
                }
                else if (pitchBend < 0) {
                  spinDial = map(constrain(pitchBend, -1023, 0), -1023, 0, 24, 29);
                   if (pitchBend == -1023){
                    spinDial--;
                  }
                }
                else spinDial = 29;
                ledRightDigit = spinDial;
                ledMiddleDigit = 22;
                ledLeftDigit = 22;
                startTimerWithPriority(1);

                pitchBendOld = pitchBend;
                pitchBend = pitchBend * 8;
                MIDI.sendPitchBend( pitchBend, midiChannel);

                OutOfRangeL = 0;
              }
            }
}

void handleVolume(){
  static byte channelVolumeOld = 127;
  if (leftSensorProcessed != 0)
          {
            byte channelVolume = map(leftSensorProcessed, minimumDistance, maximumDistance, dataNear, dataFar);
            if (channelVolume != channelVolumeOld)
            {
              digitSplit(channelVolume);
              startTimerWithPriority(1);
              MIDI.sendControlChange(7, channelVolume, midiChannel);
              channelVolumeOld = channelVolume;
            }
          }
}

void handleModulation(){
  static byte modulationOld = 0;
            if (leftSensorProcessed != 0 )
          {byte modulation = map(leftSensorProcessed, minimumDistance, maximumDistance, dataNear, dataFar);
            if (modulation != modulationOld)
            {
              digitSplit(modulation);
              startTimerWithPriority(1);      
              MIDI.sendControlChange(1, modulation, midiChannel);
              modulationOld = modulation;
            }
          }
}

void handlePortamento(){
  static byte portamentoTimeOld = 0;
  if (leftSensorProcessed !=0 )
          {          
            portamentoTime = map(leftSensorProcessed, minimumDistance, maximumDistance, dataNear, dataFar );
            if (portamentoTime != portamentoTimeOld)
            {
              digitSplit(portamentoTime);
              startTimerWithPriority(1);
              MIDI.sendControlChange(5, portamentoTime, midiChannel);
              if (portamentoTime == 0) {
                MIDI.sendControlChange (65, 0, midiChannel);
                 if (portamentoOn){
                  portamentoOn = false;
                 }
              }
                if (!portamentoOn && portamentoTime != 0){
            MIDI.sendControlChange (65, 127, midiChannel);
            portamentoOn=true;
            }
              portamentoTimeOld = portamentoTime;
            }
          }
}

void handleLeftSensor(){
      if (displayPriority < 2) {
      switch (functionSelectCurrent) {
    case 1:
      handlePitchBend();
      break;      
  case 2:
      handleModulation();        
      break;      
    case 3:
      handleVelocity();
      break;        
    case 4:
      handleVolume();
      break;
    case 5:
      handlePortamento();
      break;
    case 6:
      xyHandleLeftSensor();
      }
    }
}

void xyHandleLeftSensor(){
  static byte lastValue = -1;
   if (leftSensorProcessed > 0){
            byte dataLeft = map(leftSensorProcessed, minimumDistance, maximumDistance, dataNear, dataFar);
            if (dataLeft != lastValue && displayPriority < 1){
              digitSplit(dataLeft);
              startTimerWithPriority(0);
           }
               MIDI.sendControlChange(xyLeftControlChange, dataLeft, midiChannel);
              lastValue = dataLeft;
       }
}

void xyHandleRightSensor(){
  static byte lastValue = -1;
  if (displayPriority < 3){
     if (rightSensorProcessed > 0)
          {
            byte dataRight = map(rightSensorProcessed, minimumDistance, maximumDistance, xyDataNearRight, xyDataFarRight);
            if (dataRight != lastValue && displayPriority < 1)
            {
              digitSplit(dataRight);
              startTimerWithPriority(0);
             
            }
             MIDI.sendControlChange(xyRightControlChange, dataRight, midiChannel);
              lastValue = dataRight;
            if (leftSensorProcessed > 0 && rightSensorProcessed > 0){
              displayTimeout = 200;
      ledLeftDigit = 48;
      ledMiddleDigit = 47;
      ledRightDigit = 49;
      startTimerWithPriority(1);
      displayTimeout = 800;
      }
    }  
  }
}

void handleRightSensor(long sensorReading){
  static bool notePlaying = false;
  static byte currentNote;
  static byte oldNote;
  static byte OutOfRange = 0;
  if (sensorReading == 0)   
  {
    if (OutOfRange <= 5)
    {
      OutOfRange++;
    }
    if (OutOfRange > 5 && notePlaying == true)
    {
      MIDI.sendNoteOn(oldNote, 0, midiChannel);
      OutOfRange = 0;
      notePlaying = false;     

    }
  }
  else
  {
      OutOfRange = 0;
      currentMillis = millis();
      longTimeout = currentMillis + 60000;
  byte newNote = map(sensorReading, minimumDistance, maximumDistance, 0, notesInCurrentScale + 1);
if (newNote > notesInCurrentScale){newNote=notesInCurrentScale;}
    
    currentNote = midiNotes[newNote];

    if (notePlaying == false)
    {
      notePlaying = true;
      MIDI.sendNoteOn(currentNote, noteVelocity, midiChannel);
      oldNote = currentNote;
    }
    else
    {
      if (currentNote != oldNote)
      {
        MIDI.sendNoteOn(oldNote, 0, midiChannel);
        MIDI.sendNoteOn(currentNote, noteVelocity, midiChannel);
        notePlaying = true;
        oldNote = currentNote;
      }
    }
  }
}

long checkNoteBuffer(long reading){
   if (rightSensorProcessed > (reading + noteBuffer) || rightSensorProcessed < (reading - noteBuffer)) 
  {
    rightSensorProcessed = reading; 
  }
  return rightSensorProcessed;
}

long readRightSensor(){
  unsigned long rightSensorRaw = pingSensor(rightTriggerPin, rightEchoPin);  
  rightSensorRaw = sensorConstrain(rightSensorRaw);
    
  return checkNoteBuffer(rightSensorRaw);
}

long readLeftSensor(){
    return sensorConstrain(pingSensor(leftTriggerPin, leftEchoPin)); 
}

void initializeArticulation(){
  for (int i = 0; i < 7; i++) 
  {
    if (Pot[i] >= 10) {
      articulationMode = false;
    }
  }
  fastActionRatio = map(Pot[articulationPot], 0, 1023, 1, 17);  
}

void runFastActions(){
  for (byte i = fastActionRatio; i > 0; i--) {
  checkArticulation();     
  cycleDisplay(10);  //this,combined with the above "for loop", is the main driving point for timing 
  leftSensorProcessed = stabilizeLeftReadings(readLeftSensor());
  handleLeftSensor();
  
  if (xyMode){
    rightSensorProcessed = stabilizeRightReadings(readRightSensor());
    xyHandleRightSensor();
    xyCheckPots();
  }
}
}

void runSlowActions(){
  readMultiplex();  
  checkTimeouts();  
  if (!xyMode){
  checkPots();  
  handleRightSensor(readRightSensor());
  }  
}


///////////////////////////////////////////////
// VOID SETUP             //
///////////////////////////////////////////////

void setup() {

  Serial.begin(31250); // MIDI Begin

  pinMode(rightTriggerPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  pinMode(leftTriggerPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);

  pinMode(multiplexChannelSelectMSBPin, OUTPUT);
  pinMode(multiplexChannelSelectMiddleBitPin, OUTPUT);
  pinMode(multiplexChannelSelectLSBPin, OUTPUT);
  pinMode(multiplexDataPin, INPUT);

  pinMode(ledLatch, OUTPUT);
  pinMode(ledClock, OUTPUT);
  pinMode(ledData, OUTPUT);
  pinMode(ledLeftDigitPin, OUTPUT);
  pinMode(ledMiddleDigitPin, OUTPUT);
  pinMode(ledRightDigitPin, OUTPUT);

//Clear MIDI buffer upon startup 
  byte startupBuffer[2]= {0,0};

MIDI.sendSysEx(2,startupBuffer,false);
MIDI.sendProgramChange(81,midiChannel); //typically a square lead, appropriate to a theremin

 
  readMultiplex();
  initializeArticulation();
  if (articulationMode) { //display the software version (2.1.0)
    ledLeftDigit = 45;
    ledMiddleDigit = 46;
    ledRightDigit = 1;
    cycleDisplay(1000);
  }
  else {
   startupDisplay();
  }
  startTimerWithPriority(2);
}

///////////////////////////////////////////////////
// BEGIN VOID LOOP              //
///////////////////////////////////////////////////

void loop()
{  
  runSlowActions();
  runFastActions(); 
}



