/*
 *
 * Stjärnhimmel projekt till bibliotek
 *
 * TODO:  1. Kod för knappar
 *        2. Kod för lampor
 *
 * Frågor 1. Beräkna strömförsörjning... (är 18W agg tillräckligt?) Kan behöva köpa 36w agg...
 *        2. Hur många stjärnbilder? 15-16 bilder
 *        3. Hur många knappar? 12+
 *        4. Hur många meter kabel?
 *        5. Hur stora motstånd? (ström...) 470R
 *        6. Led-hållare? (ska de borras hål? hur stora hål?) nej!
 * 
 *  INFO OM KOD-struktur:
 *  1. Använder seriekopplade shift-registers. Totalt 13st (x8 lampor = 104 lampor!). Grupperade i grupper om 6-3-4 IC:ar (som delar latch, klocka och data, dvs 9 pins används för detta). Grupperna har jag döpt till "stora"/"large" (6), "lilla"/"small" (3) och "mellan"/"middle" (4) dvs alla variabler har dessa prefix!
 * 
 *
 */
#include <Arduino.h>
#define DEBOUNCE_TIME 200
#define BLINKING_TIME 5000
#define BLINK_PROBABILITY 10 // 1/10=10% chans för blink. 2=>1/2=50%. 20=>1/20=5%...

#define largeLatchPin 2
#define largeClockPin 3
#define largeDataPin 4

#define smallLatchPin 5
#define smallClockPin 6
#define smallDataPin 7

#define middleLatchPin 8
#define middleClockPin 9
#define middleDataPin 10

#define amtLargeSigns 7 
#define amtSmallSigns 4 
#define amtMiddleSigns 4 

const uint8_t largeOrder[6] = {24, 16, 8, 32, 0, 40};
const uint8_t middleOrder[4] = {24, 16, 8, 0};
const uint8_t smallOrder[3] = {0, 8, 16};

const uint16_t shiftRegisterPins[9] = {largeLatchPin, largeClockPin, largeDataPin, smallLatchPin, smallClockPin, smallDataPin, middleLatchPin, middleClockPin, middleDataPin};

const uint16_t switchPins[16] = {A0, A1, A2,  A3,  A4,  A5,  A6,  A7,
                                 A8, A9, A10, A11, A12, A13, A14, A15};

uint16_t state[16] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};
uint16_t lastDebounce[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint64_t largeOutput = 0, smallOutput = 0, middleOutput = 0;

//bool waitWhileBlinking = false;

char *nameOfSign = "";

struct StarSign {
  char *name;
  int noLamps, noShifts;
  bool turnedOn = false;

  uint64_t bitMask = 0;
  uint16_t switchPin;

  unsigned long blinkTime = 0;
  StarSign(char *name_, int noLamps_, uint16_t switchPin_) : name(name_), noLamps(noLamps_), switchPin(switchPin_) {}
};

/**
 * De första stjärntecknen, som tillhör den "stora" gruppen. Är totalt 48 lampor (dvs 6register*8=48)
 */
StarSign largeSigns[amtLargeSigns] = {
    {"Björnväktaren", 7, A0}, {"Pegasus", 8, A1}, {"Kräftan", 4, A2}, {"Cassiopeia", 5, A3}, //vänster
    {"Stora Björn", 7, A4}, {"Oxen", 6, A5}, {"Jungfrun", 11, A6} //höger
};

/**
 * De andra stjärntecknen, som tillhör den "lilla" gruppen. Är totalt 24 lampor (dvs 3register*8=24)
 */
StarSign smallSigns[amtSmallSigns] = {
  /*{"Draken", 8, A7},*/ {"Fiskarna", 7, A7}, {"Cepheus", 5, A8}, /*{"Perseus", 6, A10}*/ {"Lilla Björn", 6, A9}, {"Kusken", 5, A10}
};

/**
 * De tredje stjärntecknen, som tillhör "mellan" gruppen. Är totalt 32 lampor (dvs 4register*8=32)
 */
StarSign middleSigns[amtMiddleSigns] = {
   {"Väduren", 4, A11}, {"Lejonet", 10, A12},/*{"Orion", 5, A13},*/ {"Tvillingarna", 10, A13},  {"Orion", 8, A14} //just nu totalt 41 lampor... (behöver kanske en grupp till...) , TODO 17 knappar??
};

/**
 * Skapar bitmasks som används för att shifta ut ett visst tecken
 */
void createBitmasks(int amtSigns, StarSign signs[]){
  int countSign = 0;
  for (int i = 0; i < amtSigns; i++) {
    signs[i].noShifts = countSign;
    signs[i].bitMask = (1 << (signs[i].noLamps)) - 1; //denna rad skapar en sträng av ettor lika lång som antalet lampor. T.ex. 4 lampor=> 0b00001 << 4 steg => 0b10000 - 1 = 0b1111
    countSign += signs[i].noLamps;
  }
}

void writeOutput(int latchPin, int clockPin, int dataPin, uint64_t output, uint8_t outputOrder[], int amtRegisters){
  digitalWrite(latchPin, LOW);

  for (size_t i = 0; i < amtRegisters; i++)
    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output >> outputOrder[i])); 

  // shift out the bits
  digitalWrite(latchPin, HIGH);
}

void updateOutput(uint64_t& output, StarSign sign, bool blink){
  uint64_t mask = sign.bitMask;
  if(blink)
  {
    uint64_t randomMask = 0;
    for(int signIndex = 0; signIndex < sign.noLamps; signIndex++)//nödvändigt för att styra sannolikhet
      if(random(0,BLINK_PROBABILITY)==1)//10% chans att lampa blinkar
        randomMask |= (1 << signIndex);
    //mask &= random(sign.bitMask+1);
    mask &= randomMask;
  }
  output |= mask<<sign.noShifts; // blinking...
  //Serial.println((uint16_t)output);
  //Serial.println((uint16_t)(output>>16));
  //Serial.println((uint16_t)(output>>32));
  //Serial.println((uint16_t)(output>>48));
}

void clearOutput(uint64_t& output, StarSign sign){
  output &= ~(sign.bitMask<<sign.noShifts); // Clear...
}

void setup() {

  Serial.begin(115200);

  // Sätt alla knapp-pins till input_pullup
  for (int i = 0; i<16; i++) {
    pinMode(switchPins[i], INPUT_PULLUP);
    state[i] = digitalRead(switchPins[i]);
  }

  // Sätt alla shift register-pins till output
  for (auto pin : shiftRegisterPins){
    pinMode(pin, OUTPUT);
  }

  createBitmasks(amtLargeSigns, largeSigns);
  createBitmasks(amtSmallSigns, smallSigns);
  createBitmasks(amtMiddleSigns, middleSigns);

  for(StarSign sign : smallSigns)
  {
      clearOutput(smallOutput, sign);
      writeOutput(smallLatchPin, smallClockPin, smallDataPin, smallOutput, smallOrder, 3);
  }

  for(StarSign sign : middleSigns)
  {
      clearOutput(middleOutput, sign);
      writeOutput(middleLatchPin, middleClockPin, middleDataPin, middleOutput, middleOrder, 4);
  }

  for(StarSign sign : largeSigns)
  {
      clearOutput(largeOutput, sign);
      writeOutput(largeLatchPin, largeClockPin, largeDataPin, largeOutput, largeOrder, 6);
  }
  //largeSigns[5].turnedOn = true;
}

void loop() {

//if(!waitWhileBlinking)
//{
  for (int i = 0; i<16; i++) {
    if(state[i] != digitalRead(switchPins[i])){
      if((millis()-lastDebounce[i])>DEBOUNCE_TIME){
        lastDebounce[i] = millis();
        for(StarSign &sign : smallSigns){
          if(sign.switchPin == switchPins[i]){
            if(!sign.turnedOn){
              sign.turnedOn = true;
              sign.blinkTime = millis();
              nameOfSign = "";
              //waitWhileBlinking = true;
            }
            state[i] = digitalRead(switchPins[i]);
          } else if (sign.turnedOn){
            sign.turnedOn = false;
            clearOutput(smallOutput, sign);
            writeOutput(smallLatchPin, smallClockPin, smallDataPin, smallOutput, smallOrder, 3);
          }
        }       
        for(StarSign &sign : middleSigns){
          if(sign.switchPin == switchPins[i]){
            if(!sign.turnedOn){
              sign.turnedOn = true;
              sign.blinkTime = millis();
              nameOfSign = "";
              //waitWhileBlinking = true;
            }
            state[i] = digitalRead(switchPins[i]);
          } else if (sign.turnedOn){
            sign.turnedOn = false;
            clearOutput(middleOutput, sign);
            writeOutput(middleLatchPin, middleClockPin, middleDataPin, middleOutput, middleOrder, 4);
          }
        }       
        for(StarSign &sign : largeSigns){
          if(sign.switchPin == switchPins[i]){
            if(!sign.turnedOn){
              sign.turnedOn = true;
              sign.blinkTime = millis();
              nameOfSign = "";
              //waitWhileBlinking = true;
            }
            state[i] = digitalRead(switchPins[i]);
          } else if (sign.turnedOn){
            sign.turnedOn = false;
            clearOutput(largeOutput, sign);
            writeOutput(largeLatchPin, largeClockPin, largeDataPin, largeOutput, largeOrder, 6);
          }
        }
      }
    }
  }


//}

  //Serial.println(digitalRead(A0));
  //delay(1000);

  for(StarSign sign : smallSigns)
    if(sign.turnedOn){
      if((millis() - sign.blinkTime)<BLINKING_TIME){
        updateOutput(smallOutput, sign, true);
        writeOutput(smallLatchPin, smallClockPin, smallDataPin, smallOutput, smallOrder, 3);
        delay(40);
        clearOutput(smallOutput, sign);
        writeOutput(smallLatchPin, smallClockPin, smallDataPin, smallOutput, smallOrder, 3);
        delay(120);
        } 
        else if(strcmp(sign.name, nameOfSign) != 0){
          //waitWhileBlinking = false;
         updateOutput(smallOutput, sign, false);
         writeOutput(smallLatchPin, smallClockPin, smallDataPin, smallOutput, smallOrder, 3);
         delay(100);
         clearOutput(smallOutput, sign);
         //Serial.println("once");
          nameOfSign = sign.name;
        } 
    }

  for(StarSign sign : middleSigns)
    if(sign.turnedOn){
      if((millis() - sign.blinkTime)<BLINKING_TIME){
        updateOutput(middleOutput, sign, true);
        writeOutput(middleLatchPin, middleClockPin, middleDataPin, middleOutput, middleOrder, 4);
        delay(40);
        clearOutput(middleOutput, sign);
        writeOutput(middleLatchPin, middleClockPin, middleDataPin, middleOutput, middleOrder, 4);
        delay(120);
        } 
        else if(strcmp(sign.name, nameOfSign) != 0){
          //waitWhileBlinking = false;
         updateOutput(middleOutput, sign, false);
         writeOutput(middleLatchPin, middleClockPin, middleDataPin, middleOutput, middleOrder, 4);
         delay(100);
         clearOutput(middleOutput, sign);
         //Serial.println("once");
          nameOfSign = sign.name;
        } 
    }

  for(StarSign sign : largeSigns)
    if(sign.turnedOn){
      if((millis() - sign.blinkTime)<BLINKING_TIME){
        updateOutput(largeOutput, sign, true);
        writeOutput(largeLatchPin, largeClockPin, largeDataPin, largeOutput, largeOrder, 6);
        delay(40);
        clearOutput(largeOutput, sign);
        writeOutput(largeLatchPin, largeClockPin, largeDataPin, largeOutput, largeOrder, 6);
        delay(120);
        } 
        else if(strcmp(sign.name, nameOfSign) != 0){
          //waitWhileBlinking = false;
         updateOutput(largeOutput, sign, false);
         writeOutput(largeLatchPin, largeClockPin, largeDataPin, largeOutput, largeOrder, 6);
         delay(100);
         clearOutput(largeOutput, sign);
         //Serial.println("once");
          nameOfSign = sign.name;
        } 
    }
}
