/*
 *
 * Stjärnhimmel projekt till bibliotek
 *
 * TODO:  1. Kod för knappar
 *        2. Kod för lampor
 *        3.
 *        4.
 *        5.
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
#define DEBOUNCE 200

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
#define amtMiddleSigns 6 


const uint8_t largeOrder[6] = {24, 16, 8, 32, 0, 40};
const uint16_t switchPins[16] = {A0, A1, A2,  A3,  A4,  A5,  A6,  A7,
                                 A8, A9, A10, A11, A12, A13, A14, A15};

uint16_t lastDebounce[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint64_t largeOutput = 0;

struct StarSign {
  char *name;
  int noLamps, noShifts;
  bool turnedOn = false;
  uint64_t bitMask = 0;
  unsigned long blinkTime = 0;
  StarSign(char *name_, int noLamps_) : name(name_), noLamps(noLamps_) {}
};

/**
 * De första stjärntecknen, som tillhör den "stora" gruppen. Är totalt 48 lampor (dvs 6register*8=48)
 */
StarSign largeSigns[amtLargeSigns] = {
    {"Björnväktaren", 7}, {"Pegasus", 8}, {"Kräftan", 4}, {"Orion", 5}, //vänster
    {"Stora Björn", 7}, {"Lilla Björn", 6}, {"Jungfrun", 11} //höger
};

/**
 * De andra stjärntecknen, som tillhör den "lilla" gruppen. Är totalt 24 lampor (dvs 3register*8=24)
 */
StarSign smallSigns[amtSmallSigns] = {
  {"Draken", 8}, {"Cepheus", 5}, {"Cassiopeia", 5}, {"Perseus", 6}
};

/**
 * De tredje stjärntecknen, som tillhör "mellan" gruppen. Är totalt 32 lampor (dvs 4register*8=32)
 */
StarSign middleSigns[amtMiddleSigns] = {
  {"Väduren", 4}, {"Oxen", 6}, {"Orion", 5}, {"Kusken", 5}, {"Tvillingarna", 11}, {"Lejonet", 10} //just nu totalt 41 lampor... (behöver kanske en grupp till...)
};

void setup() {

  Serial.begin(115200);

  for (auto pin : switchPins) {
    pinMode(pin, INPUT_PULLUP);
  }

  int countSign = 0;
  for (int i = 0; i < amtLargeSigns; i++) {
    largeSigns[i].noShifts = countSign;
    largeSigns[i].bitMask = (1 << (largeSigns[i].noLamps)) - 1; //denna rad skapar en sträng av ettor lika lång som antalet lampor. T.ex. 4 lampor=> 0b00001 << 4 steg => 0b10000 - 1 = 0b1111
    countSign += largeSigns[i].noLamps;
  }

  pinMode(largeLatchPin, OUTPUT);
  pinMode(largeClockPin, OUTPUT);
  pinMode(largeDataPin, OUTPUT);

  pinMode(smallLatchPin, OUTPUT);
  pinMode(smallClockPin, OUTPUT);
  pinMode(smallDataPin, OUTPUT);

  pinMode(middleLatchPin, OUTPUT);
  pinMode(middleClockPin, OUTPUT);
  pinMode(middleDataPin, OUTPUT);
}

void writeOutput(int latchPin, int clockPin, int dataPin, uint64_t output, uint8_t outputOrder[], int amtRegisters){
  digitalWrite(latchPin, LOW);

  for (size_t i = 0; i < amtRegisters; i++){
    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output >> outputOrder[i])); // 3 höger
  }



  //shiftOut(largeDataPin, largeClockPin, MSBFIRST, (byte)(largeOutput >> 24)); // 3 höger
  //shiftOut(largeDataPin, largeClockPin, MSBFIRST, (byte)(largeOutput >> 16)); // 1 vänster
  //shiftOut(largeDataPin, largeClockPin, MSBFIRST, (byte)(largeOutput >> 8));  // 2 vänster
  //shiftOut(largeDataPin, largeClockPin, MSBFIRST, (byte)(largeOutput >> 32)); // 2 höger
  //shiftOut(largeDataPin, largeClockPin, MSBFIRST, (byte)(largeOutput));       // 3 vänster
  //shiftOut(largeDataPin, largeClockPin, MSBFIRST,                        // 1 höger (med 3pins uppåt)
  //          (byte)(largeOutput >> 40));                                      

  // shift out the bits
  digitalWrite(latchPin, HIGH);
}

int count = 0;
void loop() {

  //TODO fixa en blinka funktion...
  for (int i = 0; i < 30; i++) {
    //largeOutput |= (largeSigns[count].bitMask & random(largeSigns[count].bitMask+1))<<largeSigns[count].noShifts; // blinking...
    writeOutput(largeLatchPin, largeClockPin, largeDataPin, largeOutput, largeOrder, 6);
    delay(30);
    //largeOutput &= ~(largeSigns[count].bitMask<<largeSigns[count].noShifts); // Clear...
  }
  count = (count+1)%amtLargeSigns;

}
