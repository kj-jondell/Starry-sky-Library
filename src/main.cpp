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
 * Frågor 1. Beräkna strömförsörjning... (är 18W agg tillräckligt?)
 *        2. Hur många stjärnbilder? 15-16 bilder
 *        3. Hur många knappar? 12+
 *        4. Hur många meter kabel?
 *        5. Hur stora motstånd? (ström...)
 *        6. Led-hållare? (ska de borras hål? hur stora hål?)
 *        7. Lödas på kort... hur stora kort?
 *        8.
 *        9. Går det att använda LED-lister istället?
 *
 */
#include <Arduino.h>
#define DEBOUNCE 200

#define latchPin 2
#define clockPin 3
#define dataPin 4

#define amtSigns 7

const uint16_t switchPins[16] = {A0, A1, A2,  A3,  A4,  A5,  A6,  A7,
                                 A8, A9, A10, A11, A12, A13, A14, A15};

int out = 0; // TODO ta bort
uint16_t lastDebounce[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint64_t output = 0;

struct StarSign {
  char *name;
  int noLamps, noShifts;
  bool turnedOn = false;
  uint64_t bitMask = 0;
  StarSign(char *name_, int noLamps_) : name(name_), noLamps(noLamps_) {}
};

StarSign signs[amtSigns] = {
    {"Pegasus", 8}, {"Jungfrun", 3}, {"Lilla Björn", 7}, {"Björnväktaren", 6},
    {"Kräftan", 4}, {"Orion", 2},    {"Stora Björn", 8}};

uint64_t generateBinary(int start, int end) {
  if (start < 0 || end < 0 || start >= end)
    return 0;
  return ((((uint64_t)1) << (end - start)) - 1) << start;
}

void setup() {

  Serial.begin(115200);

  for (auto pin : switchPins) {
    pinMode(pin, INPUT_PULLUP);
  }

  int countSign = 0;
  for (int i = 0; i < amtSigns; i++) {
    //uint64_t binary = generateBinary(countSign, signs[i].noLamps + countSign);
    signs[i].noShifts = countSign;
    signs[i].bitMask = (1 << (signs[i].noLamps)) - 1;
    countSign += signs[i].noLamps;
    // output |= (binary * signs[i].turnedOn);
  }

  Serial.println("Click!");
  pinMode(13, OUTPUT);

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}

int count = 0;
void loop() {
  for (int i = 0; i < 30; i++) {
    output |= (signs[count].bitMask & random(signs[count].bitMask+1))<<signs[count].noShifts; // blinking...
    digitalWrite(latchPin, LOW);

    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output >> 24)); // 3 höger
    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output >> 16)); // 1 vänster
    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output >> 8));  // 2 vänster
    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output >> 32)); // 2 höger
    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output));       // 3 vänster
    shiftOut(dataPin, clockPin, MSBFIRST,
             (byte)(output >> 40)); // 1 höger (med 3pins uppåt)

    // shift out the bits

    // take the latch pin high so the pins reflect
    // the data we have sent
    digitalWrite(latchPin, HIGH);
    delay(30);
    output &= ~(signs[count].bitMask<<signs[count].noShifts); // Clear...
  }
  count = (count + 1) % amtSigns;
}