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

#define amtSigns 7 //TODO byt namn

const uint16_t switchPins[16] = {A0, A1, A2,  A3,  A4,  A5,  A6,  A7,
                                 A8, A9, A10, A11, A12, A13, A14, A15};

uint16_t lastDebounce[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint64_t output = 0;

struct StarSign {
  char *name;
  int noLamps, noShifts;
  bool turnedOn = false;
  uint64_t bitMask = 0;
  unsigned long blinkTime = 0;
  StarSign(char *name_, int noLamps_) : name(name_), noLamps(noLamps_) {}
};

/**
 * First amount of signs ...
 */
StarSign signs[amtSigns] = {
    {"Pegasus", 8}, {"Jungfrun", 11}, {"Lilla Björn", 6}, {"Björnväktaren", 7},
    {"Kräftan", 4}, {"Orion", 5},    {"Stora Björn", 7}
};

void setup() {

  Serial.begin(115200);

  for (auto pin : switchPins) {
    pinMode(pin, INPUT_PULLUP);
  }

  int countSign = 0;
  for (int i = 0; i < amtSigns; i++) {
    signs[i].noShifts = countSign;
    signs[i].bitMask = (1 << (signs[i].noLamps)) - 1;
    countSign += signs[i].noLamps;
  }

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

}

int count=0;
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

    digitalWrite(latchPin, HIGH);
    delay(30);
    output &= ~(signs[count].bitMask<<signs[count].noShifts); // Clear...
  }
  count = (count + 1) % amtSigns;
}
