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

const uint16_t switchPins [16] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};

int out = 0; //TODO ta bort
uint16_t lastDebounce [16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void setup() {

  Serial.begin(115200);

  for (auto pin : switchPins)
  {
    pinMode(pin, INPUT_PULLUP);
  }

  Serial.println("Click!");
  pinMode(13, OUTPUT);

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

}

int count = 0;
void loop() {
 for (int i = 0; i < 1; i++)
 {
    digitalWrite(latchPin, LOW);
 
    shiftOut(dataPin, clockPin, LSBFIRST, random(256));  //1
    shiftOut(dataPin, clockPin, LSBFIRST, random(256));  //2
    shiftOut(dataPin, clockPin, LSBFIRST, random(256));  //3
    shiftOut(dataPin, clockPin, LSBFIRST, random(256));  //4
    shiftOut(dataPin, clockPin, LSBFIRST, random(256));  //5
    shiftOut(dataPin, clockPin, LSBFIRST, random(256));  //6

    //shift out the bits
    // shiftOut(dataPin, clockPin, LSBFIRST, (1<<i) * (count==0 ? 1 : 0));  //1
    // shiftOut(dataPin, clockPin, LSBFIRST, (1<<i) * (count==1 ? 1 : 0));  //2
    // shiftOut(dataPin, clockPin, LSBFIRST, (1<<i) * (count==2 ? 1 : 0));  //3
    // shiftOut(dataPin, clockPin, LSBFIRST, (1<<i) * (count==5 ? 1 : 0));  //4
    // shiftOut(dataPin, clockPin, LSBFIRST, (1<<i) * (count==3 ? 1 : 0));  //5
    // shiftOut(dataPin, clockPin, LSBFIRST, (1<<i) * (count==4 ? 1 : 0));  //6
 
    //take the latch pin high so the pins reflect
    //the data we have sent
    digitalWrite(latchPin, HIGH);

    // pause before next value:
    delay(50);
 }
    count = (count+1)%6;
}