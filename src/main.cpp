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

const uint16_t switchPins [16] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};

int out = 0; //TODO ta bort
uint16_t lastDebounce [16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

struct StarSign {
  char *name;
  int noLamps;
  bool turnedOn = false;
  uint64_t bitMask = 0;
  StarSign(char* name_, int noLamps_) : name(name_), noLamps(noLamps_) {}
};

StarSign signs [amtSigns] = {
  {"Pegasus", 8},
  {"Lilla Björn", 7},
  {"Stora Björn", 8},
  {"Björnväktaren", 6},
  {"Jungfrun", 3},
  {"Kräftan", 4},
  {"Orion", 2}
};

uint64_t generateBinary(int start, int n){
  if(n>start){
    return generateBinary(start, n-1) | (((uint64_t)1)<<(n-1));
  }
  else {
    return 0;
  }
}

uint64_t output = 0;
void setup() {

  Serial.begin(115200);

  for (auto pin : switchPins)
  {
    pinMode(pin, INPUT_PULLUP);
  }

  int countSign = 0;
    for (int i = 0; i < amtSigns; i++)
   {
    //if(strcmp(sign.name, "Stora Björn"))
      //sign.turnedOn = true;
    //sign.turnedOn = (tempC%2);
    uint64_t binary = generateBinary(countSign, signs[i].noLamps+countSign);
    signs[i].bitMask = binary;
    countSign += signs[i].noLamps;
    output |= (binary*signs[i].turnedOn);
//if(sign.turnedOn){
  //  Serial.println("----");
  //  Serial.println((uint16_t) binary);
  //  Serial.println((uint16_t) (binary>>16));
  //  Serial.println((uint16_t) (binary>>32));
  //  Serial.println((uint16_t) (binary>>48));
  //  //Serial.println(binary);
  //  Serial.println(sign.noLamps);
  //  Serial.println("----");
  //  Serial.println((uint16_t) output);
  //  Serial.println((uint16_t) output>>16);
  //  Serial.println((uint16_t) output>>32);
  //  Serial.println((uint16_t) output>>48);
//}
  }

  Serial.println("Click!");
  pinMode(13, OUTPUT);

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

}

int count = 0;
void loop() {
    count = (count+1)%amtSigns;
    for (int i = 0; i < 10; i++)
{
  if(count>amtSigns/2)
    output |= (signs[count].bitMask&(((uint64_t)random(__LONG_MAX__))<<32));
    else
    output |= (signs[count].bitMask&(random(__LONG_MAX__)));
    digitalWrite(latchPin, LOW);
 
    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output>>24));  //3 höger
    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output>>16));  //1 vänster
    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output>>8));  //2 vänster
    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output>>32));  //2 höger
    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output));  //3 vänster
    shiftOut(dataPin, clockPin, MSBFIRST, (byte)(output>>40));  //1 höger (med 3pins uppåt)

    //shift out the bits
 
    //take the latch pin high so the pins reflect
    //the data we have sent
    digitalWrite(latchPin, HIGH);
    delay(100);
    output &= ~(signs[count].bitMask);
}

    // pause before next value:
    //delay(1000);
}