#include <LiquidCrystal.h>
#include <SR04.h>
#include "ServoTimer2.h"

LiquidCrystal lcd (12, 11, 10, 9, 8, 7);
ServoTimer2 Myservo;

const int ServoPin = 3;
const int latchPin = 5;
const int clockPin = 6;
const int dataPin = 4;
const int NUM_LEDS = 8;
const int PinPing = 50;
const int PinEcho = 52;
const int number = 0;

void setup() {
  pinMode (PinPing, OUTPUT);
  pinMode (PinEcho, INPUT);
  Serial.begin (9600);
  // put your setup code here, to run once:
  lcd.begin (16,2);
  pinMode (latchPin, OUTPUT);
  pinMode (dataPin, OUTPUT);
  pinMode (clockPin, OUTPUT);
  Myservo.attach (ServoPin);
  
  cli ();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 15624; // (16*10^6) / (1/1024) -1 <65536
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei ();
}

ISR (TIMER1_COMPA_vect)
{
  UltraSonicSensor ();
}

void EvasiveManeuver (int distance)
{
  SetServo ( distance * ((90-60) / 20) + 60);
  lcd.clear ();
  lcd.setCursor (0,0);
  lcd.print (String ("WARNING123"));
  lcd.setCursor (0,1);
  lcd.print (String ("Servo Pos:") + String (Myservo.read() * 9 / 180 - 90));
  updateShiftRegister (Myservo.read() * 9 / 180 - 90);
}

int UltraSonicSensor ()
{
  long duration, distcm;
  digitalWrite (PinPing, LOW);
  delayMicroseconds (2);
  digitalWrite (PinPing, HIGH);
  delayMicroseconds (10);
  digitalWrite (PinPing, LOW);
  distcm = pulseIn (PinEcho, HIGH) /29 /2;

  if (distcm < 20)
  EvasiveManeuver (distcm);
  else {
    SetServo (number + 90);
    updateShiftRegister (number);
    print1(distcm);
  }
}

void print1 (int distance)
{
  lcd.clear ();
  lcd.setCursor (0,0);
  lcd.print (String ("Distance(cm):") + String (distance));
  lcd.setCursor (0,1);
  lcd.print (String ("Servo Pos:") + String (Myservo.read() * 9 / 180 - 90));
}

int SetServo (int degree)
{
  int PWM = degree * 180 / 9;
  Myservo.write ( PWM );
  delay (200);
}

void updateShiftRegister (int storageByte)
{
  if (storageByte < 0) storageByte = -storageByte + 128;
  digitalWrite (latchPin, LOW);
  shiftOut (dataPin, clockPin, LSBFIRST, storageByte);
  digitalWrite (latchPin,HIGH);
}
void loop (){
  }
