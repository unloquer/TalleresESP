#include "Arduino.h"

int pulse_pin = D8;
// pulse in devuelve unsigned long, tener cuidado con el envio de esta variable a otros entornos

unsigned long pulse_high_time, pulse_low_time, period;
int frecuency;
int inByte = 0;

//---------------------
void setup(){

  Serial.begin(9600);
  pinMode(pulse_pin,INPUT);

}
//--------------------------------
void loop(){

  pulse_low_time = pulseIn(pulse_pin,LOW);
  pulse_high_time = pulseIn(pulse_pin, HIGH);

  period = pulse_low_time + pulse_high_time;

  //  frecuency = 1000000/period; // valores en sg, 10M de microsegundos son 1 segundo

  Serial.print("high :");
  Serial.print(pulse_high_time);
  Serial.print("\t");
  Serial.print("low :");
  Serial.print(pulse_low_time);
  Serial.print("\t");
  Serial.print("period:  ");
  Serial.println(period);

  delay(30);

}
