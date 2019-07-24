

#include <avr/io.h>

int relayPin = 5;
char incomingByte = 0;


void setup() {
  Serial.begin(9600); //opens serial port, sets data rate to 9600 bps
 
  pinMode(relayPin, OUTPUT);           // set pin as output
  digitalWrite(relayPin, HIGH);

  Serial.println("Feeder is ready :)");
}

void loop()
{
  if (Serial.available() > 0) {
    incomingByte = Serial.read(); // read the incoming byte:
  
    if (incomingByte == 'f')  {
      Serial.println("Feeding"); 
      digitalWrite(relayPin, LOW);
      delay(100);
      digitalWrite(relayPin, HIGH);
    }
  }
}

