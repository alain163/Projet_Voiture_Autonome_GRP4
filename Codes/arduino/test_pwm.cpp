#include <Servo.h>

Servo esc;

void setup() {
  esc.attach(9);
  Serial.begin(9600);
  
  Serial.println("Calibration de l'ESC...");
  esc.writeMicroseconds(2000); 
  delay(2000);
  esc.writeMicroseconds(1000);
  delay(2000);
  Serial.println("Calibration terminee");
}

void loop() {
  Serial.println("Demarrage du moteur...");
  
  for (int speed = 1000; speed <= 2000; speed += 50) { 
    esc.writeMicroseconds(speed);
    Serial.print("Vitesse: ");
    Serial.println(speed);
    delay(500);
  }

  Serial.println("Arret du moteur...");
  esc.writeMicroseconds(1000); 
  delay(2000);
}
