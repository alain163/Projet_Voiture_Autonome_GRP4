#include <Servo.h>  // Bibliothèque pour générer un signal PWM

Servo esc;  // Déclaration de l'objet pour le variateur ESC

void setup() {
  esc.attach(9);  // Connecter le signal de l'ESC à la pin 9
  Serial.begin(9600);
  
  Serial.println("Calibration de l'ESC...");
  esc.writeMicroseconds(2000);  // Envoie le signal max pour calibration
  delay(2000);
  esc.writeMicroseconds(1000);  // Retour à la position neutre
  delay(2000);
  Serial.println("Calibration terminée");
}

void loop() {
  Serial.println("Démarrage du moteur...");
  
  for (int speed = 1000; speed <= 2000; speed += 50) {  // Accélération progressive
    esc.writeMicroseconds(speed);
    Serial.print("Vitesse: ");
    Serial.println(speed);
    delay(500);
  }

  Serial.println("Arrêt du moteur...");
  esc.writeMicroseconds(1000);  // Stopper le moteur
  delay(2000);
}