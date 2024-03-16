#include "Adafruit_NeoPixel.h"
#include <SoftwareSerial.h>

SoftwareSerial mySerial(4, 3); // RX, TX

#define RING_PIN 7
#define ANZAHL_LED 16
#define DUNKELHEITSSCHALTER_PIN 6 // Pin für Dunkelheitssensor
#define BEWEGUNGSSENSOR_PIN 5
#define BUZZER_PIN 11

#define MOTOR_ENABLE_PIN 10
#define MOTOR_STEP_PIN 9
#define MOTOR_DIR_PIN 8

Adafruit_NeoPixel LEDRing(ANZAHL_LED, RING_PIN, NEO_GRB + NEO_KHZ800);

// Motor Variablen
int steps = 200;

// Zustandsvariablen
bool motorOpened = false;
bool isTonePlaying = false;
unsigned long blinkStartTime = 0;
bool ledState = false;

// Timer Variablen
unsigned long previousMillis = 0;
const long interval = 15000; // 15 Sekunden

String received; // Globale Variable für empfangene Nachrichten

void setup() {
  Serial.begin(115200);
  mySerial.begin(38400);
  
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  
  pinMode(BEWEGUNGSSENSOR_PIN, INPUT);
  pinMode(DUNKELHEITSSCHALTER_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  LEDRing.begin(); // Initialisierung des LED-Rings

  Serial.println("Setup abgeschlossen.");
}

void loop() {
  if (mySerial.available()) {
    received = mySerial.readStringUntil('\n');
    received.trim();
    Serial.println("Empfangen: " + received);
  }

  int dunkelheitsschalterStatus = digitalRead(DUNKELHEITSSCHALTER_PIN); 
  int bewegungssensorStatus = digitalRead(BEWEGUNGSSENSOR_PIN);

  handleMotorAction(dunkelheitsschalterStatus);
  handleLEDRingAndAlarm(bewegungssensorStatus, dunkelheitsschalterStatus);
  checkForAnimals();
  Spannungsmessung();
}

void handleMotorAction(int sensorStatus) {
  if (sensorStatus == HIGH && !motorOpened) {
    rotateMotorToOpen();
    motorOpened = true;
    Serial.println("Motor geöffnet bei Tag.");
  } else if (sensorStatus == LOW && motorOpened) {
    rotateMotorToClose();
    motorOpened = false;
    Serial.println("Motor geschlossen bei Nacht.");
  }
}

void handleLEDRingAndAlarm(int bewegung, int dunkelheit) {
  unsigned long currentMillis = millis();
  if (dunkelheit == LOW && bewegung == HIGH) {
    turnOnLEDRing();
    mySerial.println("Bewegung erkannt");
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      turnOffLEDRing();
    }
  } else if (currentMillis - previousMillis >= interval) {
    turnOffLEDRing();
  }
}

void checkForAnimals() {
  if (received == "Huhn") {
    rotateMotorToOpen();
    delay(5000); // Warten für 5 Sekunden
    rotateMotorToClose();
    received = ""; // Zurücksetzen der empfangenen Nachricht
    Serial.println("Huhn behandelt.");
  } else if (received == "Fuchs") {
    scareFuchs();
    received = ""; // Zurücksetzen der empfangenen Nachricht
    Serial.println("Fuchs verscheucht.");
  }
}

void turnOnLEDRing() {
  for(int i = 0; i < LEDRing.numPixels(); i++) {
    LEDRing.setPixelColor(i, LEDRing.Color(255, 255, 255)); // Hellweiß
  }
  LEDRing.show();
  Serial.println("LEDRing eingeschaltet.");
}

void turnOffLEDRing() {
  LEDRing.clear(); // Alle LEDs ausschalten
  LEDRing.show();
  Serial.println("LEDRing ausgeschaltet.");
}

void scareFuchs() {
  unsigned long currentMillis = millis();

  // Starten Sie den Ton nur einmal
  if (!isTonePlaying) {
    tone(BUZZER_PIN, 1000, 500); // Ton für 1 Sekunde
    isTonePlaying = true; // Verhindern Sie, dass der Ton erneut gestartet wird
    blinkStartTime = currentMillis; // Setzen Sie die Startzeit für das Blinken
  }

  // Überprüfen Sie, ob die Blinkzeit (1 Sekunde) noch nicht überschritten ist
  if (currentMillis - blinkStartTime < 1000) {
    // Wechseln in den LED-Zustand basierend Millis
    if ((currentMillis - blinkStartTime) < 500) {
      if (!ledState) {
        turnOnLEDRing(); // LEDs einschalten
        ledState = true;
      }
    } else {
      if (ledState) {
        turnOffLEDRing(); // LEDs ausschalten
        ledState = false;
      }
    }
  } else if (isTonePlaying) {
    // Nach 1 Sekunde LEDs und der Ton ausgeschalten 
    if (ledState) {
      turnOffLEDRing();
      ledState = false;
    }
    isTonePlaying = false; // Erlaubt, dass der Ton beim nächsten Aufruf erneut gestartet wird
  }
}

void rotateMotorToOpen() {
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  for (int stepCounter = 0; stepCounter < steps; stepCounter++) {
    digitalWrite(MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(750);
    digitalWrite(MOTOR_STEP_PIN, LOW);
    delayMicroseconds(750);
  }
  Serial.println("Motor zum Öffnen gedreht.");
}

void rotateMotorToClose() {
  digitalWrite(MOTOR_DIR_PIN, LOW);
  for (int stepCounter = 0; stepCounter < steps; stepCounter++) {
    digitalWrite(MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(750);
    digitalWrite(MOTOR_STEP_PIN, LOW);
    delayMicroseconds(750);
  }
  Serial.println("Motor zum Schließen gedreht.");
}

void Spannungsmessung() {
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0); // Anpassung für 5V und 10-bit ADC
  if (voltage < 3.3) {
    mySerial.println("Niedrige Spannung: " + String(voltage));
  }
}

