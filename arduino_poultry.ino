#include <Wire.h>

#define COOLING_FAN_PIN 10
#define EXHAUST_FAN_PIN 11
#define HEAT_LAMP_PIN 12
#define WINDOW_PIN 13

unsigned long startTime = millis();
const unsigned long duration = 35600;
String windowStatus = "closed";

void setup() {
  Serial.begin(9600);
  pinMode(COOLING_FAN_PIN, OUTPUT);
  pinMode(EXHAUST_FAN_PIN, OUTPUT);
  pinMode(HEAT_LAMP_PIN, OUTPUT);
  pinMode(WINDOW_PIN, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    int command = Serial.read();
    Serial.println(command);
    switch (command) {
      case 1:
        openCloseWindow(windowStatus, "open", millis() - startTime);
        digitalWrite(EXHAUST_FAN_PIN, HIGH);
        digitalWrite(COOLING_FAN_PIN, HIGH);
        digitalWrite(HEAT_LAMP_PIN, LOW);
        break;

      case 2:
        openCloseWindow(windowStatus, "closed", millis() - startTime);
        digitalWrite(EXHAUST_FAN_PIN, LOW);
        digitalWrite(COOLING_FAN_PIN, LOW);
        digitalWrite(HEAT_LAMP_PIN, HIGH);
        break;

      case 3:
        openCloseWindow(windowStatus, "open", millis() - startTime);
        digitalWrite(EXHAUST_FAN_PIN, LOW);
        digitalWrite(COOLING_FAN_PIN, HIGH);
        digitalWrite(HEAT_LAMP_PIN, LOW);
        break;

      case 4:
        openCloseWindow(windowStatus, "closed", millis() - startTime);
        digitalWrite(EXHAUST_FAN_PIN, HIGH);
        digitalWrite(COOLING_FAN_PIN, LOW);
        digitalWrite(HEAT_LAMP_PIN, HIGH);
        break; 
      case 5:
        openCloseWindow(windowStatus, "open", millis() - startTime);
        digitalWrite(EXHAUST_FAN_PIN, LOW);
        digitalWrite(COOLING_FAN_PIN, LOW);
        digitalWrite(HEAT_LAMP_PIN, LOW);
        break;
      case 6:
        openCloseWindow(windowStatus, "closed", millis() - startTime);
        digitalWrite(EXHAUST_FAN_PIN, HIGH);
        digitalWrite(COOLING_FAN_PIN, LOW);
        digitalWrite(HEAT_LAMP_PIN, LOW);
        break;
      case 7:
        openCloseWindow(windowStatus, "closed", millis() - startTime);
        digitalWrite(EXHAUST_FAN_PIN, LOW);
        digitalWrite(COOLING_FAN_PIN, HIGH);
        digitalWrite(HEAT_LAMP_PIN, LOW);
        break;
      case 8:
        openCloseWindow(windowStatus, "open", millis() - startTime);
        break;
      case 9:
        digitalWrite(EXHAUST_FAN_PIN, HIGH);
        break;
      case 10:
        digitalWrite(COOLING_FAN_PIN, HIGH);
        break;
      case 11:
        digitalWrite(HEAT_LAMP_PIN, HIGH);
        break;
      case 12:
        openCloseWindow(windowStatus, "closed", millis() - startTime);
        break;
      case 13:
        digitalWrite(EXHAUST_FAN_PIN, LOW);
        break;
      case 14:
        digitalWrite(COOLING_FAN_PIN, LOW);
        break;
      case 15:
        digitalWrite(HEAT_LAMP_PIN, LOW);
        break;
      case 0:
      default:
        openCloseWindow(windowStatus, "closed", millis() - startTime);
        digitalWrite(EXHAUST_FAN_PIN, LOW);
        digitalWrite(COOLING_FAN_PIN, LOW);
        digitalWrite(HEAT_LAMP_PIN, LOW);
        break;
    }
  }
}

void openCloseWindow(String &currentStatus, String triggerStatus, unsigned long time) { //Window doesn't have any detection for open-close, only rotates when powered
  if (currentStatus != triggerStatus && time > duration) {
    digitalWrite(WINDOW_PIN, LOW);
    delay(2500);
    digitalWrite(WINDOW_PIN, HIGH);
    startTime = millis();
    currentStatus = triggerStatus;
  }
}