int LED_PIN_1 = 2;   // REDLED
int LED_PIN_2 = 3;   // GREEN LED
int LED_PIN_3 = 11;  // BLUE LED

void initLED() {
  Serial.print("Initializing LED!");
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);
  digitalWrite(LED_PIN_1, LOW);
  digitalWrite(LED_PIN_2, LOW);
  digitalWrite(LED_PIN_3, LOW);
  delay(500);
  Serial.println("Successful!");
}

void operateLED(int LED_PIN, bool opened) {
  if (opened) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}