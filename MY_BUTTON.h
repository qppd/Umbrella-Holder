#define BUTTON_COUNT 4
#define DEBOUNCE_DELAY 50

#define BUTTON_1 4
#define BUTTON_2 5
#define BUTTON_3 6
#define BUTTON_4 7

int inputState[BUTTON_COUNT];
int lastInputState[BUTTON_COUNT] = { LOW, LOW, LOW, LOW };
bool inputFlags[BUTTON_COUNT] = { LOW, LOW, LOW, LOW };
long lastDebounceTime[BUTTON_COUNT] = { 0, 0, 0, 0 };
const int inputPins[BUTTON_COUNT] = { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 };

void initBUTTONS() {
  for (int i = 0; i < BUTTON_COUNT; i++) {
    pinMode(inputPins[i], INPUT);
    digitalWrite(inputPins[i], HIGH);
  }
  delay(1000);
  Serial.println("Push Buttons: Initialized!");
}

void inputAction(int BUTTON_PIN) {
  Serial.println(BUTTON_PIN);
}
void setInputFlags() {
  for (int i = 0; i < BUTTON_COUNT; i++) {
    int reading = digitalRead(inputPins[i]);
    if (reading != lastInputState[i]) {
      lastDebounceTime[i] = millis();
    }
    if (millis() - lastDebounceTime[i] > DEBOUNCE_DELAY) {
      if (reading != inputState[i]) {
        inputState[i] = reading;
        if (inputState[i] == HIGH) {
          inputFlags[i] = HIGH;
        }
      }
    }
    lastInputState[i] = reading;
  }
}




boolean first_time = true;
void resolveInputFlags() {
  for (int i = 0; i < BUTTON_COUNT; i++) {
    if (inputFlags[i] == HIGH) {
      if (i == 3 && first_time) {
        first_time = false;
      } else {
        inputAction(i);
      }
      inputFlags[i] = LOW;
    }
  }  // loop end
}
