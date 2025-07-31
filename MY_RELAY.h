uint16_t RELAY_1 = 8;
uint16_t RELAY_2 = 9;

void initRELAY(){
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
}

void operateSSR(uint16_t RELAY, boolean OPENED) {
  if (OPENED)
    digitalWrite(RELAY, HIGH);
  else
    digitalWrite(RELAY, LOW);
}