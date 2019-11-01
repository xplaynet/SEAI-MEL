void setup() {
  pinMode(7, INPUT);
  digitalWrite(7, HIGH);

  pinMode(10, OUTPUT);
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1A = _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS12) | _BV(CS10) | _BV(WGM13);
  OCR1A = 1000;
  OCR1B = 0;

  Serial.begin(57600);



}

void loop() {
  static int last = 0;
  int current;
  static unsigned long Time = 0;
  static int start = 1;

  if (start) {
    Serial.print("started:");
    Serial.print(millis());
    Serial.print("\n");
    start = 0;
  }

  current = digitalRead(7);
  if (last != current) {
    last = current;
    Time = millis();
    Serial.print(Time);
    if (last) Serial.write("on\n");
    else Serial.write("off\n");
  }

  if (Time > 5000) {
    OCR1B = 200;
  }


  if (Time > 10000) {
    Serial.print(TCCR1B);
    Serial.print(TCCR1A);
    delay(100000);
  }


}
