#define PIN_X 13
#define PIN_Y 12
#define PIN_FRAME 11
#define X_POINTS 10
#define Y_POINTS 10
int point_index = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_X, OUTPUT);
  pinMode(PIN_Y, OUTPUT);
  pinMode(PIN_FRAME, OUTPUT);
  digitalWrite(PIN_X, LOW);
  digitalWrite(PIN_Y, LOW);
  digitalWrite(PIN_FRAME, LOW);
}
void changePin1State() {
  digitalWrite(PIN_X, HIGH);
  // delayMicroseconds(2);
  digitalWrite(PIN_X, LOW);
  if (point_index % X_POINTS == 0) {
    digitalWrite(PIN_Y, HIGH);
    // delayMicroseconds(2);
    digitalWrite(PIN_Y, LOW);
    if (point_index % (X_POINTS * Y_POINTS) == 0) {
      digitalWrite(PIN_FRAME, HIGH);
      delayMicroseconds(2);
      digitalWrite(PIN_FRAME, LOW);
      point_index = 0;
    }
  }
  point_index++;
}

unsigned long previousMillis = 0;
unsigned long interval = 10;  
void loop() {
  // digitalWrite(PIN_X, HIGH);
  // delayMicroseconds(2);
  // digitalWrite(PIN_X, LOW);
  // delayMicroseconds(2);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    changePin1State();
    previousMillis = currentMillis;
  }
}
