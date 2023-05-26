#include <MsTimer2.h>

const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = LOW;
int count;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, FALLING); // 2번 pin falling edge일 때 blink 실행
  MsTimer2::set(500, timer_int); // timer interrupt : timer_int 함수를 500ms마다 호출한다
  MsTimer2::start();
  Serial.begin(115200);
}

void loop() {
  digitalWrite(LED_BUILTIN, state);
}

void blink() {
  state = !state;
  count++;
}

void timer_int() {
  Serial.println(count);
  count = 0;
}
