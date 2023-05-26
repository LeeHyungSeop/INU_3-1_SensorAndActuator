#define ECHOPIN 2
#define TRIGPIN 3

int analogPin = 0;
int val = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);
}

void loop() {
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  
  int distance = pulseIn(ECHOPIN, HIGH);
  distance = distance/58;  // cm 변환
  val = analogRead(analogPin);
  Serial.print(val);
  Serial.print(", ");
  Serial.println(distance);
  delay(50);
}
