int analogPin = 0;
int val = 0;
float V1 = 0.0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  val = analogRead(analogPin); // val: 0~1023
  V1 = convertToVoltage(val);  // V1: Output Voltage of CdS sensor
  Serial.print("ADC value: ");
  Serial.print(val);
  Serial.print(", Output voltage: ");
  Serial.print(V1, 2); 
  Serial.println(" V");
  delay(500);
}

float convertToVoltage(int adcValue) {
  float voltage = map(adcValue, 0, 1023, 0, 5.0);
  return voltage;
}
