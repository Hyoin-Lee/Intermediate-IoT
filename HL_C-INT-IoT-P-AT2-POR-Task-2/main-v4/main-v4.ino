
void setup()
{
  Serial.begin(9600);
  
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);
}

void loop()
{
  int sensorVal = digitalRead(2);

  Serial.println(sensorVal);
  if (sensorVal == 0) {
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);  
  }
}
