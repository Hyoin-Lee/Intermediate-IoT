int previousSensorValue = HIGH;


void setup()
{
  Serial.begin(9600);
  
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);
}

void loop()
{
  int sensorVal = digitalRead(2);

  if (sensorVal != previousSensorValue){
      Serial.println(sensorVal);
      previousSensorValue = sensorVal;
  }

  digitalWrite(13, !sensorVal);  
}
