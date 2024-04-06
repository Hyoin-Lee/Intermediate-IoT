int previousSensorValue = HIGH;
unsigned long previousMillis = 0;
const unsigned long ledDuration = 300;

int sensorPin = 2;
int ledPin = 13;



void setup()
{
  Serial.begin(9600);
  pinMode(sensorPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  int sensorVal = digitalRead(sensorPin);

  if (sensorVal != previousSensorValue){
    Serial.println(sensorVal);
    previousSensorValue = sensorVal;
    digitalWrite(ledPin, HIGH);
    previousMillis = millis();
  }
        
  if (millis() - previousMillis >= ledDuration){
  	  digitalWrite(ledPin, LOW);  
  }
}
