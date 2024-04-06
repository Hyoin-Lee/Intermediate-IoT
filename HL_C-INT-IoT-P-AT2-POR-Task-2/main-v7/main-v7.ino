int previousSensorValue = HIGH;
unsigned long previousMillis = 0;
const unsigned long ledDuration = 300;

int PIRSensorPin = 2;
int ledPin = 13;



void setup()
{
  Serial.begin(9600);
  pinMode(PIRSensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  int sensorVal = digitalRead(PIRSensorPin);

  if (sensorVal != previousSensorValue){
    if (sensorVal == HIGH){
      
      Serial.println("Motion detected");
      digitalWrite(ledPin, HIGH);
    }
    else{
      digitalWrite(ledPin, LOW);
    }
    previousSensorValue = sensorVal;
    previousMillis = millis();
        
  if (millis() - previousMillis >= ledDuration){
  	  digitalWrite(ledPin, LOW);  
  }
}
}
