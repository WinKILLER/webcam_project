int motorPin = 3;
int velocidad = 100;
void setup() 
{ 
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
  //while (! Serial);
  //Serial.println("Speed 0 to 255");
  
} 
 
 
void loop() 
{ 
  /*if (Serial.available())
  {
    int speed = Serial.parseInt();
    if (speed >= 0 && speed <= 255)
    {
      analogWrite(motorPin, speed);
    }
  }*/
  
  analogWrite(motorPin, velocidad);
  
  
} 
