// Define pin connections & motor's steps per revolution
const int dirPin = 2;
const int stepPin = 3;
const int hallPin = A0;
const int stepsPerRevolution = 200;

const int minMotorSpeed = 750;
const int maxMotorSpeed = 125;
const int speedStep = 25;

int stepCount = 0;
int hallCount = 0;

float motorAccum = 0.0;

int motorSpeed = minMotorSpeed;

int stepSwitch = 0;

unsigned long currentTime = 0;
unsigned long previousTime = 0;

unsigned long stopDelayTime = 0;
unsigned long prevSensorTime = 0;

int stepTreshold = 0;

unsigned long delta = 0;

void setup()
{
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(13,OUTPUT);

  digitalWrite(dirPin, HIGH);

  Serial.begin(9600);
  
}
void loop()
{
  
  digitalWrite(13,HIGH);
  
  int hallSensor = analogRead(A0);

  //if hallSensor senses magnet
  if(hallSensor < 1000){
    
    if(stepSwitch == 0){  //switch on only once
      hallCount++;
      Serial.println(hallCount);
      
      stepCount++;
      stepCount = min(2,stepCount);
      motorSpeed = maxMotorSpeed;
      previousTime = millis();
      stopDelayTime = millis();

      delta = millis()-prevSensorTime;
      //Serial.println(delta);
      prevSensorTime=millis();

      stepSwitch = 1; // switch

    }//if
    
  }else{
    stepSwitch = 0;
  }

  if(hallCount > 1){//if two sensors have passed

    currentTime = millis();

    moveMotor();

    //damping speed
    motorAccum+=0.02;
    
    motorSpeed = maxMotorSpeed + (int)motorAccum;
    motorSpeed = constrain(motorSpeed,maxMotorSpeed,minMotorSpeed); //clip motor speed
    
    //wait before counting back
    if(currentTime - stopDelayTime > 2000){
      countBack();
    }//if
    
  }else{//reset
    stopDelayTime = currentTime;
    motorAccum=0.0;
  }
  
  //Serial.println(stepCount);

}

void moveMotor(){
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(motorSpeed);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(motorSpeed);
}


void countBack(){
 
  if(currentTime - previousTime > 500){
    motorAccum+=0.05; //extra damping after delay

    stepCount--;
    stepCount = max(0,stepCount);
    if(stepCount == 0){
      hallCount = 0;
    }

    //Serial.println(stepCount);
    //Serial.println(motorSpeed);

    previousTime = currentTime;

  }//if


}
