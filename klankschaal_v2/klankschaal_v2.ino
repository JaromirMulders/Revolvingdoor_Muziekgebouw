// Define pin connections & motor's steps per revolution
const int dirPin = 6;
const int stepPin = 5;
const int hallPin = A0;
const int stepsPerRevolution = 200;


const int startMotorSpeed = 950;
const int minMotorSpeed = 750;
const int maxMotorSpeed = 300;

int stepCount = 0;
int hallCount = 0;

float motorAccum = 0.0;
float startAccum = 0.0;

int motorSpeed = minMotorSpeed;

int stepSwitch = 0;

unsigned long currentTime = 0;
unsigned long previousTime = 0;

unsigned long stopDelayTime = 0;
unsigned long prevSensorTime = 0;

int stepTreshold = 0;

unsigned long delta = 0;

bool speedSwitch = 0;

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

  if(millis() < 100){
      previousTime = millis();
      stopDelayTime = millis();  
      prevSensorTime=millis();  
  }

  
  digitalWrite(13,HIGH);
  
  int hallSensor = analogRead(A0);

  //if hallSensor senses magnet
  if(hallSensor < 500){


    
    if(stepSwitch == 0){  //switch on only once
      hallCount++;
      Serial.println("hallcount: " + (String)hallCount);
      
      stepCount++;
      stepCount = min(2,stepCount);
      motorSpeed = startMotorSpeed;
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

  currentTime = millis();


  if(hallCount > 1){//if two sensors have passed

    moveMotor();
     
    if(speedSwitch == 0){ //ramp speed up
      startAccum+=1;
      motorSpeed = startMotorSpeed - (int)startAccum;

      if(motorSpeed <= maxMotorSpeed){ //if maximum speed is reached
        speedSwitch = 1;
      }

      
    }else{ //ramp speed down

      //damping speed
      motorAccum+=0.05;//0.025;
      
      motorSpeed = maxMotorSpeed + (int)motorAccum;
      motorSpeed = constrain(motorSpeed,maxMotorSpeed,minMotorSpeed); //clip motor speed

    }
    
    //wait before counting back
    if(currentTime - stopDelayTime > 2000){
      countBack();
    }//if

    
  }else if (hallCount == 1){

    motorSpeed = startMotorSpeed;
    moveMotor();

    //wait before counting back
    if(currentTime - stopDelayTime > 2000){
      countBack();
    }//if
  
  
  }else {//reset
    stopDelayTime = currentTime;
    motorAccum=0.0;
    speedSwitch = 0;
  }
  

}

void moveMotor(){
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(motorSpeed);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(motorSpeed);
  
  //Serial.println(motorSpeed);
}


void countBack(){
 
  if(currentTime - previousTime > 1000){
    motorAccum+=0.015; //extra damping after delay

    stepCount--;
    stepCount = max(0,stepCount);
    if(stepCount == 0){
      hallCount = 0;
      startAccum = 0;
    }

    //Serial.println(stepCount);
    //Serial.println(motorSpeed);

    previousTime = currentTime;

  }//if


}
