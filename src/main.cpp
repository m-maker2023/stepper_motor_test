#include <Arduino.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>
//#include <AS5600.h>

#include <AccelStepper.h>

#define stepPin D2
#define dirPin D3

//buttons
#define right_pin D1 //s2
#define left_pin D0  //s3

volatile int stepCounter = 0;

boolean motorMove_step(int steps);
boolean motorMove_rev(float n_rev);
boolean motorMove_linearX(float distance);

#define GearRadius_mm 10
#define StepAngle 1.8
#define StepAngleFactor 1

#define motorFreq 2000 //move step every 1000us ,1000 Hz

#define timer1Tick 0.0000032 //in seconds, because prescaler divider is 256

#define motorTimer_init 1 / (motorFreq * timer1Tick) //motortimerInit * timerTick gives ISR calling time

#define N_rpm(freq) (StepAngle * StepAngleFactor * 60 / 360) * freq
#define n_rev(step) (StepAngle * StepAngleFactor * 60 / 360) * step

boolean motorMove_step(int steps)
{
  if (steps < 0)
  {
    steps = -steps;
    digitalWrite(dirPin, HIGH);
  }
  else
    digitalWrite(dirPin, LOW);

  if (stepCounter >= (2 * steps))
  {
    Serial.println("hi from stop");
    digitalWrite(stepPin, LOW);
    timer1_disable();
    stepCounter = 0;
    return true; //finished
  }              //stop timer
  else
  {
    if (stepCounter == 0)
    { //start timer
      Serial.println("hi from start");
      timer1_enable(TIM_DIV256, TIM_EDGE, TIM_SINGLE);
      timer1_write(motorTimer_init);
    }
    Serial.println("hi");
    
    return false; //still moving
  }
}

boolean motorMove_rev(float n_rev)
{
  return motorMove_step((int)(n_rev * 360 / (StepAngle * StepAngleFactor)));
}

void motor_buttons(void)
{
  bool move = 0;
  if(digitalRead(right_pin))return;
  if(digitalRead(left_pin)){move =1 ;digitalWrite(dirPin, LOW);}
  if(!digitalRead(right_pin)){move =1 ;digitalWrite(dirPin, HIGH);}
  while (digitalRead(left_pin)||(!digitalRead(right_pin)))
  {
    if (move)
    {
      timer1_enable(TIM_DIV256, TIM_EDGE, TIM_SINGLE);
      timer1_write(motorTimer_init);
      move =0 ;//
    }
    
   Serial.println("moving");
   
  }
  timer1_disable();
  stepCounter = 0;
  Serial.println("not moving");
}

void IRAM_ATTR motorMove_timer()
{
  stepCounter++;
  digitalWrite(stepPin, !(digitalRead(stepPin))); //pulsing
  timer1_write(motorTimer_init);
}

void setup()
{
  Serial.begin(9600);
  delay(10);
  // Sets the two pins as Outputs
  pinMode(right_pin, INPUT_PULLUP);
  pinMode(left_pin, INPUT_PULLDOWN_16);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);

  timer1_isr_init();
  timer1_attachInterrupt(motorMove_timer);
}

void loop()
{
  motor_buttons();
  /*
  if (motorMove_rev(3.5))
  {
    while (1)
    {
      Serial.println("finished!");
    }
  }
  */
}

/*
// Include the AccelStepper Library
#include <AccelStepper.h>


// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
	// set the maximum speed, acceleration factor,
	// initial speed and the target position
	myStepper.setMaxSpeed(1000);
	myStepper.setAcceleration(50);
	myStepper.setSpeed(1000);
	myStepper.moveTo(10000);
}

void loop() {
	// Change direction once the motor reaches target position
	if (myStepper.distanceToGo() == 0) 
		myStepper.moveTo(-myStepper.currentPosition());

	// Move the motor one step
	myStepper.run();
}
*/