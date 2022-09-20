#include <AccelStepper.h>

#define MOTOR_SPEED_PIN A0
#define MOTOR_STEP_PIN 2
#define MOTOR_SIGNAL_PIN 3
#define MOTOR_ANTICLOCKWISE_PIN 4
#define MOTOR_DISABLE_PIN 5
#define MOTOR_INDEX_PIN 7 // Input, signals shaft index

AccelStepper stepper(AccelStepper::FULL2WIRE, MOTOR_STEP_PIN, MOTOR_ANTICLOCKWISE_PIN);

int previousSignal = 0;
int currentSignal = 0;
int prevPot = 0;
int scaledSpeed = 0;

void setup() {
  //Serial.begin(9600);
  pinMode( MOTOR_SIGNAL_PIN, INPUT );
  pinMode( MOTOR_STEP_PIN, OUTPUT );
  pinMode( MOTOR_ANTICLOCKWISE_PIN, OUTPUT );
  pinMode( MOTOR_DISABLE_PIN, OUTPUT );
  pinMode( MOTOR_SPEED_PIN, INPUT );

  digitalWrite( MOTOR_STEP_PIN, LOW );
  digitalWrite( MOTOR_ANTICLOCKWISE_PIN, LOW );
  digitalWrite( MOTOR_DISABLE_PIN, LOW );

  stepper.setMaxSpeed(60 + 5); // adjust rotation speed here
  stepper.setAcceleration(100.0);
}

void loop() {
  int pot = analogRead(MOTOR_SPEED_PIN);
  if ( pot != prevPot )
  {
    scaledSpeed = map( pot, 0, 1023, 0, 10000 );
    stepper.setMaxSpeed(scaledSpeed + 1);
    /*
    Serial.print("*");
    Serial.print(pot);
    Serial.print(":");
    Serial.print(scaledSpeed + 5);
    Serial.println(" ");
    */
  }
  else
  {
    /*
      Serial.print(pot);
      Serial.print(":");
      Serial.print(prevPot);
      Serial.println(" ");
    */
  }
  prevPot = pot;

  currentSignal = digitalRead( MOTOR_SIGNAL_PIN );
  if ( previousSignal != currentSignal )
  {

    if ( currentSignal )
    {
      // start moving
      stepper.moveTo(stepper.currentPosition() + 1000000);
    }
    else
    {
      // stop
      stepper.stop();
    }
    previousSignal = currentSignal;
  }
  stepper.run();
}
