#include <AccelStepper.h>

#define MOTOR_STEP_PIN 2
#define MOTOR_SIGNAL_PIN 3
#define MOTOR_ANTICLOCKWISE_PIN 4
#define MOTOR_DISABLE_PIN 5
#define MOTOR_INDEX_PIN 7 // Input, signals shaft index

AccelStepper stepper(AccelStepper::FULL2WIRE, MOTOR_STEP_PIN, MOTOR_ANTICLOCKWISE_PIN);

int previousSignal = 0;
int currentSignal = 0;

void setup() {
  pinMode( MOTOR_SIGNAL_PIN, INPUT );
  pinMode( MOTOR_STEP_PIN, OUTPUT );
  pinMode( MOTOR_ANTICLOCKWISE_PIN, OUTPUT );
  pinMode( MOTOR_DISABLE_PIN, OUTPUT );

  digitalWrite( MOTOR_STEP_PIN, LOW );
  digitalWrite( MOTOR_ANTICLOCKWISE_PIN, LOW );
  digitalWrite( MOTOR_DISABLE_PIN, LOW );

  stepper.setMaxSpeed(400.0);
  stepper.setAcceleration(100.0);
}

void loop() {
  currentSignal = digitalRead( MOTOR_SIGNAL_PIN );
  if ( previousSignal != currentSignal )
  {
    previousSignal = currentSignal;
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
  }
  stepper.run();
}
