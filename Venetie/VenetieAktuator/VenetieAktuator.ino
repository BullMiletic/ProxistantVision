#define MOTOR_SIGNAL_PIN 3
#define ACTUATOR_OUT_PIN 5
#define ACTUATOR_IN_PIN 4

int previousSignal = 0;
int currentSignal = 0;

void setup() {
  pinMode( MOTOR_SIGNAL_PIN, INPUT );
  pinMode( ACTUATOR_OUT_PIN, OUTPUT );
  pinMode( ACTUATOR_IN_PIN, OUTPUT );
  digitalWrite( ACTUATOR_OUT_PIN, LOW );
  digitalWrite( ACTUATOR_IN_PIN, LOW );
}

void loop() {
  currentSignal = digitalRead( MOTOR_SIGNAL_PIN );
  if ( previousSignal != currentSignal )
  {
    previousSignal = currentSignal;
    if ( currentSignal )
    {
      // push out
      digitalWrite( ACTUATOR_IN_PIN, LOW );
      delay(200);
      digitalWrite( ACTUATOR_OUT_PIN, HIGH );
    }
    else
    {
      // pull in
      digitalWrite( ACTUATOR_OUT_PIN, LOW );
      delay(200);
      digitalWrite( ACTUATOR_IN_PIN, HIGH );
    }
  }

}
