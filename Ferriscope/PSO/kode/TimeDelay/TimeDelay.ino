// Based on an Adafruit RP2040 board, but any
// Feather board can do this...

int delayBeforeStarting5V = 10000;
int transistorPin = 12;

void setup() {
  Serial.begin(115200);
  pinMode( transistorPin, OUTPUT );
  pinMode( LED_BUILTIN, OUTPUT );

  digitalWrite( transistorPin, HIGH );
  digitalWrite( LED_BUILTIN, HIGH );
  Serial.println("on");
  delay(5000);
  digitalWrite( transistorPin, LOW );
  digitalWrite( LED_BUILTIN, LOW );
  Serial.println("off");
  delay( 5000 );
  digitalWrite( transistorPin, HIGH );
  digitalWrite( LED_BUILTIN, HIGH );
  Serial.println("on");

  digitalWrite( transistorPin, HIGH );
  digitalWrite( LED_BUILTIN, HIGH );
  Serial.println("on");
  delay(5000);
  digitalWrite( transistorPin, LOW );
  digitalWrite( LED_BUILTIN, LOW );
  Serial.println("off");
  delay( 2000 );
  digitalWrite( transistorPin, HIGH );
  digitalWrite( LED_BUILTIN, HIGH );
  Serial.println("on");
}

void loop() {
  // put your main code here, to run repeatedly:

}
