/******************************************************************************

  Ferriscope Control Unit firmware.

  Bull.Miletic
  Ferriscope
  1893-2020

  https://bull.miletic.info

  The Spectrum Shield code is based off of the original demo sketch by Toni Klopfenstein @SparkFun Electronics.
  This sketch is available in the Spectrum Shield repository.
  
  Tweak the value below based on what you see in Tools -> Serial monitor
*********************************************************************************/
int audioTriggerLimit = 200; // 110 = ferriscope, 200 = veneti


/******************************************************************************
 Do not change below here
*********************************************************************************/

//Declare Spectrum Shield pin connections
#define STROBE 4
#define RESET 5
#define DC_One A0
#define DC_Two A1

#define Signal_1 8
#define Signal_2 9
#define Signal_3 10
#define Signal_4 11
#define Signal_5 12

//Define spectrum variables
int freq_amp;
int Frequencies_One[7];
int Frequencies_Two[7];
int i;

/********************Setup Loop*************************/

void setup() {
  //Set spectrum Shield pin configurations
  pinMode(STROBE, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(DC_One, INPUT);
  pinMode(DC_Two, INPUT);
  pinMode(Signal_1, OUTPUT);
  pinMode(Signal_2, OUTPUT);
  pinMode(Signal_3, OUTPUT);
  pinMode(Signal_4, OUTPUT);
  pinMode(Signal_5, OUTPUT);

  //Initialize Spectrum Analyzers
  digitalWrite(STROBE, LOW);
  digitalWrite(RESET, LOW);
  delay(5);

  testLeds();

  Serial.begin(9600);
}

void testLeds()
{
  digitalWrite(Signal_1, HIGH);
  digitalWrite(Signal_2, LOW);
  digitalWrite(Signal_3, LOW);
  digitalWrite(Signal_4, LOW);
  digitalWrite(Signal_5, LOW);
  delay(500);
  digitalWrite(Signal_1, LOW);
  digitalWrite(Signal_2, HIGH);
  digitalWrite(Signal_3, LOW);
  digitalWrite(Signal_4, LOW);
  digitalWrite(Signal_5, LOW);
  delay(500);
  digitalWrite(Signal_1, LOW);
  digitalWrite(Signal_2, LOW);
  digitalWrite(Signal_3, HIGH);
  digitalWrite(Signal_4, LOW);
  digitalWrite(Signal_5, LOW);
  delay(500);
  digitalWrite(Signal_1, LOW);
  digitalWrite(Signal_2, LOW);
  digitalWrite(Signal_3, LOW);
  digitalWrite(Signal_4, HIGH);
  digitalWrite(Signal_5, LOW);
  delay(500);
  digitalWrite(Signal_1, LOW);
  digitalWrite(Signal_2, LOW);
  digitalWrite(Signal_3, LOW);
  digitalWrite(Signal_4, LOW);
  digitalWrite(Signal_5, HIGH);
  delay(500);
}


/**************************Main Function Loop*****************************/

void loop() {
  Read_Frequencies();
  // Graph_Frequencies();
 
  if (Frequencies_One[2] > audioTriggerLimit) {
    digitalWrite(Signal_1, HIGH);
    Serial.print("Signal_1 = HIGH  ");
    Serial.print(Frequencies_One[2]);
    Serial.print("\t");
  }
  else {
    digitalWrite(Signal_1, LOW);
    Serial.print("Signal_1 = LOW  ");
    Serial.print(Frequencies_One[2]);
    Serial.print("\t");
  }

  if (Frequencies_One[3] > audioTriggerLimit) { // 200
    digitalWrite(Signal_2, HIGH);
    Serial.print("Signal_2 = HIGH  ");
    Serial.print(Frequencies_One[3]);
    Serial.print("\t");
  }
  else {
    digitalWrite(Signal_2, LOW);
    Serial.print("Signal_2 = LOW  ");
    Serial.print(Frequencies_One[3]);
    Serial.print("\t");
  }

  if (Frequencies_One[4] > audioTriggerLimit) {
    digitalWrite(Signal_3, HIGH);
    Serial.print("Signal_3 = HIGH  ");
    Serial.print(Frequencies_One[4]);
    Serial.print("\t");
  }
  else {
    digitalWrite(Signal_3, LOW);
    Serial.print("Signal_3 = LOW  ");
    Serial.print(Frequencies_One[4]);
    Serial.print("\t");
  }

  if (Frequencies_One[5] > audioTriggerLimit) {
    digitalWrite(Signal_4, HIGH);
    Serial.print("Signal_4 = HIGH  ");
    Serial.print(Frequencies_One[5]);
    Serial.print("\t");
  }
  else {
    digitalWrite(Signal_4, LOW);
    Serial.print("Signal_4 = LOW  ");
    Serial.print(Frequencies_One[5]);
    Serial.print("\t");
  }

  if (Frequencies_One[6] > audioTriggerLimit) { // 200 110
    digitalWrite(Signal_5, HIGH);
    Serial.print("Signal_5 = HIGH  ");
    Serial.println(Frequencies_One[6]);
  }
  else {
    digitalWrite(Signal_5, LOW);
    Serial.print("Signal_5 = LOW  ");
    Serial.println(Frequencies_One[6]);
  }

}

/*******************Pull frquencies from Spectrum Shield********************/

void Read_Frequencies() {
  digitalWrite(RESET, HIGH);
  delayMicroseconds(200);
  digitalWrite(RESET, LOW);
  delayMicroseconds(200);

  //Read frequencies for each band
  for (freq_amp = 0; freq_amp < 7; freq_amp++)
  {
    digitalWrite(STROBE, HIGH);
    delayMicroseconds(50);
    digitalWrite(STROBE, LOW);
    delayMicroseconds(50);

    Frequencies_One[freq_amp] = analogRead(DC_One);
    Frequencies_Two[freq_amp] = analogRead(DC_Two);

  }

  //Serial.print("Frequencies_One:  ");
  //Serial.println(Frequencies_One[3]);

}

/*****************Print Out Band Values for Serial Plotter****************

  void Graph_Frequencies() {
  for (i = 0; i < 7; i++)
  {
  // Serial.print(Frequencies_One[i]);
  //  Serial.print(" ");
  //  Serial.print(Frequencies_Two[i]);
  //  Serial.print(" ");
  //  Serial.print( (Frequencies_One[i] + Frequencies_Two[i]) / 2 );
  //  Serial.print("    ");
  }
  Serial.println();
  }

*/
