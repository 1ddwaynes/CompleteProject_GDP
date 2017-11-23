#include <TinyGPS++.h>
#include <AltSoftSerial.h>
#include <EnableInterrupt.h>

static const int SERIAL_PORT_SPEED = 115200;
static const int RC_NUM_CHANNELS = 4;

static const int RC_CH1 = 0;
static const int RC_CH2 = 1;

static const int RC_CH1_INPUT = A12;
static const int RC_CH2_INPUT = A14;

unsigned long startTime;
unsigned long interval = 30000;

static const int GPS_RX_Pin = 52, GPS_TX_Pin = 53;
static const int triGPS_RX_Pin = 34, echo_TX_Pin = 35;
static const uint32_t GPSBaud = 9600;// SERIAL_PORT_SPEED = 57600;
static const int LED_G_Pin = 5, HORN_PIN = 42, ONOFF_PIN = 43, MOTOR_PIN = 4;
static const int opSens = A8;


uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

TinyGPSPlus gps;
AltSoftSerial  GPS_SS(GPS_RX_Pin, GPS_TX_Pin);

unsigned long last = 0UL;

void setup() {
  
  // put your setup code here, to run once:
  pinMode(triGPS_RX_Pin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echo_TX_Pin, INPUT); // Sets the echoPin as an Input
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(opSens, INPUT);

  pinMode(LED_G_Pin, OUTPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  pinMode(HORN_PIN,INPUT_PULLUP);
  pinMode(ONOFF_PIN,INPUT_PULLUP);
  pinMode(HORN_PIN,OUTPUT);
  pinMode(ONOFF_PIN,OUTPUT);
  Serial.begin(SERIAL_PORT_SPEED);
  GPS_SS.begin(GPSBaud);

 //Serial.write("Debug mode");
}

void loop() {
   int value_op = analogRead(opSens);
   Serial.println(value_op);
  //analogWrite(LED_G_Pin, 20);
  while (GPS_SS.available()>0)
    gps.encode(GPS_SS.read());

    debug();
    
    Serial.print(F("Sat"));
    printInt(gps.satellites.value(), gps.satellites.isValid(), 2);
    
    Serial.print(F("Lat"));
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    
    Serial.print(F("Lon"));
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    
    //printDistance();
    float distance = printDistance();
    horn_controller(distance);
    
    //rc_read_values();

    //Serial.print("CH1"); Serial.print(rc_values[RC_CH1]); Serial.println("\t");
    //Serial.print("CH3"); Serial.print(rc_values[RC_CH2]); Serial.println("\t");

    //int RC1 = rc_values[RC_CH1];
    //int RC2 = rc_values[RC_CH2];

    //motorControl( RC1, RC2);
 
//    digitalWrite(HORN_PIN, LOW);
//    digitalWrite(ONOFF_PIN, HIGH);
//    delay(300);
//    digitalWrite(HORN_PIN, HIGH);
//    digitalWrite(ONOFF_PIN, LOW);
    
// no less than 50 ms (as per JSN Ultrasonic sensor specification)
smartDelay(600);
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }

void debug(){
  if (millis() > 5000 && gps.charsProcessed() < 10) // uh oh
  {
      Serial.println("ERROR: not getting any GPS data!");
      // dump the stream to Serial
      Serial.println("GPS stream dump:");
      while (true) // infinite loop
        if (GPS_SS.available() > 0) // any data coming in?
          Serial.write(GPS_SS.read());
else{
      Serial.print("Sentences that failed checksum=");
      Serial.println(gps.failedChecksum());
 
// Testing overflow in SoftwareSerial is sometimes useful too.
Serial.print("Soft Serial device overflowed? ");
Serial.println(GPS_SS.overflow() ? "YES!" : "No");
}
  }
}

void horn_controller(float distance){
  unsigned long start = millis();
  
  if (start - startTime >= interval){
     digitalWrite(HORN_PIN, HIGH);
     smartDelay(100);
     digitalWrite(HORN_PIN, LOW);
     startTime = millis ();
  }
  
  else if ( distance < 60 && distance > 0) {
     digitalWrite(HORN_PIN, HIGH);
     smartDelay(500);
     digitalWrite(HORN_PIN, LOW);
  }
  else
  {
    digitalWrite(HORN_PIN, LOW);
  }
}

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_SS.available())
      gps.encode(GPS_SS.read());
      //printRC_Data();
      //printDistance();
      //digitalWrite(triGPS_RX_Pin, HIGH);
      //delayMicroseconds(10);
      //digitalWrite(triGPS_RX_Pin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
 
  //duration = pulseIn(echo_TX_Pin, HIGH);
  
  } while (millis() - start < ms);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  //Serial.println(sz);
  //smartDelay(0);
}

static float printFloat(float val, bool valid, int len,  int prec)
{
   if (!valid)
  {
    while (len-- > 1)
    Serial.print('*');
    Serial.println(' ');
  }
  else
  {
    Serial.println(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    //for (int i=flen; i<len; ++i)
     // Serial.print(' ');
  }
  return val;
}

static float printDistance()
{
  float duration, distance;
  
  // Clears the trigPin
  digitalWrite(triGPS_RX_Pin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(triGPS_RX_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triGPS_RX_Pin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  // Version 2.0 requires echo pin to be pulled up to VCC. 
  // A 4.7K  to 10K resistor can be used as pull-up resistor. (Uses 10k)
  duration = pulseIn(echo_TX_Pin, HIGH);
  
   // Calculating the distance (cm)
  distance = (duration/2)/29.1;
  
  //(duration / 1000000.0) * 17015;
  
  //(duration/2) / 29.1;
  
  if(distance <= 60) 
  {
     horn_controller(distance);
     //digitalWrite(LED_G_Pin,HIGH); // when Red LED turns ON, Green LED turns off
     
     //digitalWrite(ledGreenPin,LOW);
  }
  if (distance >= 600 || distance <= 0)
  {
     smartDelay(40);
     Serial.println('*');
     
  }
  else 
  {
    //Prints the distance in serial
    Serial.print("Dst");
    Serial.println(distance);
  }
  smartDelay(20);
}



