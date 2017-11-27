#include <TinyGPS++.h>
#include <AltSoftSerial.h>
#include <EnableInterrupt.h>

static const uint32_t SERIAL_PORT_BAUD = 115200;
static const int RC_NUM_CHANNELS = 4;

static const int RC_CH1 = 0;
static const int RC_CH2 = 1;

static const int RC_CH1_INPUT = A1; //A12
static const int RC_CH2_INPUT = A2; //A13

unsigned long startTime;
unsigned long interval = 30000;

int GLOBAL_DUMP_VAR = 0;

static const int triGPS_RX_Pin = 34, echo_TX_Pin = 35;
static const uint32_t GPS_BAUD = 9600;
static const int LED_G_Pin = 5, HORN_PIN = 42, ONOFF_PIN = 43, MOTOR_PIN = 4;
static const int opSens = A3;

int signalAInput1;                             // signal input 1 for encoderA
int signalAInput2;                             // signal input 2 for encoderA
int signalBInput1;                             // signal input 1 for encoderB
int signalBInput2;                             // signal input 2 for encoderB



											   // robot location service
											   // 0 is up
											   // 1 is right
											   // 2 is down
											   // 3 is left
int robotDirection = 2;

// this is the coordinates in the grid of where the robot is
// it is also the x and y indexes in the array.
// remember that the array starts at index 0.
int xcoordinate = 2;
int ycoordinate = 1;

// ultrasonic pins
const int Trig_pin = 5;  // pin for triggering pulse    INPUT
const int Echo_pin = 6;   // pin for recieving echo      OUPUT
long duration;            // how long it takes for the sound to rebound


						  // motor pins
const int Motor1Pin = 9;  // Left side
const int Motor2Pin = 10; // right side

						  // the array that it tracks with
						  // this can be an array of any size
						  // just make sure that the robot has a free space to move to from its initial position.
int arraything[6][6] =
{
	{ 1,1,1,1,1,1 }
	,
	{ 1,1,0,1,0,1 }
	,
	{ 1,1,0,1,0,1 }
	,
	{ 1,1,0,1,0,1 }
	,
	{ 1,1,0,1,1,1 }
	,
	{ 1,1,1,0,1,1 }
};

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

TinyGPSPlus gps;

// Pins for AltSoftSerial
// TXPin = 46(7) RXPin = 48(8) UsuablePins = 44, 45
AltSoftSerial  altSerial;

unsigned long last = 0UL;

void setup() {

  Serial.println("Starting");
  // put your setup code here, to run once:
  pinMode(triGPS_RX_Pin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echo_TX_Pin, INPUT); // Sets the echoPin as an Input
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(opSens, INPUT);

  // Rotary encoder
  //pinMode(encoderAPin, INPUT);
  //pinMode(encoderBPin, INPUT);

  pinMode(LED_G_Pin, OUTPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  pinMode(HORN_PIN,INPUT_PULLUP);
  pinMode(ONOFF_PIN,INPUT_PULLUP);
  pinMode(HORN_PIN,OUTPUT);
  pinMode(ONOFF_PIN,OUTPUT);

  Serial.begin(SERIAL_PORT_BAUD);
  altSerial.begin(GPS_BAUD);

 //Serial.write("Debug mode");
}

void loop() {
//  int value_op = analogRead(opSens);
//  Serial.println(value_op);
  analogWrite(LED_G_Pin, 20);
  
  while (altSerial.available()>0)
    gps.encode(altSerial.read());

 //   debug();
    
    Serial.print(F("Sat"));
    printInt(gps.satellites.value(), gps.satellites.isValid(), 2);
    
    Serial.print(F("Lat"));
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    
    Serial.print(F("Lon"));
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);

    Serial.print(F("Spd"));
    printFloat(gps.speed.mph(), gps.speed.isValid(), 6, 2);

	

    Serial.print(F("Dst"));
    printInt(getFrontDistance(),true, 2);
    //float distance = getFrontDistance();
    //horn_controller(distance);
    
    rc_read_values();

    Serial.print("CH1"); Serial.print(rc_values[RC_CH1]); Serial.println("\t");
    Serial.print("CH3"); Serial.print(rc_values[RC_CH2]); Serial.println("\t");

    int RC1 = rc_values[RC_CH1];
    int RC2 = rc_values[RC_CH2];

    //motorControl( RC1, RC2);
    
// no less than 50 ms (as per JSN Ultrasonic sensor specification)
smartDelay(600);
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }

void debug()
{
    if (millis() > 5000 && gps.charsProcessed() < 10) // uh oh
    {
        Serial.println("ERROR: not getting any GPS data!");
        // dump the stream to Serial
        Serial.println("GPS stream dump:");
        while (true) // infinite loop
          if (altSerial.available() > 0) // any data coming in?
            Serial.write(altSerial.read());
    }
    else
    {
        Serial.print("Sentences that failed checksum=");
        Serial.println(gps.failedChecksum());
       
        // Testing overflow in SoftwareSerial is sometimes useful too.
        Serial.print("Soft Serial device overflowed? ");
        Serial.println(altSerial.overflow() ? "YES!" : "No");
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
		while (altSerial.available())
			gps.encode(altSerial.read());
	} while (millis() - start < ms);
}

static void printInt(unsigned long val, bool valid, int len)
{
	char sz[32] = "*****************";
	if (valid)
		sprintf(sz, "%ld", val);
	sz[len] = 0;
	for (int i = strlen(sz); i<len; ++i)
		sz[i] = ' ';
	if (len > 0)
		sz[len - 1] = ' ';
	Serial.println(sz);
	smartDelay(0);
}

static float printFloat(float val, bool valid, int len, int prec)
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
			for (int i = flen; i < len; ++i)
				Serial.print(' ');
		}
		return val;
	}

static void printStr(const char *str, int len)
	{
		int slen = strlen(str);
		for (int i = 0; i < len; ++i)
			Serial.print(i < slen ? str[i] : ' ');
		smartDelay(0);
	}

static float getFrontDistance()
{
	float duration, distanceCm;

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
	distanceCm = duration * 0.034 / 2;

	//Checks if see sensor value is within sensor range
  //  if (distance >= 600 || distance <= 18)
  //  {
  //     smartDelay(40);
  //     Serial.println('*');
  //  }
  //  else 
  //  {
  //    if(distance <= 60) 
  //    {
  //     horn_controller(distance);
  //    }
  //  }

	if (distanceCm < 600 || distanceCm > 18)
	{
		if (distanceCm <= 60)
		{
			horn_controller(distanceCm);
		}
		else
			Serial.println("***");
	}
	return distanceCm;
}

static void control() 
{
	while (1 == 1) {
		if (isFrontOpen() == true) 
		{
			moveForward();
			delay(2000);
		}
		else
			if (isRightOpen() == true)
			{
				turnRight();
				delay(2000);
			}
			else
				if (isLeftOpen() == true) 
				{
					turnLeft();
					delay(2000);
				}
				else {
					turnAround();
					delay(2000);
				}
	}
}

// Checks if there is something right in front of it using Grids
boolean isFrontOpen() {
	int nextNumber = getFrontNumber();
	if (nextNumber == 0) {
		return true;
	}
	else {
		return false;
	}
}

// Checks if there is something to the Right of it using Grids
boolean isRightOpen() {
	int nextNumber = getRightNumber();
	if (nextNumber == 0) {
		return true;
	}
	else {
		return false;
	}
}

// Checks if there is something to the Left of it using Grids
boolean isLeftOpen() {
	int nextNumber = getLeftNumber();
	if (nextNumber == 0) {
		return true;
	}
	else {
		return false;
	}
}

// Moves straight forward.
void moveForward() {
	//motor1.write(180);
	//motor2.write(0);

	Serial.println("Forward");
	if (robotDirection == 0)
		ycoordinate = ycoordinate - 1;
	if (robotDirection == 1)
		xcoordinate = xcoordinate + 1;
	if (robotDirection == 2)
		ycoordinate = ycoordinate + 1;
	if (robotDirection == 3)
		xcoordinate = xcoordinate - 1;
	delay(100);
	/*Serial.print("  xcoordinate " );
	Serial.print(xcoordinate);
	delay (500);
	Serial.print(" ycoordinate ");
	Serial.print(ycoordinate);
	delay (500);
	Serial.print("  robot direction: ");
	Serial.print(robotDirection);
	delay(500);
	Serial.println ();
	delay(1000);

	*/
	delay(800);
}

// Turns 90 degrees to the Right
void turnRight() {
	//motor1.write(60);
	//motor2.write(60);
	delay(178);
	//motor2.write(95);
	delay(65);
	//motor1.write(90);
	Serial.println("Right");
	if (robotDirection == 0)
		robotDirection = 1;
	else if (robotDirection == 1)
		robotDirection = 2;
	else if (robotDirection == 2)
		robotDirection = 3;
	else if (robotDirection == 3)
		robotDirection = 0;
	delay(500);
	Serial.print("  xcoordinate ");
	Serial.print(xcoordinate);
	delay(500);
	Serial.print(" ycoordinate ");
	Serial.print(ycoordinate);
	delay(500);
	Serial.print("  robot direction: ");
	Serial.print(robotDirection);
	delay(500);
	Serial.println();

	delay(1000);
}

// Turns 90 degrees to the Left
void turnLeft() {
	//motor1.write(120);
	//motor2.write(120);
	delay(325);
	//motor2.write(95);
	delay(65);
	//motor.write(90);
	Serial.println("Left");
	if (robotDirection == 0)
		robotDirection = 3;
	else if (robotDirection == 1)
		robotDirection = 0;
	else if (robotDirection == 2)
		robotDirection = 1;
	else if (robotDirection == 3)
		robotDirection = 2;
	delay(500);
	Serial.print("  xcoordinate ");
	Serial.print(xcoordinate);
	delay(500);
	Serial.print(" ycoordinate ");
	Serial.print(ycoordinate);
	delay(500);
	Serial.print("  robot direction: ");
	Serial.print(robotDirection);
	delay(500);
	Serial.println();
	delay(1000);
}

// Turns 180 degrees
void turnAround() {
	//  delay(1000);
	Serial.println("Around");
	if (robotDirection == 0)
		robotDirection = 2;
	else if (robotDirection == 1)
		robotDirection = 3;
	else if (robotDirection == 2)
		robotDirection = 0;
	else if (robotDirection == 3)
		robotDirection = 1;
	delay(500);
	Serial.print("  xcoordinate ");
	Serial.print(xcoordinate);
	delay(500);
	Serial.print(" ycoordinate ");
	Serial.print(ycoordinate);
	delay(500);
	Serial.print("  robot direction: ");
	Serial.print(robotDirection);
	delay(500);
	Serial.println();

	delay(1000);
}

// Gets the number on the Grid of the space right in front of it.
int getFrontNumber() {
	if (robotDirection == 0) {
		return arraything[ycoordinate - 1][xcoordinate];
	}
	if (robotDirection == 1) {
		return arraything[ycoordinate][xcoordinate + 1];
	}
	if (robotDirection == 2) {
		return arraything[ycoordinate + 1][xcoordinate];
	}
	if (robotDirection == 3) {
		return arraything[ycoordinate][xcoordinate - 1];
	}
}

// Gets the number on the Grid of the space to the Right of it.
int getRightNumber() {
	if (robotDirection == 0) {
		return arraything[ycoordinate][xcoordinate + 1];

	}
	if (robotDirection == 1) {
		return arraything[ycoordinate + 1][xcoordinate];

	}
	if (robotDirection == 2) {
		return arraything[ycoordinate][xcoordinate - 1];
	}
	if (robotDirection == 3) {
		return arraything[ycoordinate - 1][xcoordinate];
	}
}

// Gets the number on the Grid of the Space to the Left of it.
int getLeftNumber() {
	if (robotDirection == 0) {
		return arraything[ycoordinate][xcoordinate - 1];
	}
	if (robotDirection == 1) {
		return arraything[ycoordinate - 1][xcoordinate];
	}
	if (robotDirection == 2) {
		return arraything[ycoordinate][xcoordinate + 1];
	}
	if (robotDirection == 3) {
		return arraything[ycoordinate + 1][xcoordinate];
	}
}


