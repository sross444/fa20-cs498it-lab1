#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 8;
unsigned char rxBuf[8];
unsigned char data[8];
   
#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10


// FOR ULTRASONIC
const int pingPin = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 6; // Echo Pin of Ultrasonic Sensor
float cm_between_sensors = 2.50;
float cm_from_center_to_sensors = cm_between_sensors / 2;
long speed_of_sound_in_cm_per_sec = 34326;
int picN = 100;

// FOR LED
const int ledPin = 8;


void setup() {
  Serial.begin(9600);

  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin, 1);
  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {

  // ..... measure distance .....
  unsigned long duration;
  double distance;
  double sum = 0;
  int i = 0;
  while (i < picN) {
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pingPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    if(duration > 0 & duration < 32767){
      sum=sum+duration;
      i++;
    }
  }

  duration = sum / picN;
  distance = sqrt(pow(duration * speed_of_sound_in_cm_per_sec / 1000000 / 2,2)- pow(cm_from_center_to_sensors,2));
   //distance = duration /  29 / 2;
   //Serial.print("Measured Duration: ");
   //Serial.println(duration);
   //Serial.print("Measured Distance: ");
   //Serial.println(distance);

   // SEND DISTANCE
   //this will be cm to object to eight significant digits
   memcpy(&data, &distance, 8);
   byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, data);
   //Serial.print("Sent Distance");
  
  // RECEIVE IF MESSAGE AVAILABLE
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    unsigned char j[8];
    memcpy(j, rxBuf, 8);
    Serial.print("Received on CAN");
    //Serial.println(*j);
    //Serial.println(*rxBuf);
    if(*j == '1'){
      //Serial.print("Got 1");
      digitalWrite(ledPin, 1);
    } else {digitalWrite(ledPin, 0);}
  }
  delay(25);
}
