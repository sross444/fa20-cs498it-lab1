/*
 UDPSendReceiveString:
 This sketch receives UDP message strings, prints them to the serial port
 and sends an "acknowledge" string back to the sender
 A Processing sketch is included at the end of file that can be used to send
 and received messages for testing with a computer.
 created 21 Aug 2010
 by Michael Margolis
 This code is in the public domain.
 */

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02
};

IPAddress ip(192,168,86,248);
IPAddress dest(192,168,86,247);

unsigned int localPort = 8888;      // local port to listen on
unsigned int remotePort = 54321;

char buffer[4];

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
//unsigned char ReplyBuffer[8];        // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;


// ............... for CAN BUS .....................
#include <mcp_can.h>
long unsigned int rxId;
unsigned char len = 8;
unsigned char rxBuf[8];
unsigned char ledMsg[8];
#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10

byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
/*
  // start the Ethernet connection:
  Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // no point in carrying on, so do nothing forevermore:
    while (true) {
      delay(1);
    }
  }*/

  Ethernet.begin(mac, ip);
  // print your local IP address:
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());

  // start UDP
  Udp.begin(localPort);

  // start the CAN BUS:
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

  
}

void loop() {
  // let's first check to receive on CAN BUS
  // RECEIVE IF MESSAGE AVAILABLE
  float distance;
 
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    memcpy(&distance, (float*)rxBuf,4);
    memcpy(buffer, (float*)rxBuf,4);
    //Serial.print("Received distance Measurement: ");
    //Serial.println(distance);

    //Serial.print("Sending distance measurement......");
    Udp.beginPacket(dest, remotePort); //Initialize packet send
    Udp.write((uint8_t *)buffer, 4); //Send the temperature data
    //Serial.print("Wrote buffer");
    Udp.endPacket(); //End the packet
    //Serial.println("sent!");
  }

  
  // if there's data available, read it
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    //Serial.print("Received packet of size ");
    //Serial.println(packetSize);
    //Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i=0; i < 4; i++) {
      //Serial.print(remote[i], DEC);
      if (i < 3) {
        //Serial.print(".");
      }
    }
    //Serial.print(", port ");
    //Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    //Serial.println("Contents:");
    //Serial.println(packetBuffer);
    
    char z[3];
    memcpy(z, packetBuffer, 3);
    
    char i;
    if(z[1] == 'N'){
      //Serial.println("Light-Off");
      i = '1';
    } else {
      //Serial.println("Light-On");
      i = '0';
    }
    memcpy(data, &i, 1);
    CAN0.sendMsgBuf(0x200, 0, 8, data);
  }
  delay(25);
  memset(packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);
}
