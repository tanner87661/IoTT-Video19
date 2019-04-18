#include <NmraDcc.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Adafruit_MCP23017.h"

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

#define THROWN 1
#define CLOSED 0

#define SERVO 0
#define RELAY 1

//#define measureMode

#ifdef measureMode
  int loopCounter;
  uint32_t loopTimer;
#endif

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MCP23017 mcp;

NmraDcc  Dcc ;

const int DccAckPin = 3 ; //not connected
//int firstServo = 21; //no longer used
#define numServos 24

typedef struct {
  uint16_t dccAddr;
  uint8_t  driveType;
  uint16_t portA;
  uint16_t portB;
  uint16_t  minPos;
  uint16_t  maxPos;
  uint16_t  targetPos;
  uint16_t  currPos;
  uint16_t  moveDelayCL;
  uint16_t  moveDelayTH;
  uint32_t nextMove;
} myServo;

myServo servoArray[numServos] = {
                            {24, SERVO, 0, 0, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,10,0, 0},
                            {25, SERVO, 1, 1, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            {50, SERVO, 2, 2, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,10, 0},
                            {51, SERVO, 3, 3, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            
                            {10, SERVO, 4, 4, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            {11, SERVO, 5, 5, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            {16, SERVO, 6, 6,SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            {17, SERVO, 7, 7, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            
                            {18, SERVO, 8, 8, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            {23, SERVO, 9, 9, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            {33, SERVO, 10, 10, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            {34, SERVO, 11, 11, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            
                            {10800, SERVO, 12, 12, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            {10802, SERVO, 13, 13, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            {10804, SERVO, 14, 14, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},
                            {10806, SERVO, 15, 15, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0,0},

                            {61, RELAY, 0, 1, CLOSED, THROWN,THROWN, CLOSED,0,0,0},
                            {62, RELAY, 2, 3, CLOSED, THROWN,THROWN, CLOSED,4000,2000,0},
                            {63, RELAY, 4, 5, CLOSED, THROWN,THROWN, CLOSED,400,400,0},
                            {64, RELAY, 6, 7, CLOSED, THROWN,THROWN, CLOSED,200,200,0},

                            {65, RELAY, 8, 9, CLOSED, THROWN,THROWN, CLOSED,0,0,0},
                            {66, RELAY, 10, 11, CLOSED, THROWN,THROWN, CLOSED,0,0,0},
                            {67, RELAY, 12, 13, CLOSED, THROWN,THROWN, CLOSED,0,0,0},
                            {68, RELAY, 14, 15, CLOSED, THROWN,THROWN, CLOSED,0,0,0},
                          };

// This function is called whenever a normal DCC Turnout Packet is received and we're in Output Addressing Mode
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
  Serial.print("notifyDccAccTurnoutOutput: ") ;
  Serial.print(Addr,DEC) ;
  Serial.print(',');
  Serial.print(Direction,DEC) ;
  Serial.print(',');
  Serial.println(OutputPower, HEX) ;
  for (int i=0; i < numServos; i++)
    if (Addr == servoArray[i].dccAddr)
    {
      if (Direction == 0)
        servoArray[i].targetPos = (servoArray[i].driveType==SERVO) ? servoArray[i].minPos: CLOSED;
      else
        servoArray[i].targetPos = (servoArray[i].driveType==SERVO) ? servoArray[i].maxPos: THROWN;
      if (servoArray[i].driveType==RELAY)
        servoArray[i].currPos = (servoArray[i].targetPos == THROWN) ? CLOSED:THROWN;
    }
}

// This function is called whenever a DCC Signal Aspect Packet is received
void notifyDccSigOutputState( uint16_t Addr, uint8_t State)
{
  Addr += 10000;
  Serial.print("notifyDccSigOutputState: ") ;
  Serial.print(Addr,DEC) ;
  Serial.print(',');
  Serial.println(State, HEX) ;
  for (int i=0; i < numServos; i++)
    if ((Addr == servoArray[i].dccAddr) && (servoArray[i].driveType == SERVO))
      servoArray[i].targetPos = min(servoArray[i].maxPos, servoArray[i].minPos +(State * ((servoArray[i].maxPos - servoArray[i].minPos)/8)));
}

void processServo(myServo * thisServo)
{
  if ((thisServo->nextMove < millis()))
  {
    if (thisServo->targetPos > thisServo->currPos)
    {
      thisServo->currPos++;
      pwm.setPWM(thisServo->portA, 0, thisServo->currPos);
      thisServo->nextMove = millis() + thisServo->moveDelayCL; 
    }
    else
    {
      thisServo->currPos--;
      pwm.setPWM(thisServo->portA, 0, thisServo->currPos);
      thisServo->nextMove = millis() + thisServo->moveDelayTH; 
    }
  }
}

void processRelay(myServo * thisServo)
{
  switch (thisServo->currPos)
  {
    case 0:;
    case 1: 
    {
      mcp.digitalWrite(thisServo->portA,thisServo->targetPos==THROWN);
      mcp.digitalWrite(thisServo->portB,thisServo->targetPos==CLOSED);
      thisServo->currPos = 0xFFFF;
      thisServo->nextMove = millis() + ((thisServo->targetPos==THROWN) ? thisServo->moveDelayTH : thisServo->moveDelayCL); 
      break;
    }
    default:
      if ((thisServo->nextMove < millis()) && ((thisServo->moveDelayTH != 0) || (thisServo->moveDelayCL != 0)))
      {
        mcp.digitalWrite(thisServo->portA,1);
        mcp.digitalWrite(thisServo->portB,1);
        thisServo->currPos = thisServo->targetPos;
      }
      break;
  }
}

void processLocations()
{
  for (int i=0; i < numServos; i++)
    if (servoArray[i].targetPos != servoArray[i].currPos) 
    {
      switch (servoArray[i].driveType)
      {
        case SERVO: processServo(&servoArray[i]); break;
        case RELAY: processRelay(&servoArray[i]); break;
      }
    }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Configure the DCC CV Programing ACK pin for an output
  pinMode( DccAckPin, OUTPUT );

  Serial.println("NMRA DCC Example 1");
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, 2, 1);
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );


  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  mcp.begin();      // use default address 0
  for (int i=0; i<16; i++)
  {
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i,HIGH);
  }
  Serial.println("Init Port Expander Done");

}

void loop() {
  // put your main code here, to run repeatedly:
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  processLocations();

#ifdef measureMode
  if (loopTimer < millis())
  {
    Serial.println(loopCounter);
    loopCounter = 0;
    loopTimer = millis() + 1000;
  }
  loopCounter++;
#endif

  
}
