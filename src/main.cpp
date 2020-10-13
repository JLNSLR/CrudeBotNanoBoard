/*  Code to run on the Arduino Nano PCB, which controls and collects 
    Joint Sensor Data and sends it via CAN-BUS to the main processor. 
    Main Function should be the same for every CAN-Board.
*/
#include <mcp_can.h>
#include <SPI.h>
#include <AS5048A.h>
#include <Wire.h>
#include "RobotDataStructures.h"
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802
#include <FastLED.h>
#include <EEPROM.h>

#include <avr/pgmspace.h>

/*--- Global Robot Definitions --- */

/*--- CAN-Network Definitions --- */
#define CAN_ID_ENCODERDATA 0x00 //CAN-ID to identify encoder-angle-data of tje joint
#define CAN_ID_TORQUESENSOR 0x01 //CAN-ID to identify torque-sensor-data of joint j-1;
#define CAN_ID_ENCODER_ERROR 0x02 // CAN_ID to identify Encoder Error-States
#define CAN_ID_STATUS_MSG 0x03 //CAN-ID to identify status-msgs from the joint-sensors
#define CAN_ID_STATUS_REQUEST 0x04
#define CAN_ID_LIGHT_COMMAND 0x05
#define CAN_ID_ENCODER_COMMAND 0x06 //CAN-ID to identify command
#define CAN_ID_TORQUESENSOR_COMMAND 0x07 //CAN-ID to identify torque sensor command
#define CAN_ID_DEBUG_ENABLE_COMMAND 0x08 //CAN-ID to identify debug enable command


/* --- Board Definitions --- */
#define JOINT_ID 5

#define  TORQUE_SENSOR_ID (JOINT_ID-1)
/* --- Hardware Definitions --- */
#define CAN_PIN 10
#define ENCODERPIN 9

#define LED_PIN 2 //HX_Data

#define NUM_LEDS 12

/*--- Global Data Structures --- */


CRGB leds[NUM_LEDS];

/*--- Torque Sensor width NAU7802-Amplifier and ADC --- */
NAU7802 torqueSensor; //Create instance of the NAU7802 class
int32_t torqueData;
int32_t torqueOffset;
torqueDataPacket torquePaket;
uint8_t torqueSensorGain = NAU7802_GAIN_4;


/*--- Magnetic rotary Encoder AS5048 --- */
int16_t encoderData; 
AS5048A encoder(ENCODERPIN);
encoderDataPacket encoderPaket;

int16_t encoderOffset = 0;
const int eeOffsetAddress = 0;


/*--- CAN Controller IC MCP2515 --- */
MCP_CAN CAN(CAN_PIN);        

jointSensorBoardState state;


/*--- Realtime Timing Parameters --- */

//Sample Periods
const long encoderPeriod PROGMEM =  3333; //2ms ==500Hz
const long torqueSensorPeriod PROGMEM = 3333; //3,33ms == 300Hz
const long ReadingCanInputsPeriod PROGMEM  = 200000; // 20ms = 50Hz

//Sample Period Tracking Variables
long encoderLastTime = 0;
long torqueLastTime = 0;
long readCANInputLastTime = 0;
long blinkTime = 0;


/*--- System state flags --- */
bool debug = true;
bool setup_error = false;
bool torqueSensorAvailable = true;
bool ledsAvailable = true;


/* --- Sensor Board functions --- */

void handleStatusRequest(unsigned char* msg_buffer, unsigned char len);
void handleLightCommand(unsigned char* msg_buffer, unsigned char len);
void handleEncoderZeroCommand(unsigned char* msg_buffer, unsigned char len);
void handleTorqueSensorCommand(unsigned char* msg_buffer, unsigned char len);

void ErrorLights();
void speedLights();

int16_t getEncoderOffset(){
  int16_t offset;
  EEPROM.get(eeOffsetAddress,offset);
  return offset;
};


void storeEncoderOffset(int16_t offset){
  EEPROM.put(eeOffsetAddress,offset);
};




void setup(){
    Serial.begin(115200);

    /* --- Initialize Sensors --- */
    encoder.init(); //Encoder

    Serial.println(F("Encoder initialized: "));
    encoder.printErrors();
    encoder.printState();

    //Torque Sensor
    Wire.begin();
    if(torqueSensorAvailable){
      if(torqueSensor.begin()){
        Serial.println(F("NAU7802 initialized"));
        torqueSensor.setSampleRate(320);
        torqueSensor.setGain(torqueSensorGain);
      }else{
          Serial.println(F("NAU7802 initialization failed"));
          torqueSensorAvailable = false;
      }
    }

      /* --- Initialize CAN Communication --- */
    for(int i = 0; i < 100; i++){
      if(CAN_OK == CAN.begin(CAN_1000KBPS, MCP_8MHz)){                // init can bus : baudrate = 1000k
        Serial.println(F("CAN BUS Shield init successful!"));

        //--- Set Masks --- //
        
        CAN.init_Mask(0,0,0x7FF); //all ID Bits are relevant
        CAN.init_Mask(1,0,0x7FF);

        //--- Set Filters --- //
        // we can receive CAN-Messages width IDs from 0x004 - 0x008
        CAN.init_Filt(0,0,0x004);
        CAN.init_Filt(1,0,0x005);
        CAN.init_Filt(2,0,0x006);
        CAN.init_Filt(3,0,0x007);
        CAN.init_Filt(4,0,0x008);
        

        break;
      }
      else
      {
        Serial.println(F("CAN BUS Shield init fail"));
        Serial.println(F("Init CAN BUS Shield again"));
        delay(5);
        if(i == 99){
          setup_error = true;
        }
      }
    }

  /* --- Initiate LED-RING --- */
  if(ledsAvailable){
    FastLED.addLeds<WS2812B,LED_PIN,GRB>(leds,NUM_LEDS);
	  FastLED.setBrightness(80);

    for(int i = 0; i<12; i++){
      leds[i] = CRGB::Orange;
    }
    FastLED.show();
      
  }


  if(setup_error){
 

    Serial.print(F("Initialization of Joint-Sensor-CAN-Board at Joint "));
    Serial.print(JOINT_ID);
    Serial.println(F(" failed"));
    }else{

    Serial.print(F("Initialization of Joint-Sensor-CAN-Board at Joint "));
    Serial.print(JOINT_ID);
    Serial.println(F(" was successful."));

  }

    // --- Initial State Paket Setup --- //
    state.encoderError = false;
    state.joint_id = JOINT_ID;
    state.torqueSensor_id = TORQUE_SENSOR_ID;
    state.torqueSensorGainVal = torqueSensorGain;
    state.torqueSensorOffsetVal = 0;




}

void loop()
{
  
  if(micros() - encoderLastTime >= encoderPeriod ){    
      //Read Value from Sensor and put it into the buffer
      encoderPaket.encoderValue = encoder.getRotation();

      //Serialize the encoderData for Sending
      uint8_t bytes[ENCODERPAKET_SIZE];
      serializeEncoderData(bytes,&encoderPaket);

      //Send Encoder Data via CAN BUS
      CAN.sendMsgBuf(CAN_ID_ENCODERDATA,0,ENCODERPAKET_SIZE,bytes);
      encoderLastTime = micros();

      if(encoder.error()){
        Serial.println(F("Encoder Error"));
        encoder.printErrors();
        encoder.printState();
        setup_error = true;
      }else{
        if(setup_error){
          setup_error = false;
        }
      }


    if(debug &&!setup_error){
      Serial.print(F("Reading Encoder Value: "));
      Serial.println(encoder.getRotation());
    }
  }

  

  if(micros() - torqueLastTime >= torqueSensorPeriod){

    int32_t torqueValue = torqueSensor.getReading();

    torquePaket.torqueValue = torqueValue;

    //Serialize the TorqueData for Sending
    uint8_t torque_bytes[TORQUEPAKET_SIZE];
    serializeTorqueData(torque_bytes,&torquePaket);

    //send Torquesensor Data via CAN BUS
    CAN.sendMsgBuf(CAN_ID_TORQUESENSOR,0,TORQUEPAKET_SIZE,torque_bytes);

    


  if(debug){
      Serial.print(F("Reading Torque Value: "));
      Serial.println(torqueValue);
    }

  }
  




if(micros() - readCANInputLastTime >= ReadingCanInputsPeriod ){   
  if (CAN_MSGAVAIL == CAN.checkReceive()) {  // check if data coming 
      unsigned char len = 0;
      unsigned char *msg_buffer;    
      //CAN.readMsgBuf(&len, msg_buffer); //Problem?!!  
      unsigned long canId = CAN.getCanId();
      switch(canId){
        case CAN_ID_STATUS_REQUEST:
          handleStatusRequest(msg_buffer,len);
          break;
        case CAN_ID_ENCODER_COMMAND:
          handleEncoderZeroCommand(msg_buffer,len);
          break;
        case CAN_ID_TORQUESENSOR_COMMAND:
          handleTorqueSensorCommand(msg_buffer,len);
          break;
        case CAN_ID_LIGHT_COMMAND:
          handleLightCommand(msg_buffer,len);
          break;
    }

    readCANInputLastTime = micros();
  }
}


if(setup_error){
  ErrorLights();
  Serial.println("Error");
}

}

void handleStatusRequest(unsigned char* msg_buffer, unsigned char len){
  
  if(msg_buffer[0] == JOINT_ID){
    uint8_t *bytes;
    serializeJointStateData(bytes ,&state);
    CAN.sendMsgBuf(CAN_ID_STATUS_MSG,0,JOINTSTATE_PAKET_SIZE,bytes);
  }else{
    return;
  }
};


void handleLightCommand(unsigned char* msg_buffer, unsigned char len){
  if(msg_buffer[0] == JOINT_ID){
    //Do something
  }else{
    return;
  }
};

void handleEncoderZeroCommand(unsigned char* msg_buffer, unsigned char len){
 if(msg_buffer[0] == JOINT_ID){
   encoderZeroCommand* encoderCommand;
   deSerializeEncoderZeroCommand(msg_buffer,encoderCommand);

   if(debug){

     Serial.print(F("Received Encoder Command for Joint Sensor "));
     Serial.println(encoderCommand->joint_id);
     Serial.print(F("Setting Encoder Zero Position to: "));
     Serial.println(encoderCommand->zeroPosition);
   }

   storeEncoderOffset(encoderCommand->zeroPosition);
 }else{
    return;
  }
}




void handleTorqueSensorCommand(unsigned char* msg_buffer, unsigned char len){
  if(msg_buffer[0] == JOINT_ID){
    torqueSensorCommand* torqueSensorCommand;
    deSerializeTorqueSensorCommand(msg_buffer,torqueSensorCommand);

    if(debug){
     Serial.print(F("Received Torque Sensor Command for Joint Sensor "));
     Serial.println(torqueSensorCommand->joint_id);
     Serial.print(F("Setting Torque Sensor Gain to to: "));
     Serial.println(torqueSensorCommand->gain);
     Serial.print(F("Setting Torque Offset to to: "));
     Serial.println(torqueSensorCommand->offset);

   }

    torqueSensor.setGain(torqueSensorCommand->gain);
    torqueSensorGain = torqueSensorCommand->gain;
    torqueOffset = torqueSensorCommand->offset;


  }else{
    return;
  }
}

void ErrorLights(){
  static bool toggled = false;
  static long blinkPeriod = 250;
  if(millis() - blinkTime > blinkPeriod){
    if(toggled){
      for(int i = 0; i<12; i++){
        FastLED.setBrightness(80);
        leds[i] = CRGB::Red;
        
      }
      FastLED.show();
      toggled = false;
    }else {
      for(int i = 0; i<12; i++){
        FastLED.setBrightness(50);
        leds[i] = CRGB::ForestGreen;
        
      }
      FastLED.show();
      toggled = true;
    }
    blinkTime = millis();
  }
  
}


