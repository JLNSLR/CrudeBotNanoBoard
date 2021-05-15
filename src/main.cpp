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
#include <avr/wdt.h>
#include <avr/pgmspace.h>

/*--- Global Robot Definitions --- */

/*--- CAN-Network Definitions --- */
#define CAN_ID_ENCODERDATA 0x00   //CAN-ID to identify encoder-angle-data of tje joint
#define CAN_ID_TORQUESENSOR 0x01  //CAN-ID to identify torque-sensor-data of joint j-1;
#define CAN_ID_ENCODER_ERROR 0x02 // CAN_ID to identify Encoder Error-States
#define CAN_ID_STATUS_MSG 0x03    //CAN-ID to identify status-msgs from the joint-sensors
#define CAN_ID_STATUS_REQUEST 0x04
#define CAN_ID_LIGHT_COMMAND 0x05
#define CAN_ID_ENCODER_COMMAND 0x06          //CAN-ID to identify command
#define CAN_ID_TORQUESENSOR_COMMAND 0x07     //CAN-ID to identify torque sensor command
#define CAN_ID_SENSORCONTROLLER_COMMAND 0x08 //CAN-ID to identify sensor controller command

/* --- Board Definitions --- */
#define JOINT_ID 2
#define TORQUE_SENSOR_ID (JOINT_ID - 1)

/* --- Hardware Definitions --- */
#define CAN_PIN 10
#define ENCODERPIN 9
#define LED_PIN 2 //HX_data
#define NUM_LEDS 12

#define LED_BUILTIN 13

#define TORQUESENSOR_AVAILABLE
#define LEDS_AVAILABLE

/*--- Global Data Structures --- */

CRGB leds[NUM_LEDS];

#ifdef TORQUESENSOR_AVAILABLE
/*--- Torque Sensor width NAU7802-Amplifier and ADC --- */
NAU7802 torqueSensor; //Create instance of the NAU7802 class
int32_t torqueData;
int32_t torqueOffset;
torqueDataPacket torquePaket;
uint8_t torqueSensorGain = NAU7802_GAIN_32;
#endif //TORQUESENSOR_AVAILABLE

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
#define EMPIRIC_DELAY 900
const long encoderPeriod = 3333 - EMPIRIC_DELAY;      //3,33ms == 300Hz
const long torqueSensorPeriod = 3333 - EMPIRIC_DELAY; //3,33ms == 300Hz
const long ReadingCanInputsPeriod = 10000;            // 10ms = 100Hz

//Sample Period Tracking Variables
long encoderLastTime = 0;
long torqueLastTime = 0;
long readCANInputLastTime = 0;
long blinkTime = 0;
long blinkTime2 = 0;

/*--- System state flags --- */
bool debug = true;
bool setup_error = false;

/* --- Sensor Board functions --- */

void handleStatusRequest(unsigned char *msg_buffer, unsigned char len);
void handleLightCommand(unsigned char *msg_buffer, unsigned char len);
void handleEncoderZeroCommand(unsigned char *msg_buffer, unsigned char len);
void handleTorqueSensorCommand(unsigned char *msg_buffer, unsigned char len);
void handleSensorControllerCommand(unsigned char *msg_buffer, unsigned char len);
void ErrorLights();
void lightsOff();
void controlLightsRGB(uint8_t r, uint8_t g, uint8_t b);
void controlLightsHSV(uint8_t h, uint8_t s, uint8_t v);

int16_t getEncoderOffset()
{
  int16_t offset;
  EEPROM.get(eeOffsetAddress, offset);
  return offset;
};

void storeEncoderOffset(int16_t offset)
{
  EEPROM.put(eeOffsetAddress, offset);
};

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  /* --- Initialize Sensors --- */
  encoder.init(); //Encoder
  sei();

  Serial.println(F("Encoder initialized: "));
  encoder.printErrors();
  encoder.printState();

//Torque Sensor Initialization
#ifdef TORQUESENSOR_AVAILABLE
  Wire.begin();
  for (int i = 5; i++; i++)
  {
    if (torqueSensor.begin())
    {
      Serial.println(F("NAU7802 initialized"));
      torqueSensor.setSampleRate(320);
      torqueSensor.setGain(torqueSensorGain);
      torqueSensor.calibrateAFE();
      delay(10);
      break;
    }
    else
    {
      Serial.println(F("NAU7802 initialization failed"));
    }
  }
#endif //TORQUESENSOR_AVAILABLE

  /* --- Initialize CAN Communication --- */
  for (int i = 0; i < 10; i++)
  {
    if (CAN_OK == CAN.begin(CAN_1000KBPS, MCP_8MHz))
    { // init can bus : baudrate = 1000k
      Serial.println(F("CAN BUS Shield init successful!"));

      //--- Set Masks --- //
      CAN.init_Mask(0, 0, 0x7FF); //all ID Bits are relevant
      CAN.init_Mask(1, 0, 0x7FF);

      //--- Set Filters --- //
      // we can receive CAN-Messages width IDs from 0x004 - 0x008
      CAN.init_Filt(0, 0, 0x004);
      CAN.init_Filt(1, 0, 0x005);
      CAN.init_Filt(2, 0, 0x006);
      CAN.init_Filt(3, 0, 0x007);
      CAN.init_Filt(4, 0, 0x008);
      CAN.init_Filt(5, 0, 0x004);

      break;
    }
    else
    {
      Serial.println(F("CAN BUS Shield init fail"));
      Serial.println(F("Init CAN BUS Shield again"));
      if (i == 9)
      {
        setup_error = true;
      }
    }
  }

/* --- Initiate LED-RING --- */
#ifdef LEDS_AVAILABLE
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(80);

  for (int i = 0; i < 12; i++)
  {
    leds[i] = CRGB::Orange;
  }
  FastLED.show();
#endif //LEDS_AVAILABLE

  if (setup_error)
  {

    Serial.print(F("Initialization of Joint-Sensor-CAN-Board at Joint "));
    Serial.print(JOINT_ID);
    Serial.println(F(" failed"));
  }
  else
  {

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

  encoderPaket.joint_id = JOINT_ID;
  torquePaket.joint_id = TORQUE_SENSOR_ID;

  wdt_enable(WDTO_30MS); //start watchdog Timer, 30ms
}

void loop()
{

  if (micros() - encoderLastTime >= encoderPeriod)
  {
    //Read Value from Sensor and put it into the buffer
    encoderPaket.encoderValue = encoder.getRotation();

    //Serialize the encoderData for Sending
    uint8_t bytes[ENCODERPAKET_SIZE];
    serializeEncoderData(bytes, &encoderPaket);

    //Send Encoder Data via CAN BUS
    CAN.sendMsgBuf(CAN_ID_ENCODERDATA, 0, ENCODERPAKET_SIZE, bytes);
    encoderLastTime = micros();

    if (encoder.error())
    {
      Serial.println(F("Encoder Error"));
      encoder.printErrors();
      encoder.printState();
      setup_error = true;
    }
    else
    {
      if (setup_error)
      {
        setup_error = false;
      }
    }

    if (debug && !setup_error)
    {
      Serial.print(F("Reading Encoder Value: "));
      Serial.println(encoderPaket.encoderValue);
    }
  }

#ifdef TORQUESENSOR_AVAILABLE
  if (micros() - torqueLastTime >= torqueSensorPeriod)
  {

    if (torqueSensor.available())
    {
      int32_t torqueValue = torqueSensor.getReading();

      if (torqueValue == 0)
      {
        torqueSensor.begin();
        torqueSensor.setSampleRate(320);
        torqueSensor.setGain(torqueSensorGain);
        torqueSensor.calibrateAFE();
      }
      else
      {
        torquePaket.torqueValue = torqueValue;

        //Serialize the TorqueData for Sending
        uint8_t torque_bytes[TORQUEPAKET_SIZE];
        serializeTorqueData(torque_bytes, &torquePaket);

        //send Torquesensor Data via CAN BUS
        CAN.sendMsgBuf(CAN_ID_TORQUESENSOR, 0, TORQUEPAKET_SIZE, torque_bytes);
        torqueLastTime = micros();

        if (debug)
        {
          Serial.print(F("Reading Torque Value: "));
          Serial.println(torqueValue);
        }
      }
    }
  }
#endif //TORQUESENSOR_AVAILABLE

  if (micros() - readCANInputLastTime >= ReadingCanInputsPeriod)
  {
    if (CAN_MSGAVAIL == CAN.checkReceive())
    {

      // check if data coming
      unsigned char len = 0;
      unsigned char msg_buffer[8];
      CAN.readMsgBuf(&len, msg_buffer);
      unsigned long canId = CAN.getCanId();
      switch (canId)
      {
      case CAN_ID_STATUS_REQUEST:
        handleStatusRequest(msg_buffer, len);
        break;
      case CAN_ID_ENCODER_COMMAND:
        handleEncoderZeroCommand(msg_buffer, len);
        break;
#ifdef TORQUESENSOR_AVAILABLE
      case CAN_ID_TORQUESENSOR_COMMAND:
        handleTorqueSensorCommand(msg_buffer, len);
        break;
#endif // TORQUESENSOR_AVAILABLE

#ifdef LEDS_AVAILABLE
      case CAN_ID_LIGHT_COMMAND:
        handleLightCommand(msg_buffer, len);
        break;
#endif //LEDS_AVAILABLE

      case CAN_ID_SENSORCONTROLLER_COMMAND:
        handleSensorControllerCommand(msg_buffer, len);
        break;
      }

      readCANInputLastTime = micros();
    }
  }

  wdt_reset();
  CAN.clearBufferTransmitIfFlags();

#ifdef LEDS_AVAILABLE
  if (setup_error)
  {
    ErrorLights();
  }
#endif //LEDS_AVAILABLE
}

/* --- Function Implementations --- */

void handleStatusRequest(unsigned char *msg_buffer, unsigned char len)
{

  if (msg_buffer[0] == JOINT_ID)
  {
    uint8_t *bytes;
    serializeJointStateData(bytes, &state);
    CAN.sendMsgBuf(CAN_ID_STATUS_MSG, 0, JOINTSTATE_PAKET_SIZE, bytes);
  }
  else
  {
    return;
  }
};

void handleLightCommand(unsigned char *msg_buffer, unsigned char len)
{
  if (msg_buffer[0] == JOINT_ID)
  {
    lightCommand *lightCommand;
    deSerializeLightCommand(msg_buffer, lightCommand);

#define LIGHTMODE_OFF 0
#define LIGHTMODE_SET_RGB 1
#define LIGHTMODE_SET_HSV 2

    switch (lightCommand->mode)
    {
    case LIGHTMODE_OFF:
      lightsOff();
      break;
    case LIGHTMODE_SET_RGB:
      controlLightsRGB(lightCommand->value_0, lightCommand->value_1, lightCommand->value_2);
      break;
    case LIGHTMODE_SET_HSV:
      controlLightsHSV(lightCommand->value_0, lightCommand->value_1, lightCommand->value_2);
      break;

    default:
      break;
    }
  }
  else
  {
    return;
  }
};

void handleEncoderZeroCommand(unsigned char *msg_buffer, unsigned char len)
{
  if (msg_buffer[0] == JOINT_ID)
  {
    encoderZeroCommand *encoderCommand;
    deSerializeEncoderZeroCommand(msg_buffer, encoderCommand);

    if (debug)
    {
      Serial.print(F("Setting Encoder Zero Position to: "));
      Serial.println(encoderCommand->zeroPosition);
    }

    storeEncoderOffset(encoderCommand->zeroPosition);
  }
  else
  {
    return;
  }
}

void handleTorqueSensorCommand(unsigned char *msg_buffer, unsigned char len)
{
  if (msg_buffer[0] == JOINT_ID)
  {
    torqueSensorCommand *torqueSensorCommand;
    deSerializeTorqueSensorCommand(msg_buffer, torqueSensorCommand);

    if (debug)
    {
      Serial.print(F("Setting Torque Sensor Gain to: "));
      Serial.println(torqueSensorCommand->gain);
      Serial.print(F("Setting Torque Offset to: "));
      Serial.println(torqueSensorCommand->offset);
    }

    torqueSensor.setGain(torqueSensorCommand->gain);
    torqueSensorGain = torqueSensorCommand->gain;
    torqueOffset = torqueSensorCommand->offset;
  }
  else
  {
    return;
  }
}

void handleSensorControllerCommand(unsigned char *msg_buffer, unsigned char len)
{
  if (msg_buffer[0] == JOINT_ID)
  {
    sensorControllerCommand *sensorCommand;
    deSerializeSensorControllerCommand(msg_buffer, sensorCommand);

    switch (sensorCommand->mode)
    {
    case SENSORBOARD_MODE_DEBUG_ON:
      debug = true;
      break;
    case SENSORBOARD_MODE_DEBUG_OFF:
      if (debug)
      {
        debug = false;
        Serial.println(F("turning debug off"));
      }
      break;
    case SENSORBOARD_MODE_RESET_CALL:
      delay(100);
      break;
    }
  }
  else
  {
    return;
  }
}

#ifdef LEDS_AVAILABLE
void ErrorLights()
{
  static bool toggled = false;
  static long blinkPeriod = 250;
  if (millis() - blinkTime > blinkPeriod)
  {
    if (toggled)
    {
      for (int i = 0; i < 12; i++)
      {
        FastLED.setBrightness(80);
        leds[i] = CRGB::Red;
      }
      FastLED.show();
      toggled = false;
    }
    else
    {
      for (int i = 0; i < 12; i++)
      {
        FastLED.setBrightness(50);
        leds[i] = CRGB::ForestGreen;
      }
      FastLED.show();
      toggled = true;
    }
    blinkTime = millis();
  }
}

void lightsOff()
{
  FastLED.setBrightness(0);

  for (int i = 0; i < 12; i++)
  {
    leds[i].r = CRGB::Black;
  }
  FastLED.show();
}

void controlLightsRGB(uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = 0; i < 12; i++)
  {
    leds[i].setRGB(r, g, b);
  }
  FastLED.show();
}
void controlLightsHSV(uint8_t h, uint8_t s, uint8_t v)
{
  for (int i = 0; i < 12; i++)
  {
    leds[i].setHSV(h, s, v);
  }
  FastLED.show();
}

#endif //LEDS_AVAILABLE
