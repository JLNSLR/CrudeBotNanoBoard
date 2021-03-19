#ifndef ROBOTDATASTRUCTURES_H
#define ROBOTDATASTRUCTURES_H

#include "Arduino.h"
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

#define ENCODERPAKET_SIZE 3
struct encoderDataPacket
{
    uint8_t joint_id : 8;
    int16_t encoderValue : 16;
};

#define TORQUEPAKET_SIZE 5
struct torqueDataPacket
{
    uint8_t joint_id;
    int32_t torqueValue;
};

#define JOINTSTATE_PAKET_SIZE 8
struct jointSensorBoardState
{
    uint8_t joint_id;
    uint8_t torqueSensor_id;
    uint8_t torqueSensorGainVal;
    int32_t torqueSensorOffsetVal;
    uint8_t encoderError;
};
#define ENCODERCOMMAND_PAKET_SIZE 3
struct encoderZeroCommand
{
    uint8_t joint_id;
    int16_t zeroPosition;
};
#define TORQUESENSORCOMMAND_PAKET_SIZE 6
struct torqueSensorCommand
{
    uint8_t joint_id;
    uint8_t gain;
    int32_t offset;
};
#define LIGHTCOMMAND_PAKET_SIZE 5
#define LIGHTCOMMAND_OFF 0
#define LIGHTCOMMAND_RGB 1
#define LIGHTCOMMAND_HSV 2
struct lightCommand
{
    uint8_t joint_id;
    uint8_t mode;
    uint8_t value_0;
    uint8_t value_1;
    uint8_t value_2;
};

#define SENSORCONTROLLER_COMMAND_SIZE 2
#define SENSORBOARD_MODE_DEBUG_ON 0
#define SENSORBOARD_MODE_DEBUG_OFF 1
struct sensorControllerCommand
{
    uint8_t joint_id;
    uint8_t mode;
};

void serializeEncoderData(uint8_t *bytes, const encoderDataPacket *dataPaket);

void deSerializeEncoderData(const uint8_t *bytes, encoderDataPacket *dataPaket);

void serializeTorqueData(uint8_t *bytes, const torqueDataPacket *dataPaket);

void deSerializeTorqueData(const uint8_t *bytes, torqueDataPacket *dataPaket);

void serializeJointStateData(uint8_t *bytes, const jointSensorBoardState *dataPaket);

void deSerializeJointStateData(const uint8_t *bytes, jointSensorBoardState *dataPaket);

void serializeEncoderZeroCommand(uint8_t *bytes, const encoderZeroCommand *dataPaket);

void deSerializeEncoderZeroCommand(const uint8_t *bytes, encoderZeroCommand *dataPaket);

void serializeTorqueSensorCommand(uint8_t *bytes, const torqueSensorCommand *dataPaket);

void deSerializeTorqueSensorCommand(const uint8_t *bytes, torqueSensorCommand *dataPaket);

void serializeLightCommand(uint8_t *bytes, const lightCommand *dataPaket);

void deSerializeLightCommand(const uint8_t *bytes, lightCommand *dataPaket);

void serializeSensorControllerCommand(uint8_t *bytes, const sensorControllerCommand *dataPaket);

void deSerializeSensorControllerCommand(const uint8_t *bytes, sensorControllerCommand *dataPaket);

#endif //ROBOTDATASTRUCTURES