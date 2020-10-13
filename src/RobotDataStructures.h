#ifndef ROBOTDATASTRUCTURES_H
#define ROBOTDATASTRUCTURES_H


#include "Arduino.h"


#define ENCODERPAKET_SIZE 3
struct encoderDataPacket{
    uint8_t joint_id :8;
    int16_t encoderValue:16;
};

#define TORQUEPAKET_SIZE 5
struct torqueDataPacket{
    uint8_t joint_id;
    int32_t torqueValue;
};

#define JOINTSTATE_PAKET_SIZE 8
struct jointSensorBoardState{
    uint8_t joint_id;
    uint8_t torqueSensor_id;
    uint8_t torqueSensorGainVal;
    int32_t torqueSensorOffsetVal;
    uint8_t encoderError;

};
#define ENCODERCOMMAND_PAKET_SIZE 3
struct encoderZeroCommand{
    uint8_t joint_id;
    int16_t zeroPosition;
};
#define TORQUESENSORCOMMAND_PAKET_SIZE 6
struct torqueSensorCommand{
    uint8_t joint_id;
    uint8_t gain;
    int32_t offset;
};
#define LIGHTCOMMAND_PAKET_SIZE 2
struct lightCommand{
    uint8_t joint_id;
    uint8_t mode;
};

void serializeEncoderData(uint8_t * bytes, const encoderDataPacket* dataPaket);

void deSerializeEncoderData(const uint8_t* bytes, encoderDataPacket* dataPaket);

void serializeTorqueData(uint8_t * bytes, const torqueDataPacket* dataPaket);

void deSerializeTorqueData(const uint8_t* bytes, torqueDataPacket* dataPaket);

void serializeJointStateData(uint8_t * bytes, const jointSensorBoardState* dataPaket);

void deSerializeJointStateData(const uint8_t* bytes, jointSensorBoardState* dataPaket);

void serializeEncoderZeroCommand(uint8_t * bytes, const encoderZeroCommand* dataPaket);

void deSerializeEncoderZeroCommand(const uint8_t* bytes, encoderZeroCommand* dataPaket);

void serializeTorqueSensorCommand(uint8_t * bytes, const torqueSensorCommand* dataPaket);

void deSerializeTorqueSensorCommand(const uint8_t* bytes, torqueSensorCommand* dataPaket);

void serializeLightCommand(uint8_t * bytes, const lightCommand* dataPaket);

void deSerializeLightCommand(const uint8_t* bytes, lightCommand* dataPaket);







#endif //ROBOTDATASTRUCTURES