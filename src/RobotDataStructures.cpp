#include "RobotDataStructures.h"

void serializeEncoderData(uint8_t * bytes, const encoderDataPacket* dataPaket){
    bytes[0] = dataPaket->joint_id;
    bytes[1] = dataPaket->encoderValue & 0xFF;
    bytes[2] = (dataPaket->encoderValue >> 8) &0xFF;
}

void deSerializeEncoderData(const uint8_t* bytes, encoderDataPacket* dataPaket){
    dataPaket->joint_id = bytes[0];
    dataPaket->encoderValue = ((uint16_t) bytes[1]) | (((uint16_t) bytes[2]) << 8);
}

void serializeTorqueData(uint8_t * bytes, const torqueDataPacket* dataPaket){
    bytes[0] = dataPaket->joint_id;
    bytes[1] = dataPaket->torqueValue & 0xFF;
    bytes[2] = (dataPaket->torqueValue >> 8) &0xFF;
    bytes[3] = (dataPaket->torqueValue >> 16) &0xFF;
    bytes[4] = (dataPaket->torqueValue >> 24) &0xFF;
}

void deSerializeTorqueData(const uint8_t* bytes, torqueDataPacket* dataPaket){
    dataPaket->joint_id = bytes[0];
    dataPaket->torqueValue = ((int32_t) bytes[1]) | (((int32_t) bytes[2]) << 8) | (((int32_t) bytes[3]) << 16) | (((int32_t) bytes[4]) << 24 );
}

void serializeJointStateData(uint8_t * bytes, const jointSensorBoardState* dataPaket){
    bytes[0] = dataPaket->joint_id;
    bytes[1] = dataPaket->torqueSensor_id;
    bytes[2] = dataPaket->torqueSensorGainVal;
    bytes[3] = dataPaket->torqueSensorOffsetVal & 0xFF;
    bytes[4] = (dataPaket->torqueSensorGainVal >> 8) &0xFF;
    bytes[5] = (dataPaket->torqueSensorGainVal >> 16) &0xFF;
    bytes[6] = (dataPaket->torqueSensorGainVal >> 24) &0xFF;
    bytes[7] = dataPaket->encoderError;
}

void deSerializeJointStateData(const uint8_t* bytes, jointSensorBoardState* dataPaket){
    dataPaket->joint_id = bytes[0];
    dataPaket->torqueSensor_id = bytes[1];
    dataPaket->torqueSensorGainVal = bytes[2];
    dataPaket->torqueSensorOffsetVal = ((int32_t) bytes[3]) | (((int32_t) bytes[4]) << 8) | (((int32_t) bytes[5]) << 16) | (((int32_t) bytes[6]) << 24 );
    dataPaket->encoderError = bytes[7];
}

void serializeEncoderZeroCommand(uint8_t * bytes, const encoderZeroCommand* dataPaket){
    bytes[0] = dataPaket->joint_id;
    bytes[1] = dataPaket->zeroPosition &0xFF;
    bytes[2] = (dataPaket->zeroPosition >> 8) &0xFF;
}

void deSerializeEncoderZeroCommand(const uint8_t* bytes, encoderZeroCommand* dataPaket){
    dataPaket->joint_id = bytes[0];
    dataPaket->zeroPosition = ((uint16_t) bytes[2]) | (((uint16_t) bytes[3]) << 8);
}


void serializeTorqueSensorCommand(uint8_t * bytes, const torqueSensorCommand* dataPaket){
    bytes[0] = dataPaket->joint_id;
    bytes[1] = dataPaket->gain;
    bytes[2] = dataPaket->offset &0xFF;
    bytes[3] = (dataPaket->offset >> 8) &0xFF;
    bytes[4] = (dataPaket->offset >> 16) &0xFF;
    bytes[5] = (dataPaket->offset >> 24) &0xFF;
}

void deSerializeTorqueSensorCommand(const uint8_t* bytes, torqueSensorCommand* dataPaket){
    dataPaket->joint_id = bytes[0];
    dataPaket->gain = bytes[1];
    dataPaket->offset =((int32_t) bytes[2]) | (((int32_t) bytes[3]) << 8) | (((int32_t) bytes[4]) << 16) | (((int32_t) bytes[5]) << 24 );
}

void serializeLightCommand(uint8_t * bytes, const lightCommand* dataPaket){
    bytes[0] = dataPaket->joint_id;
    bytes[1] = dataPaket->mode;
    bytes[2] = dataPaket->value_0;
    bytes[3] = dataPaket->value_1;
    bytes[4] = dataPaket->value_2;
}

void deSerializeLightCommand(const uint8_t* bytes, lightCommand* dataPaket){
    dataPaket->joint_id = bytes[0];
    dataPaket->mode = bytes[1];
    dataPaket->value_0 = bytes[2];
    dataPaket->value_1 = bytes[3];
    dataPaket->value_2 = bytes[4];
}

void serializeSensorControllerCommand(uint8_t * bytes, const sensorControllerCommand* dataPaket){
    bytes[0] = dataPaket->joint_id;
    bytes[1] = dataPaket->mode;
}

void deSerializeSensorControllerCommand(const uint8_t* bytes, sensorControllerCommand* dataPaket){
    dataPaket->joint_id = bytes[0];
    dataPaket->mode = bytes[1];
}