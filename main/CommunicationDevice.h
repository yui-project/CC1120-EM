#pragma once
#include "Arduino.h"
#include "StorageDefine.h"
#include "StorageGrobalVariable.h"
#include "LoRa.h"
// #include "I2cFram.h"
// #include "GlobalStruct.h"
#include "IoExpander.h"

class CommunicationDevice
{
private:
    // GlobalVariable global;
    // I2cFram i2cFram;
    IoExpander ex;

public:
    bool sendDl(uint8_t data, uint8_t whichRadio);
    bool sendDlPacket(uint8_t *data, int dataSize, uint8_t whichRadio);
    bool recvUl(uint8_t *ulCommand, uint8_t whichRadio);
    void switchMode(bool sendOrRecv, uint8_t whichRadio);
};
