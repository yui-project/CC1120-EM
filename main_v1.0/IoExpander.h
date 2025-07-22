#pragma once
#include "Arduino.h"
#include "Decoder.h"
#include "StorageDefine.h"
class IoExpander
{
private:
    Decoder decoder;

// MCP23S08のレジスタアドレス
#define MCP23S08_IODIR 0x00 // I/Oディレクションレジスタ
#define MCP23S08_GPIO 0x09  // GPIOレジスタ
#define MCP23S08_OLAT 0x0A  // 出力ラッチレジスタ

public:
    void init();
    void setPin(int pin, int value);
    void writeIO1(byte reg, byte data);
    void writeIO2(byte reg, byte data);

    byte readIO1(byte reg);
    byte readIO2(byte reg);
};
