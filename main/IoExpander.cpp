#include "IoExpander.h"
#include "Arduino.h"
#include <SPI.h>

/*
引数：なし
戻り値：なし
概要：IoExpanderの初期設定を行う
*/
void IoExpander::init()
{
    decoder.init();
    // NOTE: 初期設定ではデコーダは7番をLOWにしている
    decoder.write(DEFALT);

    SPI5.begin();

    // MCP23S08の初期設定
    // 全てのピンを出力に設定
    writeIO1(MCP23S08_IODIR, 0x00);
    writeIO2(MCP23S08_IODIR, 0x00);
    // 全てのピンをLOWに設定
    byte gpioState = 0x00;
    writeIO1(MCP23S08_GPIO, gpioState);
    writeIO2(MCP23S08_GPIO, gpioState);
}

/*
引数：int pin, int value(HIGH\LOW)
戻り値：なし
概要：指定したIOエキスパンダのピンに指定した値(HIGH\lOW)を設定する
*/
void IoExpander::setPin(int pin, int value)
{
    // NOTE:指定したピンがIOエキスパンダ1の範囲内の場合
    if (pin <= NUM_PIN_IO_EX1 )
    {
        byte gpioState = readIO1(MCP23S08_IODIR);
        if (value == HIGH)
        {
            gpioState |= (1 << pin); // 指定ピンをHIGHに設定
        }
        else
        {
            gpioState &= ~(1 << pin); // 指定ピンをLOWに設定
        }
        writeIO1(MCP23S08_GPIO, gpioState);
        return;
    }
    if (pin >= NUM_PIN_IO_EX2)
    {
        pin = pin - 8;
        byte gpioState = readIO2(MCP23S08_IODIR);
        Serial.println(gpioState, BIN);
        if (value == HIGH)
        {
            gpioState |= (1 << pin); // 指定ピンをHIGHに設定
        }
        else
        {
            gpioState &= ~(1 << pin); // 指定ピンをLOWに設定
        }
        writeIO2(MCP23S08_GPIO, gpioState);
        return;
    }
}

void IoExpander::writeIO1(byte reg, byte data)
{
    decoder.write(CS_IO_1);
    SPI5.transfer(0x40); // MCP23S08のアドレス（デフォルトは0x40）
    SPI5.transfer(reg);
    SPI5.transfer(data);
    decoder.write(DEFALT);
}

void IoExpander::writeIO2(byte reg, byte data)
{
    decoder.write(CS_IO_2);
    SPI5.transfer(0x40); // MCP23S08のアドレス（デフォルトは0x40）
    SPI5.transfer(reg);
    SPI5.transfer(data);
    decoder.write(DEFALT);
}

byte IoExpander::readIO1(byte reg)
{
    decoder.write(CS_IO_1);
    SPI5.transfer(0x41); // MCP23S08のアドレス（読み取りは0x41）
    SPI5.transfer(0x09);
    byte data = SPI5.transfer(0x00);
    decoder.write(DEFALT);
    return (data >> reg);
}

byte IoExpander::readIO2(byte reg)
{
    decoder.write(CS_IO_2);
    SPI5.transfer(0x41); // MCP23S08のアドレス（読み取りは0x41）
    SPI5.transfer(0x09);
    byte data = SPI5.transfer(0x00);
    decoder.write(DEFALT);
    return (data >> reg);
}
