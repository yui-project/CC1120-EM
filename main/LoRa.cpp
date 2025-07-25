// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "LoRa.h"

// registers
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_OCP 0x0b
#define REG_LNA 0x0c
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_TX_BASE_ADDR 0x0e
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1a
#define REG_RSSI_VALUE 0x1b
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_FREQ_ERROR_MSB 0x28
#define REG_FREQ_ERROR_MID 0x29
#define REG_FREQ_ERROR_LSB 0x2a
#define REG_RSSI_WIDEBAND 0x2c
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_INVERTIQ 0x33
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_INVERTIQ2 0x3b
#define REG_DIO_MAPPING_1 0x40
#define REG_VERSION 0x42
#define REG_PA_DAC 0x4d

// modes
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06
#define MODE_CAD 0x07

// PA config
#define PA_BOOST 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK 0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK 0x40
#define IRQ_CAD_DONE_MASK 0x04
#define IRQ_CAD_DETECTED_MASK 0x01

#define RF_MID_BAND_THRESHOLD 525E6
#define RSSI_OFFSET_HF_PORT 157
#define RSSI_OFFSET_LF_PORT 164

#define MAX_PKT_LENGTH 255

#if (ESP8266 || ESP32)
#define ISR_PREFIX ICACHE_RAM_ATTR
#else
#define ISR_PREFIX
#endif

LORAClass::LORAClass() : _spiSettings(LORA_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
                         _spi(&LORA_DEFAULT_SPI),
                         _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
                         _frequency(0),
                         _packetIndex(0),
                         _implicitHeaderMode(0),
                         _onReceive(NULL),
                         _onCadDone(NULL),
                         _onTxDone(NULL)
{
    // overide Stream timeout value
    setTimeout(0);
}

int LORAClass::begin(long frequency)
{
#if defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310)
    pinMode(LORA_IRQ_DUMB, OUTPUT);
    digitalWrite(LORA_IRQ_DUMB, LOW);

    // Hardware reset
    pinMode(LORA_BOOT0, OUTPUT);
    digitalWrite(LORA_BOOT0, LOW);

    pinMode(LORA_RESET, OUTPUT);
    digitalWrite(LORA_RESET, HIGH);
    delay(200);
    digitalWrite(LORA_RESET, LOW);
    delay(200);
    digitalWrite(LORA_RESET, HIGH);
    delay(50);
#endif

    decoder.write(7);           // NOTE:csピンをHIGHにしておく
    ex.setPin(LORA_RESET, LOW); // NOTE:Ra-01への通電前はRESETピンをLOWにしておく必要がある
    delay(10);
    digitalWrite(LORA_PIN, LOW); // NOTE:PchMOSFETなので、LOWでオン
    delay(10);
    ex.setPin(LORA_RESET, HIGH);

    // check version
    uint8_t version = readRegister(REG_VERSION);
    Serial.println(version);

    // put in sleep mode
    sleep();

    // set frequency
    setFrequency(frequency);

    // set base addresses
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

    // set auto AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x04);

    // set output power to 17 dBm
    setTxPower(17);

    // put in standby mode
    idle();

    return 1;
}

void LORAClass::end()
{
    // put in sleep mode
    sleep();

    // stop SPI
    _spi->end();
}

int LORAClass::beginPacket(int implicitHeader)
{
    if (isTransmitting())
    {
        return 0;
    }

    // put in standby mode
    idle();

    if (implicitHeader)
    {
        implicitHeaderMode();
    }
    else
    {
        explicitHeaderMode();
    }

    // reset FIFO address and paload length
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    writeRegister(REG_PAYLOAD_LENGTH, 0);

    return 1;
}

int LORAClass::endPacket(bool async)
{

    if ((async) && (_onTxDone))
        writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE

    // put in TX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    /*if (!async)
    {
        // wait for TX done
        while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
        {
            Serial.println("BAD!!!!!");
            yield();
        }
        // clear IRQ's
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }*/

    return 1;
}

bool LORAClass::isTransmitting()
{
    if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX)
    {
        return true;
    }

    if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)
    {
        // clear IRQ's
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return false;
}

int LORAClass::parsePacket(int size)
{
    int packetLength = 0;
    int irqFlags = readRegister(REG_IRQ_FLAGS);

    if (size > 0)
    {
        implicitHeaderMode();

        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    }
    else
    {
        explicitHeaderMode();
    }

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
    {
        // received a packet
        _packetIndex = 0;

        // read packet length
        if (_implicitHeaderMode)
        {
            packetLength = readRegister(REG_PAYLOAD_LENGTH);
        }
        else
        {
            packetLength = readRegister(REG_RX_NB_BYTES);
        }

        // set FIFO address to current RX address
        writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

        // put in standby mode
        idle();
    }
    else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
    {
        // not currently in RX mode

        // reset FIFO address
        writeRegister(REG_FIFO_ADDR_PTR, 0);

        // put in single RX mode
        writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }

    return packetLength;
}

int LORAClass::packetRssi()
{
    // return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
    return (readRegister(REG_RSSI_VALUE)); // ここを調整中
}

float LORAClass::packetSnr()
{
    return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long LORAClass::packetFrequencyError()
{
    int32_t freqError = 0;
    freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & 0b111);
    freqError <<= 8L;
    freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
    freqError <<= 8L;
    freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

    if (readRegister(REG_FREQ_ERROR_MSB) & 0b1000)
    {                        // Sign bit is on
        freqError -= 524288; // 0b1000'0000'0000'0000'0000
    }

    const float fXtal = 32E6;                                                                                         // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
    const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

    return static_cast<long>(fError);
}

int LORAClass::rssi()
{
    return (readRegister(REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

size_t LORAClass::write(uint8_t byte)
{
    return write(&byte, sizeof(byte));
}

size_t LORAClass::write(const uint8_t *buffer, size_t size)
{
    int currentLength = readRegister(REG_PAYLOAD_LENGTH);

    // check size
    if ((currentLength + size) > MAX_PKT_LENGTH)
    {
        size = MAX_PKT_LENGTH - currentLength;
    }

    // write data
    for (size_t i = 0; i < size; i++)
    {
        writeRegister(REG_FIFO, buffer[i]);
    }

    // update length
    writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

    return size;
}

int LORAClass::available()
{
    return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LORAClass::read()
{
    if (!available())
    {
        return -1;
    }

    _packetIndex++;

    return readRegister(REG_FIFO);
}

int LORAClass::peek()
{
    if (!available())
    {
        return -1;
    }

    // store current FIFO address
    int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

    // read
    uint8_t b = readRegister(REG_FIFO);

    // restore FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

    return b;
}

void LORAClass::flush()
{
}

#ifndef ARDUINO_SAMD_MKRWAN1300
void LORAClass::onReceive(void (*callback)(int))
{
    _onReceive = callback;

    if (callback)
    {
        pinMode(_dio0, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI5.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
        attachInterrupt(digitalPinToInterrupt(_dio0), LORAClass::onDio0Rise, RISING);
    }
    else
    {
        detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI5.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    }
}

void LORAClass::onCadDone(void (*callback)(boolean))
{
    _onCadDone = callback;

    if (callback)
    {
        pinMode(_dio0, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI5.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
        attachInterrupt(digitalPinToInterrupt(_dio0), LORAClass::onDio0Rise, RISING);
    }
    else
    {
        detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI5.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    }
}

void LORAClass::onTxDone(void (*callback)())
{
    _onTxDone = callback;

    if (callback)
    {
        pinMode(_dio0, INPUT);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI5.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
        attachInterrupt(digitalPinToInterrupt(_dio0), LORAClass::onDio0Rise, RISING);
    }
    else
    {
        detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI5.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    }
}

void LORAClass::receive(int size)
{

    writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

    if (size > 0)
    {
        implicitHeaderMode();

        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    }
    else
    {
        explicitHeaderMode();
    }

    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LORAClass::channelActivityDetection(void)
{
    writeRegister(REG_DIO_MAPPING_1, 0x80); // DIO0 => CADDONE
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
}
#endif

void LORAClass::idle()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LORAClass::sleep()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LORAClass::setTxPower(int level, int outputPin)
{
    if (PA_OUTPUT_RFO_PIN == outputPin)
    {
        // RFO
        if (level < 0)
        {
            level = 0;
        }
        else if (level > 14)
        {
            level = 14;
        }

        writeRegister(REG_PA_CONFIG, 0x70 | level);
    }
    else
    {
        // PA BOOST
        if (level > 17)
        {
            if (level > 20)
            {
                level = 20;
            }

            // subtract 3 from level, so 18 - 20 maps to 15 - 17
            level -= 3;

            // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
            writeRegister(REG_PA_DAC, 0x87);
            setOCP(140);
        }
        else
        {
            if (level < 2)
            {
                level = 2;
            }
            // Default value PA_HF/LF or +17dBm
            writeRegister(REG_PA_DAC, 0x84);
            setOCP(100);
        }

        Serial.print("LEVEL:");
        Serial.println(level - 2);
        writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

void LORAClass::setFrequency(long frequency)
{
    _frequency = frequency;

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int LORAClass::getSpreadingFactor()
{
    return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

void LORAClass::setSpreadingFactor(int sf)
{
    if (sf < 6)
    {
        sf = 6;
    }
    else if (sf > 12)
    {
        sf = 12;
    }

    if (sf == 6)
    {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    }
    else
    {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
    setLdoFlag();
}

long LORAClass::getSignalBandwidth()
{
    byte bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

    switch (bw)
    {
    case 0:
        return 7.8E3;
    case 1:
        return 10.4E3;
    case 2:
        return 15.6E3;
    case 3:
        return 20.8E3;
    case 4:
        return 31.25E3;
    case 5:
        return 41.7E3;
    case 6:
        return 62.5E3;
    case 7:
        return 125E3;
    case 8:
        return 250E3;
    case 9:
        return 500E3;
    }

    return -1;
}

void LORAClass::setSignalBandwidth(long sbw) // 使用しない方針（扱いづらいため）
{
    int bw;

    if (sbw <= 7.8E3)
    {
        bw = 0;
    }
    else if (sbw <= 10.4E3)
    {
        bw = 1;
    }
    else if (sbw <= 15.6E3)
    {
        bw = 2;
    }
    else if (sbw <= 20.8E3)
    {
        bw = 3;
    }
    else if (sbw <= 31.25E3)
    {
        bw = 4;
    }
    else if (sbw <= 41.7E3)
    {
        bw = 5;
    }
    else if (sbw <= 62.5E3)
    {
        bw = 6;
    }
    else if (sbw <= 125E3)
    {
        bw = 7;
    }
    else if (sbw <= 250E3)
    {
        bw = 8;
    }
    else /*if (sbw <= 250E3)*/
    {
        bw = 9;
    }

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
    setLdoFlag();
}

void LORAClass::setLdoFlag()
{
    // Section 4.1.1.5
    long symbolDuration = 1000 / (getSignalBandwidth() / (1L << getSpreadingFactor()));

    // Section 4.1.1.6
    boolean ldoOn = symbolDuration > 16;

    uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
    bitWrite(config3, 3, ldoOn);
    writeRegister(REG_MODEM_CONFIG_3, config3);
}

void LORAClass::setLdoFlagForced(const boolean ldoOn)
{
    uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
    bitWrite(config3, 3, ldoOn);
    writeRegister(REG_MODEM_CONFIG_3, config3);
}

void LORAClass::setCodingRate4(int denominator)
{
    if (denominator < 5)
    {
        denominator = 5;
    }
    else if (denominator > 8)
    {
        denominator = 8;
    }

    int cr = denominator - 4;

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LORAClass::setPreambleLength(long length)
{
    writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LORAClass::setSyncWord(int sw)
{
    writeRegister(REG_SYNC_WORD, sw);
}

void LORAClass::enableCrc()
{
    writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LORAClass::disableCrc()
{
    writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LORAClass::enableInvertIQ()
{
    writeRegister(REG_INVERTIQ, 0x66);
    writeRegister(REG_INVERTIQ2, 0x19);
}

void LORAClass::disableInvertIQ()
{
    writeRegister(REG_INVERTIQ, 0x27);
    writeRegister(REG_INVERTIQ2, 0x1d);
}

void LORAClass::enableLowDataRateOptimize()
{
    setLdoFlagForced(true);
}

void LORAClass::disableLowDataRateOptimize()
{
    setLdoFlagForced(false);
}

void LORAClass::setOCP(uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if (mA <= 120)
    {
        ocpTrim = (mA - 45) / 5;
    }
    else if (mA <= 240)
    {
        ocpTrim = (mA + 30) / 10;
    }

    writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LORAClass::setGain(uint8_t gain)
{
    // check allowed range
    if (gain > 6)
    {
        gain = 6;
    }

    // set to standby
    idle();

    // set gain
    if (gain == 0)
    {
        // if gain = 0, enable AGC
        writeRegister(REG_MODEM_CONFIG_3, 0x04);
    }
    else
    {
        // disable AGC
        writeRegister(REG_MODEM_CONFIG_3, 0x00);

        // clear Gain and set LNA boost
        writeRegister(REG_LNA, 0x03);

        // set gain
        writeRegister(REG_LNA, readRegister(REG_LNA) | (gain << 5));
    }
}

byte LORAClass::random()
{
    return readRegister(REG_RSSI_WIDEBAND);
}

void LORAClass::setPins(int ss, int reset, int dio0)
{
    _ss = ss;
    _reset = reset;
    _dio0 = dio0;
}

void LORAClass::setSPI(SPIClass &spi)
{
    _spi = &spi;
}

void LORAClass::setSPIFrequency(uint32_t frequency)
{
    _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void LORAClass::dumpRegisters(Stream &out)
{
    for (int i = 0; i < 128; i++)
    {
        out.print("0x");
        out.print(i, HEX);
        out.print(": 0x");
        out.println(readRegister(i), HEX);
    }
}

void LORAClass::explicitHeaderMode()
{
    _implicitHeaderMode = 0;

    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LORAClass::implicitHeaderMode()
{
    int implicitHeaderMode = 1;

    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void LORAClass::handleDio0Rise()
{
    int irqFlags = readRegister(REG_IRQ_FLAGS);

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_CAD_DONE_MASK) != 0)
    {
        if (_onCadDone)
        {
            _onCadDone((irqFlags & IRQ_CAD_DETECTED_MASK) != 0);
        }
    }
    else if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
    {

        if ((irqFlags & IRQ_RX_DONE_MASK) != 0)
        {
            // received a packet
            _packetIndex = 0;

            // read packet length
            int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

            // set FIFO address to current RX address
            writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

            if (_onReceive)
            {
                _onReceive(packetLength);
            }
        }
        else if ((irqFlags & IRQ_TX_DONE_MASK) != 0)
        {
            if (_onTxDone)
            {
                _onTxDone();
            }
        }
    }
}

uint8_t LORAClass::readRegister(uint8_t address)
{
    return singleTransfer(address & 0x7f, 0x00);
}

void LORAClass::writeRegister(uint8_t address, uint8_t value)
{
    singleTransfer(address | 0x80, value);
}

uint8_t LORAClass::singleTransfer(uint8_t address, uint8_t value)
{
    uint8_t response;

    _spi->beginTransaction(_spiSettings);
    decoder.write(0);
    _spi->transfer(address);
    response = _spi->transfer(value);
    decoder.write(7);
    _spi->endTransaction();

    return response;
}

void LORAClass::getRegPaConfig()
{
    Serial.print("READ:");
    Serial.println(LoRa.readRegister(9));
}

bool LORAClass::sendDL(uint8_t *data, int dlDataSize)
{
    LoRa.beginPacket();
    for (int i = 0; i < dlDataSize; i++)
    {
        LoRa.write(data[i]);
    }
    LoRa.endPacket();
    return true;
}

bool LORAClass::sendDL(uint8_t data)
{
    LoRa.beginPacket();
    LoRa.write(data);
    LoRa.endPacket();
    return true;
}

/*
引数：受信データを格納する配列
戻り値：受信成功か否か
概要：LoRaを用いてULを受信する.ULを受信したらtrueを返す.この時配列の先頭にはコマンド番号が格納される.2番目の配列からpacketが格納される.
*/
bool LORAClass::recvUL(uint8_t *recvCommand)
{
    int loraPacketSize = LoRa.parsePacket();
    if (loraPacketSize)
    {
        while (LoRa.available())
        {
            int startCode = LoRa.read();
            Serial.print("START_CODE");
            Serial.println(startCode);
            if (startCode == SOH)
            {
                recvCommand[0] = (uint8_t)LoRa.read();
                int subjectNumber = (int)recvCommand[0];
                int8_t recvPacketSize = packetSize[subjectNumber];
                Serial.print("Received Command Number: ");
                Serial.println(subjectNumber);
                for (int i = 1; i < recvPacketSize; i++)
                {
                    recvCommand[i] = (uint8_t)LoRa.read();
                    Serial.print("Received Command Data");
                    Serial.print(i);
                    Serial.print(": ");
                    Serial.println(recvCommand[i]);
                }

                int crc = (LoRa.read() << BYTE_1) + LoRa.read();
                Serial.print("CRC:");
                Serial.println(crc);
                checkCrc(recvCommand, recvPacketSize, crc);
                int endCode = LoRa.read();
                Serial.print("END_CODE:");
                Serial.println(endCode);
                if (endCode == EOT)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
    }
    return false;
}

bool LORAClass::sendDLfromFram(uint16_t addrStart, uint16_t addrEnd)
{
    /*uint8_t logicalAddrStart = getLogicalAddr(phygicalAddrStart);
    uint8_t logicalAddrEnd = getLogicalAddr(phygicalAddrEnd);
    for(uint8_t* data = logicalAddrStart; data <= logicalAddrEnd; data++) {
        sendDL(data);
    }*/
    // TODO FRAMからデータを指定して送信する
    return true;
}

void LORAClass::checkCrc(uint8_t *addr, uint8_t size, uint crc)
{
    // TODO CRCのチェック
}

int LORAClass::convertRecvData(int recvData)
{
    // NOTE:LoRaから受信したデータはASCIIコードで送られてくるので、それを数値に変換する
    int convertedData = recvData - 0x30;
    return convertedData;
}

ISR_PREFIX void LORAClass::onDio0Rise()
{
    LoRa.handleDio0Rise();
}

LORAClass LoRa;
