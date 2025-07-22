#include "HardwareSerial.h"
#include "stdint.h"
#include "CC1120.h"
#include "Decoder.h"
#include "IoExpander.h"
// #include "SpiFram.h"
#include <SPI.h>

SPISettings settings(100000, MSBFIRST, SPI_MODE0);
Decoder DECODER;
IoExpander IoEx;
// SpiFram FRAM;


/// CC1120の初期化
/// @return エラーコード(0は異常、1は正常)
bool CC1120Class::CWbegin(){
  bool ret = 1;
  reset();
  Serial.println("Reset");
  // CC1120_SERIAL.begin(9600);
  CC1120_SPI.begin();
  Serial.println("SPI start");  
  Serial.println(ret);  
  ret = IDLE();
  Serial.println(ret);

  pinMode(CC1120_POWER, OUTPUT);
  pinMode(LoRa_POWER, OUTPUT);

  DECODER.init();

  DECODER.write(1);

  digitalWrite(CC1120_POWER, HIGH);
  digitalWrite(LoRa_POWER, HIGH);
  delay(100);

  IoEx.setPin(15, LOW);
  IoEx.setPin(13, LOW);
  delay(100);

  digitalWrite(CC1120_POWER, LOW);
  digitalWrite(LoRa_POWER, LOW);
  delay(100);

  IoEx.setPin(15, HIGH);
  IoEx.setPin(13, LOW);
  delay(100);

  // Serial.print("IDLE");
  // Serial.println(ret);
  Serial.println(ret);
  ret = setCW();
  Serial.println("CW");
  return ret;
}

bool CC1120Class::FSKbegin(){
  bool ret = 1;
  reset();
  Serial.println("Reset");
  // CC1120_SERIAL.begin(9600);
  CC1120_SPI.begin();
  Serial.println("SPI start");  
  Serial.println(ret);  
  ret = IDLE();
  Serial.println(ret);

  pinMode(CC1120_POWER, OUTPUT);
  pinMode(LoRa_POWER, OUTPUT);

  DECODER.init();

  DECODER.write(1);

  digitalWrite(CC1120_POWER, HIGH);
  digitalWrite(LoRa_POWER, HIGH);
  delay(100);

  IoEx.setPin(15, LOW);
  IoEx.setPin(13, LOW);
  delay(100);

  digitalWrite(CC1120_POWER, LOW);
  digitalWrite(LoRa_POWER, LOW);
  delay(100);

  IoEx.setPin(15, HIGH);
  IoEx.setPin(13, LOW);
  delay(100);

  // Serial.print("IDLE");
  // Serial.println(ret);
  Serial.println(ret);
  ret = setFSK();
  Serial.println("FSK");
  return ret;
}

bool CC1120Class::calibration(){
  strobeSPI(SCAL);
  bool ret = 1;
  ret = waitIDLE(ret, waitTime);
  return ret;
}

bool CC1120Class::setRegister(bool extAddr, uint8_t addr, uint8_t value){
  bool ret = 1;
  strobeSPI(SIDLE);
  ret = waitIDLE(ret, waitTime);
  if(extAddr == 1){
    writeExtAddrSPI(addr, value);
  }
  else{
    writeSPI(addr, value);
  }
  return ret;
}

uint8_t CC1120Class::showResister(bool extAddr, uint8_t addr){
  uint8_t val;
  if(extAddr == 1){
  readExtAddrSPI(addr);
  }
  else{
  readSPI(addr);
  }
  return val;
}

uint32_t CC1120Class::showRSSI(){
  uint8_t rssi1 = showResister(1, RSSI1);
  uint8_t rssi0 = showResister(1, RSSI0);

  uint32_t rssi = (rssi1 << 4) + (rssi0 & 0b01111000);
  return rssi*0.0625-128;
}

bool CC1120Class::IDLE(){
  bool ret = 1;
  strobeSPI(SIDLE);
  ret = waitIDLE(ret, waitTime);
  return ret;
}

bool CC1120Class::sendDL(uint8_t data){
  bool ret = 1;
  Serial.println(data);
  Serial.println((uint32_t)&data);
  ret = TX(&data, 1);
  return ret;
}

bool CC1120Class::sendDL(uint8_t *data, uint32_t len){
  bool ret = 1;
  ret = TX(data, len);
  return ret;
}

bool CC1120Class::recvUL(uint8_t *recvCommand)
{
  bool ret = 1;
  uint8_t packet[128];
  // Serial.println(ret);
  ret = RX(packet);    
  // Serial.println(ret);

  if (ret == 1)
  {
    uint8_t startCode = packet[2];
    // Serial.println(startCode, HEX);
    if(startCode == 0x02){
      uint8_t dataLen = packet[0];
      // Serial.println(dataLen);
      uint8_t subjectNumber = packet[3];
      uint8_t payload[dataLen-1];
      
      for(int i=0; i<dataLen-1; i++){
        payload[i] = packet[i+2];
      }
      uint8_t crc[2] = {packet[dataLen-2], packet[dataLen-1]};
      uint8_t endCode = packet[dataLen];
      // Serial.println(endCode, HEX);
      if(endCode != 0x04){
        // Serial.println(ret);
        ret = 0;
      }
      // recvCommand[0] = startCode;
      // recvCommand[1] = subjectNumber;
      for(int i=0; i<dataLen-1; i++){
        recvCommand[i] = payload[i];
      }
      // recvCommand[dataLen+1] = crc[0];
      // recvCommand[dataLen+2] = crc[1];
      // recvCommand[dataLen+3] = endCode;
    }
  }
  // Serial.println(ret);
  return ret;
}

bool CC1120Class::setFREQ(uint8_t FREQ){
  bool ret = 1;
  if(FREQ == 0){ // 437.05MHz
    setRegister(1, 0x0C, 0x6D);  
    setRegister(1, 0x0D, 0x43);  
    setRegister(1, 0x0E, 0x33);  
  }
  if(FREQ == 1){ // 435.00MHz
    setRegister(1, 0x0C, 0x6C);  
    setRegister(1, 0x0D, 0xC0);  
    setRegister(1, 0x0E, 0x00);  
  }
  if(FREQ == 2){ // 437.3MHz
    setRegister(1, 0x0C, 0x6D);  
    setRegister(1, 0x0D, 0x5B);  
    setRegister(1, 0x0E, 0x33); 
  }
  ret = calibration();
  return ret;
}

bool CC1120Class::setPWR(uint8_t PWR){
  bool ret = 1;
  if(PWR == 3){   // 15dBm
    setRegister(0, 0x2B, 0x7F);
  }
  if(PWR == 2){   // 10dBm
    setRegister(0, 0x2B, 0x74);
  }
  if(PWR == 1){  // 5dBm
    setRegister(0, 0x2B, 0x69);
  }
  if(PWR == 0){   // -11dBm
    setRegister(0, 0x2B, 0x43);
  }
  ret = calibration();
  return ret;
}



// bool CC1120Class::sendDLfromFram(uint64_t start, uint64_t end){
//   uint64_t len = end-start+1;
//   uint8_t payload[len];
//   FRAM.begin();
//   for(uint64_t i=0; i<=len; i++){
//     payload[i] = (uint8_t)FRAM.read(start+i);
//   }
//   begin();
//   TX(payload, len);
// }

bool CC1120Class::TX(uint8_t *payload, int32_t len)
{
  bool ret = 1;
  len++;

  Serial.println("TX");
  Serial.println(len);
  int32_t index=0;

  while(len>0){
    writeSPI(TXRX_FIFO, (uint8_t) min(len, 125));
    Serial.print("len: ");
    Serial.println(readFIFO(0x00));
    writeSPI(TXRX_FIFO, 0x55);
    Serial.print("addr: 0x");
    Serial.println(readFIFO(0x01), HEX);

    for(uint32_t i=0; ((i<len) && (i<125)); i++)
    {
      writeSPI(TXRX_FIFO, payload[index++]);
      // Serial.print("payload");
      // Serial.print(i);
      // Serial.print(": ");
      // Serial.println(readFIFO(i+2));
      // Serial.println((i<len));
      // Serial.println((i<126));
      // Serial.println((i<len) || (i<126));
    }
    len-=124;
    Serial.println(len);
    strobeSPI(STX);
    // Serial.println(marcstate(), BIN);
    // delay(100);
    // Serial.println(marcstate(), BIN);
    // delay(10);
    // Serial.println(marcstate(), BIN);
    // Serial.println(ret);
    ret = waitIDLEorTXFIFOERROR(ret, waitTime);
    // Serial.println(ret);
    ret = FIFOFlush();

    int8_t RXByte;
    RXByte = readExtAddrSPI(NUM_RXBYTES);
    if(RXByte > 4){
      ret = 0;
      break;
      // Serial.println(RXByte);
    }
    delay(3000);
  }

  // Serial.println(marcstate(), BIN);

  // delay(100);

  // Serial.println(marcstate(), BIN);

  // delay(1000);

  // Serial.println(marcstate(), BIN);

  // ret = waitTXFIFOERROR(ret, waitTime);

  // strobeSPI(SIDLE);

  // ret = waitIDLE(ret, waitTime);

  ret = FIFOFlush();
  strobeSPI(SRX);
  ret = waitRX(ret, timerTime);
  return ret;
}

bool CC1120Class::RX(uint8_t *data, uint16_t limit=0)
{
  bool ret = 1;
  uint8_t DataLen = 0;
  uint8_t DataAddr = 0;
  strobeSPI(SRX);

  ret = waitRX(ret, waitTime);

  uint8_t RXByte = 0;

  // ret = waitRXPKT(ret, timerTime);

  if(readExtAddrSPI(NUM_RXBYTES) < 1){
    ret = 0;
  }

  if(ret == 1){
    DataLen = readSPI(0b10111111);
    Serial.print(DataLen);
    data[0] = DataLen;
    if(limit != 0){
      DataLen = limit;
    }
    DataAddr = readSPI(0b10111111);
    data[1] = DataAddr;

    DataLen+=1;

    // if(DataLen < 127)
    // {
      for(int i=0; i<DataLen; i++)
      {
        data[i+2] = readSPI(0b10111111);
      }
    // }
    // else if(DataLen > 127)
    // {
    //   uint32_t index = 0;
    //   while(DataLen > 0)
    //   {
    //     if(DataLen >= 127)
    //     {
    //       for(int i=0; i<127; i++)
    //       {
    //         data[index++] = readSPI(0b10111111);
    //       }
    //       DataLen -= 127;
    //     }
    //     else if(DataLen < 127)
    //     {
    //       for(int i=0; i<DataLen; i++)
    //       {
    //         data[index++] = readSPI(0b10111111);
    //       }
    //       DataLen = 0;
    //     }
    //   }
    // }
  
    // strobeSPI(SIDLE);
    return ret;
  }
  
  IDLE();

  ret = waitIDLE(ret, waitTime);

  FIFOFlush();

  strobeSPI(SRX);
  ret = waitRX(ret, timerTime);

  return ret;
}











uint8_t CC1120Class::readSPI(uint8_t addr) {
  // Serial.println("reading SPI");
  CC1120_SPI.beginTransaction(settings);
  DECODER.write(1);
  CC1120_SPI.transfer(CC1120_R_BIT | addr);
  uint8_t v = CC1120_SPI.transfer(0x00);
  DECODER.write(7);
  CC1120_SPI.endTransaction();
  return v;
}
 
void CC1120Class::writeSPI(uint8_t addr, uint8_t value) {
  CC1120_SPI.beginTransaction(settings);
  DECODER.write(1);
  CC1120_SPI.transfer(addr);
  CC1120_SPI.transfer(value);
  DECODER.write(7);
  CC1120_SPI.endTransaction();
}
 
void CC1120Class::strobeSPI(uint8_t cmd)
{
  CC1120_SPI.beginTransaction(settings);
  DECODER.write(1);
  CC1120_SPI.transfer(CC1120_R_BIT | cmd);
  DECODER.write(7);
  CC1120_SPI.endTransaction();
}
 
uint8_t CC1120Class::readExtAddrSPI(uint8_t addr) {
  CC1120_SPI.beginTransaction(settings);
  static uint8_t v;
  // CC1120_SPI.beginTransaction(settings);
  DECODER.write(1);
  CC1120_SPI.transfer(CC1120_R_BIT | EXT_ADDR);
  CC1120_SPI.transfer(addr);
  delayMicroseconds(10);
  v = CC1120_SPI.transfer(0xff);
  // Serial.println(CC1120_SPI.transfer(0xff), BIN);
  DECODER.write(7);
  CC1120_SPI.endTransaction();
  return v;
}
 
void CC1120Class::writeExtAddrSPI(uint8_t addr, uint8_t value) {
  CC1120_SPI.beginTransaction(settings);
  int v;
  DECODER.write(1);
  v = CC1120_SPI.transfer(EXT_ADDR);
  v = CC1120_SPI.transfer(addr);
  v = CC1120_SPI.transfer(value);
  DECODER.write(7);
  CC1120_SPI.endTransaction();
}

uint8_t CC1120Class::readFIFO(uint8_t addr){
  CC1120_SPI.beginTransaction(settings);
  static uint8_t v;
  // CC1120_SPI.beginTransaction(settings);
  DECODER.write(1);
  CC1120_SPI.transfer(0b10111110);
  CC1120_SPI.transfer(addr);
  delayMicroseconds(10);
  v = CC1120_SPI.transfer(0xff);
  // Serial.println(CC1120_SPI.transfer(0xff), BIN);
  DECODER.write(7);
  CC1120_SPI.endTransaction();
  return v;
}

void CC1120Class::writeFIFO(uint8_t addr, uint8_t value){
  CC1120_SPI.beginTransaction(settings);
  int v;
  DECODER.write(1);
  v = CC1120_SPI.transfer(0b10111110);
  v = CC1120_SPI.transfer(addr);
  v = CC1120_SPI.transfer(value);
  DECODER.write(7);
  CC1120_SPI.endTransaction();
}

void CC1120Class::timerStart(uint32_t time)
{
  timerTime = millis() + time;
}

bool CC1120Class::timeout()
{
  if(timerTime < millis()){
    return 1;
  }
  return 0;
}

bool CC1120Class::waitIDLE(bool ret, uint32_t time){
  Serial.println("waiting IDLE");
  timerStart(time);
  while(marcstate() != MARCSTATE_IDLE){
    // Serial.println(marcstate());
    if(timeout()){
      ret = 0;
      break;
    }
  }
  return ret;
}

bool CC1120Class::waitRX(bool ret, uint32_t time){
  timerStart(time);
  while(marcstate() != MARCSTATE_RX){
    delay(1);
    if(timeout()){
      ret = 0;
      break;
    }
  }
  return ret;
}

bool CC1120Class::waitRXPKT(bool ret, uint32_t time){
  timerStart(time);
  while(readExtAddrSPI(NUM_RXBYTES) < 1){
    // Serial.println(readExtAddrSPI(NUM_RXBYTES));
    delay(10);
    if(timeout()){
      Serial.println("timeout!!!");
      ret = 0;
      break;
    }
  }
  return ret;
}

bool CC1120Class::waitTXFIFOERROR(bool ret, uint32_t time){
  Serial.println("waiting TX_FIFO_ERROR");
  timerStart(time);
  while(marcstate() != MARCSTATE_TXFIFOERROR){
    delay(1);
    Serial.println("USB hub");
    if(timeout()){
      Serial.println("timeout!!!");
      ret = 0;
      break;
    }
  }
  return ret;
}

bool CC1120Class::waitIDLEorTXFIFOERROR(bool ret, uint32_t time){
  timerStart(time);
  while(true){
    uint8_t marcstateValue = marcstate();
    if((marcstateValue == MARCSTATE_IDLE) || (marcstateValue == MARCSTATE_TXFIFOERROR)){
      break;
    }
    
    if(timeout()){
      Serial.println("timeout!!!");
      ret = 0;
      break;
    }

    delay(1);
  }
  return ret;
}

bool CC1120Class::FIFOFlush(){
  bool ret = 1;
  strobeSPI(SFRX); // Flush the RX FIFO

  ret = waitIDLE(ret, waitTime);
  
  strobeSPI(SFTX); // Flush the TX FIFO

  ret = waitIDLE(ret, waitTime);
  return ret;
}

uint8_t CC1120Class::marcstate(){
  return readExtAddrSPI(MARCSTATE);
}


void CC1120Class::reset(){
  Serial.println("Start reset");
  CC1120_SPI.beginTransaction(settings);
  Serial.println("beginTransaction");
  DECODER.write(1);
  CC1120_SPI.transfer(SRES);
  Serial.println("transfer");
  delay(1);
  while(digitalRead(17) == 1){
    delay(1);
    Serial.println("hi");
  }
  Serial.println("digitalRead");
  DECODER.write(7);
  CC1120_SPI.endTransaction();
}


bool CC1120Class::setFSK(){
  Serial.println("Setting registers (FSK)...");
  writeSPI(IOCFG3, FSK_IOCFG3_VALUE);
  writeSPI(IOCFG2, FSK_IOCFG2_VALUE);
  writeSPI(IOCFG1, FSK_IOCFG1_VALUE);
  writeSPI(IOCFG0, FSK_IOCFG0_VALUE);
  writeSPI(SYNC3, FSK_SYNC3_VALUE);
  writeSPI(SYNC2, FSK_SYNC2_VALUE);
  writeSPI(SYNC1, FSK_SYNC1_VALUE);
  writeSPI(SYNC0, FSK_SYNC0_VALUE);
  writeSPI(SYNC_CFG1, FSK_SYNC_CFG1_VALUE);
  writeSPI(SYNC_CFG0, FSK_SYNC_CFG0_VALUE);
  writeSPI(DEVIATION_M, FSK_DEVIATION_M_VALUE);
  writeSPI(MODCFG_DEV_E, FSK_MODCFG_DEV_E_VALUE);
  writeSPI(DCFILT_CFG, FSK_DCFILT_CFG_VALUE);
  writeSPI(PREAMBLE_CFG1, FSK_PREAMBLE_CFG1_VALUE);
  writeSPI(PREAMBLE_CFG0, FSK_PREAMBLE_CFG0_VALUE);
  writeSPI(FREQ_IF_CFG, FSK_FREQ_IF_CFG_VALUE);
  writeSPI(IQIC, FSK_IQIC_VALUE);
  writeSPI(CHAN_BW, FSK_CHAN_BW_VALUE);
  writeSPI(MDMCFG1, FSK_MDMCFG1_VALUE);
  writeSPI(MDMCFG0, FSK_MDMCFG0_VALUE);
  writeSPI(SYMBOL_RATE2, FSK_SYMBOL_RATE2_VALUE);
  writeSPI(SYMBOL_RATE1, FSK_SYMBOL_RATE1_VALUE);
  writeSPI(SYMBOL_RATE0, FSK_SYMBOL_RATE0_VALUE);
  writeSPI(AGC_REF, FSK_AGC_REF_VALUE);
  writeSPI(AGC_CS_THR, FSK_AGC_CS_THR_VALUE);
  writeSPI(AGC_GAIN_ADJUST, FSK_AGC_GAIN_ADJUST_VALUE);
  writeSPI(AGC_CFG3, FSK_AGC_CFG3_VALUE);
  writeSPI(AGC_CFG2, FSK_AGC_CFG2_VALUE);
  writeSPI(AGC_CFG1, FSK_AGC_CFG1_VALUE);
  writeSPI(AGC_CFG0, FSK_AGC_CFG0_VALUE);
  writeSPI(FIFO_CFG, FSK_FIFO_CFG_VALUE);
  writeSPI(DEV_ADDR, FSK_DEV_ADDR_VALUE);
  writeSPI(SETTLING_CFG, FSK_SETTLING_CFG_VALUE);
  writeSPI(FS_CFG, FSK_FS_CFG_VALUE);
  writeSPI(WOR_CFG1, FSK_WOR_CFG1_VALUE);
  writeSPI(WOR_CFG0, FSK_WOR_CFG0_VALUE);
  writeSPI(WOR_EVENT0_MSB, FSK_WOR_EVENT0_MSB_VALUE);
  writeSPI(WOR_EVENT0_LSB, FSK_WOR_EVENT0_LSB_VALUE);
  writeSPI(PKT_CFG2, FSK_PKT_CFG2_VALUE);
  writeSPI(PKT_CFG1, FSK_PKT_CFG1_VALUE);
  writeSPI(PKT_CFG0, FSK_PKT_CFG0_VALUE);
  writeSPI(RFEND_CFG1, FSK_RFEND_CFG1_VALUE);
  writeSPI(RFEND_CFG0, FSK_RFEND_CFG0_VALUE);
  writeSPI(PA_CFG2, FSK_PA_CFG2_VALUE);
  writeSPI(PA_CFG1, FSK_PA_CFG1_VALUE);
  writeSPI(PA_CFG0, FSK_PA_CFG0_VALUE);
  writeSPI(PKT_LEN, FSK_PKT_LEN_VALUE);

  writeExtAddrSPI(FREQOFF_CFG, FSK_FREQOFF_CFG_VALUE);
  writeExtAddrSPI(TOC_CFG, FSK_TOC_CFG_VALUE);
  writeExtAddrSPI(MARC_SPARE, FSK_MARC_SPARE_VALUE);
  writeExtAddrSPI(ECG_CFG, FSK_ECG_CFG_VALUE);
  writeExtAddrSPI(CFM_DATA_CFG, FSK_CFM_DATA_CFG_VALUE);
  writeExtAddrSPI(EXT_CTRL, FSK_EXT_CTRL_VALUE);
  writeExtAddrSPI(RCCAL_FINE, FSK_RCCAL_FINE_VALUE);
  writeExtAddrSPI(RCCAL_COARSE, FSK_RCCAL_COARSE_VALUE);
  writeExtAddrSPI(RCCAL_OFFSET, FSK_RCCAL_OFFSET_VALUE);
  writeExtAddrSPI(FREQOFF1, FSK_FREQOFF1_VALUE);
  writeExtAddrSPI(FREQOFF0, FSK_FREQOFF0_VALUE);
  writeExtAddrSPI(FREQ2, FSK_FREQ2_VALUE);
  writeExtAddrSPI(FREQ1, FSK_FREQ1_VALUE);
  writeExtAddrSPI(FREQ0, FSK_FREQ0_VALUE);
  writeExtAddrSPI(IF_ADC2, FSK_IF_ADC2_VALUE);
  writeExtAddrSPI(IF_ADC1, FSK_IF_ADC1_VALUE);
  writeExtAddrSPI(IF_ADC0, FSK_IF_ADC0_VALUE);
  writeExtAddrSPI(FS_DIG1, FSK_FS_DIG1_VALUE);
  writeExtAddrSPI(FS_DIG0, FSK_FS_DIG0_VALUE);
  writeExtAddrSPI(FS_CAL3, FSK_FS_CAL3_VALUE);
  writeExtAddrSPI(FS_CAL2, FSK_FS_CAL2_VALUE);
  writeExtAddrSPI(FS_CAL1, FSK_FS_CAL1_VALUE);
  writeExtAddrSPI(FS_CAL0, FSK_FS_CAL0_VALUE);
  writeExtAddrSPI(FS_CHP, FSK_FS_CHP_VALUE);
  writeExtAddrSPI(FS_DIVTWO, FSK_FS_DIVTWO_VALUE);
  writeExtAddrSPI(FS_DSM1, FSK_FS_DSM1_VALUE);
  writeExtAddrSPI(FS_DSM0, FSK_FS_DSM0_VALUE);
  writeExtAddrSPI(FS_DVC1, FSK_FS_DVC1_VALUE);
  writeExtAddrSPI(FS_DVC0, FSK_FS_DVC0_VALUE);
  writeExtAddrSPI(FS_LBI, FSK_FS_LBI_VALUE);
  writeExtAddrSPI(FS_PFD, FSK_FS_PFD_VALUE);
  writeExtAddrSPI(FS_PRE, FSK_FS_PRE_VALUE);
  writeExtAddrSPI(FS_REG_DIV_CML, FSK_FS_REG_DIV_CML_VALUE);
  writeExtAddrSPI(FS_SPARE, FSK_FS_SPARE_VALUE);
  writeExtAddrSPI(FS_VCO4, FSK_FS_VCO4_VALUE);
  writeExtAddrSPI(FS_VCO3, FSK_FS_VCO3_VALUE);
  writeExtAddrSPI(FS_VCO2, FSK_FS_VCO2_VALUE);
  writeExtAddrSPI(FS_VCO1, FSK_FS_VCO1_VALUE);
  writeExtAddrSPI(FS_VCO0, FSK_FS_VCO0_VALUE);
  writeExtAddrSPI(GBIAS6, FSK_GBIAS6_VALUE);
  writeExtAddrSPI(GBIAS5, FSK_GBIAS5_VALUE);
  writeExtAddrSPI(GBIAS4, FSK_GBIAS4_VALUE);
  writeExtAddrSPI(GBIAS3, FSK_GBIAS3_VALUE);
  writeExtAddrSPI(GBIAS2, FSK_GBIAS2_VALUE);
  writeExtAddrSPI(GBIAS1, FSK_GBIAS1_VALUE);
  writeExtAddrSPI(GBIAS0, FSK_GBIAS0_VALUE);
  writeExtAddrSPI(IFAMP, FSK_IFAMP_VALUE);
  writeExtAddrSPI(LNA, FSK_LNA_VALUE);
  writeExtAddrSPI(RXMIX, FSK_RXMIX_VALUE);
  writeExtAddrSPI(XOSC5, FSK_XOSC5_VALUE);
  writeExtAddrSPI(XOSC4, FSK_XOSC4_VALUE);
  writeExtAddrSPI(XOSC3, FSK_XOSC3_VALUE);
  writeExtAddrSPI(XOSC2, FSK_XOSC2_VALUE);
  writeExtAddrSPI(XOSC1, FSK_XOSC1_VALUE);
  writeExtAddrSPI(XOSC0, FSK_XOSC0_VALUE);
  writeExtAddrSPI(ANALOG_SPARE, FSK_ANALOG_SPARE_VALUE);
  writeExtAddrSPI(PA_CFG3, FSK_PA_CFG3_VALUE);
  writeExtAddrSPI(WOR_TIME1, FSK_WOR_TIME1_VALUE);
  writeExtAddrSPI(WOR_TIME0, FSK_WOR_TIME0_VALUE);
  writeExtAddrSPI(WOR_CAPTURE1, FSK_WOR_CAPTURE1_VALUE);
  writeExtAddrSPI(WOR_CAPTURE0, FSK_WOR_CAPTURE0_VALUE);
  writeExtAddrSPI(BIST, FSK_BIST_VALUE);
  writeExtAddrSPI(DCFILTOFFSET_I1, FSK_DCFILTOFFSET_I1_VALUE);
  writeExtAddrSPI(DCFILTOFFSET_I0, FSK_DCFILTOFFSET_I0_VALUE);
  writeExtAddrSPI(DCFILTOFFSET_Q1, FSK_DCFILTOFFSET_Q1_VALUE);
  writeExtAddrSPI(DCFILTOFFSET_Q0, FSK_DCFILTOFFSET_Q0_VALUE);
  writeExtAddrSPI(IQIE_I1, FSK_IQIE_I1_VALUE);
  writeExtAddrSPI(IQIE_I0, FSK_IQIE_I0_VALUE);
  writeExtAddrSPI(IQIE_Q1, FSK_IQIE_Q1_VALUE);
  writeExtAddrSPI(IQIE_Q0, FSK_IQIE_Q0_VALUE);
  writeExtAddrSPI(RSSI1, FSK_RSSI1_VALUE);
  writeExtAddrSPI(RSSI0, FSK_RSSI0_VALUE);
  writeExtAddrSPI(MARCSTATE, FSK_MARCSTATE_VALUE);
  writeExtAddrSPI(LQI_VAL, FSK_LQI_VAL_VALUE);
  writeExtAddrSPI(PQT_SYNC_ERR, FSK_PQT_SYNC_ERR_VALUE);
  writeExtAddrSPI(DEM_STATUS, FSK_DEM_STATUS_VALUE);
  writeExtAddrSPI(FREQOFF_EST1, FSK_FREQOFF_EST1_VALUE);
  writeExtAddrSPI(FREQOFF_EST0, FSK_FREQOFF_EST0_VALUE);
  writeExtAddrSPI(AGC_GAIN3, FSK_AGC_GAIN3_VALUE);
  writeExtAddrSPI(AGC_GAIN2, FSK_AGC_GAIN2_VALUE);
  writeExtAddrSPI(AGC_GAIN1, FSK_AGC_GAIN1_VALUE);
  writeExtAddrSPI(AGC_GAIN0, FSK_AGC_GAIN0_VALUE);
  writeExtAddrSPI(CFM_RX_DATA_OUT, FSK_CFM_RX_DATA_OUT_VALUE);
  writeExtAddrSPI(CFM_TX_DATA_IN, FSK_CFM_TX_DATA_IN_VALUE);
  writeExtAddrSPI(ASK_SOFT_RX_DATA, FSK_ASK_SOFT_RX_DATA_VALUE);
  writeExtAddrSPI(RNDGEN, FSK_RNDGEN_VALUE);
  writeExtAddrSPI(MAGN2, FSK_MAGN2_VALUE);
  writeExtAddrSPI(MAGN1, FSK_MAGN1_VALUE);
  writeExtAddrSPI(MAGN0, FSK_MAGN0_VALUE);
  writeExtAddrSPI(ANG1, FSK_ANG1_VALUE);
  writeExtAddrSPI(ANG0, FSK_ANG0_VALUE);
  writeExtAddrSPI(CHFILT_I2, FSK_CHFILT_I2_VALUE);
  writeExtAddrSPI(CHFILT_I1, FSK_CHFILT_I1_VALUE);
  writeExtAddrSPI(CHFILT_I0, FSK_CHFILT_I0_VALUE);
  writeExtAddrSPI(CHFILT_Q2, FSK_CHFILT_Q2_VALUE);
  writeExtAddrSPI(CHFILT_Q1, FSK_CHFILT_Q1_VALUE);
  writeExtAddrSPI(CHFILT_Q0, FSK_CHFILT_Q0_VALUE);
  writeExtAddrSPI(GPIO_STATUS, FSK_GPIO_STATUS_VALUE);
  writeExtAddrSPI(FSCAL_CTRL, FSK_FSCAL_CTRL_VALUE);
  writeExtAddrSPI(PHASE_ADJUST, FSK_PHASE_ADJUST_VALUE);
  // writeExtAddrSPI(PARTNUMBER, PARTNUMBER_VALUE);
  // writeExtAddrSPI(PARTVERSION, PARTVERSION_VALUE);
  writeExtAddrSPI(SERIAL_STATUS, FSK_SERIAL_STATUS_VALUE);
  writeExtAddrSPI(MODEM_STATUS1, FSK_MODEM_STATUS1_VALUE);
  writeExtAddrSPI(MODEM_STATUS0, FSK_MODEM_STATUS0_VALUE);
  writeExtAddrSPI(MARC_STATUS1, FSK_MARC_STATUS1_VALUE);
  writeExtAddrSPI(MARC_STATUS0, FSK_MARC_STATUS0_VALUE);
  writeExtAddrSPI(PA_IFAMP_TEST, FSK_PA_IFAMP_TEST_VALUE);
  writeExtAddrSPI(FSRF_TEST, FSK_FSRF_TEST_VALUE);
  writeExtAddrSPI(PRE_TEST, FSK_PRE_TEST_VALUE);
  writeExtAddrSPI(PRE_OVR, FSK_PRE_OVR_VALUE);
  writeExtAddrSPI(ADC_TEST, FSK_ADC_TEST_VALUE);
  writeExtAddrSPI(DVC_TEST, FSK_DVC_TEST_VALUE);
  writeExtAddrSPI(ATEST, FSK_ATEST_VALUE);
  writeExtAddrSPI(ATEST_LVDS, FSK_ATEST_LVDS_VALUE);
  writeExtAddrSPI(ATEST_MODE, FSK_ATEST_MODE_VALUE);
  writeExtAddrSPI(XOSC_TEST1, FSK_XOSC_TEST1_VALUE);
  writeExtAddrSPI(XOSC_TEST0, FSK_XOSC_TEST0_VALUE);
  // writeExtAddrSPI(RXFIRST, RXFIRST_VALUE);
  // writeExtAddrSPI(TXFIRST, TXFIRST_VALUE);
  // writeExtAddrSPI(RXLAST, RXLAST_VALUE);
  // writeExtAddrSPI(TXLAST, TXLAST_VALUE);
  // writeExtAddrSPI(NUM_TXBYTES, NUM_TXBYTES_VALUE);
  // writeExtAddrSPI(NUM_RXBYTES, NUM_RXBYTES_VALUE);
  // writeExtAddrSPI(FIFO_NUM_TXBYTES, FIFO_NUM_TXBYTES_VALUE);
  // writeExtAddrSPI(FIFO_NUM_RXBYTES, FIFO_NUM_RXBYTES_VALUE);

  Serial.print("Checking registers .");

  bool ret = 1;

  if(readSPI(IOCFG3)           != FSK_IOCFG3_VALUE              ) ret=0;
  if(readSPI(IOCFG2)           != FSK_IOCFG2_VALUE              ) ret=0;
  if(readSPI(IOCFG1)           != FSK_IOCFG1_VALUE              ) ret=0;
  if(readSPI(IOCFG0)           != FSK_IOCFG0_VALUE              ) ret=0;
  if(readSPI(SYNC3)            != FSK_SYNC3_VALUE               ) ret=0;
  if(readSPI(SYNC2)            != FSK_SYNC2_VALUE               ) ret=0;
  if(readSPI(SYNC1)            != FSK_SYNC1_VALUE               ) ret=0;
  if(readSPI(SYNC0)            != FSK_SYNC0_VALUE               ) ret=0;
  if(readSPI(SYNC_CFG1)        != FSK_SYNC_CFG1_VALUE           ) ret=0;
  if(readSPI(SYNC_CFG0)        != FSK_SYNC_CFG0_VALUE           ) ret=0;
  if(readSPI(DEVIATION_M)      != FSK_DEVIATION_M_VALUE         ) ret=0;
  if(readSPI(MODCFG_DEV_E)     != FSK_MODCFG_DEV_E_VALUE        ) ret=0;
  if(readSPI(DCFILT_CFG)       != FSK_DCFILT_CFG_VALUE          ) ret=0;
  if(readSPI(PREAMBLE_CFG1)    != FSK_PREAMBLE_CFG1_VALUE       ) ret=0;
  if(readSPI(PREAMBLE_CFG0)    != FSK_PREAMBLE_CFG0_VALUE       ) ret=0;
  if(readSPI(FREQ_IF_CFG)      != FSK_FREQ_IF_CFG_VALUE         ) ret=0;
  if(readSPI(IQIC)             != FSK_IQIC_VALUE                ) ret=0;
  if(readSPI(CHAN_BW)          != FSK_CHAN_BW_VALUE             ) ret=0;
  if(readSPI(MDMCFG1)          != FSK_MDMCFG1_VALUE             ) ret=0;
  if(readSPI(MDMCFG0)          != FSK_MDMCFG0_VALUE             ) ret=0;
  if(readSPI(SYMBOL_RATE2)     != FSK_SYMBOL_RATE2_VALUE        ) ret=0;
  if(readSPI(SYMBOL_RATE1)     != FSK_SYMBOL_RATE1_VALUE        ) ret=0;
  if(readSPI(SYMBOL_RATE0)     != FSK_SYMBOL_RATE0_VALUE        ) ret=0;
  if(readSPI(AGC_REF)          != FSK_AGC_REF_VALUE             ) ret=0;
  if(readSPI(AGC_CS_THR)       != FSK_AGC_CS_THR_VALUE          ) ret=0;
  if(readSPI(AGC_GAIN_ADJUST)  != FSK_AGC_GAIN_ADJUST_VALUE     ) ret=0;
  if(readSPI(AGC_CFG3)         != FSK_AGC_CFG3_VALUE            ) ret=0;
  if(readSPI(AGC_CFG2)         != FSK_AGC_CFG2_VALUE            ) ret=0;
  if(readSPI(AGC_CFG1)         != FSK_AGC_CFG1_VALUE            ) ret=0;
  if(readSPI(AGC_CFG0)         != FSK_AGC_CFG0_VALUE            ) ret=0;
  if(readSPI(FIFO_CFG)         != FSK_FIFO_CFG_VALUE            ) ret=0;
  if(readSPI(DEV_ADDR)         != FSK_DEV_ADDR_VALUE            ) ret=0;
  if(readSPI(SETTLING_CFG)     != FSK_SETTLING_CFG_VALUE        ) ret=0;
  if(readSPI(FS_CFG)           != FSK_FS_CFG_VALUE              ) ret=0;
  if(readSPI(WOR_CFG1)         != FSK_WOR_CFG1_VALUE            ) ret=0;
  if(readSPI(WOR_CFG0)         != FSK_WOR_CFG0_VALUE            ) ret=0;
  if(readSPI(WOR_EVENT0_MSB)   != FSK_WOR_EVENT0_MSB_VALUE      ) ret=0;
  if(readSPI(WOR_EVENT0_LSB)   != FSK_WOR_EVENT0_LSB_VALUE      ) ret=0;
  if(readSPI(PKT_CFG2)         != FSK_PKT_CFG2_VALUE            ) ret=0;
  if(readSPI(PKT_CFG1)         != FSK_PKT_CFG1_VALUE            ) ret=0;
  if(readSPI(PKT_CFG0)         != FSK_PKT_CFG0_VALUE            ) ret=0;
  if(readSPI(RFEND_CFG1)       != FSK_RFEND_CFG1_VALUE          ) ret=0;
  if(readSPI(RFEND_CFG0)       != FSK_RFEND_CFG0_VALUE          ) ret=0;
  if(readSPI(PA_CFG2)          != FSK_PA_CFG2_VALUE             ) ret=0;
  if(readSPI(PA_CFG1)          != FSK_PA_CFG1_VALUE             ) ret=0;
  if(readSPI(PA_CFG0)          != FSK_PA_CFG0_VALUE             ) ret=0;
  if(readSPI(PKT_LEN)          != FSK_PKT_LEN_VALUE             ) ret=0;

  Serial.print(".");

  if(readExtAddrSPI(IF_MIX_CFG)       != FSK_IF_MIX_CFG_VALUE          ) ret=0;
  if(readExtAddrSPI(FREQOFF_CFG)      != FSK_FREQOFF_CFG_VALUE         ) ret=0;
  if(readExtAddrSPI(TOC_CFG)          != FSK_TOC_CFG_VALUE             ) ret=0;
  if(readExtAddrSPI(MARC_SPARE)       != FSK_MARC_SPARE_VALUE          ) ret=0;
  if(readExtAddrSPI(ECG_CFG)          != FSK_ECG_CFG_VALUE             ) ret=0;
  if(readExtAddrSPI(CFM_DATA_CFG)     != FSK_CFM_DATA_CFG_VALUE        ) ret=0;
  if(readExtAddrSPI(EXT_CTRL)         != FSK_EXT_CTRL_VALUE            ) ret=0;
  if(readExtAddrSPI(RCCAL_FINE)       != FSK_RCCAL_FINE_VALUE          ) ret=0;
  if(readExtAddrSPI(RCCAL_COARSE)     != FSK_RCCAL_COARSE_VALUE        ) ret=0;
  if(readExtAddrSPI(RCCAL_OFFSET)     != FSK_RCCAL_OFFSET_VALUE        ) ret=0;
  if(readExtAddrSPI(FREQOFF1)         != FSK_FREQOFF1_VALUE            ) ret=0;
  if(readExtAddrSPI(FREQOFF0)         != FSK_FREQOFF0_VALUE            ) ret=0;
  if(readExtAddrSPI(FREQ2)            != FSK_FREQ2_VALUE               ) ret=0;
  if(readExtAddrSPI(FREQ1)            != FSK_FREQ1_VALUE               ) ret=0;
  if(readExtAddrSPI(FREQ0)            != FSK_FREQ0_VALUE               ) ret=0;
  if(readExtAddrSPI(IF_ADC2)          != FSK_IF_ADC2_VALUE             ) ret=0;
  if(readExtAddrSPI(IF_ADC1)          != FSK_IF_ADC1_VALUE             ) ret=0;
  if(readExtAddrSPI(IF_ADC0)          != FSK_IF_ADC0_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_DIG1)          != FSK_FS_DIG1_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_DIG0)          != FSK_FS_DIG0_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_CAL3)          != FSK_FS_CAL3_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_CAL2)          != FSK_FS_CAL2_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_CAL1)          != FSK_FS_CAL1_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_CAL0)          != FSK_FS_CAL0_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_CHP)           != FSK_FS_CHP_VALUE              ) ret=0;
  if(readExtAddrSPI(FS_DIVTWO)        != FSK_FS_DIVTWO_VALUE           ) ret=0;
  if(readExtAddrSPI(FS_DSM1)          != FSK_FS_DSM1_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_DSM0)          != FSK_FS_DSM0_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_DVC1)          != FSK_FS_DVC1_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_DVC0)          != FSK_FS_DVC0_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_LBI)           != FSK_FS_LBI_VALUE              ) ret=0;
  if(readExtAddrSPI(FS_PFD)           != FSK_FS_PFD_VALUE              ) ret=0;
  if(readExtAddrSPI(FS_PRE)           != FSK_FS_PRE_VALUE              ) ret=0;
  if(readExtAddrSPI(FS_REG_DIV_CML)   != FSK_FS_REG_DIV_CML_VALUE      ) ret=0;
  if(readExtAddrSPI(FS_SPARE)         != FSK_FS_SPARE_VALUE            ) ret=0;
  if(readExtAddrSPI(FS_VCO4)          != FSK_FS_VCO4_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_VCO3)          != FSK_FS_VCO3_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_VCO2)          != FSK_FS_VCO2_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_VCO1)          != FSK_FS_VCO1_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_VCO0)          != FSK_FS_VCO0_VALUE             ) ret=0;
  if(readExtAddrSPI(GBIAS6)           != FSK_GBIAS6_VALUE              ) ret=0;
  if(readExtAddrSPI(GBIAS5)           != FSK_GBIAS5_VALUE              ) ret=0;
  if(readExtAddrSPI(GBIAS4)           != FSK_GBIAS4_VALUE              ) ret=0;
  if(readExtAddrSPI(GBIAS3)           != FSK_GBIAS3_VALUE              ) ret=0;
  if(readExtAddrSPI(GBIAS2)           != FSK_GBIAS2_VALUE              ) ret=0;
  if(readExtAddrSPI(GBIAS1)           != FSK_GBIAS1_VALUE              ) ret=0;
  if(readExtAddrSPI(GBIAS0)           != FSK_GBIAS0_VALUE              ) ret=0;
  if(readExtAddrSPI(IFAMP)            != FSK_IFAMP_VALUE               ) ret=0;
  if(readExtAddrSPI(LNA)              != FSK_LNA_VALUE                 ) ret=0;
  if(readExtAddrSPI(RXMIX)            != FSK_RXMIX_VALUE               ) ret=0;
  if(readExtAddrSPI(XOSC5)            != FSK_XOSC5_VALUE               ) ret=0;
  if(readExtAddrSPI(XOSC4)            != FSK_XOSC4_VALUE               ) ret=0;
  if(readExtAddrSPI(XOSC3)            != FSK_XOSC3_VALUE               ) ret=0;
  if(readExtAddrSPI(XOSC2)            != FSK_XOSC2_VALUE               ) ret=0;
  if(readExtAddrSPI(XOSC1)            != FSK_XOSC1_VALUE               ) ret=0;
  if(readExtAddrSPI(XOSC0)            != FSK_XOSC0_VALUE               ) ret=0;
  if(readExtAddrSPI(ANALOG_SPARE)     != FSK_ANALOG_SPARE_VALUE        ) ret=0;
  if(readExtAddrSPI(PA_CFG3)          != FSK_PA_CFG3_VALUE             ) ret=0;
  if(readExtAddrSPI(WOR_TIME1)        != FSK_WOR_TIME1_VALUE           ) ret=0;
  if(readExtAddrSPI(WOR_TIME0)        != FSK_WOR_TIME0_VALUE           ) ret=0;
  if(readExtAddrSPI(WOR_CAPTURE1)     != FSK_WOR_CAPTURE1_VALUE        ) ret=0;
  if(readExtAddrSPI(WOR_CAPTURE0)     != FSK_WOR_CAPTURE0_VALUE        ) ret=0;
  if(readExtAddrSPI(BIST)             != FSK_BIST_VALUE                ) ret=0;
  if(readExtAddrSPI(DCFILTOFFSET_I1)  != FSK_DCFILTOFFSET_I1_VALUE     ) ret=0;
  if(readExtAddrSPI(DCFILTOFFSET_I0)  != FSK_DCFILTOFFSET_I0_VALUE     ) ret=0;
  if(readExtAddrSPI(DCFILTOFFSET_Q1)  != FSK_DCFILTOFFSET_Q1_VALUE     ) ret=0;
  if(readExtAddrSPI(DCFILTOFFSET_Q0)  != FSK_DCFILTOFFSET_Q0_VALUE     ) ret=0;
  if(readExtAddrSPI(IQIE_I1)          != FSK_IQIE_I1_VALUE             ) ret=0;
  if(readExtAddrSPI(IQIE_I0)          != FSK_IQIE_I0_VALUE             ) ret=0;
  if(readExtAddrSPI(IQIE_Q1)          != FSK_IQIE_Q1_VALUE             ) ret=0;
  if(readExtAddrSPI(IQIE_Q0)          != FSK_IQIE_Q0_VALUE             ) ret=0;
  if(readExtAddrSPI(RSSI1)            != FSK_RSSI1_VALUE               ) ret=0;
  if(readExtAddrSPI(RSSI0)            != FSK_RSSI0_VALUE               ) ret=0;
  if(readExtAddrSPI(MARCSTATE)        != FSK_MARCSTATE_VALUE           ) ret=0;
  if(readExtAddrSPI(LQI_VAL)          != FSK_LQI_VAL_VALUE             ) ret=0;
  if(readExtAddrSPI(PQT_SYNC_ERR)     != FSK_PQT_SYNC_ERR_VALUE        ) ret=0;
  if(readExtAddrSPI(DEM_STATUS)       != FSK_DEM_STATUS_VALUE          ) ret=0;
  if(readExtAddrSPI(FREQOFF_EST1)     != FSK_FREQOFF_EST1_VALUE        ) ret=0;
  if(readExtAddrSPI(FREQOFF_EST0)     != FSK_FREQOFF_EST0_VALUE        ) ret=0;
  if(readExtAddrSPI(AGC_GAIN3)        != FSK_AGC_GAIN3_VALUE           ) ret=0;
  if(readExtAddrSPI(AGC_GAIN2)        != FSK_AGC_GAIN2_VALUE           ) ret=0;
  if(readExtAddrSPI(AGC_GAIN1)        != FSK_AGC_GAIN1_VALUE           ) ret=0;
  if(readExtAddrSPI(AGC_GAIN0)        != FSK_AGC_GAIN0_VALUE           ) ret=0;
  if(readExtAddrSPI(CFM_RX_DATA_OUT)  != FSK_CFM_RX_DATA_OUT_VALUE     ) ret=0;
  if(readExtAddrSPI(CFM_TX_DATA_IN)   != FSK_CFM_TX_DATA_IN_VALUE      ) ret=0;
  if(readExtAddrSPI(ASK_SOFT_RX_DATA) != FSK_ASK_SOFT_RX_DATA_VALUE    ) ret=0;
  if(readExtAddrSPI(RNDGEN)           != FSK_RNDGEN_VALUE              ) ret=0;
  if(readExtAddrSPI(MAGN2)            != FSK_MAGN2_VALUE               ) ret=0;
  if(readExtAddrSPI(MAGN1)            != FSK_MAGN1_VALUE               ) ret=0;
  if(readExtAddrSPI(MAGN0)            != FSK_MAGN0_VALUE               ) ret=0;
  if(readExtAddrSPI(ANG1)             != FSK_ANG1_VALUE                ) ret=0;
  if(readExtAddrSPI(ANG0)             != FSK_ANG0_VALUE                ) ret=0;
  if(readExtAddrSPI(CHFILT_I2)        != FSK_CHFILT_I2_VALUE           ) ret=0;
  if(readExtAddrSPI(CHFILT_I1)        != FSK_CHFILT_I1_VALUE           ) ret=0;
  if(readExtAddrSPI(CHFILT_I0)        != FSK_CHFILT_I0_VALUE           ) ret=0;
  if(readExtAddrSPI(CHFILT_Q2)        != FSK_CHFILT_Q2_VALUE           ) ret=0;
  if(readExtAddrSPI(CHFILT_Q1)        != FSK_CHFILT_Q1_VALUE           ) ret=0;
  if(readExtAddrSPI(CHFILT_Q0)        != FSK_CHFILT_Q0_VALUE           ) ret=0;
  if(readExtAddrSPI(GPIO_STATUS)      != FSK_GPIO_STATUS_VALUE         ) ret=0;
  if(readExtAddrSPI(FSCAL_CTRL)       != FSK_FSCAL_CTRL_VALUE          ) ret=0;
  if(readExtAddrSPI(PHASE_ADJUST)     != FSK_PHASE_ADJUST_VALUE        ) ret=0;
  // if(readExtAddrSPI(PARTNUMBER)       != FSK_PARTNUMBER_VALUE          ) ret=0;
  // if(readExtAddrSPI(PARTVERSION)      != FSK_PARTVERSION_VALUE         ) ret=0;
  if(readExtAddrSPI(SERIAL_STATUS)    != FSK_SERIAL_STATUS_VALUE       ) ret=0;
  if(readExtAddrSPI(MODEM_STATUS1)    != FSK_MODEM_STATUS1_VALUE       ) ret=0;
  if(readExtAddrSPI(MODEM_STATUS0)    != FSK_MODEM_STATUS0_VALUE       ) ret=0;
  if(readExtAddrSPI(MARC_STATUS1)     != FSK_MARC_STATUS1_VALUE        ) ret=0;
  if(readExtAddrSPI(MARC_STATUS0)     != FSK_MARC_STATUS0_VALUE        ) ret=0;
  if(readExtAddrSPI(PA_IFAMP_TEST)    != FSK_PA_IFAMP_TEST_VALUE       ) ret=0;
  if(readExtAddrSPI(FSRF_TEST)        != FSK_FSRF_TEST_VALUE           ) ret=0;
  if(readExtAddrSPI(PRE_TEST)         != FSK_PRE_TEST_VALUE            ) ret=0;
  if(readExtAddrSPI(PRE_OVR)          != FSK_PRE_OVR_VALUE             ) ret=0;
  if(readExtAddrSPI(ADC_TEST)         != FSK_ADC_TEST_VALUE            ) ret=0;
  if(readExtAddrSPI(DVC_TEST)         != FSK_DVC_TEST_VALUE            ) ret=0;
  if(readExtAddrSPI(ATEST)            != FSK_ATEST_VALUE               ) ret=0;
  if(readExtAddrSPI(ATEST_LVDS)       != FSK_ATEST_LVDS_VALUE          ) ret=0;
  if(readExtAddrSPI(ATEST_MODE)       != FSK_ATEST_MODE_VALUE          ) ret=0;
  if(readExtAddrSPI(XOSC_TEST1)       != FSK_XOSC_TEST1_VALUE          ) ret=0;
  if(readExtAddrSPI(XOSC_TEST0)       != FSK_XOSC_TEST0_VALUE          ) ret=0;
  // if(readExtAddrSPI(RXFIRST)          != FSK_RXFIRST_VALUE             ) ret=0;
  // if(readExtAddrSPI(TXFIRST)          != FSK_TXFIRST_VALUE             ) ret=0;
  // if(readExtAddrSPI(RXLAST)           != FSK_RXLAST_VALUE              ) ret=0;
  // if(readExtAddrSPI(TXLAST)           != FSK_TXLAST_VALUE              ) ret=0;
  // if(readExtAddrSPI(NUM_TXBYTES)      != FSK_NUM_TXBYTES_VALUE         ) ret=0;
  // if(readExtAddrSPI(NUM_RXBYTES)      != FSK_NUM_RXBYTES_VALUE         ) ret=0;
  // if(readExtAddrSPI(FIFO_NUM_TXBYTES) != FSK_FIFO_NUM_TXBYTES_VALUE    ) ret=0;
  // if(readExtAddrSPI(FIFO_NUM_RXBYTES) != FSK_FIFO_NUM_RXBYTES_VALUE    ) ret=0;
  Serial.println(".");

  Serial.println("Calibrating...");
  strobeSPI(SCAL); //Calibrate frequency synthesizer and turn it off
 
  ret = waitIDLE(ret, waitTime);
  return ret;
}

bool CC1120Class::setCW(){
  writeSPI(IOCFG3, CW_IOCFG3_VALUE);
  writeSPI(IOCFG2, CW_IOCFG2_VALUE);
  writeSPI(IOCFG1, CW_IOCFG1_VALUE);
  writeSPI(IOCFG0, CW_IOCFG0_VALUE);
  writeSPI(SYNC3, CW_SYNC3_VALUE);
  writeSPI(SYNC2, CW_SYNC2_VALUE);
  writeSPI(SYNC1, CW_SYNC1_VALUE);
  writeSPI(SYNC0, CW_SYNC0_VALUE);
  writeSPI(SYNC_CFG1, CW_SYNC_CFG1_VALUE);
  writeSPI(SYNC_CFG0, CW_SYNC_CFG0_VALUE);
  writeSPI(DEVIATION_M, CW_DEVIATION_M_VALUE);
  writeSPI(MODCFG_DEV_E, CW_MODCFG_DEV_E_VALUE);
  writeSPI(DCFILT_CFG, CW_DCFILT_CFG_VALUE);
  writeSPI(PREAMBLE_CFG1, CW_PREAMBLE_CFG1_VALUE);
  writeSPI(PREAMBLE_CFG0, CW_PREAMBLE_CFG0_VALUE);
  writeSPI(FREQ_IF_CFG, CW_FREQ_IF_CFG_VALUE);
  writeSPI(IQIC, CW_IQIC_VALUE);
  writeSPI(CHAN_BW, CW_CHAN_BW_VALUE);
  writeSPI(MDMCFG1, CW_MDMCFG1_VALUE);
  writeSPI(MDMCFG0, CW_MDMCFG0_VALUE);
  writeSPI(SYMBOL_RATE2, CW_SYMBOL_RATE2_VALUE);
  writeSPI(SYMBOL_RATE1, CW_SYMBOL_RATE1_VALUE);
  writeSPI(SYMBOL_RATE0, CW_SYMBOL_RATE0_VALUE);
  writeSPI(AGC_REF, CW_AGC_REF_VALUE);
  writeSPI(AGC_CS_THR, CW_AGC_CS_THR_VALUE);
  writeSPI(AGC_GAIN_ADJUST, CW_AGC_GAIN_ADJUST_VALUE);
  writeSPI(AGC_CFG3, CW_AGC_CFG3_VALUE);
  writeSPI(AGC_CFG2, CW_AGC_CFG2_VALUE);
  writeSPI(AGC_CFG1, CW_AGC_CFG1_VALUE);
  writeSPI(AGC_CFG0, CW_AGC_CFG0_VALUE);
  writeSPI(FIFO_CFG, CW_FIFO_CFG_VALUE);
  writeSPI(DEV_ADDR, CW_DEV_ADDR_VALUE);
  writeSPI(SETTLING_CFG, CW_SETTLING_CFG_VALUE);
  writeSPI(FS_CFG, CW_FS_CFG_VALUE);
  writeSPI(WOR_CFG1, CW_WOR_CFG1_VALUE);
  writeSPI(WOR_CFG0, CW_WOR_CFG0_VALUE);
  writeSPI(WOR_EVENT0_MSB, CW_WOR_EVENT0_MSB_VALUE);
  writeSPI(WOR_EVENT0_LSB, CW_WOR_EVENT0_LSB_VALUE);
  writeSPI(PKT_CFG2, CW_PKT_CFG2_VALUE);
  writeSPI(PKT_CFG1, CW_PKT_CFG1_VALUE);
  writeSPI(PKT_CFG0, CW_PKT_CFG0_VALUE);
  writeSPI(RFEND_CFG1, CW_RFEND_CFG1_VALUE);
  writeSPI(RFEND_CFG0, CW_RFEND_CFG0_VALUE);
  writeSPI(PA_CFG2, CW_PA_CFG2_VALUE);
  writeSPI(PA_CFG1, CW_PA_CFG1_VALUE);
  writeSPI(PA_CFG0, CW_PA_CFG0_VALUE);
  writeSPI(PKT_LEN, CW_PKT_LEN_VALUE);

  writeExtAddrSPI(FREQOFF_CFG, CW_FREQOFF_CFG_VALUE);
  writeExtAddrSPI(TOC_CFG, CW_TOC_CFG_VALUE);
  writeExtAddrSPI(MARC_SPARE, CW_MARC_SPARE_VALUE);
  writeExtAddrSPI(ECG_CFG, CW_ECG_CFG_VALUE);
  writeExtAddrSPI(CFM_DATA_CFG, CW_CFM_DATA_CFG_VALUE);
  writeExtAddrSPI(EXT_CTRL, CW_EXT_CTRL_VALUE);
  writeExtAddrSPI(RCCAL_FINE, CW_RCCAL_FINE_VALUE);
  writeExtAddrSPI(RCCAL_COARSE, CW_RCCAL_COARSE_VALUE);
  writeExtAddrSPI(RCCAL_OFFSET, CW_RCCAL_OFFSET_VALUE);
  writeExtAddrSPI(FREQOFF1, CW_FREQOFF1_VALUE);
  writeExtAddrSPI(FREQOFF0, CW_FREQOFF0_VALUE);
  writeExtAddrSPI(FREQ2, CW_FREQ2_VALUE);
  writeExtAddrSPI(FREQ1, CW_FREQ1_VALUE);
  writeExtAddrSPI(FREQ0, CW_FREQ0_VALUE);
  writeExtAddrSPI(IF_ADC2, CW_IF_ADC2_VALUE);
  writeExtAddrSPI(IF_ADC1, CW_IF_ADC1_VALUE);
  writeExtAddrSPI(IF_ADC0, CW_IF_ADC0_VALUE);
  writeExtAddrSPI(FS_DIG1, CW_FS_DIG1_VALUE);
  writeExtAddrSPI(FS_DIG0, CW_FS_DIG0_VALUE);
  writeExtAddrSPI(FS_CAL3, CW_FS_CAL3_VALUE);
  writeExtAddrSPI(FS_CAL2, CW_FS_CAL2_VALUE);
  writeExtAddrSPI(FS_CAL1, CW_FS_CAL1_VALUE);
  writeExtAddrSPI(FS_CAL0, CW_FS_CAL0_VALUE);
  writeExtAddrSPI(FS_CHP, CW_FS_CHP_VALUE);
  writeExtAddrSPI(FS_DIVTWO, CW_FS_DIVTWO_VALUE);
  writeExtAddrSPI(FS_DSM1, CW_FS_DSM1_VALUE);
  writeExtAddrSPI(FS_DSM0, CW_FS_DSM0_VALUE);
  writeExtAddrSPI(FS_DVC1, CW_FS_DVC1_VALUE);
  writeExtAddrSPI(FS_DVC0, CW_FS_DVC0_VALUE);
  writeExtAddrSPI(FS_LBI, CW_FS_LBI_VALUE);
  writeExtAddrSPI(FS_PFD, CW_FS_PFD_VALUE);
  writeExtAddrSPI(FS_PRE, CW_FS_PRE_VALUE);
  writeExtAddrSPI(FS_REG_DIV_CML, CW_FS_REG_DIV_CML_VALUE);
  writeExtAddrSPI(FS_SPARE, CW_FS_SPARE_VALUE);
  writeExtAddrSPI(FS_VCO4, CW_FS_VCO4_VALUE);
  writeExtAddrSPI(FS_VCO3, CW_FS_VCO3_VALUE);
  writeExtAddrSPI(FS_VCO2, CW_FS_VCO2_VALUE);
  writeExtAddrSPI(FS_VCO1, CW_FS_VCO1_VALUE);
  writeExtAddrSPI(FS_VCO0, CW_FS_VCO0_VALUE);
  writeExtAddrSPI(GBIAS6, CW_GBIAS6_VALUE);
  writeExtAddrSPI(GBIAS5, CW_GBIAS5_VALUE);
  writeExtAddrSPI(GBIAS4, CW_GBIAS4_VALUE);
  writeExtAddrSPI(GBIAS3, CW_GBIAS3_VALUE);
  writeExtAddrSPI(GBIAS2, CW_GBIAS2_VALUE);
  writeExtAddrSPI(GBIAS1, CW_GBIAS1_VALUE);
  writeExtAddrSPI(GBIAS0, CW_GBIAS0_VALUE);
  writeExtAddrSPI(IFAMP, CW_IFAMP_VALUE);
  writeExtAddrSPI(LNA, CW_LNA_VALUE);
  writeExtAddrSPI(RXMIX, CW_RXMIX_VALUE);
  writeExtAddrSPI(XOSC5, CW_XOSC5_VALUE);
  writeExtAddrSPI(XOSC4, CW_XOSC4_VALUE);
  writeExtAddrSPI(XOSC3, CW_XOSC3_VALUE);
  writeExtAddrSPI(XOSC2, CW_XOSC2_VALUE);
  writeExtAddrSPI(XOSC1, CW_XOSC1_VALUE);
  writeExtAddrSPI(XOSC0, CW_XOSC0_VALUE);
  writeExtAddrSPI(ANALOG_SPARE, CW_ANALOG_SPARE_VALUE);
  writeExtAddrSPI(PA_CFG3, CW_PA_CFG3_VALUE);
  writeExtAddrSPI(WOR_TIME1, CW_WOR_TIME1_VALUE);
  writeExtAddrSPI(WOR_TIME0, CW_WOR_TIME0_VALUE);
  writeExtAddrSPI(WOR_CAPTURE1, CW_WOR_CAPTURE1_VALUE);
  writeExtAddrSPI(WOR_CAPTURE0, CW_WOR_CAPTURE0_VALUE);
  writeExtAddrSPI(BIST, CW_BIST_VALUE);
  writeExtAddrSPI(DCFILTOFFSET_I1, CW_DCFILTOFFSET_I1_VALUE);
  writeExtAddrSPI(DCFILTOFFSET_I0, CW_DCFILTOFFSET_I0_VALUE);
  writeExtAddrSPI(DCFILTOFFSET_Q1, CW_DCFILTOFFSET_Q1_VALUE);
  writeExtAddrSPI(DCFILTOFFSET_Q0, CW_DCFILTOFFSET_Q0_VALUE);
  writeExtAddrSPI(IQIE_I1, CW_IQIE_I1_VALUE);
  writeExtAddrSPI(IQIE_I0, CW_IQIE_I0_VALUE);
  writeExtAddrSPI(IQIE_Q1, CW_IQIE_Q1_VALUE);
  writeExtAddrSPI(IQIE_Q0, CW_IQIE_Q0_VALUE);
  writeExtAddrSPI(RSSI1, CW_RSSI1_VALUE);
  writeExtAddrSPI(RSSI0, CW_RSSI0_VALUE);
  writeExtAddrSPI(MARCSTATE, CW_MARCSTATE_VALUE);
  writeExtAddrSPI(LQI_VAL, CW_LQI_VAL_VALUE);
  writeExtAddrSPI(PQT_SYNC_ERR, CW_PQT_SYNC_ERR_VALUE);
  writeExtAddrSPI(DEM_STATUS, CW_DEM_STATUS_VALUE);
  writeExtAddrSPI(FREQOFF_EST1, CW_FREQOFF_EST1_VALUE);
  writeExtAddrSPI(FREQOFF_EST0, CW_FREQOFF_EST0_VALUE);
  writeExtAddrSPI(AGC_GAIN3, CW_AGC_GAIN3_VALUE);
  writeExtAddrSPI(AGC_GAIN2, CW_AGC_GAIN2_VALUE);
  writeExtAddrSPI(AGC_GAIN1, CW_AGC_GAIN1_VALUE);
  writeExtAddrSPI(AGC_GAIN0, CW_AGC_GAIN0_VALUE);
  writeExtAddrSPI(CFM_RX_DATA_OUT, CW_CFM_RX_DATA_OUT_VALUE);
  writeExtAddrSPI(CFM_TX_DATA_IN, CW_CFM_TX_DATA_IN_VALUE);
  writeExtAddrSPI(ASK_SOFT_RX_DATA, CW_ASK_SOFT_RX_DATA_VALUE);
  writeExtAddrSPI(RNDGEN, CW_RNDGEN_VALUE);
  writeExtAddrSPI(MAGN2, CW_MAGN2_VALUE);
  writeExtAddrSPI(MAGN1, CW_MAGN1_VALUE);
  writeExtAddrSPI(MAGN0, CW_MAGN0_VALUE);
  writeExtAddrSPI(ANG1, CW_ANG1_VALUE);
  writeExtAddrSPI(ANG0, CW_ANG0_VALUE);
  writeExtAddrSPI(CHFILT_I2, CW_CHFILT_I2_VALUE);
  writeExtAddrSPI(CHFILT_I1, CW_CHFILT_I1_VALUE);
  writeExtAddrSPI(CHFILT_I0, CW_CHFILT_I0_VALUE);
  writeExtAddrSPI(CHFILT_Q2, CW_CHFILT_Q2_VALUE);
  writeExtAddrSPI(CHFILT_Q1, CW_CHFILT_Q1_VALUE);
  writeExtAddrSPI(CHFILT_Q0, CW_CHFILT_Q0_VALUE);
  writeExtAddrSPI(GPIO_STATUS, CW_GPIO_STATUS_VALUE);
  writeExtAddrSPI(FSCAL_CTRL, CW_FSCAL_CTRL_VALUE);
  writeExtAddrSPI(PHASE_ADJUST, CW_PHASE_ADJUST_VALUE);
  // writeExtAddrSPI(PARTNUMBER, PARTNUMBER_VALUE);
  // writeExtAddrSPI(PARTVERSION, PARTVERSION_VALUE);
  writeExtAddrSPI(SERIAL_STATUS, CW_SERIAL_STATUS_VALUE);
  writeExtAddrSPI(MODEM_STATUS1, CW_MODEM_STATUS1_VALUE);
  writeExtAddrSPI(MODEM_STATUS0, CW_MODEM_STATUS0_VALUE);
  writeExtAddrSPI(MARC_STATUS1, CW_MARC_STATUS1_VALUE);
  writeExtAddrSPI(MARC_STATUS0, CW_MARC_STATUS0_VALUE);
  writeExtAddrSPI(PA_IFAMP_TEST, CW_PA_IFAMP_TEST_VALUE);
  writeExtAddrSPI(FSRF_TEST, CW_FSRF_TEST_VALUE);
  writeExtAddrSPI(PRE_TEST, CW_PRE_TEST_VALUE);
  writeExtAddrSPI(PRE_OVR, CW_PRE_OVR_VALUE);
  writeExtAddrSPI(ADC_TEST, CW_ADC_TEST_VALUE);
  writeExtAddrSPI(DVC_TEST, CW_DVC_TEST_VALUE);
  writeExtAddrSPI(ATEST, CW_ATEST_VALUE);
  writeExtAddrSPI(ATEST_LVDS, CW_ATEST_LVDS_VALUE);
  writeExtAddrSPI(ATEST_MODE, CW_ATEST_MODE_VALUE);
  writeExtAddrSPI(XOSC_TEST1, CW_XOSC_TEST1_VALUE);
  writeExtAddrSPI(XOSC_TEST0, CW_XOSC_TEST0_VALUE);
  // writeExtAddrSPI(RXFIRST, RXFIRST_VALUE);
  // writeExtAddrSPI(TXFIRST, TXFIRST_VALUE);
  // writeExtAddrSPI(RXLAST, RXLAST_VALUE);
  // writeExtAddrSPI(TXLAST, TXLAST_VALUE);
  // writeExtAddrSPI(NUM_TXBYTES, NUM_TXBYTES_VALUE);
  // writeExtAddrSPI(NUM_RXBYTES, NUM_RXBYTES_VALUE);
  // writeExtAddrSPI(FIFO_NUM_TXBYTES, FIFO_NUM_TXBYTES_VALUE);
  // writeExtAddrSPI(FIFO_NUM_RXBYTES, FIFO_NUM_RXBYTES_VALUE);



  bool ret = 1;

  if(readSPI(IOCFG3)           != CW_IOCFG3_VALUE              ) ret=0;
  if(readSPI(IOCFG2)           != CW_IOCFG2_VALUE              ) ret=0;
  if(readSPI(IOCFG1)           != CW_IOCFG1_VALUE              ) ret=0;
  if(readSPI(IOCFG0)           != CW_IOCFG0_VALUE              ) ret=0;
  if(readSPI(SYNC3)            != CW_SYNC3_VALUE               ) ret=0;
  if(readSPI(SYNC2)            != CW_SYNC2_VALUE               ) ret=0;
  if(readSPI(SYNC1)            != CW_SYNC1_VALUE               ) ret=0;
  if(readSPI(SYNC0)            != CW_SYNC0_VALUE               ) ret=0;
  if(readSPI(SYNC_CFG1)        != CW_SYNC_CFG1_VALUE           ) ret=0;
  if(readSPI(SYNC_CFG0)        != CW_SYNC_CFG0_VALUE           ) ret=0;
  if(readSPI(DEVIATION_M)      != CW_DEVIATION_M_VALUE         ) ret=0;
  if(readSPI(MODCFG_DEV_E)     != CW_MODCFG_DEV_E_VALUE        ) ret=0;
  if(readSPI(DCFILT_CFG)       != CW_DCFILT_CFG_VALUE          ) ret=0;
  if(readSPI(PREAMBLE_CFG1)    != CW_PREAMBLE_CFG1_VALUE       ) ret=0;
  if(readSPI(PREAMBLE_CFG0)    != CW_PREAMBLE_CFG0_VALUE       ) ret=0;
  if(readSPI(FREQ_IF_CFG)      != CW_FREQ_IF_CFG_VALUE         ) ret=0;
  if(readSPI(IQIC)             != CW_IQIC_VALUE                ) ret=0;
  if(readSPI(CHAN_BW)          != CW_CHAN_BW_VALUE             ) ret=0;
  if(readSPI(MDMCFG1)          != CW_MDMCFG1_VALUE             ) ret=0;
  if(readSPI(MDMCFG0)          != CW_MDMCFG0_VALUE             ) ret=0;
  if(readSPI(SYMBOL_RATE2)     != CW_SYMBOL_RATE2_VALUE        ) ret=0;
  if(readSPI(SYMBOL_RATE1)     != CW_SYMBOL_RATE1_VALUE        ) ret=0;
  if(readSPI(SYMBOL_RATE0)     != CW_SYMBOL_RATE0_VALUE        ) ret=0;
  if(readSPI(AGC_REF)          != CW_AGC_REF_VALUE             ) ret=0;
  if(readSPI(AGC_CS_THR)       != CW_AGC_CS_THR_VALUE          ) ret=0;
  if(readSPI(AGC_GAIN_ADJUST)  != CW_AGC_GAIN_ADJUST_VALUE     ) ret=0;
  if(readSPI(AGC_CFG3)         != CW_AGC_CFG3_VALUE            ) ret=0;
  if(readSPI(AGC_CFG2)         != CW_AGC_CFG2_VALUE            ) ret=0;
  if(readSPI(AGC_CFG1)         != CW_AGC_CFG1_VALUE            ) ret=0;
  if(readSPI(AGC_CFG0)         != CW_AGC_CFG0_VALUE            ) ret=0;
  if(readSPI(FIFO_CFG)         != CW_FIFO_CFG_VALUE            ) ret=0;
  if(readSPI(DEV_ADDR)         != CW_DEV_ADDR_VALUE            ) ret=0;
  if(readSPI(SETTLING_CFG)     != CW_SETTLING_CFG_VALUE        ) ret=0;
  if(readSPI(FS_CFG)           != CW_FS_CFG_VALUE              ) ret=0;
  if(readSPI(WOR_CFG1)         != CW_WOR_CFG1_VALUE            ) ret=0;
  if(readSPI(WOR_CFG0)         != CW_WOR_CFG0_VALUE            ) ret=0;
  if(readSPI(WOR_EVENT0_MSB)   != CW_WOR_EVENT0_MSB_VALUE      ) ret=0;
  if(readSPI(WOR_EVENT0_LSB)   != CW_WOR_EVENT0_LSB_VALUE      ) ret=0;
  if(readSPI(PKT_CFG2)         != CW_PKT_CFG2_VALUE            ) ret=0;
  if(readSPI(PKT_CFG1)         != CW_PKT_CFG1_VALUE            ) ret=0;
  if(readSPI(PKT_CFG0)         != CW_PKT_CFG0_VALUE            ) ret=0;
  if(readSPI(RFEND_CFG1)       != CW_RFEND_CFG1_VALUE          ) ret=0;
  if(readSPI(RFEND_CFG0)       != CW_RFEND_CFG0_VALUE          ) ret=0;
  if(readSPI(PA_CFG2)          != CW_PA_CFG2_VALUE             ) ret=0;
  if(readSPI(PA_CFG1)          != CW_PA_CFG1_VALUE             ) ret=0;
  if(readSPI(PA_CFG0)          != CW_PA_CFG0_VALUE             ) ret=0;
  if(readSPI(PKT_LEN)          != CW_PKT_LEN_VALUE             ) ret=0;



  if(readExtAddrSPI(IF_MIX_CFG)       != CW_IF_MIX_CFG_VALUE          ) ret=0;
  if(readExtAddrSPI(FREQOFF_CFG)      != CW_FREQOFF_CFG_VALUE         ) ret=0;
  if(readExtAddrSPI(TOC_CFG)          != CW_TOC_CFG_VALUE             ) ret=0;
  if(readExtAddrSPI(MARC_SPARE)       != CW_MARC_SPARE_VALUE          ) ret=0;
  if(readExtAddrSPI(ECG_CFG)          != CW_ECG_CFG_VALUE             ) ret=0;
  if(readExtAddrSPI(CFM_DATA_CFG)     != CW_CFM_DATA_CFG_VALUE        ) ret=0;
  if(readExtAddrSPI(EXT_CTRL)         != CW_EXT_CTRL_VALUE            ) ret=0;
  if(readExtAddrSPI(RCCAL_FINE)       != CW_RCCAL_FINE_VALUE          ) ret=0;
  if(readExtAddrSPI(RCCAL_COARSE)     != CW_RCCAL_COARSE_VALUE        ) ret=0;
  if(readExtAddrSPI(RCCAL_OFFSET)     != CW_RCCAL_OFFSET_VALUE        ) ret=0;
  if(readExtAddrSPI(FREQOFF1)         != CW_FREQOFF1_VALUE            ) ret=0;
  if(readExtAddrSPI(FREQOFF0)         != CW_FREQOFF0_VALUE            ) ret=0;
  if(readExtAddrSPI(FREQ2)            != CW_FREQ2_VALUE               ) ret=0;
  if(readExtAddrSPI(FREQ1)            != CW_FREQ1_VALUE               ) ret=0;
  if(readExtAddrSPI(FREQ0)            != CW_FREQ0_VALUE               ) ret=0;
  if(readExtAddrSPI(IF_ADC2)          != CW_IF_ADC2_VALUE             ) ret=0;
  if(readExtAddrSPI(IF_ADC1)          != CW_IF_ADC1_VALUE             ) ret=0;
  if(readExtAddrSPI(IF_ADC0)          != CW_IF_ADC0_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_DIG1)          != CW_FS_DIG1_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_DIG0)          != CW_FS_DIG0_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_CAL3)          != CW_FS_CAL3_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_CAL2)          != CW_FS_CAL2_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_CAL1)          != CW_FS_CAL1_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_CAL0)          != CW_FS_CAL0_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_CHP)           != CW_FS_CHP_VALUE              ) ret=0;
  if(readExtAddrSPI(FS_DIVTWO)        != CW_FS_DIVTWO_VALUE           ) ret=0;
  if(readExtAddrSPI(FS_DSM1)          != CW_FS_DSM1_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_DSM0)          != CW_FS_DSM0_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_DVC1)          != CW_FS_DVC1_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_DVC0)          != CW_FS_DVC0_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_LBI)           != CW_FS_LBI_VALUE              ) ret=0;
  if(readExtAddrSPI(FS_PFD)           != CW_FS_PFD_VALUE              ) ret=0;
  if(readExtAddrSPI(FS_PRE)           != CW_FS_PRE_VALUE              ) ret=0;
  if(readExtAddrSPI(FS_REG_DIV_CML)   != CW_FS_REG_DIV_CML_VALUE      ) ret=0;
  if(readExtAddrSPI(FS_SPARE)         != CW_FS_SPARE_VALUE            ) ret=0;
  if(readExtAddrSPI(FS_VCO4)          != CW_FS_VCO4_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_VCO3)          != CW_FS_VCO3_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_VCO2)          != CW_FS_VCO2_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_VCO1)          != CW_FS_VCO1_VALUE             ) ret=0;
  if(readExtAddrSPI(FS_VCO0)          != CW_FS_VCO0_VALUE             ) ret=0;
  if(readExtAddrSPI(GBIAS6)           != CW_GBIAS6_VALUE              ) ret=0;
  if(readExtAddrSPI(GBIAS5)           != CW_GBIAS5_VALUE              ) ret=0;
  if(readExtAddrSPI(GBIAS4)           != CW_GBIAS4_VALUE              ) ret=0;
  if(readExtAddrSPI(GBIAS3)           != CW_GBIAS3_VALUE              ) ret=0;
  if(readExtAddrSPI(GBIAS2)           != CW_GBIAS2_VALUE              ) ret=0;
  if(readExtAddrSPI(GBIAS1)           != CW_GBIAS1_VALUE              ) ret=0;
  if(readExtAddrSPI(GBIAS0)           != CW_GBIAS0_VALUE              ) ret=0;
  if(readExtAddrSPI(IFAMP)            != CW_IFAMP_VALUE               ) ret=0;
  if(readExtAddrSPI(LNA)              != CW_LNA_VALUE                 ) ret=0;
  if(readExtAddrSPI(RXMIX)            != CW_RXMIX_VALUE               ) ret=0;
  if(readExtAddrSPI(XOSC5)            != CW_XOSC5_VALUE               ) ret=0;
  if(readExtAddrSPI(XOSC4)            != CW_XOSC4_VALUE               ) ret=0;
  if(readExtAddrSPI(XOSC3)            != CW_XOSC3_VALUE               ) ret=0;
  if(readExtAddrSPI(XOSC2)            != CW_XOSC2_VALUE               ) ret=0;
  if(readExtAddrSPI(XOSC1)            != CW_XOSC1_VALUE               ) ret=0;
  if(readExtAddrSPI(XOSC0)            != CW_XOSC0_VALUE               ) ret=0;
  if(readExtAddrSPI(ANALOG_SPARE)     != CW_ANALOG_SPARE_VALUE        ) ret=0;
  if(readExtAddrSPI(PA_CFG3)          != CW_PA_CFG3_VALUE             ) ret=0;
  if(readExtAddrSPI(WOR_TIME1)        != CW_WOR_TIME1_VALUE           ) ret=0;
  if(readExtAddrSPI(WOR_TIME0)        != CW_WOR_TIME0_VALUE           ) ret=0;
  if(readExtAddrSPI(WOR_CAPTURE1)     != CW_WOR_CAPTURE1_VALUE        ) ret=0;
  if(readExtAddrSPI(WOR_CAPTURE0)     != CW_WOR_CAPTURE0_VALUE        ) ret=0;
  if(readExtAddrSPI(BIST)             != CW_BIST_VALUE                ) ret=0;
  if(readExtAddrSPI(DCFILTOFFSET_I1)  != CW_DCFILTOFFSET_I1_VALUE     ) ret=0;
  if(readExtAddrSPI(DCFILTOFFSET_I0)  != CW_DCFILTOFFSET_I0_VALUE     ) ret=0;
  if(readExtAddrSPI(DCFILTOFFSET_Q1)  != CW_DCFILTOFFSET_Q1_VALUE     ) ret=0;
  if(readExtAddrSPI(DCFILTOFFSET_Q0)  != CW_DCFILTOFFSET_Q0_VALUE     ) ret=0;
  if(readExtAddrSPI(IQIE_I1)          != CW_IQIE_I1_VALUE             ) ret=0;
  if(readExtAddrSPI(IQIE_I0)          != CW_IQIE_I0_VALUE             ) ret=0;
  if(readExtAddrSPI(IQIE_Q1)          != CW_IQIE_Q1_VALUE             ) ret=0;
  if(readExtAddrSPI(IQIE_Q0)          != CW_IQIE_Q0_VALUE             ) ret=0;
  if(readExtAddrSPI(RSSI1)            != CW_RSSI1_VALUE               ) ret=0;
  if(readExtAddrSPI(RSSI0)            != CW_RSSI0_VALUE               ) ret=0;
  if(readExtAddrSPI(MARCSTATE)        != CW_MARCSTATE_VALUE           ) ret=0;
  if(readExtAddrSPI(LQI_VAL)          != CW_LQI_VAL_VALUE             ) ret=0;
  if(readExtAddrSPI(PQT_SYNC_ERR)     != CW_PQT_SYNC_ERR_VALUE        ) ret=0;
  if(readExtAddrSPI(DEM_STATUS)       != CW_DEM_STATUS_VALUE          ) ret=0;
  if(readExtAddrSPI(FREQOFF_EST1)     != CW_FREQOFF_EST1_VALUE        ) ret=0;
  if(readExtAddrSPI(FREQOFF_EST0)     != CW_FREQOFF_EST0_VALUE        ) ret=0;
  if(readExtAddrSPI(AGC_GAIN3)        != CW_AGC_GAIN3_VALUE           ) ret=0;
  if(readExtAddrSPI(AGC_GAIN2)        != CW_AGC_GAIN2_VALUE           ) ret=0;
  if(readExtAddrSPI(AGC_GAIN1)        != CW_AGC_GAIN1_VALUE           ) ret=0;
  if(readExtAddrSPI(AGC_GAIN0)        != CW_AGC_GAIN0_VALUE           ) ret=0;
  if(readExtAddrSPI(CFM_RX_DATA_OUT)  != CW_CFM_RX_DATA_OUT_VALUE     ) ret=0;
  if(readExtAddrSPI(CFM_TX_DATA_IN)   != CW_CFM_TX_DATA_IN_VALUE      ) ret=0;
  if(readExtAddrSPI(ASK_SOFT_RX_DATA) != CW_ASK_SOFT_RX_DATA_VALUE    ) ret=0;
  if(readExtAddrSPI(RNDGEN)           != CW_RNDGEN_VALUE              ) ret=0;
  if(readExtAddrSPI(MAGN2)            != CW_MAGN2_VALUE               ) ret=0;
  if(readExtAddrSPI(MAGN1)            != CW_MAGN1_VALUE               ) ret=0;
  if(readExtAddrSPI(MAGN0)            != CW_MAGN0_VALUE               ) ret=0;
  if(readExtAddrSPI(ANG1)             != CW_ANG1_VALUE                ) ret=0;
  if(readExtAddrSPI(ANG0)             != CW_ANG0_VALUE                ) ret=0;
  if(readExtAddrSPI(CHFILT_I2)        != CW_CHFILT_I2_VALUE           ) ret=0;
  if(readExtAddrSPI(CHFILT_I1)        != CW_CHFILT_I1_VALUE           ) ret=0;
  if(readExtAddrSPI(CHFILT_I0)        != CW_CHFILT_I0_VALUE           ) ret=0;
  if(readExtAddrSPI(CHFILT_Q2)        != CW_CHFILT_Q2_VALUE           ) ret=0;
  if(readExtAddrSPI(CHFILT_Q1)        != CW_CHFILT_Q1_VALUE           ) ret=0;
  if(readExtAddrSPI(CHFILT_Q0)        != CW_CHFILT_Q0_VALUE           ) ret=0;
  if(readExtAddrSPI(GPIO_STATUS)      != CW_GPIO_STATUS_VALUE         ) ret=0;
  if(readExtAddrSPI(FSCAL_CTRL)       != CW_FSCAL_CTRL_VALUE          ) ret=0;
  if(readExtAddrSPI(PHASE_ADJUST)     != CW_PHASE_ADJUST_VALUE        ) ret=0;
  // if(readExtAddrSPI(PARTNUMBER)       != CW_PARTNUMBER_VALUE          ) ret=0;
  // if(readExtAddrSPI(PARTVERSION)      != CW_PARTVERSION_VALUE         ) ret=0;
  if(readExtAddrSPI(SERIAL_STATUS)    != CW_SERIAL_STATUS_VALUE       ) ret=0;
  if(readExtAddrSPI(MODEM_STATUS1)    != CW_MODEM_STATUS1_VALUE       ) ret=0;
  if(readExtAddrSPI(MODEM_STATUS0)    != CW_MODEM_STATUS0_VALUE       ) ret=0;
  if(readExtAddrSPI(MARC_STATUS1)     != CW_MARC_STATUS1_VALUE        ) ret=0;
  if(readExtAddrSPI(MARC_STATUS0)     != CW_MARC_STATUS0_VALUE        ) ret=0;
  if(readExtAddrSPI(PA_IFAMP_TEST)    != CW_PA_IFAMP_TEST_VALUE       ) ret=0;
  if(readExtAddrSPI(FSRF_TEST)        != CW_FSRF_TEST_VALUE           ) ret=0;
  if(readExtAddrSPI(PRE_TEST)         != CW_PRE_TEST_VALUE            ) ret=0;
  if(readExtAddrSPI(PRE_OVR)          != CW_PRE_OVR_VALUE             ) ret=0;
  if(readExtAddrSPI(ADC_TEST)         != CW_ADC_TEST_VALUE            ) ret=0;
  if(readExtAddrSPI(DVC_TEST)         != CW_DVC_TEST_VALUE            ) ret=0;
  if(readExtAddrSPI(ATEST)            != CW_ATEST_VALUE               ) ret=0;
  if(readExtAddrSPI(ATEST_LVDS)       != CW_ATEST_LVDS_VALUE          ) ret=0;
  if(readExtAddrSPI(ATEST_MODE)       != CW_ATEST_MODE_VALUE          ) ret=0;
  if(readExtAddrSPI(XOSC_TEST1)       != CW_XOSC_TEST1_VALUE          ) ret=0;
  if(readExtAddrSPI(XOSC_TEST0)       != CW_XOSC_TEST0_VALUE          ) ret=0;
  // if(readExtAddrSPI(RXFIRST)          != CW_RXFIRST_VALUE             ) ret=0;
  // if(readExtAddrSPI(TXFIRST)          != CW_TXFIRST_VALUE             ) ret=0;
  // if(readExtAddrSPI(RXLAST)           != CW_RXLAST_VALUE              ) ret=0;
  // if(readExtAddrSPI(TXLAST)           != CW_TXLAST_VALUE              ) ret=0;
  // if(readExtAddrSPI(NUM_TXBYTES)      != CW_NUM_TXBYTES_VALUE         ) ret=0;
  // if(readExtAddrSPI(NUM_RXBYTES)      != CW_NUM_RXBYTES_VALUE         ) ret=0;
  // if(readExtAddrSPI(FIFO_NUM_TXBYTES) != CW_FIFO_NUM_TXBYTES_VALUE    ) ret=0;
  // if(readExtAddrSPI(FIFO_NUM_RXBYTES) != CW_FIFO_NUM_RXBYTES_VALUE    ) ret=0;

  strobeSPI(SCAL); //Calibrate frequency synthesizer and turn it off
 
  ret = waitIDLE(ret, waitTime);
  return ret;
}