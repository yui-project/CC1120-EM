#include "arch/types.h"
#include "HardwareSerial.h"
#include "stdint.h"
#include "CC1120.h"
#include "Decoder.h"
#include "IoExpander.h"
// #include "SpiFram.h"
#include <SPI.h>
#include <math.h>

SPISettings settings(100000, MSBFIRST, SPI_MODE0);
Decoder DECODER;
IoExpander IoEx;
// SpiFram FRAM;

const unsigned long F_XOSC = 32000000;

struct PowerSetting {
  float dbm;
  byte pa_power_ramp; // PA_CFG2のビット0-5に入る値
};

const PowerSetting fsk_power_table[] = {
  {15.0, 0x3F}, {14.0, 0x3D}, {13.0, 0x3B}, {12.0, 0x39},
  {11.0, 0x37}, {10.0, 0x34}, { 9.0, 0x32}, { 8.0, 0x2F},
  { 7.0, 0x2D}, { 6.0, 0x2B}, { 5.0, 0x29}, { 4.0, 0x26},
  { 3.0, 0x24}, { 2.0, 0x22}, { 1.0, 0x1F}, { 0.0, 0x1D},
  {-3.0, 0x16}, {-6.0, 0x0F}, {-11.0, 0x03}
};

// --- ASK/OOK用ルックアップテーブル (12.5dBm to -11.5dBm) ---
const PowerSetting ask_ook_power_table[] = {
  {12.5, 0x3C}, {10.5, 0x38}, { 8.5, 0x34}, { 6.5, 0x30},
  { 4.5, 0x2C}, { 2.5, 0x28}, { 0.5, 0x24}, {-1.5, 0x20},
  {-3.5, 0x1C}, {-5.5, 0x18}, {-7.5, 0x14}, {-9.5, 0x10},
  {-11.5, 0x0C}
};

const uint8_t IQIC_table[] = {0x00, 0x46, 0x46, 0x46, 0x46, 0x46,
                                    0x46, 0xC6, 0xC6, 0xC6, 0xC6,
                                    0xC6, 0xC6, 0xC6, 0xC6, 0xC6,
                                    0xC6, 0xC6, 0xC6, 0xC6, 0xC6,
                                    0xC6, 0xC6, 0xC6, 0xC6, 0xC6};

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

bool CC1120Class::setFREQ(float FREQ){ // target_freq (MHz)
  bool ret = 1;
  const int LO_Divider = 8;
  const unsigned long TWO_POW_16 = 65536;
  byte FREQ2_value;
  byte FREQ1_value;
  byte FREQ0_value;

  unsigned long FREQ_value = FREQ * LO_Divider * TWO_POW_16 / (F_XOSC / 1000000);
  FREQ2_value = (FREQ >> 16) & 0xFF;
  FREQ1_value = (FREQ >> 8) & 0xFF;
  FREQ0_value = FREQ & 0xFF;

  // Serial.print("FREQ ");   Serial.printl(FREQ);         Serial.prinln(" Mhz");
  // Serial.print("FREQ2: "); Serial.println(FREQ2_value); Serial.print(" (0x"); Serial.print(FREQ2_value, HEX); Serial.println(")");
  // Serial.print("FREQ1: "); Serial.println(FREQ1_value); Serial.print(" (0x"); Serial.print(FREQ1_value, HEX); Serial.println(")");
  // Serial.print("FREQ0: "); Serial.println(FREQ0_value); Serial.print(" (0x"); Serial.print(FREQ0_value, HEX); Serial.println(")");

  setRegister(1, FREQ2, FREQ2_value);
  setRegister(1, FREQ1, FREQ1_value);
  setRegister(1, FREQ0, FREQ0_value);

  ret = calibration();
  return ret;
}

bool CC1120Class::setPWR(uint8_t modulation, float PWR){
  bool ret = 1;
  byte PA_POWER_RAMP = 0x03;
  byte PA_CFG2_value = 0x03;
  byte ASK_DEPTH = 0x03;
  byte PA_CFG0_value = 0x1C;
  const byte PA_CFG2_RESERVED_BIT6 = 0x01;
  const byte PA_CFG0_RESERVED_BIT6 = 0x04;

  // 使用するテーブルを選択
  const PowerSetting* table;
  size_t table_size;
  if (modulation == 0) { // (FSK)
    table = fsk_power_table;
    table_size = sizeof(fsk_power_table) / sizeof(PowerSetting);
  } else { // modulation == 1 (ASK_OOK)
    table = ask_ook_power_table;
    table_size = sizeof(ask_ook_power_table) / sizeof(PowerSetting);
  }

  // テーブルから最も近い値を探す
  int best_index = -1;
  float min_diff = 1000.0;
  for (size_t i = 0; i < table_size; i++) {
    float diff = abs(table[i].dbm - PWR);
    if (diff < min_diff) {
      min_diff = diff;
      best_index = i;
    }
  }

  PA_POWER_RAMP = table[best_index].pa_power_ramp;
  PA_CFG2_value = (PA_CFG2_RESERVED_BIT6 << 6) | (PA_POWER_RAMP & 0x3F);
  setRegister(0, PA_CFG2, PA_CFG2_value);

  // CWの場合、PA_CFG0も設定
  if (modulation == 1) {
    ASK_DEPTH = PA_POWER_RAMP / 4;
    PA_CFG0_value = (ASK_DEPTH << 3) | (PA_CFG0_RESERVED_BIT6 & 0x07);
    setRegister(0, PA_CFG0, PA_CFG0_value);
  }
  
  // Serial.print("PA_POWER_RAMP: "); Serial.print(PA_POWER_RAMP); Serial.print(" (0x"); Serial.print(PA_POWER_RAMP, HEX); Seria.println(")");
  // Serial.print("PA_CFG2_value: "); Serial.print(PA_CFG2_value); Serial.print(" (0x"); Serial.print(PA_CFG2_value, HEX); Seria.println(")");
  // Serial.print("ASK_DEPTH: ");     Serial.print(ASK_DEPTH);     Serial.print(" (0x"); Serial.print(ASK_DEPTH, HEX);     Seria.println(")");
  // Serial.print("PA_CFG0_value: "); Serial.print(PA_CFG0_value); Serial.print(" (0x"); Serial.print(PA_CFG0_value, HEX); Seria.println(")");

  ret = calibration();
  return ret;
}

bool CC1120Class::setDEV(uint8_t modulation, float DEV){
  bool ret = 1;
  byte MODCFG_DEV_E_value = 0;
  byte mode = 0x05;
  DEV = DEV * 1000; // kHz→Hzに変換
  if (modulation == 0) { // (FSK)
    mode = 0x00;
  }
  else {// modulation == 1 (ASK/OOK)
    mode = 0x03;
  }

  // DEV_E (指数部) を大きい方から試すことで、DEV_M (仮数部) の精度を最大化します
  for (int DEV_E_value = 7; DEV_E_value >= 0; DEV_E_value--) {
    double DEV_M_value;
    if (DEV_E_value == 0) {
      // DEV_E = 0 の場合の式を逆算
      // m = f_dev * 2^23 / f_XOSC
      DEV_M_value = ((double)DEV * 8388608.0) / F_XOSC;
    } else {
      // DEV_E > 0 の場合の式を逆算
      // m = (f_dev * 2^24 / (f_XOSC * 2^DEV_E_value)) - 256
      uint64_t power_of_2_e = 1ULL << DEV_E_value;
      DEV_M_value = (((double)DEV * 16777216.0) / ((double)F_XOSC * power_of_2_e)) - 256.0;
      DEV_M_value = round(DEV_M_value);
    }
    MODCFG_DEV_E_value = (mode << 3) | (DEV_E_value & 0x07);

    // 計算されたmがレジスタの範囲内(0-255)かチェック
    if (DEV_M_value >= 0.0 && DEV_M_value < 256.0) {
      // 範囲内であれば、このeとmのペアを採用
      // Serial.print("DEV: "); Serial.print(DEV / 1000); Serial.println("kHz");
      // Serial.print("DEVIATION_M: "); Serial.print(DEV_M_value); Serial.print(" (0x"); Serial.print(DEV_M_value, HEX); Serial.println(")");
      // Serial.print("MODCFG_DEV_E: "); Serial.print(MODCFG_DEV_E_value); Serial.print(" (0x"); Serial.print(MODCFG_DEV_E_value, HEX); Serial.println(")");
      // Serial.print("DEV_E: "); Serial.print(DEV_E_value); Serial.print(" (0x"); Serial.print(DEV_E_value, HEX); Serial.println(")");
      
      setRegister(0, DEVIATION_M, DEV_M_value);
      setRegister(0, MODCFG_DEV_E, MODCFG_DEV_E_value);
      
      ret = calibration();
      return ret; // 最適な値が見つかったので終了
    }
  }

  // すべてのeで範囲内のmが見つからなかった場合
  // (偏差が大きすぎるか小さすぎる)
  return 0;
}

bool CC1120Class::setSR(float R_symbol_k) {
  bool ret = 1;
  byte SYMBOL_RATE2 = 0;
  byte SYMBOL_RATE1 = 0;
  byte SYMBOL_RATE0 = 0;
  double R_symbol = R_symbol_k * 1000.0;
  
  // 巨大な定数を64ビットで定義
  const uint64_t TWO_POW_39 = 1ULL << 39; // 2^39
  const uint64_t TWO_POW_38 = 1ULL << 38; // 2^38
  const uint64_t TWO_POW_20 = 1ULL << 20; // 2^20

  // Equation 8: SRATE_E = floor(log2(R_symbol * 2^39 / f_XOSC) - 20)
  double e_float = floor(log(((double)R_symbol * TWO_POW_39) / F_XOSC) / log(2) - 20.0);
  
  byte SRATE_E;
  uint32_t SRATE_M;

  if (e_float < 1.0) {
    // SRATE_E = 0 のケース
    SRATE_E = 0;
    // Equation 7 (逆算): SRATE_M = R_symbol * 2^38 / f_XOSC
    SRATE_M = (uint32_t)round(((double)R_symbol * TWO_POW_38) / F_XOSC);
  } else {
    // SRATE_E > 0 のケース
    SRATE_E = (byte)e_float;
    // Equation 9: SRATE_M = (R_symbol * 2^39 / (f_XOSC * 2^SRATE_E)) - 2^20
    SRATE_M = (uint32_t)round((((double)R_symbol * TWO_POW_39) / (F_XOSC * pow(2.0, SRATE_E))) - TWO_POW_20);
  }

  // なんでかわからんけど1ずれる
  SRATE_M = SRATE_M - 1;

  // データシートの注記: もしMが2^20に丸められたら、EをインクリメントしMを0にする
  if (SRATE_M >= TWO_POW_20) {
    SRATE_E++;
    SRATE_M = 0;
  }

  // 20ビットのSRATE_Mと4ビットのSRATE_Eを3つのレジスタに分割
  SYMBOL_RATE2 = ((SRATE_E & 0x0F) << 4) | ((SRATE_M >> 16) & 0x0F);
  SYMBOL_RATE1 = (SRATE_M >> 8) & 0xFF;
  SYMBOL_RATE0 = SRATE_M & 0xFF;

  // Serial.print("Symbol Rate: "); Serial.print(R_symbol_k); Serial.println("ksps");
  // Serial.print("SYMBOL_RATE2: "); Serial.print(SYMBOL_RATE2); Serial.print(" (0x"); Serial.print(SYMBOL_RATE2, HEX); Serial.println(")");
  // Serial.print("SRATE_E: "); Serial.print(SRATE_E); Serial.print(" (0x"); Serial.print(SRATE_E, HEX); Serial.println(")");
  // Serial.print("SRATE_M: "); Serial.print(SRATE_M); Serial.print(" (0x"); Serial.print(SRATE_M, HEX); Serial.println(")");
  // Serial.print("SRATE_M_19_16: "); Serial.print((SRATE_M >> 16) & 0x0F); Serial.print(" (0x"); Serial.print((SRATE_M >> 16) & 0x0F, HEX); Serial.println(")");
  // Serial.print("SYMBOL_RATE1: "); Serial.print(SYMBOL_RATE1); Serial.print(" (0x"); Serial.print(SYMBOL_RATE1, HEX); Serial.println(")");
  // Serial.print("SYMBOL_RATE0: "); Serial.print(SYMBOL_RATE0); Serial.print(" (0x"); Serial.print(SYMBOL_RATE0, HEX); Serial.println(")");

  setRegister(0, SYMBOL_RATE2, SYMBOL_RATE2);
  setRegister(0, SYMBOL_RATE1, SYMBOL_RATE1);
  setRegister(0, SYMBOL_RATE0, SYMBOL_RATE0);
  
  ret = calibration();
  return ret;
}

bool CC1120Class::setBW(unsigned long RX_Filter_BW_k) {
  bool ret = 1;
  float actual_bw = 0;
  byte IQIC_value = 0;
  byte CHAN_BW_value = 0;
  byte ADC_CIC_DECFACT = 0;
  byte BB_CIC_DECFACT = 0;
  unsigned long RX_Filter_BW = RX_Filter_BW_k * 1000;

  int Decimation_Factor = 20;
  byte BB_CIC_DECFACT1 = round((double)F_XOSC / ((double)Decimation_Factor * RX_Filter_BW * 8.0));
  // レジスタの有効範囲(1-25)にクランプ
  if (BB_CIC_DECFACT1 < 1) BB_CIC_DECFACT1 = 1;
  if (BB_CIC_DECFACT1 > 25) BB_CIC_DECFACT1 = 25;
  float actual_bw1 = (double)F_XOSC / ((double)Decimation_Factor * BB_CIC_DECFACT1 * 8.0);
  float diff1 = abs(actual_bw1 - RX_Filter_BW);

  Decimation_Factor = 32;
  byte BB_CIC_DECFACT2 = round((double)F_XOSC / ((double)Decimation_Factor * RX_Filter_BW * 8.0));
  // レジスタの有効範囲(1-25)にクランプ
  if (BB_CIC_DECFACT2 < 1) BB_CIC_DECFACT2 = 1;
  if (BB_CIC_DECFACT2 > 16) BB_CIC_DECFACT2 = 16;
  float actual_bw2 = (double)F_XOSC / ((double)Decimation_Factor * BB_CIC_DECFACT2 * 8.0);
  float diff2 = abs(actual_bw2 - RX_Filter_BW);
  
  if (diff1 > diff2 && (BB_CIC_DECFACT2 == 1 || BB_CIC_DECFACT2 == 2 || BB_CIC_DECFACT2 == 16)) {
    BB_CIC_DECFACT = BB_CIC_DECFACT2;
    actual_bw = actual_bw2 / 1000; // (kHz)
  } else {
    ADC_CIC_DECFACT = 0;
    BB_CIC_DECFACT = BB_CIC_DECFACT1;
    actual_bw = actual_bw1 / 1000; // (kHz)
    ADC_CIC_DECFACT = 1;
  }
  IQIC_value = IQIC_table[BB_CIC_DECFACT];
  CHAN_BW_value = (ADC_CIC_DECFACT << 6) | (BB_CIC_DECFACT & 0x3F);

  Serial.print("RX_Filter_BW: "); Serial.print(RX_Filter_BW); Serial.println("kHz");
  Serial.print("IQIC: "); Serial.print(IQIC_value); Serial.print(" (0x"); Serial.print(IQIC_value, HEX); Serial.println(")");
  Serial.print("CHAN_BW: "); Serial.print(CHAN_BW_value); Serial.print(" (0x"); Serial.print(CHAN_BW_value, HEX); Serial.println(")");
  Serial.print("ADC_CIC_DECFACT: "); Serial.print(ADC_CIC_DECFACT); Serial.print(" (0x"); Serial.print(ADC_CIC_DECFACT, HEX); Serial.println(")");
  Serial.print("BB_CIC_DECFACT: "); Serial.print(BB_CIC_DECFACT); Serial.print(" (0x"); Serial.print(BB_CIC_DECFACT, HEX); Serial.println(")");
  Serial.print("Actual BW: "); Serial.print(actual_bw); Serial.println("kHz");
  Serial.println();
    
  // setRegister(0, IQIC, IQIC_value);
  // setRegister(0, CHAN_BW, CHAN_BW);
  
  // ret = calibration();
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