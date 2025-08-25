#include <SPI.h>
#include "CC1120.h"
#include "IoExpander.h"
#include "CommunicationDevice.h"

CC1120Class CC1120;
IoExpander  IE;
CommunicationDevice CD;

bool ret = 1;

void setup() {
  IE.init();
  IE.setPin(15, HIGH);
  
  Serial.begin(9600);
  Serial.println("Serial start");

  // Initialize CC1120
  // ret = CC1120.CWbegin(); // and Set Value of Register for CW(ASK/OOK)
  // ret = CC1120.FSKbegin(); // and Set Value of Register for FSK
  // CD.switchMode(SEND, FSK);
}

void loop() {  
  // --- Set Frequency ---
  // setFREQ(float Frequency)
  // Frequency: 410 ~ 480 (MHz)
  ret = CC1120.setFREQ(437.05);
  
  // --- Set Power ---
  // setPWR(uint8_t modulation, float PWR)
  // modulation → FSK:0, CW:1
  // PWR → FSK:[15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -3, -6, -11] (dBm)
  //       CW :[12.5, 10.5, 8.5, 6.5, 4.5, 2.5, 0.5, -1.5, -3.5, -5.5, -7.5, -9.5, -11.5] (dBm)
  // テーブルの中からPWRに最も近い値を採用する
  ret = CC1120.setPWR(0, 15);

  // --- Set Deviation ---
  // setDEV(uint8_t modulation, float DEV)
  // modulation → FSK:0, CW:1
  // DEV → 0.6 ~ 200 (kHz) (maybe)
  // 入力からレジスタ値を計算できなかった場合、ret=0を返す
  ret = CC1120.setDEV(0, 4);

  // --- Set Symbol Rate ---
  // setSR(float SymbolRate)
  // SymbolRate: <= RX Filter BW / 2 (ksps) (RX Filter BW >= 2 * Symbol Rate)
  ret = CC1120.setSR(1.2);

  // --- Set RX Filter BW ---
  // setBW(unsigned long RX_Filter_BW)
  // RX_Filter_BW: 7.8 ~ 200 (kHz)
  // チップの仕様上、実際に設定される値は離散地
  // (200, 125, 100, 66.7, 62.5, 50, 40, 33.3, 28.6, 25, 22.2, 20, 18.2, 16.7,
  //  15.4, 14.3, 13.3, 12.5, 11.8, 11.1, 10.5, 10, 9.5, 9.1, 8.7, 8.3, 8, 7.8)
  // 引数に渡した値に最も近い値が採用される
  ret == CC1120.setBW();

  // --- Tx ---
  Serial.println("loop");
  uint8_t payload[1005]= "In twilight's glow, the silence calls, a whisper drifting, soft and small. The wind, it dances, wild and free, a song of time, a melody. The stars appear, their light so bright, a quiet shimmer in the night. And in the breeze, a secret shared, a fleeting thought, a dream declared. The world around us slows its pace, as shadows hide from dawn's embrace. Yet in the stillness, hearts will find the endless rhythm, intertwined. Beneath the sky, so vast and wide, we walk this earth, side by side. Unspoken words, yet understood, the quiet bond of soul and blood. Through all the years, both calm and storm, we seek the truth, we search for warm embraces held in fleeting moments, lost within the world's vast currents. But still, the whispers call again, to guide us through the sorrow, pain. A journey long, yet full of grace, as time and tide, they interlace. So listen close, and feel the wind, it carries dreams, where hope begins. For in the quiet of the night, we find our peace, and take our flight.";
  ret = CC1120.TX(payload, 1005);
  Serial.println(ret);

  // --- Rx ---
  // uint8_t data[128];
  // ret = CC1120.recvUL(data);
  // Serial.print("RSSI: ");
  // Serial.print(CC1120.showRSSI());
  // Serial.println(ret);
  // if(ret == 1){
  //   for(int i=0; i<19; i++) Serial.println((char)data[i]);
  // }
}
