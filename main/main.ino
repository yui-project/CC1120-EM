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
  ret = CC1120.CWbegin(); // and Set Value of Register for CW(ASK/OOK)
  // ret = CC1120.FSKbegin(); // and Set Value of Register for FSK
  
  CD.switchMode(SEND, FSK);
}

void loop() {
  // なにこれ。纐纈に聞きたい。
  uint8_t rxdata[128] = {};
  ret = CC1120.recvUL(rxdata);
  if(rxdata[1] == 48){
    CC1120.setPWR(rxdata[3]);
  } 
  
  // --- Set Frequency ---
  // 0:437.05MHz, 1:435.00MHz, 2:437.3MHz
  ret = CC1120.setFREQ(0);
  
  // --- Tx ---
  // 0:-11dBm, 1:5dBm, 2:10dBm, 3:15dBm
  ret = CC1120.setPWR(3);
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
