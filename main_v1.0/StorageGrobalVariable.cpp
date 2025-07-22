#include "StorageGrobalVariable.h"

//NOTE:アンテナ展開カウンタ
bool counterForAntennaDeployment = false;

double counterForSerialInterrupt;

bool counterForInitialStartUp = false;

//NOTE:初期ミッションカウンタ
bool IsMissionInitial = true;
//NOTE:通常ダウンリンクフラグ(trueであったら通常ダウンリンクを行う)
bool FragRegularDL = true;


//NOTE:最終HKデータ格納用配列
uint8_t HKData[27] = {0};

//NOTE:熱真空用HKデータ格納用配列
uint8_t HKDataForHeatVacuum[HKDATA_FOR_HEAT_VACUUM_SIZE] = {0};

//NOTE:熱真空用通信機データ格納用配列
uint8_t ComDataForHeatVacuum[COMDEVICE_DATA_SIZE] = {0};

//NOTE:放出直後データ
uint8_t PostDeploymentData[4] = {1,2,3,4};
//NOTE:一定時間経過後データ
uint8_t SteadyStateData[21] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21};

uint8_t SubjectMission = 0;
uint32_t MissionStartTime = 0;

bool PowerMCUncontrolled = false; // 電源マイコン通信途絶フラグ
bool IsMode1 = true;
bool IsMode2 = false;
bool IsMode3_1 = false;
bool IsMode3_2 = false;
bool IsMode3_3 = false;
bool IsMode3_4 = false;
bool IsMode3_5 = false;

bool IsMode5_1 = false;
bool IsMode5_2 = false;
bool IsMode6_1 = false;
bool IsMode6_2 = false;
bool IsMode7_1 = false;
bool IsMode7_2 = false;
bool IsMode7_3 = false;

uint32_t EndTimeOfMode2 = 0;
uint32_t RadioSwitchTimeOfMode5_2 = 0;
uint32_t RadioSwitchTimeOfMode6_2 = 0;
uint32_t SwitchTimeToMode7_2 = 0;
uint32_t SwitchTimeToMode7_3 = 0;
uint32_t PreviousTimeOfSaveHKdata = 0;
uint32_t PreviousTimeOfSendHKdata = 0;
uint32_t PreviousTimeOfMissionOperation = 0;
uint32_t PreviousTimeOfImageDL = 0;
uint32_t PreviousTimeOfSwichComDevice = 0;

bool mode2DataDL = true;

bool IsDLdata = false;
int WaitingCommand = 0;
bool IsShutDownedMission = false;

// NOTE: 0:CW 1:FSK 2:LoRa
uint8_t DlRadio = 2;
// NOTE: 1:FSK 2:LoRa
uint8_t UlRadio = 2;

bool InitialBatteryProtect = false;
bool MediumBatteryProtect = false;
bool LastBatteryProtect = false;
bool MissionUncontrolled = false; // NOTE：ミッション部通信断絶フラグ

uint32_t TSUKUTO_TIME = 0;
int32_t TLE = 0;

//NOTE:HKデータのサンプリングレート
uint32_t HKdataSamplingRate = 60000;

//NOTE:packetsize[i] := i番目のULコマンドのパケットサイズ
int8_t packetSize[56] = {
    -1, // コマンド0は存在しない
     1,  9,  7,  1,  1,  1,  1,  1,  1,  1,       // コマンド 1-10
     2,  1,  2,  2,  2,  1,  1,  1,  1,  2,       // コマンド 11-20
     2,  9,  1,  1,  1,  1,  8,  1,  1,  1,       // コマンド 21-30
     2,  1,  1,  3,  3,  3,  7, 28,  8,  3,       // コマンド 31-40
     7,  7,  1,  1,  7,  2,  2,  2,  1,  1,       // コマンド 41-50
     2,  2,  1,  1,  1                            // コマンド 51-55
}; // FIXME: 仕様に合わせて修正する

//NOTE:A系統かB系統かのフラグ
bool SYSTEM_A = false;
//NOTE:A系統かB系統かのフラグ
bool SYSTEM_B = true;
