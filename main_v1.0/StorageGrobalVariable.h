#pragma once
#include <Arduino.h>
#include "StorageDefine.h"

extern bool counterForAntennaDeployment;
extern double counterForSerialInterrupt;
extern bool counterForInitialStartUp;
extern bool IsMissionInitial;

extern bool FragRegularDL;

extern uint8_t HKData[27];//最終HKデータ格納用配列

extern uint8_t HKDataForHeatVacuum[HKDATA_FOR_HEAT_VACUUM_SIZE];//熱真空用HKデータ格納用配列

extern uint8_t ComDataForHeatVacuum[COMDEVICE_DATA_SIZE];//熱真空用通信機データ格納用配列

//放出直後データ
extern uint8_t PostDeploymentData[4];
//一定時間経過後データ
extern uint8_t SteadyStateData[21];

extern uint8_t SubjectMission;
extern uint32_t MissionStartTime;

extern bool PowerMCUncontrolled; // 電源マイコン通信途絶フラグ
extern bool IsMode1;
extern bool IsMode2;
extern bool IsMode3_1;
extern bool IsMode3_2;
extern bool IsMode3_3;
extern bool IsMode3_4;
extern bool IsMode3_5;

extern bool IsMode5_1;
extern bool IsMode5_2;
extern bool IsMode6_1;
extern bool IsMode6_2;
extern bool IsMode7_1;
extern bool IsMode7_2;
extern bool IsMode7_3;
extern uint32_t EndTimeOfMode2;
extern uint32_t RadioSwitchTimeOfMode5_2;
extern uint32_t RadioSwitchTimeOfMode6_2;
extern uint32_t SwitchTimeToMode7_2;
extern uint32_t SwitchTimeToMode7_3;
extern uint32_t PreviousTimeOfSaveHKdata;
extern uint32_t PreviousTimeOfSendHKdata;
extern uint32_t PreviousTimeOfMissionOperation;
extern uint32_t PreviousTimeOfImageDL;
extern uint32_t PreviousTimeOfSwichComDevice;

//NOTE:trueだったらEmitData, falseだったらRotateDataをDLする
extern bool mode2DataDL;
// TODO 前回実行されたモード用の変数を作る
extern bool IsDLdata;
extern int WaitingCommand;
extern bool IsShutDownedMission;
// NOTE:0:CW 1:FSK 2:LoRa
extern uint8_t DlRadio; 
// NOTE:1:FSK 2:LoRa
extern uint8_t UlRadio;
extern bool InitialBatteryProtect;
extern bool MediumBatteryProtect;
extern bool LastBatteryProtect;
extern bool PowerMCUncontrolled; // NOTE：電源マイコン通信断絶フラグ
extern bool MissionUncontrolled; // NOTE：ミッション部通信断絶フラグ*/

extern uint32_t TSUKUTO_TIME;
extern int32_t TLE;

extern uint32_t HKdataSamplingRate;

extern int8_t packetSize[56];

//NOTE:A系統かB系統かのフラグ
extern bool SYSTEM_A;
//NOTE:A系統かB系統かのフラグ
extern bool SYSTEM_B;
