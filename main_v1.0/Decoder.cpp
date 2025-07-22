#include "Arduino.h"
#include "Decoder.h"

/*
引数：なし
戻り値：なし
概要：デコーダの初期化を行う
*/
void Decoder::init()
{
    pinMode(CS_A, OUTPUT);
    pinMode(CS_B, OUTPUT);
    pinMode(CS_C, OUTPUT);
    //NOTE デコーダを使用しない時はY7をLOWにする
    write(7);
}

/*
引数：int pin(0～7)
戻り値：なし
概要：引数に入れたデコーダのピンをLOWにする.処理が終わったらwrite(7)を呼ぶこと
*/
void Decoder::write(int pin)
{
    digitalWrite(CS_A, pin & 0x01); // LSB
    digitalWrite(CS_B, (pin >> 1) & 0x01);
    digitalWrite(CS_C, (pin >> 2) & 0x01); // MSB
}
