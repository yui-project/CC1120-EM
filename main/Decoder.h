#pragma once
#include "Arduino.h"
#include "StorageDefine.h"

// NOTE:メインボードのデコーダ用のクラス　IoExpander、MCP3204のクラスで使用するためデコーダ用のクラスを作成した

class Decoder
{
private:

public:
    void init();
    void write(int pin);
};