#include "CommunicationDevice.h"

/*
引数：uint8_t data, uint8_t whichRadio
戻り値：true / false
概要：ダウンリンクを行う.引数には送信するデータとグローバル変数のDlRadioを入れる
*/
bool CommunicationDevice::sendDl(uint8_t data, uint8_t whichRadio)
{
    switchMode(SEND, whichRadio);
    if (whichRadio == CW)
    {
        // TODO CWでダウンリンク
    }
    else if (whichRadio == LORA)
    {
        // TODO LORAでダウンリンク
    }
    else if (whichRadio == FSK)
    {
        // TODO FSKでダウンリンク
    }

#ifdef DEBUG
    else if (whichRadio == SERIAL)
    {
        // TODO シリアルでダウンリンク
    }
#endif

    else
    {
        // NOTE:万が一値が変わっていたらCWにする
        DlRadio = CW;
        return false;
    }
    return false;
}

/*
引数：uint8_t *data, int dataSize,  uint8_t whichRadio
戻り値：true / false
概要：ダウンリンクを行う.引数には送信するデータ配列と配列のサイズとグローバル変数のDlRadioを入れる
*/
bool CommunicationDevice::sendDlPacket(uint8_t *data, int dataSize, uint8_t whichRadio)
{
    switchMode(SEND, whichRadio);
    if (whichRadio == CW)
    {
        // TODO CWでパケットのダウンリンク
        return true;
    }
    else if (whichRadio == LORA)
    {
        LoRa.sendDL(data, dataSize);
        //switchMode(RECEIVE, whichRadio);
        return true;
    }
    else if (whichRadio == FSK)
    {
        // TODO FSKでパケットのダウンリンク
        return true;
    }
#ifdef DEBUG
    else if (whichRadio == SERIAL)
    {
        // TODO シリアルでパケットのダウンリンク
    }
#endif
    else
    {
        // NOTE:万が一値が変わっていたらCWにする
        DlRadio = CW;
        return false;
    }
}

/*
引数：uint8_t *data, uint8_t whichRadio
戻り値：true / false
概要：アップリンクを受け取る.引数には受信するコマンド格納用の変数アドレスとグローバル変数のUlRadioを入れる
*/
bool CommunicationDevice::recvUl(uint8_t *ulCommand, uint8_t whichRadio)
{
    switchMode(RECEIVE, whichRadio);
    if (whichRadio == FSK)
    {
        // TODO FSKでアップリンク受信
    }
    else if (whichRadio == LORA)
    {
        if (LoRa.recvUL(ulCommand))
        {
            /*i2cFram.readMemory(I2cFram::Memory::GlobalVariableArea, global);
            delay(5000);
            // TODO LoRaアップリンクフラグの管理はここじゃない、コマンドで管理するべき
            if (SYSTEM_A == true)
            {
                global.loRaAUplinkCheck = 1;
            }
            if (SYSTEM_B == true)
            {
                global.loRaBUplinkCheck = 1;
            }
            i2cFram.writeMemory(I2cFram::Memory::GlobalVariableArea, global);
            */
            return true;
        }
        else
        {
            return false;
        }
    }
    else if (whichRadio == SERIAL)
    {
        if (Serial.available() > 0)
        {
            int subNo = Serial.parseInt(); // コマンド番号を受け取る
            ulCommand[0] = subNo;          // コマンド番号をパケットの最初に保存
            int8_t recvPacketSize = packetSize[subNo];

            Serial.print("Received Command Number: ");
            Serial.println(subNo);
            Serial.print("Packet Size: ");
            Serial.println(recvPacketSize);

            // サブ番号が1〜52の範囲内であれば、パケットの読み込みを開始
            if (subNo >= 1 && subNo <= 52)
            {
                int i = 1;
                while (true)
                {
                    if (Serial.available() > 0)
                    {
                        uint8_t byteReceived = Serial.parseInt();
                        if (byteReceived == 99)
                        {
                            break;
                        }

                        // 受け取ったバイトをulCommandに格納
                        ulCommand[i] = byteReceived;
                        i++;
                    }
                    if (i >= recvPacketSize)
                    {
                        break;
                    }

                    Serial.print("Please Enter Command Data: ");
                    Serial.println(recvPacketSize - i);
                    delay(1000);
                }
                for (int j = 0; j < i; j++)
                {
                    Serial.print("Received Command Data: ");
                    Serial.println(ulCommand[j]);
                }
                return true;
            }
        }
        {
            return false;
        }
    }
    else
    {
        UlRadio = FSK;
        return false;
    }
    return false;
}

/*
引数：uint8_t whichRadio, bool sendOrRecv
戻り値：なし
概要：無線機のモードを切り替える.引数にはSEND/RECEIVEとUlRadioを入れる
*/
void CommunicationDevice::switchMode(bool sendOrRecv, uint8_t whichRadio)
{
    if (sendOrRecv == SEND)
    {
        if (whichRadio == FSK)
        {
            // TODO FSKで送信モード
            ex.setPin(RF_SWITCH_1_CTRL, LOW);
            delay(100);
            ex.setPin(FRONT_END_TR, HIGH);
            delay(100);
            ex.setPin(FRONT_END_EN, HIGH);
            delay(100);
            ex.setPin(FRONT_END_BYP, HIGH);
            delay(100);
            ex.setPin(RF_SWITCH_2_CTRL, LOW);
            delay(100); // NOTE: 元々は10msだったが、それではフロントエンドICのピン切り替えが正常に行われていなかったため、20msに変更（動作確認済み）
        }
        else if (whichRadio == LORA)
        {
            ex.setPin(RF_SWITCH_1_CTRL, HIGH);
            ex.setPin(FRONT_END_TR, HIGH);
            ex.setPin(FRONT_END_EN, HIGH);
            ex.setPin(FRONT_END_BYP, LOW);
            ex.setPin(RF_SWITCH_2_CTRL, LOW);
            delay(20); // NOTE: 元々は10msだったが、それではフロントエンドICのピン切り替えが正常に行われていなかったため、20msに変更（動作確認済み）
        }
        else
        {
            DlRadio = FSK;
            // TODO FSKで送信モード
            return 0;
        }
    }
    else if (sendOrRecv == RECEIVE)
    {
        if (whichRadio == FSK)
        {
            // TODO FSKで受信モード
            ex.setPin(RF_SWITCH_1_CTRL, LOW);
            ex.setPin(FRONT_END_TR, LOW);
            ex.setPin(FRONT_END_EN, HIGH);
            ex.setPin(FRONT_END_BYP, LOW);
            ex.setPin(RF_SWITCH_2_CTRL, HIGH);
            delay(20);
        }
        else if (whichRadio == LORA)
        {
            ex.setPin(RF_SWITCH_1_CTRL, HIGH);
            ex.setPin(FRONT_END_TR, LOW);
            ex.setPin(FRONT_END_EN, HIGH);
            ex.setPin(FRONT_END_BYP, LOW);
            ex.setPin(RF_SWITCH_2_CTRL, HIGH);
            delay(20);
        }
        else
        {
            UlRadio = FSK;
            // TODO FSKで受信モード
            return 0;
        }
    }
    return 0;
}
