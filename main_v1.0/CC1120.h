#include "stdint.h"
#include "Stream.h"

#ifndef CC1120_H
#define CC1120_H

#include <Arduino.h>
#include <SPI.h>

#define CC1120_SPI            SPI5
#define CC1120_SERIAL         Serial
#define CC1120_R_BIT          0x80
#define CC1120_POWER          20
#define LoRa_POWER            21
#define DECA_SS_PIN           19
#define DECB_SS_PIN           25
#define DECC_SS_PIN           22

#define MARCSTATE_IDLE        0b01000001
#define MARCSTATE_RX          0b01101101
#define MARCSTATE_TXFIFOERROR 0b00010110

#define SOH                   0x02
#define EOH                   0x04



class CC1120Class
{
  public:
    bool    begin();
    bool    calibration();
    bool    setRegister(bool extAddr, uint8_t addr, uint8_t value);
    uint8_t showResister(bool extAddr, uint8_t addr);
    uint32_t showRSSI();
    bool    sendDL(uint8_t data);
    bool    sendDL(uint8_t *data, uint32_t len);
    bool    recvUL(uint8_t *recvCommand);
    bool    setFREQ(uint8_t FREQ);
    bool    setPWR(uint8_t PWR);
    // bool    sendDLfromFram(uint64_t start, uint64_t end);
    bool    TX(uint8_t *payload, int32_t len);
    bool    RX(uint8_t *data, uint16_t limit=0);
    bool    IDLE();
    bool    setFSK();
    bool    setCW();
    uint8_t marcstate();
    

  private:
    uint8_t readSPI(uint8_t addr);
    void    writeSPI(uint8_t addr, uint8_t value);
    void    strobeSPI(uint8_t cmd);
    uint8_t readExtAddrSPI(uint8_t addr);
    void    writeExtAddrSPI(uint8_t addr, uint8_t value);
    uint8_t readFIFO(uint8_t addr);
    void    writeFIFO(uint8_t addr, uint8_t value);
    void    timerStart(uint32_t time);
    bool    timeout();
    bool    FIFOFlush();
    
    bool    waitIDLE(bool ret, uint32_t time);
    bool    waitRX(bool ret, uint32_t time);
    bool    waitRXPKT(bool ret, uint32_t time);
    bool    waitTXFIFOERROR(bool ret, uint32_t time);
    bool    waitIDLEorTXFIFOERROR(bool ret, uint32_t time);
    void    reset();

    uint32_t timerTime;
    uint32_t waitTime = 3000;
};

//SPI Address Space
#define IOCFG3                0x00
#define IOCFG2                0x01
#define IOCFG1                0x02
#define IOCFG0                0x03
#define SYNC3                 0x04
#define SYNC2                 0x05
#define SYNC1                 0x06
#define SYNC0                 0x07
#define SYNC_CFG1             0x08
#define SYNC_CFG0             0x09
#define DEVIATION_M           0x0A
#define MODCFG_DEV_E          0x0B
#define DCFILT_CFG            0x0C
#define PREAMBLE_CFG1         0x0D
#define PREAMBLE_CFG0         0x0E
#define FREQ_IF_CFG           0x0F
#define IQIC                  0x10
#define CHAN_BW               0x11
#define MDMCFG1               0x12
#define MDMCFG0               0x13
#define SYMBOL_RATE2          0x14
#define SYMBOL_RATE1          0x15
#define SYMBOL_RATE0          0x16
#define AGC_REF               0x17
#define AGC_CS_THR            0x18
#define AGC_GAIN_ADJUST       0x19
#define AGC_CFG3              0x1A
#define AGC_CFG2              0x1B
#define AGC_CFG1              0x1C
#define AGC_CFG0              0x1D
#define FIFO_CFG              0x1E
#define DEV_ADDR              0x1F
#define SETTLING_CFG          0x20
#define FS_CFG                0x21
#define WOR_CFG1              0x22
#define WOR_CFG0              0x23
#define WOR_EVENT0_MSB        0x24
#define WOR_EVENT0_LSB        0x25
#define PKT_CFG2              0x26
#define PKT_CFG1              0x27
#define PKT_CFG0              0x28
#define RFEND_CFG1            0x29
#define RFEND_CFG0            0x2A
#define PA_CFG2               0x2B
#define PA_CFG1               0x2C
#define PA_CFG0               0x2D
#define PKT_LEN               0x2E
#define EXT_ADDR              0x2F
#define SRES                  0x30
#define SFSTXON               0x31
#define SXOFF                 0x32
#define SCAL                  0x33
#define SRX                   0x34
#define STX                   0x35
#define SIDLE                 0x36
#define SAFC                  0x37
#define SWOR                  0x38
#define SPWD                  0x39
#define SFRX                  0x3A
#define SFTX                  0x3B
#define SWORRST               0x3C
#define SNOP                  0x3D
#define DIRECT_MEMORY_ACCESS  0x3E
#define TXRX_FIFO             0x3F

//Extended Register Space Mapping
#define IF_MIX_CFG            0x00
#define FREQOFF_CFG           0x01
#define TOC_CFG               0x02
#define MARC_SPARE            0x03
#define ECG_CFG               0x04
#define CFM_DATA_CFG          0x05
#define EXT_CTRL              0x06
#define RCCAL_FINE            0x07
#define RCCAL_COARSE          0x08
#define RCCAL_OFFSET          0x09
#define FREQOFF1              0x0A
#define FREQOFF0              0x0B
#define FREQ2                 0x0C
#define FREQ1                 0x0D
#define FREQ0                 0x0E
#define IF_ADC2               0x0F
#define IF_ADC1               0x10
#define IF_ADC0               0x11
#define FS_DIG1               0x12
#define FS_DIG0               0x13
#define FS_CAL3               0x14
#define FS_CAL2               0x15
#define FS_CAL1               0x16
#define FS_CAL0               0x17
#define FS_CHP                0x18
#define FS_DIVTWO             0x19
#define FS_DSM1               0x1A
#define FS_DSM0               0x1B
#define FS_DVC1               0x1C
#define FS_DVC0               0x1D
#define FS_LBI                0x1E
#define FS_PFD                0x1F
#define FS_PRE                0x20
#define FS_REG_DIV_CML        0x21
#define FS_SPARE              0x22
#define FS_VCO4               0x23
#define FS_VCO3               0x24
#define FS_VCO2               0x25
#define FS_VCO1               0x26
#define FS_VCO0               0x27
#define GBIAS6                0x28
#define GBIAS5                0x29
#define GBIAS4                0x2A
#define GBIAS3                0x2B
#define GBIAS2                0x2C
#define GBIAS1                0x2D
#define GBIAS0                0x2E
#define IFAMP                 0x2F
#define LNA                   0x30
#define RXMIX                 0x31
#define XOSC5                 0x32
#define XOSC4                 0x33
#define XOSC3                 0x34
#define XOSC2                 0x35
#define XOSC1                 0x36
#define XOSC0                 0x37
#define ANALOG_SPARE          0x38
#define PA_CFG3               0x39
//0x3A~0x63 are not used
#define WOR_TIME1             0x64
#define WOR_TIME0             0x65
#define WOR_CAPTURE1          0x66
#define WOR_CAPTURE0          0x67
#define BIST                  0x68
#define DCFILTOFFSET_I1       0x69
#define DCFILTOFFSET_I0       0x6A
#define DCFILTOFFSET_Q1       0x6B
#define DCFILTOFFSET_Q0       0x6C
#define IQIE_I1               0x6D
#define IQIE_I0               0x6E
#define IQIE_Q1               0x6F
#define IQIE_Q0               0x70
#define RSSI1                 0x71
#define RSSI0                 0x72
#define MARCSTATE             0x73
#define LQI_VAL               0x74
#define PQT_SYNC_ERR          0x75
#define DEM_STATUS            0x76
#define FREQOFF_EST1          0x77
#define FREQOFF_EST0          0x78
#define AGC_GAIN3             0x79
#define AGC_GAIN2             0x7A
#define AGC_GAIN1             0x7B
#define AGC_GAIN0             0x7C
#define CFM_RX_DATA_OUT       0x7D
#define CFM_TX_DATA_IN        0x7E
#define ASK_SOFT_RX_DATA      0x7F
#define RNDGEN                0x80
#define MAGN2                 0x81
#define MAGN1                 0x82
#define MAGN0                 0x83
#define ANG1                  0x84
#define ANG0                  0x85
#define CHFILT_I2             0x86
#define CHFILT_I1             0x87
#define CHFILT_I0             0x88
#define CHFILT_Q2             0x89
#define CHFILT_Q1             0x8A
#define CHFILT_Q0             0x8B
#define GPIO_STATUS           0x8C
#define FSCAL_CTRL            0x8D
#define PHASE_ADJUST          0x8E
#define PARTNUMBER            0x8F
#define PARTVERSION           0x90
#define SERIAL_STATUS         0x91
#define MODEM_STATUS1         0x92
#define MODEM_STATUS0         0x93
#define MARC_STATUS1          0x94
#define MARC_STATUS0          0x95
#define PA_IFAMP_TEST         0x96
#define FSRF_TEST             0x97
#define PRE_TEST              0x98
#define PRE_OVR               0x99
#define ADC_TEST              0x9A
#define DVC_TEST              0x9B
#define ATEST                 0x9C
#define ATEST_LVDS            0x9D
#define ATEST_MODE            0x9E
#define XOSC_TEST1            0x9F
#define XOSC_TEST0            0xA0
//0xA1~0xD1 are not used
#define RXFIRST               0xD2
#define TXFIRST               0xD3
#define RXLAST                0xD4
#define TXLAST                0xD5
#define NUM_TXBYTES           0xD6
#define NUM_RXBYTES           0xD7
#define FIFO_NUM_TXBYTES      0xD8
#define FIFO_NUM_RXBYTES      0xD9



#define CW_IOCFG3_VALUE            0xB0
#define CW_IOCFG2_VALUE            0x06
#define CW_IOCFG1_VALUE            0xB0
#define CW_IOCFG0_VALUE            0x40
#define CW_SYNC3_VALUE             0x93
#define CW_SYNC2_VALUE             0x0B
#define CW_SYNC1_VALUE             0x51
#define CW_SYNC0_VALUE             0xDE
#define CW_SYNC_CFG1_VALUE         0x0A
#define CW_SYNC_CFG0_VALUE         0x17
#define CW_DEVIATION_M_VALUE       0x26
#define CW_MODCFG_DEV_E_VALUE      0x1D
#define CW_DCFILT_CFG_VALUE        0x13
#define CW_PREAMBLE_CFG1_VALUE     0x18
#define CW_PREAMBLE_CFG0_VALUE     0x33
#define CW_FREQ_IF_CFG_VALUE       0x40
#define CW_IQIC_VALUE              0x00
#define CW_CHAN_BW_VALUE           0x03
#define CW_MDMCFG1_VALUE           0x46
#define CW_MDMCFG0_VALUE           0x04
#define CW_SYMBOL_RATE2_VALUE      0x63
#define CW_SYMBOL_RATE1_VALUE      0xA9
#define CW_SYMBOL_RATE0_VALUE      0x2A
#define CW_AGC_REF_VALUE           0x30
#define CW_AGC_CS_THR_VALUE        0xEC
#define CW_AGC_GAIN_ADJUST_VALUE   0x00
#define CW_AGC_CFG3_VALUE          0xD1
#define CW_AGC_CFG2_VALUE          0x3F
#define CW_AGC_CFG1_VALUE          0x32
#define CW_AGC_CFG0_VALUE          0x9F
#define CW_FIFO_CFG_VALUE          0x00
#define CW_DEV_ADDR_VALUE          0x55
#define CW_SETTLING_CFG_VALUE      0x0B
#define CW_FS_CFG_VALUE            0x12
#define CW_WOR_CFG1_VALUE          0x08
#define CW_WOR_CFG0_VALUE          0x21
#define CW_WOR_EVENT0_MSB_VALUE    0x00
#define CW_WOR_EVENT0_LSB_VALUE    0x00
#define CW_PKT_CFG2_VALUE          0x04
#define CW_PKT_CFG1_VALUE          0x05
#define CW_PKT_CFG0_VALUE          0x20
#define CW_RFEND_CFG1_VALUE        0x0F
#define CW_RFEND_CFG0_VALUE        0x00
#define CW_PA_CFG2_VALUE           0x7F
#define CW_PA_CFG1_VALUE           0x56
#define CW_PA_CFG0_VALUE           0x7D
#define CW_PKT_LEN_VALUE           0xFF
#define CW_IF_MIX_CFG_VALUE        0x00
#define CW_FREQOFF_CFG_VALUE       0x00
#define CW_TOC_CFG_VALUE           0x0A
#define CW_MARC_SPARE_VALUE        0x00
#define CW_ECG_CFG_VALUE           0x00
#define CW_CFM_DATA_CFG_VALUE      0x00
#define CW_EXT_CTRL_VALUE          0x01
#define CW_RCCAL_FINE_VALUE        0x00
#define CW_RCCAL_COARSE_VALUE      0x00
#define CW_RCCAL_OFFSET_VALUE      0x00
#define CW_FREQOFF1_VALUE          0x00
#define CW_FREQOFF0_VALUE          0x00
#define CW_FREQ2_VALUE             0x6C
#define CW_FREQ1_VALUE             0x80
#define CW_FREQ0_VALUE             0x00
#define CW_IF_ADC2_VALUE           0x02
#define CW_IF_ADC1_VALUE           0xA6
#define CW_IF_ADC0_VALUE           0x04
#define CW_FS_DIG1_VALUE           0x00
#define CW_FS_DIG0_VALUE           0x5F
#define CW_FS_CAL3_VALUE           0x00
#define CW_FS_CAL2_VALUE           0x20
#define CW_FS_CAL1_VALUE           0x40
#define CW_FS_CAL0_VALUE           0x0E
#define CW_FS_CHP_VALUE            0x28
#define CW_FS_DIVTWO_VALUE         0x03
#define CW_FS_DSM1_VALUE           0x00
#define CW_FS_DSM0_VALUE           0x33
#define CW_FS_DVC1_VALUE           0xFF
#define CW_FS_DVC0_VALUE           0x17
#define CW_FS_LBI_VALUE            0x00
#define CW_FS_PFD_VALUE            0x50
#define CW_FS_PRE_VALUE            0x6E
#define CW_FS_REG_DIV_CML_VALUE    0x14
#define CW_FS_SPARE_VALUE          0xAC
#define CW_FS_VCO4_VALUE           0x14
#define CW_FS_VCO3_VALUE           0x00
#define CW_FS_VCO2_VALUE           0x00
#define CW_FS_VCO1_VALUE           0x00
#define CW_FS_VCO0_VALUE           0xB4
#define CW_GBIAS6_VALUE            0x00
#define CW_GBIAS5_VALUE            0x02
#define CW_GBIAS4_VALUE            0x00
#define CW_GBIAS3_VALUE            0x00
#define CW_GBIAS2_VALUE            0x10
#define CW_GBIAS1_VALUE            0x00
#define CW_GBIAS0_VALUE            0x00
#define CW_IFAMP_VALUE             0x01
#define CW_LNA_VALUE               0x01
#define CW_RXMIX_VALUE             0x01
#define CW_XOSC5_VALUE             0x0E
#define CW_XOSC4_VALUE             0xA0
#define CW_XOSC3_VALUE             0x03
#define CW_XOSC2_VALUE             0x04
#define CW_XOSC1_VALUE             0x03
#define CW_XOSC0_VALUE             0x00
#define CW_ANALOG_SPARE_VALUE      0x00
#define CW_PA_CFG3_VALUE           0x00
#define CW_WOR_TIME1_VALUE         0x00
#define CW_WOR_TIME0_VALUE         0x00
#define CW_WOR_CAPTURE1_VALUE      0x00
#define CW_WOR_CAPTURE0_VALUE      0x00
#define CW_BIST_VALUE              0x00
#define CW_DCFILTOFFSET_I1_VALUE   0x00
#define CW_DCFILTOFFSET_I0_VALUE   0x00
#define CW_DCFILTOFFSET_Q1_VALUE   0x00
#define CW_DCFILTOFFSET_Q0_VALUE   0x00
#define CW_IQIE_I1_VALUE           0x00
#define CW_IQIE_I0_VALUE           0x00
#define CW_IQIE_Q1_VALUE           0x00
#define CW_IQIE_Q0_VALUE           0x00
#define CW_RSSI1_VALUE             0x80
#define CW_RSSI0_VALUE             0x00
#define CW_MARCSTATE_VALUE         0x41
#define CW_LQI_VAL_VALUE           0x00
#define CW_PQT_SYNC_ERR_VALUE      0xFF
#define CW_DEM_STATUS_VALUE        0x00
#define CW_FREQOFF_EST1_VALUE      0x00
#define CW_FREQOFF_EST0_VALUE      0x00
#define CW_AGC_GAIN3_VALUE         0x00
#define CW_AGC_GAIN2_VALUE         0xD1
#define CW_AGC_GAIN1_VALUE         0x00
#define CW_AGC_GAIN0_VALUE         0x3F
#define CW_CFM_RX_DATA_OUT_VALUE   0x00
#define CW_CFM_TX_DATA_IN_VALUE    0x00
#define CW_ASK_SOFT_RX_DATA_VALUE  0x30
#define CW_RNDGEN_VALUE            0x7F
#define CW_MAGN2_VALUE             0x00
#define CW_MAGN1_VALUE             0x00
#define CW_MAGN0_VALUE             0x00
#define CW_ANG1_VALUE              0x00
#define CW_ANG0_VALUE              0x00
#define CW_CHFILT_I2_VALUE         0x08
#define CW_CHFILT_I1_VALUE         0x00
#define CW_CHFILT_I0_VALUE         0x00
#define CW_CHFILT_Q2_VALUE         0x00
#define CW_CHFILT_Q1_VALUE         0x00
#define CW_CHFILT_Q0_VALUE         0x00
#define CW_GPIO_STATUS_VALUE       0x00
#define CW_FSCAL_CTRL_VALUE        0x01
#define CW_PHASE_ADJUST_VALUE      0x00
#define CW_PARTNUMBER_VALUE        0x48
#define CW_PARTVERSION_VALUE       0x23
#define CW_SERIAL_STATUS_VALUE     0x00
#define CW_MODEM_STATUS1_VALUE     0x10
#define CW_MODEM_STATUS0_VALUE     0x00
#define CW_MARC_STATUS1_VALUE      0x00
#define CW_MARC_STATUS0_VALUE      0x00
#define CW_PA_IFAMP_TEST_VALUE     0x00
#define CW_FSRF_TEST_VALUE         0x00
#define CW_PRE_TEST_VALUE          0x00
#define CW_PRE_OVR_VALUE           0x00
#define CW_ADC_TEST_VALUE          0x00
#define CW_DVC_TEST_VALUE          0x0B
#define CW_ATEST_VALUE             0x40
#define CW_ATEST_LVDS_VALUE        0x00
#define CW_ATEST_MODE_VALUE        0x00
#define CW_XOSC_TEST1_VALUE        0x3C
#define CW_XOSC_TEST0_VALUE        0x00
#define CW_RXFIRST_VALUE           0x00
#define CW_TXFIRST_VALUE           0x00
#define CW_RXLAST_VALUE            0x00
#define CW_TXLAST_VALUE            0x00
#define CW_NUM_TXBYTES_VALUE       0x00
#define CW_NUM_RXBYTES_VALUE       0x00
#define CW_FIFO_NUM_TXBYTES_VALUE  0x0F
#define CW_FIFO_NUM_RXBYTES_VALUE  0x00



// Register values
#define FSK_IOCFG3_VALUE            0xB0
#define FSK_IOCFG2_VALUE            0x06
#define FSK_IOCFG1_VALUE            0xB0
#define FSK_IOCFG0_VALUE            0x40
#define FSK_SYNC3_VALUE             0x93
#define FSK_SYNC2_VALUE             0x0B
#define FSK_SYNC1_VALUE             0x51
#define FSK_SYNC0_VALUE             0xDE
#define FSK_SYNC_CFG1_VALUE         0x0B
#define FSK_SYNC_CFG0_VALUE         0x17
#define FSK_DEVIATION_M_VALUE       0x89
#define FSK_MODCFG_DEV_E_VALUE      0x01
#define FSK_DCFILT_CFG_VALUE        0x1C
#define FSK_PREAMBLE_CFG1_VALUE     0x18
#define FSK_PREAMBLE_CFG0_VALUE     0x2A
#define FSK_FREQ_IF_CFG_VALUE       0x40
#define FSK_IQIC_VALUE              0xC6
#define FSK_CHAN_BW_VALUE           0x08
#define FSK_MDMCFG1_VALUE           0x46
#define FSK_MDMCFG0_VALUE           0x05
#define FSK_SYMBOL_RATE2_VALUE      0x48
#define FSK_SYMBOL_RATE1_VALUE      0x93
#define FSK_SYMBOL_RATE0_VALUE      0x75
#define FSK_AGC_REF_VALUE           0x20
#define FSK_AGC_CS_THR_VALUE        0x19
#define FSK_AGC_GAIN_ADJUST_VALUE   0x00
#define FSK_AGC_CFG3_VALUE          0x91
#define FSK_AGC_CFG2_VALUE          0x20
#define FSK_AGC_CFG1_VALUE          0xA9
#define FSK_AGC_CFG0_VALUE          0xCF
#define FSK_FIFO_CFG_VALUE          0x00
#define FSK_DEV_ADDR_VALUE          0x55
#define FSK_SETTLING_CFG_VALUE      0x0B
#define FSK_FS_CFG_VALUE            0x14
#define FSK_WOR_CFG1_VALUE          0x08
#define FSK_WOR_CFG0_VALUE          0x21
#define FSK_WOR_EVENT0_MSB_VALUE    0x00
#define FSK_WOR_EVENT0_LSB_VALUE    0x00
#define FSK_PKT_CFG2_VALUE          0x04
#define FSK_PKT_CFG1_VALUE          0x25
#define FSK_PKT_CFG0_VALUE          0x20
#define FSK_RFEND_CFG1_VALUE        0x0F
#define FSK_RFEND_CFG0_VALUE        0x00
#define FSK_PA_CFG2_VALUE           0x7F
#define FSK_PA_CFG1_VALUE           0x56
#define FSK_PA_CFG0_VALUE           0x7E
#define FSK_PKT_LEN_VALUE           0xFF
#define FSK_IF_MIX_CFG_VALUE        0x00
#define FSK_FREQOFF_CFG_VALUE       0x22
#define FSK_TOC_CFG_VALUE           0x0B
#define FSK_MARC_SPARE_VALUE        0x00
#define FSK_ECG_CFG_VALUE           0x00
#define FSK_CFM_DATA_CFG_VALUE      0x00
#define FSK_EXT_CTRL_VALUE          0x01
#define FSK_RCCAL_FINE_VALUE        0x00
#define FSK_RCCAL_COARSE_VALUE      0x00
#define FSK_RCCAL_OFFSET_VALUE      0x00
#define FSK_FREQOFF1_VALUE          0x00
#define FSK_FREQOFF0_VALUE          0x00
#define FSK_FREQ2_VALUE             0x6D
#define FSK_FREQ1_VALUE             0x43
#define FSK_FREQ0_VALUE             0x33
#define FSK_IF_ADC2_VALUE           0x02
#define FSK_IF_ADC1_VALUE           0xA6
#define FSK_IF_ADC0_VALUE           0x04
#define FSK_FS_DIG1_VALUE           0x00
#define FSK_FS_DIG0_VALUE           0x5F
#define FSK_FS_CAL3_VALUE           0x00
#define FSK_FS_CAL2_VALUE           0x20
#define FSK_FS_CAL1_VALUE           0x40
#define FSK_FS_CAL0_VALUE           0x0E
#define FSK_FS_CHP_VALUE            0x28
#define FSK_FS_DIVTWO_VALUE         0x03
#define FSK_FS_DSM1_VALUE           0x00
#define FSK_FS_DSM0_VALUE           0x33
#define FSK_FS_DVC1_VALUE           0xFF
#define FSK_FS_DVC0_VALUE           0x17
#define FSK_FS_LBI_VALUE            0x00
#define FSK_FS_PFD_VALUE            0x50
#define FSK_FS_PRE_VALUE            0x6E
#define FSK_FS_REG_DIV_CML_VALUE    0x14
#define FSK_FS_SPARE_VALUE          0xAC
#define FSK_FS_VCO4_VALUE           0x14
#define FSK_FS_VCO3_VALUE           0x00
#define FSK_FS_VCO2_VALUE           0x00
#define FSK_FS_VCO1_VALUE           0x00
#define FSK_FS_VCO0_VALUE           0xB4
#define FSK_GBIAS6_VALUE            0x00
#define FSK_GBIAS5_VALUE            0x02
#define FSK_GBIAS4_VALUE            0x00
#define FSK_GBIAS3_VALUE            0x00
#define FSK_GBIAS2_VALUE            0x10
#define FSK_GBIAS1_VALUE            0x00
#define FSK_GBIAS0_VALUE            0x00
#define FSK_IFAMP_VALUE             0x01
#define FSK_LNA_VALUE               0x01
#define FSK_RXMIX_VALUE             0x01
#define FSK_XOSC5_VALUE             0x0E
#define FSK_XOSC4_VALUE             0xA0
#define FSK_XOSC3_VALUE             0x03
#define FSK_XOSC2_VALUE             0x04
#define FSK_XOSC1_VALUE             0x03
#define FSK_XOSC0_VALUE             0x00
#define FSK_ANALOG_SPARE_VALUE      0x00
#define FSK_PA_CFG3_VALUE           0x00
#define FSK_WOR_TIME1_VALUE         0x00
#define FSK_WOR_TIME0_VALUE         0x00
#define FSK_WOR_CAPTURE1_VALUE      0x00
#define FSK_WOR_CAPTURE0_VALUE      0x00
#define FSK_BIST_VALUE              0x00
#define FSK_DCFILTOFFSET_I1_VALUE   0x00
#define FSK_DCFILTOFFSET_I0_VALUE   0x00
#define FSK_DCFILTOFFSET_Q1_VALUE   0x00
#define FSK_DCFILTOFFSET_Q0_VALUE   0x00
#define FSK_IQIE_I1_VALUE           0x00
#define FSK_IQIE_I0_VALUE           0x00
#define FSK_IQIE_Q1_VALUE           0x00
#define FSK_IQIE_Q0_VALUE           0x00
#define FSK_RSSI1_VALUE             0x80
#define FSK_RSSI0_VALUE             0x00
#define FSK_MARCSTATE_VALUE         0x41
#define FSK_LQI_VAL_VALUE           0x00
#define FSK_PQT_SYNC_ERR_VALUE      0xFF
#define FSK_DEM_STATUS_VALUE        0x00
#define FSK_FREQOFF_EST1_VALUE      0x00
#define FSK_FREQOFF_EST0_VALUE      0x00
#define FSK_AGC_GAIN3_VALUE         0x00
#define FSK_AGC_GAIN2_VALUE         0xD1
#define FSK_AGC_GAIN1_VALUE         0x00
#define FSK_AGC_GAIN0_VALUE         0x3F
#define FSK_CFM_RX_DATA_OUT_VALUE   0x00
#define FSK_CFM_TX_DATA_IN_VALUE    0x00
#define FSK_ASK_SOFT_RX_DATA_VALUE  0x30
#define FSK_RNDGEN_VALUE            0x7F
#define FSK_MAGN2_VALUE             0x00
#define FSK_MAGN1_VALUE             0x00
#define FSK_MAGN0_VALUE             0x00
#define FSK_ANG1_VALUE              0x00
#define FSK_ANG0_VALUE              0x00
#define FSK_CHFILT_I2_VALUE         0x08
#define FSK_CHFILT_I1_VALUE         0x00
#define FSK_CHFILT_I0_VALUE         0x00
#define FSK_CHFILT_Q2_VALUE         0x00
#define FSK_CHFILT_Q1_VALUE         0x00
#define FSK_CHFILT_Q0_VALUE         0x00
#define FSK_GPIO_STATUS_VALUE       0x00
#define FSK_FSCAL_CTRL_VALUE        0x01
#define FSK_PHASE_ADJUST_VALUE      0x00
#define FSK_PARTNUMBER_VALUE        0x00
#define FSK_PARTVERSION_VALUE       0x00
#define FSK_SERIAL_STATUS_VALUE     0x00
#define FSK_MODEM_STATUS1_VALUE     0x01
#define FSK_MODEM_STATUS0_VALUE     0x00
#define FSK_MARC_STATUS1_VALUE      0x00
#define FSK_MARC_STATUS0_VALUE      0x00
#define FSK_PA_IFAMP_TEST_VALUE     0x00
#define FSK_FSRF_TEST_VALUE         0x00
#define FSK_PRE_TEST_VALUE          0x00
#define FSK_PRE_OVR_VALUE           0x00
#define FSK_ADC_TEST_VALUE          0x00
#define FSK_DVC_TEST_VALUE          0x0B
#define FSK_ATEST_VALUE             0x40
#define FSK_ATEST_LVDS_VALUE        0x00
#define FSK_ATEST_MODE_VALUE        0x00
#define FSK_XOSC_TEST1_VALUE        0x3C
#define FSK_XOSC_TEST0_VALUE        0x00
#define FSK_RXFIRST_VALUE           0x00
#define FSK_TXFIRST_VALUE           0x00
#define FSK_RXLAST_VALUE            0x00
#define FSK_TXLAST_VALUE            0x00
#define FSK_NUM_TXBYTES_VALUE       0x00
#define FSK_NUM_RXBYTES_VALUE       0x00
#define FSK_FIFO_NUM_TXBYTES_VALUE  0x0F
#define FSK_FIFO_NUM_RXBYTES_VALUE  0x00

#endif