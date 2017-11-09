#ifndef _ADE9KLIB_h
#define _ADE9KLIB_h

#include <Arduino.h>
#include <SPI.h>

#define PHASE_A 0x00
#define PHASE_B 0x01
#define PHASE_C 0x02
#define PHASE_N 0x03

#define ANGL_VA_VB 0
#define ANGL_VB_VC 1
#define ANGL_VA_VC 2
#define ANGL_VA_IA 3
#define ANGL_VB_IB 4
#define ANGL_VC_IC 5
#define ANGL_IA_IB 6
#define ANGL_IB_IC 7
#define ANGL_IA_IC 8

#ifndef VRMSCONSTANT
#define VRMSCONSTANT 13.29629989
#endif
#ifndef ARMSCONSTANT
#define ARMSCONSTANT 3.72695927003422 
#endif
#ifndef WATTCONSTANT
#define WATTCONSTANT 6.65112834224501
#endif
#ifndef WATHCONSTANT
#define WATHCONSTANT 1.89187650623858
#endif
#ifndef ANGLCONSTANT
#define ANGLCONSTANT 0.017578125
#endif

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

class ADE9K
{
  private:
    //Private Vars
    SPIClass *_ADE9KSPI;
    int _SSpin;
    uint8_t uc_ADEChipID;

    //ISR CallBack Functions
    void (*_temperatureCallback)(void);

    //Private functions
    void enableChip();
    void disableChip();
    void Init_ADE9000_SPI();
    float cal27(int32_t sample);
    uint16_t read16bit(uint16_t ADDR);
    uint32_t read32bit(uint16_t ADDR);
    uint16_t Checkcrc16( uint8_t puc_pCheck_Data[],  uint16_t  us_Nr_Bytes);
    void Read_ADE9000_SPI(uint16_t  us_ADE_Addr, uint16_t us_Nr_Bytes, uint8_t *puc_Reg_Data);
    void Write_ADE9000_SPI(uint16_t  us_ADE_Addr, uint8_t uc_Nr_Bytes, uint8_t *puc_Reg_Data);
    uint16_t Read_ADE9000_CRC_SPI(uint16_t  us_ADE_Addr, uint16_t us_Nr_Bytes, uint8_t *puc_Reg_Data);

  public:
    ADE9K (SPIClass *, int);
    uint8_t begin();
    uint8_t IsADE9000();
    void setTemperatureCallback(void (*temperatureCallback)(void));
    float getISUMRMS();
    float getIRMS(uint8_t phase);
    float getVRMS(uint8_t phase);
    float getWatt(uint8_t phase);
    float getVAR(uint8_t phase);
    float getVA(uint8_t phase);
    float getVTHD(uint8_t phase);
    float getITHD(uint8_t phase);
    float getPF(uint8_t phase);
    long IRMS(uint8_t phase);
    float getAngl(uint8_t phase);
    float getFreq(uint8_t phase);
    float getTemperature(void);
    void requestTemperature(void); 
    void ISRStatus0(void);
};

#endif
