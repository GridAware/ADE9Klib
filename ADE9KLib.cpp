#include "ADE9KLib.h"
#include "ADE9000.h"

SPISettings ADESETTINGS(200000, MSBFIRST, SPI_MODE0);

ADE9K::ADE9K(SPIClass *ADESPI, int SSpin)
{
    _ADE9KSPI=ADESPI;
    _SSpin=SSpin;
    uc_ADEChipID=0;
}

uint8_t ADE9K::begin()
{
    pinMode(_SSpin, OUTPUT);
    digitalWrite(_SSpin, HIGH);

    //Set configuration
	ADE_REGISTER_UNION ADE9000TempData;
	ADE9000TempData.ul_Register = 1;
	Write_ADE9000_SPI(ADDR_RUN, 2, ADE9000TempData.uc_Register);
	Write_ADE9000_SPI(ADDR_EP_CFG, 2, ADE9000TempData.uc_Register);

    //Enable temperature sensor
    uint8_t tempRegister[2];
    tempRegister[0] = 0 | BIT0 | BIT1 | BIT2 | BIT3;
    Write_ADE9000_SPI(ADDR_TEMP_CFG, 2, tempRegister);

    //Check if chip is ADE9K
	uc_ADEChipID = IsADE9000();
    return uc_ADEChipID;
}

float ADE9K::getIRMS(uint8_t phase)
{
    int32_t data = read32bit(ADDR_AIRMS + (phase * 0x20));
    return ((float)data * ARMSCONSTANT) / 1000000 ;
}

float ADE9K::getISUMRMS()
{
    return read32bit(ADDR_ISUMRMS) * ARMSCONSTANT;
}

float ADE9K::getVRMS(uint8_t phase)
{
    return read32bit(ADDR_AVRMS + (phase * 0x20)) * VRMSCONSTANT / 1000000;
}

float ADE9K::getWatt(uint8_t phase)
{
    int32_t value = read32bit(ADDR_AWATT + (phase * 0x20));
    return (float)value * WATTCONSTANT / 1000.0;
}

float ADE9K::getVAR(uint8_t phase)
{
    int32_t value = read32bit(ADDR_AVAR + (phase * 0x20));
    return (float)value * WATTCONSTANT / 1000.0;
}

float ADE9K::getVA(uint8_t phase)
{
    int32_t value = read32bit(ADDR_AVA + (phase * 0x20));
    return (float)value * WATTCONSTANT / 1000.0;
}

long ADE9K::IRMS(uint8_t phase)
{
  char i=0;
  long current=0;
  getIRMS(phase);//Ignore first reading
  for(i=0;i<10;++i){
    current+=getIRMS(phase);
    delayMicroseconds(50);
  }
  //average
  return current/10;
}

/*
 * The THD calculation is stored in signed 5.27 format. The
 * highest THD value is 0x2000 0000, which corresponds to a
 * THD of 400%. To calculate the THD value as a percentage, use
 * the following equation:
 * %THD on Current Channel A = AITHD × 2−27 × 100%
 */

float ADE9K::getVTHD(uint8_t phase)
{
    return cal27(read32bit(ADDR_AVTHD + (phase * 0x20))) * 100;
}

float ADE9K::getITHD(uint8_t phase)
{
    return cal27(read32bit(ADDR_AITHD + (phase * 0x20))) * 100;
}


/*
 * The power factor result is stored in 5.27 format. The highest
 * power factor value is 0x07FF FFFF, which corresponds to a power
 * factor of 1. A power factor of −1 is stored as 0xF800 0000. To
 * determine the power factor from the xPF register value, use the
 * following equation:
 * Power Factor = xPF × 2^−27
 */
float ADE9K::getPF(uint8_t phase)
{
    return cal27(read32bit(ADDR_APF + (phase * 0x20)));
}

/*
 * Get V/I phase angles
 * Options
 * 0: ANGL_VA_VB
 * 1: ANGL_VB_VC
 * 2: ANGL_VA_VC
 * 3: ANGL_VA_IA
 * 4: ANGL_VB_IB
 * 5: ANGL_VC_IC
 * 6: ANGL_IA_IB
 * 7: ANGL_IB_IC
 * 8: ANGL_IA_IC
 */
float ADE9K::getAngl(uint8_t angle)
{
    int16_t data = read16bit(ADDR_ANGL_VA_VB + angle);
    return (float)data * ANGLCONSTANT;
}

float ADE9K::getFreq(uint8_t phase)
{
    return 524288000 / (float)read32bit(ADDR_APERIOD + phase);
}

/*
 * Function for checking if there is a chip
 */
uint8_t ADE9K::IsADE9000(void)
{
	uint8_t uc_Read_Data[4];
	uint8_t uc_iChipID = 0;

	Read_ADE9000_SPI(ADDR_PART_ID,0x4,uc_Read_Data);
	if(uc_Read_Data[2] == 0x10)
		uc_iChipID = 1;
	return uc_iChipID;
}
/*
 * Temperature(°C) = TEMP_RSLT × (−TEMP_GAIN/65536) + (TEMP_OFFSET/32)
 */
float ADE9K::getTemperature(void)
{
    //Read Gain and Offset
    uint8_t uc_Read_Data[4];
    Read_ADE9000_SPI(ADDR_TEMP_TRIM,0x4,uc_Read_Data);
    uint16_t TEMP_GAIN = uc_Read_Data[0] | (uc_Read_Data[1] << 8);
    uint16_t TEMP_OFFSET = uc_Read_Data[2] | (uc_Read_Data[3] << 8);

    //Read temperatuur
    uint8_t uc_Read_Temp_Data[2];
    Read_ADE9000_SPI(ADDR_TEMP_RSLT,0x2,uc_Read_Temp_Data);
    uint16_t Temp_RSLT = 0 | uc_Read_Temp_Data[0] | (uc_Read_Temp_Data[1] << 8);
    float Temperature = ((float)Temp_RSLT * (((float)TEMP_GAIN * -1 )/65536)) + ((float)TEMP_OFFSET/32);

    //Enable temperature sensor (next reading)
    uint8_t tempRegister[2];
    tempRegister[0] = 0 | BIT0 | BIT1 | BIT2 | BIT3;
    Write_ADE9000_SPI(ADDR_TEMP_CFG, 2, tempRegister);
    return Temperature;
}

//Private functions
float ADE9K::cal27(int32_t sample)
{
    int32_t integer_bits = sample >> 27;
    int32_t integer = integer_bits <= ( (1 << 5 ) - 1 >> 1) ? integer_bits : - ((integer_bits ^ (1 << 5 ) - 1) + 1);

    int32_t fractional_bits = sample & (1 << 27) - 1;
    float fractional = fractional_bits * powf(2, -27);

    return (float)integer + fractional;
}
uint16_t ADE9K::read16bit(uint16_t ADDR)
{
    uint8_t ret[4];
    uint16_t data = 0x0000;
    Read_ADE9000_SPI(ADDR,0x02,ret);
    data = ret[0] | (ret[1] << 8); 
    return data;
}
uint32_t ADE9K::read32bit(uint16_t ADDR)
{
    uint8_t ret[4];
    uint32_t data = 0x00000000;
    Read_ADE9000_SPI(ADDR,0x04,ret);
    data = ret[0] | (ret[1] << 8) | (ret[2] << 16) | (ret[3] << 24); 
    return data;
}

void ADE9K::enableChip()
{
    digitalWrite(_SSpin, LOW);
}

void ADE9K::disableChip()
{
    digitalWrite(_SSpin, HIGH);
}

uint16_t  ADE9K::Checkcrc16( uint8_t puc_pCheck_Data[],  uint16_t  us_Nr_Bytes)
{
    uint8_t   uc_b   = 0;
    uint16_t  us_crc = 0xffff;
    uint16_t  uc_i, uc_j;

    for (uc_i = 0 ; uc_i < us_Nr_Bytes; uc_i ++ )
    {        
        for (uc_j = 0 ; uc_j < 8 ; uc_j ++ )
        {
            uc_b = ((puc_pCheck_Data[uc_i] << uc_j) & 0x80 ) ^ ((us_crc & 0x8000 ) >> 8 );
            us_crc <<= 1 ;
            if (uc_b != 0 )
                us_crc ^= 0x1021 ;
        }  
    }  
    return us_crc;
} 

void ADE9K::Read_ADE9000_SPI(uint16_t  us_ADE_Addr, uint16_t us_Nr_Bytes, uint8_t *puc_Reg_Data)
{
    uint16_t us_iCounter;
    uint16_t us_iAddress; 
    uint8_t *puc_TempData;
    uint8_t  uc_LS_Addr,uc_MS_Addr;

    us_iAddress = us_ADE_Addr;
    us_iAddress = (us_iAddress << 4);
    us_iAddress = (us_iAddress | 0x08);
    puc_TempData = puc_Reg_Data;

    uc_LS_Addr = (uint8_t) us_iAddress;
    uc_MS_Addr =(uint8_t) (us_iAddress >> 8);

    _ADE9KSPI->beginTransaction(ADESETTINGS);
    enableChip();
    delay(10);
    _ADE9KSPI->transfer(uc_MS_Addr);
    _ADE9KSPI->transfer(uc_LS_Addr);

    //ADE90xx comm data format: MSB--LSB ; So need send out the MSB first.
    puc_TempData=puc_TempData+(us_Nr_Bytes-1);

    for (us_iCounter=0;us_iCounter<us_Nr_Bytes;us_iCounter++)
    {
        *puc_TempData=_ADE9KSPI->transfer(0x00);
        puc_TempData--;
    }
    disableChip();
    _ADE9KSPI->endTransaction();    
}


uint16_t ADE9K::Read_ADE9000_CRC_SPI(uint16_t  us_ADE_Addr, uint16_t us_Nr_Bytes, uint8_t *puc_Reg_Data)
{
	uint16_t us_iCounter;
	uint16_t us_iAddress;  	
	uint8_t  uc_LS_Addr,uc_MS_Addr;
	uint8_t uc_Read_Data[4];

	us_iAddress = us_ADE_Addr;
	us_iAddress = (us_iAddress << 4);
	us_iAddress = (us_iAddress | 0x08);
	
  	uc_LS_Addr = (uint8_t) us_iAddress;
	uc_MS_Addr =(uint8_t) (us_iAddress >> 8);
	
    _ADE9KSPI->beginTransaction(ADESETTINGS);
    enableChip();
    delay(10);
    _ADE9KSPI->transfer(uc_MS_Addr);
    _ADE9KSPI->transfer(uc_LS_Addr);


	//ADE90xx comm data format: MSB--LSB ; So need send out the MSB first.
	puc_Reg_Data=puc_Reg_Data+(us_Nr_Bytes-1);

	for (us_iCounter=0;us_iCounter<us_Nr_Bytes;us_iCounter++)
	{
		*puc_Reg_Data=_ADE9KSPI->transfer(0x00);
		puc_Reg_Data--;
	 }
    disableChip();
    _ADE9KSPI->endTransaction();

	puc_Reg_Data=puc_Reg_Data+us_Nr_Bytes;
	for (us_iCounter=0;us_iCounter<us_Nr_Bytes-2;us_iCounter++)
	{
		uc_Read_Data[us_iCounter] = *puc_Reg_Data;
		puc_Reg_Data--;
	 }
	
	us_iAddress = Checkcrc16(uc_Read_Data, us_Nr_Bytes-2);

	return us_iAddress;
}

void ADE9K::Write_ADE9000_SPI(uint16_t us_ADE_Addr, uint8_t uc_Nr_Bytes, uint8_t *puc_Reg_Data)
{
    uint8_t uc_iCounter;
    uint16_t us_iAddress;  	
    uint8_t uc_MS_Addr;
    uint8_t uc_LS_Addr;

    us_iAddress = us_ADE_Addr;
    us_iAddress = (us_iAddress << 4);
    us_iAddress = (us_iAddress & 0xFFF7);

    uc_LS_Addr = (uint8_t) us_iAddress;
    uc_MS_Addr =(uint8_t) (us_iAddress >> 8);

    _ADE9KSPI->beginTransaction(ADESETTINGS);
    enableChip();
    delay(10);
    _ADE9KSPI->transfer(uc_MS_Addr);
    _ADE9KSPI->transfer(uc_LS_Addr);

    //ADE90xx comm data format: MSB--LSB ; So need send out the MSB first.
    puc_Reg_Data=puc_Reg_Data+(uc_Nr_Bytes-1);
    for(uc_iCounter=0;uc_iCounter<uc_Nr_Bytes;uc_iCounter++)
    {
        _ADE9KSPI->transfer(*puc_Reg_Data);
        puc_Reg_Data--;
    }
    disableChip();
    _ADE9KSPI->endTransaction();    
}
