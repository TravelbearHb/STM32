//---------------------------------------------------------------------

/*!
 *  @file DS18b20Device.cpp
    @brief  TDS18b20Device
    Classes to access Dallas DS18b20 Temperature sensors.

 *  A single GPIO pin is needed.
 *  Using Bit Banging routines. Multiple devices and be controlled using just 1 pin as each
 *
    @author Attila Horvath
        Console Data Systems (CDS)

    @date 6.12.2019

    @version 1.1


 */

#include "DS18b20Device.h"
#include "HbDelay.h"

HBOneWire HBOneWireDS18[NUM_ONEWIRE];



 TDS18b20Device::TDS18b20Device()
{
//	 HBOneWireDS18.GPIO_Pin_x = GPIO_Pin_5;
//	 HBOneWireDS18.GPIOx = GPIOB;
//	 HBOneWireDS18.RCC_GPIOx = RCC_APB2Periph_GPIOB;
	 valid = false;
}

void TDS18b20Device::DsInit(uint8_t owNum, GPIO_TypeDef * inGPIOx, uint16_t inGPIO_Pin_x, uint32_t inRCC_GPIOx)
{
	HBOneWireDS18[owNum].OW_Init(inGPIOx, inGPIO_Pin_x, inRCC_GPIOx);
	OwDevice = owNum;
	valid = true;

}
 bool  TDS18b20Device::ReadROM()
 {
 	bool retCode = false;

 	if (!HBOneWireDS18[OwDevice].OW_Reset()){
 		HBOneWireDS18[OwDevice].OW_Write(DS18_CMD_READ_ROM);
 		ds18b20Family = HBOneWireDS18[OwDevice].OW_Read();
 		for (int i = 0; i < 6; i++)
 	 		ds18b20ROM[i] = HBOneWireDS18[OwDevice].OW_Read();
 		ds18b20CRC = HBOneWireDS18[OwDevice].OW_Read();
 		retCode = true;
    	}
 	return retCode;
 }
bool  TDS18b20Device::ResetSkipROM()
{
	bool retCode = false;
	if (!HBOneWireDS18[OwDevice].OW_Reset()){
		HBOneWireDS18[OwDevice].OW_Write(DS18_CMD_SKIP_ROM);
		retCode = true;
   	}

   return retCode;
}

bool  TDS18b20Device::MatchROM()
{
	bool retCode = false;
	OwDevice = ds18b20Devices[dsIndex].owDevice;
//	ds18b20Devices[dsIndex].
	if (!HBOneWireDS18[OwDevice].OW_Reset()){
		HBOneWireDS18[OwDevice].OW_Write(DS18_CMD_MATCH_ROM);

		HBOneWireDS18[OwDevice].OW_Write(ds18b20Devices[dsIndex].ds18b20Family);
		for (int i = 0; i < 6; i++)
			HBOneWireDS18[OwDevice].OW_Write(ds18b20Devices[dsIndex].rom[i]);
		HBOneWireDS18[OwDevice].OW_Write(ds18b20Devices[dsIndex].ds18b20CRC);
		retCode = true;
   	}

   return retCode;
}

bool  TDS18b20Device::StartConversion()
{
//	pController->flushRecBuffer();
	bool retCode = false;
	if (ResetSkipROM()){
		HBOneWireDS18[OwDevice].OW_Write(DS18_CMD_START_CONVERSION);
		retCode = true;
	}

   return retCode;
}

/*!
    Read the current available result in the 18b20
    Make sure the conversion was previously started and the necessary delay was done
*/
bool  TDS18b20Device::ReadTemp()
{
	int8_t t;
	int16_t temp2;

	bool retCode = false;
	bool doReadFlag = false;
	OwDevice = ds18b20Devices[dsIndex].owDevice;
	if (ds18b20Devices[dsIndex].matchRom){
		if (MatchROM())
			doReadFlag = true;
	}
	else
		if (ResetSkipROM())
			doReadFlag = true;
	if (doReadFlag){
		HBOneWireDS18[OwDevice].OW_Write(DS18_CMD_READ_TEMP);
		scratch[0] = HBOneWireDS18[OwDevice].OW_Read();
		scratch[1] = HBOneWireDS18[OwDevice].OW_Read();
//			if (HBOneWireDS18[OwDevice].OW_Read(scratch, DS18_SCRATCHPAD_SIZE)){
			t = ((scratch[0] >> 4) | ((scratch[1] & 0xF) << 4));
			/**
			temp1 = (scratch[1] & (0x7)) * 16;
			temp1 += (scratch[0] / 16);

			finalTemp = (scratch[0] & (0xF)) * 0.0625 + temp1;
			**/

			temp1 = ((int16_t) scratch[1] << 8) | scratch[0];
			if (temp1 & 0x1000){
				temp2 = (temp1 ^ 0xFFFF) + 1;
				temp2 = -temp2;
			}
			else
				temp2 = temp1;
			finalTemp =(float) temp2 * 0.0625;
			retCode = true;
      		}

   return retCode;
}


/*!
    Test routine for 1 cycle start conversion and read
*/

bool  TDS18b20Device::GetTemp()
{
	StartConversion();
    delay_us(750000);
//	Sleep(750);
	ReadTemp();
	return true;
}
