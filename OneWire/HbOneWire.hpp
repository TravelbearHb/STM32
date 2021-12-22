#ifndef HBONEWIRE_H
#define HBONEWIRE_H

/*!
 *  @file HbOneWire.h
    @brief \class HbOneWire
    Classes to Bit Bang GPIO Port

 *  A single GPIO pin is needed.
 *  Using Bit Banging routines. Multiple devices and be controlled using just 1 pin as each
 *
    @author Attila Horvath
        Console Data Systems (CDS)

    @date 6.12.2019

    @version 1.1


 */


#define USE_STDPERIPH_DRIVER
#include "common.h"

/*!

    @brief \class HBOneWire
    Classes to Bit Bang GPIO Port

    */
class HBOneWire
{

private:

	GPIO_InitTypeDef* GPIO_InitStruct;
    GPIO_TypeDef * GPIOx;
    uint16_t GPIO_Pin_x;
    uint32_t RCC_GPIOx;

	char* owDdr;
	char owPin;
	char* owOut;
	char* owIn;
	char owRecTime;
	uint32_t pinMask;
	uint32_t pinIn;
	uint32_t pinOut;
	uint32_t pinPos;

	uint8_t owGetIn();
	void owOutLow();
	void owOutHigh();
	void owDirIn();
	void owDirOut();
//char OW_Reset();
	uint8_t OW_ReadBit();
	void OW_WriteBit(uint8_t bit);


public:
	HBOneWire();
	void OW_Init(GPIO_TypeDef * inGPIOx, uint16_t inGPIO_Pin_x, uint32_t inRCC_GPIOx);

    uint8_t OW_Reset();
    uint8_t OW_Read();
    void OW_Write(uint8_t b);

};


#endif
