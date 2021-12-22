/*!
 *  @file HbOneWire.cpp
    @brief  HbOneWire


 *  Access Dallas 1-Wire Devices
 *  A single GPIO pin is needed.
 *  Using Bit Banging routines. Multiple devices and be controlled using just 1 pin as each
 *
    @author Attila Horvath
        Console Data Systems (CDS)

    @date 6.12.2019

    @version 1.1


 */

#include "HBOneWire.hpp"

#include "HbDelay.h"
#include "hbGPIO.h"

#define OW_GET_IN()   ( OW_IN & (1<<OW_PIN))
#define OW_OUT_LOW()  ( OW_OUT &= (~(1 << OW_PIN)) )
#define OW_OUT_HIGH() ( OW_OUT |= (1 << OW_PIN) )
#define OW_DIR_IN()   ( OW_DDR &= (~(1 << OW_PIN )) )
#define OW_DIR_OUT()  ( OW_DDR |= (1 << OW_PIN) )

#define pin5_mask	0x00F00000
#define pin5_in		0x00400000
#define pin5_out	0x00700000

#define RESET_DELAY1	10			// 10
#define RESET_DELAY2	500			// 500
#define RESET_DELAY3	100			// 80
#define RESET_DELAY4	400			// 420

#define READ_BIT_DELAY1	2			// 2
#define READ_BIT_DELAY2	12			// 12
#define READ_BIT_DELAY3	53			// 53

#define WRITE_BIT_DELAY1	5		// 10
#define WRITE_BIT_DELAY2	60		// 55
#define WRITE_BIT_DELAY3	65		// 65
#define WRITE_BIT_DELAY4	5      		// 5

#define  BEFORE_READ_DELAY	5
#define  BEFORE_WRITE_DELAY	5

HBOneWire::HBOneWire()
{

}
uint8_t HBOneWire::owGetIn()
{
//	return (*owIn & owPin);
	return GPIO_ReadInputDataBit(GPIOx, GPIO_Pin_x);

}
void HBOneWire::owOutLow()
{
	//*owOut &= ~owPin;
	GPIO_ResetBits(GPIOx, GPIO_Pin_x);
}
void HBOneWire::owOutHigh()
{
	GPIO_SetBits(GPIOx, GPIO_Pin_x);
//	*owOut |= owPin;

}
void HBOneWire::owDirIn()
{
//	hbSetGPIO(GPIOx, GPIO_Pin_x, RCC_GPIOx, GPIO_Mode_IN_FLOATING);
//	GPIOB->CRL &= ~pin5_mask;
//	GPIOB->CRL |= pin5_in;


#ifdef STM32F103


	if (GPIO_Pin_x < 0x0100){
		GPIOB->CRL &= ~pinMask;
		GPIOB->CRL |= pinIn;
	}
	else {
		GPIOB->CRH &= ~pinMask;
		GPIOB->CRH |= pinIn;
	}

#else
    GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (pinPos * 2));
    GPIOx->MODER |= (((uint32_t)GPIO_Mode_IN) << (pinPos * 2));

#endif

}
void HBOneWire::owDirOut()
{


#ifdef STM32F103


	if (GPIO_Pin_x < 0x0100){
		GPIOB->CRL &= ~pinMask;
		GPIOB->CRL |= pinOut;
	}
	else {
		GPIOB->CRH &= ~pinMask;
		GPIOB->CRH |= pinOut;
	}

#else
	GPIOx->PUPDR &= ~(3 << (pinPos * 2));
    GPIOx->OTYPER   &= ~(1 << (pinPos));
    GPIOx->OTYPER   |= (1 << (pinPos));
    GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (pinPos * 2));
    GPIOx->MODER |= (((uint32_t)GPIO_Mode_OUT) << (pinPos * 2));

#endif
}

void HBOneWire::OW_Init(GPIO_TypeDef * inGPIOx, uint16_t inGPIO_Pin_x, uint32_t inRCC_GPIOx)
{
    GPIOx = inGPIOx;
    GPIO_Pin_x = inGPIO_Pin_x;
    RCC_GPIOx = inRCC_GPIOx;
	pinPos = 0;
    while (!(inGPIO_Pin_x & 0x0001)){
    	pinPos++;
		inGPIO_Pin_x >>= 1;
	}
#ifdef STM32F103
    hbSetGPIO(GPIOx, GPIO_Pin_x, RCC_GPIOx, GPIO_Mode_IN_FLOATING);
#else
    hbSetGPIO(GPIOx, GPIO_Pin_x, RCC_GPIOx, GPIO_Mode_IN);
#endif
}


uint8_t HBOneWire::OW_Reset()
{
    uint8_t retCode;


    //** make sure pullup on
	owDirIn();
    delay_us(RESET_DELAY1);  //10us

	owOutLow();
	owDirOut();
    delay_us(RESET_DELAY2);  //500us

	owDirIn();
    delay_us(RESET_DELAY3);  // 80us

    retCode = owGetIn();
//    retCode = 1;        // test!!! must remove later

    delay_us(RESET_DELAY4);  //420us

    return retCode;
}
uint8_t HBOneWire::OW_ReadBit()
{
	uint8_t retCode;

	owOutLow();
	owDirOut();
    delay_us(READ_BIT_DELAY1); // 2us

	owDirIn();


    delay_us(READ_BIT_DELAY2); // 12us


    retCode = owGetIn();
	delay_us(READ_BIT_DELAY3);  // 53us

	return retCode;
}
void HBOneWire::OW_WriteBit(uint8_t bit)
{

	owOutLow();
	owDirOut();

    if ( bit){
    	delay_us(WRITE_BIT_DELAY1);  // 10us
    	owDirIn();
    	delay_us(WRITE_BIT_DELAY2); // 55us
    }
    else{
    	delay_us(WRITE_BIT_DELAY3); // 65us
    	owDirIn();
    	delay_us(WRITE_BIT_DELAY4);  // 5us
    }

}

uint8_t HBOneWire::OW_Read()
{
    uint8_t b = 0;

    uint8_t i = 8;
    delay_us(BEFORE_READ_DELAY);  // space 2 consecutive reads
    while ( i--){
      b>>=1;
      if (OW_ReadBit())
         b |= 0x80;
      }
	return b;
}

void HBOneWire::OW_Write(uint8_t b)
{
    uint8_t i = 8;

    delay_us(BEFORE_WRITE_DELAY);  // space 2 consecutive WRITES
    while ( i--){
      OW_WriteBit((b & 0x01));
      b>>=1;
      }
	owOutLow();

}
