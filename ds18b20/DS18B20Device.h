//----------------------------------------------------------------------------
#ifndef DS18b20DeviceH
#define DS18b20DeviceH
//----------------------------------------------------------------------------

/*!
 *  @file DS18b20DeviceH.h
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


#include "HBOneWire.hpp"


#define MAX_DEVICE_PER_PIN				3
#define DS18_CMD_SEARCH_ROM				0xF0
#define DS18_CMD_SKIP_ROM				0xCC
#define DS18_CMD_MATCH_ROM				0x55
#define DS18_CMD_READ_ROM				0x33
#define DS18_CMD_READ_TEMP				0xBE
#define DS18_CMD_START_CONVERSION	    0x44
#define DS18_SCRATCHPAD_SIZE			9

#define NUM_ONEWIRE	7
extern HBOneWire HBOneWireDS18[NUM_ONEWIRE];

/*! \struct ds18b20Device
 * Defines a DS18b20 device and on which wire it is connected to
 */

typedef struct {
	uint8_t owDevice;          //!< index into OneWire definition Array
	uint8_t ds18b20Family;
	uint8_t rom[6];
	uint8_t ds18b20CRC;
	bool matchRom;
} ds18b20Device;

//#define NUM_TEMP_SENSORS 4

extern ds18b20Device ds18b20Devices[];
//extern ds18b20Device ds18b20Devices[NUM_TEMP_SENSORS];

/*!
 *  Routines to access Dallas DS18b20 Temperature Sensor Using One wire
 *  Using Bit Banging routines
 */
class TDS18b20Device
{
private:

	bool ResetSkipROM();  //!< Ignore ROM check

	bool MatchROM();      //!< Used if more than one device on one PIN


public:
	TDS18b20Device();


   /** Initialize the OW array with the port and pin
        @param owNum - index into OW Array (List of OW Pins)
        @param inGPIOx - Port for OW
        @param inGPIO_Pin_x - Pin for OW
        @param inRCC_GPIOx - RCC for port
        */

	void DsInit(uint8_t owNum, GPIO_TypeDef * inGPIOx, uint16_t inGPIO_Pin_x, uint32_t inRCC_GPIOx);

	bool StartConversion();        //!< start Temperature Conversion
	bool ReadTemp();               //!< Get the temperature from device : Store in finalTemp
	bool GetTemp();                //!< Start Conversion, Wait for result, Read Result
	bool ReadROM();                //!< read the ROM of current Device. MAKE SURE ONLY ONE DEVICE CONNECTED

	bool DSStartConversion();      //!< NOT IMPLEMENTED
	bool DSReadTemp();             //!< NOT IMPLEMENTED

	double finalTemp;              //!< Last temperature read
	uint16_t temp1;
	bool valid;


	uint8_t dsIndex;
	uint8_t OwDevice;
	uint8_t ds18b20Family;
	uint8_t ds18b20ROM[6];
	uint8_t ds18b20CRC;
	uint8_t scratch[DS18_SCRATCHPAD_SIZE];


};


//----------------------------------------------------------------------------
#endif
