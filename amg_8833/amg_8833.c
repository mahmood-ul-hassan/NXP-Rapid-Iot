#include <stdio.h>
#include <stdint.h>

#include "amg_8833.h"

#include "sensors.h"
#include "board.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

		 // The power control register
struct pctl {
            // 0x00 = Normal Mode
			// 0x01 = Sleep Mode
			// 0x20 = Stand-by mode (60 sec intermittence)
			// 0x21 = Stand-by mode (10 sec intermittence)
         uint8_t PCTL : 8;
};


//reset register
struct rst {
			//0x30 = flag reset (all clear status reg 0x04, interrupt flag and interrupt table)
			//0x3F = initial reset (brings flag reset and returns to initial setting)
			uint8_t RST : 8;
};

//frame rate register
struct fpsc {
			//0 = 10FPS
			//1 = 1FPS
			uint8_t FPS : 1;

};


		//interrupt control register
struct intc {

			// 0 = INT output reactive (Hi-Z)
			// 1 = INT output active
			uint8_t INTEN : 1;

			// 0 = Difference interrupt mode
			// 1 = absolute value interrupt mode
			uint8_t INTMOD : 1;

};

		//status register
struct stat {
			uint8_t unused : 1;
			//interrupt outbreak (val of interrupt table reg)
			uint8_t INTF : 1;

			//temperature output overflow (val of temperature reg)
			uint8_t OVF_IRS : 1;

			//thermistor temperature output overflow (value of thermistor)
			uint8_t OVF_THS : 1;


};

		//status clear register
		//write to clear overflow flag and interrupt flag
		//after writing automatically turns to 0x00
struct sclr {
			uint8_t unused : 1;
			//interrupt flag clear
			uint8_t INTCLR : 1;
			//temp output overflow flag clear
			uint8_t OVS_CLR : 1;
			//thermistor temp output overflow flag clear
			uint8_t OVT_CLR : 1;
};

		//average register
		//for setting moving average output mode
struct ave {
			uint8_t unused : 5;
			//1 = twice moving average mode
			uint8_t MAMOD : 1;
};

		//interrupt level registers
		//for setting upper / lower limit hysteresis on interrupt level

		//interrupt level upper limit setting. Interrupt output
		// and interrupt pixel table are set when value exceeds set value
struct inthl {
			uint8_t INT_LVL_H : 8;
};

struct inthh {
			uint8_t INT_LVL_H : 4;
};

		//interrupt level lower limit. Interrupt output
		//and interrupt pixel table are set when value is lower than set value
struct intll {
			uint8_t INT_LVL_L : 8;
};

struct intlh {
			uint8_t INT_LVL_L : 4;
};

		//setting of interrupt hysteresis level when interrupt is generated.
		//should not be higher than interrupt level
struct ihysl {
			uint8_t INT_HYS : 8;
};

struct ihysh {
			uint8_t INT_HYS : 4;
};

		//thermistor register
		//SIGNED MAGNITUDE FORMAT
struct tthl {
			uint8_t TEMP : 8;
};

struct tthh {
			uint8_t TEMP : 3;
			uint8_t SIGN : 1;
};

		//temperature registers 0x80 - 0xFF
		/*
		//read to indicate temperature data per 1 pixel
		//SIGNED MAGNITUDE FORMAT
		struct t01l {
			uint8_t TEMP : 8;

			uint8_t get(){
				return TEMP;
			}
		};
		struct t01l _t01l;

		struct t01h {
			uint8_t TEMP : 3;
			uint8_t SIGN : 1;

			uint8_t get(){
				return ( (SIGN << 3) | TEMP) & 0xF;
			}
		};
		struct t01h _t01h;
		*/

struct pctl _pctl;
struct rst _rst;
struct fpsc _fpsc;
struct intc _intc;
struct stat _stat;
struct sclr _sclr;
struct ave _ave;
struct inthl _inthl;
struct inthh _inthh;
struct intll _intll;
struct intlh _intlh;
struct ihysl _ihysl;
struct ihysh _ihysh;
struct tthl _tthl;
struct tthh _tthh;

static uint8_t getPCTL(void){ return _pctl.PCTL; }
static uint8_t getRST(void){	return _rst.RST; }
static uint8_t getFPSC(void){ return _fpsc.FPS & 0x01; }
static uint8_t getINTC(void){ return (_intc.INTMOD << 1 | _intc.INTEN) & 0x03; }
static uint8_t getSTAT(void){ return ( (_stat.OVF_THS << 3) | (_stat.OVF_IRS << 2) | (_stat.INTF << 1) ) & 0x07; }
static uint8_t getSCLR(void){ return ((_sclr.OVT_CLR << 3) | (_sclr.OVS_CLR << 2) | (_sclr.INTCLR << 1)) & 0x07; }
static uint8_t getAVE(void){ return (_ave.MAMOD << 5); }
static uint8_t getINTHL(void){ return _inthl.INT_LVL_H; }
static uint8_t getINTHH(void){ return _inthh.INT_LVL_H; }
static uint8_t getINTLL(void){ return _intll.INT_LVL_L; }
static uint8_t getINTLH(void){ return (_intlh.INT_LVL_L & 0xF); }
static uint8_t getIHYSL(void){ return _ihysl.INT_HYS; }
static uint8_t getIHYSH(void){ return (_ihysh.INT_HYS & 0xF); }
static uint8_t getTTHL(void){ return _tthl.TEMP; }
static uint8_t getTTHH(void){ return ( (_tthh.SIGN << 3) | _tthh.TEMP) & 0xF; }
static uint8_t min(uint8_t a, uint8_t b){ return a < b ? a : b; }


static void write8(uint8_t reg, uint8_t value);
static uint8_t read8(uint8_t reg);

static void read(uint8_t reg, uint8_t *buf, uint8_t num);
static void write(uint8_t reg, uint8_t *buf, uint8_t num);

static float signedMag12ToFloat(uint16_t val);


/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
    @param  addr Optional I2C address the sensor can be found on. Default is 0x69
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/
int amg88xxInit(void)
{
	//enter normal mode
	_pctl.PCTL = AMG88xx_NORMAL_MODE;
	write8(AMG88xx_PCTL, getPCTL());
	
	//software reset
	_rst.RST = AMG88xx_INITIAL_RESET;
	write8(AMG88xx_RST, getRST());
	
	//disable interrupts by default
	disableInterrupt();
	
	//set to 10 FPS
	_fpsc.FPS = AMG88xx_FPS_10;
	write8(AMG88xx_FPSC, getFPSC());

	App_WaitMsec(100);

	return 0;
}

/**************************************************************************/
/*! 
    @brief  Set the moving average mode.
    @param  mode if True is passed, output will be twice the moving average
*/
/**************************************************************************/
void setMovingAverageMode(int mode)
{
	_ave.MAMOD = mode;
	write8(AMG88xx_AVE, getAVE());
}

/**************************************************************************/
/*! 
    @brief  Set the interrupt levels. The hysteresis value defaults to .95 * high
    @param  high the value above which an interrupt will be triggered
    @param  low the value below which an interrupt will be triggered
*/
/**************************************************************************/
void setInterruptLevels(float high, float low)
{
	setInterruptLevelsHist(high, low, high * .95f);
}

/**************************************************************************/
/*! 
    @brief  Set the interrupt levels
    @param  high the value above which an interrupt will be triggered
    @param  low the value below which an interrupt will be triggered
    @param hysteresis the hysteresis value for interrupt detection
*/
/**************************************************************************/
void setInterruptLevelsHist(float high, float low, float hysteresis)
{
	int highConv = high / AMG88xx_PIXEL_TEMP_CONVERSION;
	highConv = constrain(highConv, -4095, 4095);
	_inthl.INT_LVL_H = highConv & 0xFF;
	_inthh.INT_LVL_H = (highConv & 0xF) >> 4;
	write8(AMG88xx_INTHL, getINTHL());
	write8(AMG88xx_INTHH, getINTHH());
	
	int lowConv = low / AMG88xx_PIXEL_TEMP_CONVERSION;
	lowConv = constrain(lowConv, -4095, 4095);
	_intll.INT_LVL_L = lowConv & 0xFF;
	_intlh.INT_LVL_L = (lowConv & 0xF) >> 4;
	write8(AMG88xx_INTLL, getINTLL());
	write8(AMG88xx_INTLH, getINTLH());
	
	int hysConv = hysteresis / AMG88xx_PIXEL_TEMP_CONVERSION;
	hysConv = constrain(hysConv, -4095, 4095);
	_ihysl.INT_HYS = hysConv & 0xFF;
	_ihysh.INT_HYS = (hysConv & 0xF) >> 4;
	write8(AMG88xx_IHYSL, getIHYSL());
	write8(AMG88xx_IHYSH, getIHYSH());
}

/**************************************************************************/
/*! 
    @brief  enable the interrupt pin on the device.
*/
/**************************************************************************/
void enableInterrupt()
{
	_intc.INTEN = 1;
	write8(AMG88xx_INTC, getINTC());
}

/**************************************************************************/
/*! 
    @brief  disable the interrupt pin on the device
*/
/**************************************************************************/
void disableInterrupt()
{
	_intc.INTEN = 0;
	write8(AMG88xx_INTC, getINTC());
}

/**************************************************************************/
/*! 
    @brief  Set the interrupt to either absolute value or difference mode
    @param  mode passing AMG88xx_DIFFERENCE sets the device to difference mode, AMG88xx_ABSOLUTE_VALUE sets to absolute value mode.
*/
/**************************************************************************/
void setInterruptMode(uint8_t mode)
{
	_intc.INTMOD = mode;
	write8(AMG88xx_INTC, getINTC());
}

/**************************************************************************/
/*! 
    @brief  Read the state of the triggered interrupts on the device. The full interrupt register is 8 bytes in length.
    @param  buf the pointer to where the returned data will be stored
    @param  size Optional number of bytes to read. Default is 8 bytes.
    @returns up to 8 bytes of data in buf
*/
/**************************************************************************/
void getInterrupt(uint8_t *buf, uint8_t size)
{
	uint8_t bytesToRead = min(size, (uint8_t)8);
	
	read(AMG88xx_INT_OFFSET, buf, bytesToRead);
}

/**************************************************************************/
/*! 
    @brief  Clear any triggered interrupts
*/
/**************************************************************************/
void clearInterrupt()
{
	_rst.RST = AMG88xx_FLAG_RESET;
	write8(AMG88xx_RST, getRST());
}

/**************************************************************************/
/*! 
    @brief  read the onboard thermistor
    @returns a the floating point temperature in degrees Celsius
*/
/**************************************************************************/
float readThermistor()
{
	uint8_t raw[2];
	read(AMG88xx_TTHL, raw, 2);
	uint16_t recast = ((uint16_t)raw[1] << 8) | ((uint16_t)raw[0]);

	return signedMag12ToFloat(recast) * AMG88xx_THERMISTOR_CONVERSION;
}

/**************************************************************************/
/*! 
    @brief  Read Infrared sensor values
    @param  buf the array to place the pixels in
    @param  size Optionsl number of bytes to read (up to 64). Default is 64 bytes.
    @return up to 64 bytes of pixel data in buf
*/
/**************************************************************************/
void readPixels(float *buf, uint8_t size)
{
	uint16_t recast;
	float converted;
	uint8_t bytesToRead = min((uint8_t)(size << 1), (uint8_t)(AMG88xx_PIXEL_ARRAY_SIZE << 1));
	uint8_t rawArray[bytesToRead];
	read(AMG88xx_PIXEL_OFFSET, rawArray, bytesToRead);
	
	for(int i=0; i<size; i++){
		uint8_t pos = i << 1;
		recast = ((uint16_t)rawArray[pos + 1] << 8) | ((uint16_t)rawArray[pos]);
		
		converted = signedMag12ToFloat(recast) * AMG88xx_PIXEL_TEMP_CONVERSION;
		buf[i] = converted;
	}
}


void readPixelsRaw(int16_t* buf)
{
	read(AMG88xx_PIXEL_OFFSET, (uint8_t*)buf, 128);
}

/**************************************************************************/
/*! 
    @brief  write one byte of data to the specified register
    @param  reg the register to write to
    @param  value the value to write
*/
/**************************************************************************/
static void write8(uint8_t reg, uint8_t value)
{
	write(reg, &value, 1);
}

/**************************************************************************/
/*! 
    @brief  read one byte of data from the specified register
    @param  reg the register to read
    @returns one byte of register data
*/
/**************************************************************************/
static uint8_t read8(uint8_t reg)
{
	uint8_t ret;
	read(reg, &ret, 1);
	
	return ret;
}


static void read(uint8_t reg, uint8_t *buf, uint8_t num)
{
	App_I2C2_Read(AMG88xx_ADDRESS, &reg, 1, buf, num);
}

static void write(uint8_t reg, uint8_t *buf, uint8_t num)
{
	App_I2C2_Write(AMG88xx_ADDRESS, &reg, 1);
	App_I2C2_Write(AMG88xx_ADDRESS, buf, num);
}

/**************************************************************************/
/*! 
    @brief  convert a 12-bit signed magnitude value to a floating point number
    @param  val the 12-bit signed magnitude value to be converted
    @returns the converted floating point value
*/
/**************************************************************************/
static float signedMag12ToFloat(uint16_t val)
{
	//take first 11 bits as absolute val
	uint16_t absVal = (val & 0x7FF);
	
	return (val & 0x8000) ? 0 - (float)absVal : (float)absVal ;
}
