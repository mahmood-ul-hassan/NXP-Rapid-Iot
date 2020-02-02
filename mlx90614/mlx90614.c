/*!
    \file  mlx90614.c
    \brief the read and write function file
    
    \version 2019-06-05, V1.0.0, demo for GD32VF103
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include <stdio.h>
#include "mlx90614.h"

#include "sensors.h"

static float ir_readTemp(uint8_t reg);
static uint16_t ir_read16(uint8_t addr);
static void ir_sensor_buffer_read(uint8_t* p_buffer, uint8_t read_address, uint16_t number_of_byte);


uint32_t ir_readID(void){

}

double ir_readObjectTempF(void) {
  return (ir_readTemp(MLX90614_TOBJ1) * 9 / 5) + 32;
}

double ir_readAmbientTempF(void) {
  return (ir_readTemp(MLX90614_TA) * 9 / 5) + 32;
}

double ir_readObjectTempC(void) {
  return ir_readTemp(MLX90614_TOBJ1);
}

double ir_readAmbientTempC(void) {
  return ir_readTemp(MLX90614_TA);
}

static float ir_readTemp(uint8_t reg) {
  float temp;
  
  temp = ir_read16(reg);
  temp *= .02;
  temp  -= 273.15;
  return temp;
}

/*!
    \brief      I2C read 16bit function
    \param[in]  Address
    \param[out] none
    \retval     16bit read Value 
*/
static uint16_t ir_read16(uint8_t addr) {
  uint16_t data_temp =0;
  uint8_t data[4] = {0};
  ir_sensor_buffer_read(data, addr, 2);
  data_temp = data[0]|(data[1]<<8);
  return data_temp;
}

static void ir_sensor_buffer_read(uint8_t* p_buffer, uint8_t read_address, uint16_t number_of_byte)
{
	uint8_t ret_i2c = I2C_RESULT_OK;
//	ret_i2c = App_I2C2_Read(MLX90614_I2CADDR, &read_address, 1, p_buffer, number_of_byte);
	ret_i2c = App_I2C0_Read(MLX90614_I2CADDR, &read_address, 1, p_buffer, number_of_byte);
}
