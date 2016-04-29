/**
*   @file     cn0216.cpp
*   @brief    Source file for CN0216
*   @author   Analog Devices Inc.
*
* For support please go to:
* Github: https://github.com/analogdevicesinc/mbed-adi
* Support: https://ez.analog.com/community/linux-device-drivers/microcontroller-no-os-drivers
* Product: www.analog.com/EVAL-CN0216-ARDZ
* More: https://wiki.analog.com/resources/tools-software/mbed-drivers-all

********************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
********************************************************************************/

#include "mbed.h"
#include "AD7791.h"
#include "CN0216.h"
extern Serial pc;

  CN0216::CN0216(PinName CSAD7791, PinName MOSI, PinName MISO, PinName SCK) : ad7791(1.2, CSAD7791, MOSI, MISO, SCK)
  {
	_cal_weight = 0;
    _zero_scale_value = 0;
    _full_scale_value = 0;
    _grams_per_bit = 0;

  }
  void CN0216::init(float cal_weight, uint8_t mode_val, uint8_t filter_val)
  {
    _cal_weight = cal_weight;
    ad7791.frequency(500000);
    wait_ms(50);
    ad7791.reset();
    wait_ms(50);
    ad7791.write_mode_reg(mode_val);
    wait_us(2);
    ad7791.write_filter_reg(filter_val);
    wait_ms(50);
  }

  void CN0216::calibrate(CalibrationStep_t cal)
  {
    uint64_t sum = 0;
    uint32_t min = 0xFFFFFFFF;
    uint32_t sample = 0;
    switch(cal)
    {
    case ZERO_SCALE_CALIBRATION:
    case FULL_SCALE_CALIBRATION:
      for(int i = 0;i < _NUMBER_OF_SAMPLES;i++)
      {
    	  sample = ad7791.read_u32();
    	  min = (min<sample) ? min : sample;
    	  sum += ad7791.read_u32();
		  wait_us(5);
      }
      if(cal == ZERO_SCALE_CALIBRATION)
      {
    	 // pc.printf("ZERO SCALE VALUE = %x",sum);
    	  _zero_scale_value = min;
      }
      else
      {
    	//  pc.printf("FULL SCALE VALUE = %x",sum);
    	  sum = sum / _NUMBER_OF_SAMPLES;
    	  _full_scale_value = sum;
      }
    break;

    case COMPUTE_GRAM_PER_BIT:
      _grams_per_bit = _cal_weight / (static_cast<float> (_full_scale_value - _zero_scale_value));  /* Calculate number of grams per LSB */
     // pc.printf("GRAMS/LSB = %f", _grams_per_bit);
    break;
    default:
    break;
    }

  }

  float CN0216::compute_weight(uint32_t data)
  {
//	pc.printf("\r\nFULL_SCALE_VALUE = %x\r\nZERO_SCALE_VALUE = %x\r\nDATA READ = %x\r\nGRAMS/LSB = %f\r\n",_full_scale_value,data,_zero_scale_value,_grams_per_bit);
	if(data<_zero_scale_value)
		data = _zero_scale_value; // clamp data to 0
	float weight_in_grams =  (static_cast<float>((data) - _zero_scale_value)) * _grams_per_bit;         /* Calculate weight */
    return weight_in_grams;
  }
  uint32_t CN0216::read_u32()
  {
    return ad7791.read_u32();
  }
  float CN0216::read_weight()  
  {
	uint32_t weight =  read_u32();
    return compute_weight(weight);
  }
  
