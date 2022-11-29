/*
 * bldc_driver.cpp
 *
 *  Created on: Nov 24, 2022
 *      Author: hemingshan
 */

#include "bldc_driver.hpp"

/***************************************************************************/
// int array instead of float array
// 4x200 points per 360 deg
// 2x storage save (int 2Byte float 4 Byte )
// sin*10000
const int sine_array[200] = {0,79,158,237,316,395,473,
		552,631,710,789,867,946,1024,1103,1181,1260,
		1338,1416,1494,1572,1650,1728,1806,1883,1961,
		2038,2115,2192,2269,2346,2423,2499,2575,2652,
		2728,2804,2879,2955,3030,3105,3180,3255,3329,
		3404,3478,3552,3625,3699,3772,3845,3918,3990,
		4063,4135,4206,4278,4349,4420,4491,4561,4631,
		4701,4770,4840,4909,4977,5046,5113,5181,5249,
		5316,5382,5449,5515,5580,5646,5711,5775,5839,
		5903,5967,6030,6093,6155,6217,6279,6340,6401,
		6461,6521,6581,6640,6699,6758,6815,6873,6930,
		6987,7043,7099,7154,7209,7264,7318,7371,7424,
		7477,7529,7581,7632,7683,7733,7783,7832,7881,
		7930,7977,8025,8072,8118,8164,8209,8254,8298,
		8342,8385,8428,8470,8512,8553,8594,8634,8673,
		8712,8751,8789,8826,8863,8899,8935,8970,9005,
		9039,9072,9105,9138,9169,9201,9231,9261,9291,
		9320,9348,9376,9403,9429,9455,9481,9506,9530,
		9554,9577,9599,9621,9642,9663,9683,9702,9721,
		9739,9757,9774,9790,9806,9821,9836,9850,9863,
		9876,9888,9899,9910,9920,9930,9939,9947,9955,
		9962,9969,9975,9980,9985,9989,9992,9995,9997,
		9999,10000,10000};
/***************************************************************************/
// function approximating the sine calculation by using fixed size array
// ~40us (float array)
// ~50us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _sin(float a){
  if(a < _PI_2){
    //return sine_array[(int)(126.6873* a)];           // float array optimized
    return 0.0001*sine_array[_round(126.6873* a)];      // int array optimized
  }else if(a < _PI){
    //return sine_array[398 - (int)(126.6873*a)];          // float array optimized
    return 0.0001*sine_array[398 - _round(126.6873*a)];     // int array optimized
  }else if(a < _3PI_2){
    //return -sine_array[-398 + (int)(126.6873*a)];           // float array optimized
    return -0.0001*sine_array[-398 + _round(126.6873*a)];      // int array optimized
  } else {
    //return -sine_array[796 - (int)(126.6873*a)];           // float array optimized
    return -0.0001*sine_array[796 - _round(126.6873*a)];      // int array optimized
  }
}
/***************************************************************************/
// function approximating cosine calculation by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _cos(float a){
  float a_sin = a + _PI_2;
  a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
  return _sin(a_sin);
}
/***************************************************************************/
// normalizing radian angle to [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}


Motor_FOC::Motor_FOC(float voltage_power_supply_)
{
	voltage_power_supply = voltage_power_supply_;
	pole_pairs = 12;
}

void Motor_FOC::loopFOC(void)
{
	setPhaseVoltage();
}

void Motor_FOC::move(float new_target)
{
// Type_Torque:
	Voltage_q = new_target;
}

void Motor_FOC::shaftAngle(float target)
{
	// if no sensor linked return previous value (for open loop)
	shaft_angle = target;
}

void Motor_FOC::electricalAngle(void)
{
	electrical_angle = _normalizeAngle(shaft_angle*pole_pairs);
//	return _normalizeAngle((shaft_angle + sensor_offset) * pole_pairs - zero_electric_angle);
}
void Motor_FOC::setPhaseVoltage(void)
{

	// 6 STEP: Discrete Mode:

//    static int trap_120_map[6][3] = {
//      {0,1,-1},{-1,1,0},{-1,0,1},{0,-1,1},{1,-1,0},{1,0,-1} // each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
//    };
//    // static int trap_120_state = 0;
//    for(int i = 0; i<6; i++)
//    {
//    	float Ua, Ub, Uc;
//        Ua = 0.5 + trap_120_map[i][0] * 0.5;
//        Ub = 0.5 + trap_120_map[i][1] * 0.5;
//        Uc = 0.5 + trap_120_map[i][2] * 0.5;
//        _writeDutyCyclePWM(Ua, Ub, Uc);
//        HAL_Delay(500);
//    }

    // SINE PWM:

    float _ca, _sa, Ualpha, Ubeta;
    float Ua, Ub, Uc;
    for(int i = 0; i<24;i++)
    {
    	 float angle_el = _PI_12*i;
		 _ca = _cos(angle_el);
		 _sa = _sin(angle_el);
		 Ualpha =  - _sa*0.5;
		 Ubeta  =    _ca*0.5;

		 Ua = Ualpha/2 + 0.5;
		 Ub = (-0.5 * Ualpha  + _SQRT3_2 * Ubeta)/2+0.5;
		 Uc = (-0.5 * Ualpha - _SQRT3_2 * Ubeta)/2+0.5;
		 _writeDutyCyclePWM(Ua, Ub, Uc);
//		 HAL_Delay(1);
		 DWT_Delay_us(1);
    }

     // others:
//	float Uout;
//	float T0, T1, T2;
//	float Ta, Tb, Tc;
//	uint32_t sector;
//
//	Uout = Voltage_q / voltage_power_supply;
//	float angle_el = _normalizeAngle(electrical_angle+_PI_2);
//
//	if(Uout> 0.577)Uout= 0.577;
//	if(Uout<-0.577)Uout=-0.577;
//
//	sector = (angle_el / _PI_3) + 1;
//
//	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
//	T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;
//	T0 = 1 - T1 - T2;
//
//	// calculate the duty cycles(times)
//	switch(sector)
//	{
//		case 1:
//			Ta = T1 + T2 + T0/2;
//			Tb = T2 + T0/2;
//			Tc = T0/2;
//			break;
//		case 2:
//			Ta = T1 +  T0/2;
//			Tb = T1 + T2 + T0/2;
//			Tc = T0/2;
//			break;
//		case 3:
//			Ta = T0/2;
//			Tb = T1 + T2 + T0/2;
//			Tc = T2 + T0/2;
//			break;
//		case 4:
//			Ta = T0/2;
//			Tb = T1+ T0/2;
//			Tc = T1 + T2 + T0/2;
//			break;
//		case 5:
//			Ta = T2 + T0/2;
//			Tb = T0/2;
//			Tc = T1 + T2 + T0/2;
//			break;
//		case 6:
//			Ta = T1 + T2 + T0/2;
//			Tb = T0/2;
//			Tc = T1 + T0/2;
//			break;
//		default:  // possible error state
//			Ta = 0;
//			Tb = 0;
//			Tc = 0;
//	}

//	_writeDutyCyclePWM(Ta, Tb, Tc);
}


uint32_t DWT_Delay_Init(void) {
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

     /* 3 NO OPERATION instructions */
     __ASM volatile ("NOP");
     __ASM volatile ("NOP");
  __ASM volatile ("NOP");

  /* Check if clock cycle counter has started */
     if(DWT->CYCCNT)
     {
       return 0; /*clock cycle counter started*/
     }
     else
  {
    return 1; /*clock cycle counter not started*/
  }
}
