
#include "board.h"
#include "Filters.h"
#include <asf.h>
//#include <arm_math.h>
///////////////////////////////////////////////////////////////////////////////

fourthOrderData_t fourthOrder1000Hz[6];	//MAg and accel 3 axis

float B_1000HZ[11]=
{
	0.0001,
	-0.0008,
	0.0036,
	-0.0091,
	0.0156,
	-0.0185,
	0.0156,
	-0.0091,
	0.0036,
	-0.0008,
	0.0001
};

float A_1000HZ[10]=
{

	-9.4289,
	40.0228,
	-100.7104,
	166.3694,
	-188.5281,
	148.4136,
	-80.1436,
	28.4109,
	-5.9705,
	0.5648



};
// 
// float B_1000HZ_40CO[11]=
// {
// 	0.0000290224581581177,
// 	0.000290224581581177,
// 	0.00130601061711530,
// 	0.00348269497897413,
// 	0.00609471621320472,
// 	0.00731365945584566,
// 	0.00609471621320472,
// 	0.00348269497897413,
// 	0.00130601061711530,
// 	0.000290224581581177,
// 	0.0000290224581581177,
// 	
// 
// };
// 
// float A_1000HZ_40CO[10]=
// {
// 
// 	-5.31382564426177,
// 	14.9704009940599,
// 	-28.1875367016212,
// 	38.6678121770246,
// 	-40.0304008601612,
// 	31.5471848674149,
// 	-18.6783711810891,
// 	7.97494775847136,
// 	-2.23386836558090,
// 	0.317002218993552
// };

// float LPF_Fs1000Hz(float currentInput, fourthOrderData_t * filterParameters)
// {
//     // cheby2(4,60,12.5/100)
// #define B0_200HZ    0.0018
// #define B1_200HZ    0.0073
// #define B2_200HZ   0.0110
// #define B3_200HZ 0.0073
// #define B4_200HZ    0.0018
// 
// #define A1_200HZ  -3.0543
// #define A2_200HZ  3.8290
// #define A3_200HZ  -2.2925
// #define A4_200HZ  0.5507
//     float output;
// 
//     output = B0_200HZ * currentInput + B1_200HZ * filterParameters->inputTm1 + B2_200HZ * filterParameters->inputTm2 + B3_200HZ * filterParameters->inputTm3 + B4_200HZ * filterParameters->inputTm4 - A1_200HZ * filterParameters->outputTm1 - A2_200HZ * filterParameters->outputTm2 - A3_200HZ * filterParameters->outputTm3 - A4_200HZ * filterParameters->outputTm4;
// 
//     filterParameters->inputTm4 = filterParameters->inputTm3;
//     filterParameters->inputTm3 = filterParameters->inputTm2;
//     filterParameters->inputTm2 = filterParameters->inputTm1;
//     filterParameters->inputTm1 = currentInput;
// 
//     filterParameters->outputTm4 = filterParameters->outputTm3;
//     filterParameters->outputTm3 = filterParameters->outputTm2;
//     filterParameters->outputTm2 = filterParameters->outputTm1;
//     filterParameters->outputTm1 = output;
// 
//     return output;
// }

///////////////////////////////////////////////////////////////////////////////

// void HPF_setupFourthOrder200Hz(void)
// {
//     fourthOrder200Hz[AX_FILTER].inputTm1 = 0.0f;
//     fourthOrder200Hz[AX_FILTER].inputTm2 = 0.0f;
//     fourthOrder200Hz[AX_FILTER].inputTm3 = 0.0f;
//     fourthOrder200Hz[AX_FILTER].inputTm4 = 0.0f;
// 
//     fourthOrder200Hz[AX_FILTER].outputTm1 = 0.0f;
//     fourthOrder200Hz[AX_FILTER].outputTm2 = 0.0f;
//     fourthOrder200Hz[AX_FILTER].outputTm3 = 0.0f;
//     fourthOrder200Hz[AX_FILTER].outputTm4 = 0.0f;
// 
//     /////////////////////////////////////
// 
//     fourthOrder200Hz[AY_FILTER].inputTm1 = 0.0f;
//     fourthOrder200Hz[AY_FILTER].inputTm2 = 0.0f;
//     fourthOrder200Hz[AY_FILTER].inputTm3 = 0.0f;
//     fourthOrder200Hz[AY_FILTER].inputTm4 = 0.0f;
// 
//     fourthOrder200Hz[AY_FILTER].outputTm1 = 0.0f;
//     fourthOrder200Hz[AY_FILTER].outputTm2 = 0.0f;
//     fourthOrder200Hz[AY_FILTER].outputTm3 = 0.0f;
//     fourthOrder200Hz[AY_FILTER].outputTm4 = 0.0f;
// 
//     /////////////////////////////////////
// 
//     fourthOrder200Hz[AZ_FILTER].inputTm1 = -9.8065f;
//     fourthOrder200Hz[AZ_FILTER].inputTm2 = -9.8065f;
//     fourthOrder200Hz[AZ_FILTER].inputTm3 = -9.8065f;
//     fourthOrder200Hz[AZ_FILTER].inputTm4 = -9.8065f;
// 
//     fourthOrder200Hz[AZ_FILTER].outputTm1 = -9.8065f;
//     fourthOrder200Hz[AZ_FILTER].outputTm2 = -9.8065f;
//     fourthOrder200Hz[AZ_FILTER].outputTm3 = -9.8065f;
//     fourthOrder200Hz[AZ_FILTER].outputTm4 = -9.8065f;
// }

/************************************************************************/
/* cut off 100HZ                                                        */
/************************************************************************/
float LPF_Fs1000Hz_50CutOff(float currentInput, fourthOrderData_t * filterParameters)
{	
	
	// cheby2(4,60,12.5/100)
// 	#define B0_1000HZ  B_1000HZ[0]
// 	#define B1_1000HZ  B_1000HZ[1]
// 	#define B2_1000HZ  B_1000HZ[2]
// 	#define B3_1000HZ  B_1000HZ[3]
// 	#define B4_1000HZ  B_1000HZ[4]
// 	#define B5_1000HZ  B_1000HZ[5]
// 	#define B6_1000HZ  B_1000HZ[6]
// 	#define B7_1000HZ  B_1000HZ[7]
// 	#define B8_1000HZ  B_1000HZ[8]
// 	#define B9_1000HZ  B_1000HZ[9]
// 	#define B10_1000HZ B_1000HZ[10]

// 	#define A1_1000HZ  A_1000HZ[0]
// 	#define A2_1000HZ  A_1000HZ[1]
// 	#define A3_1000HZ  A_1000HZ[2]
// 	#define A4_1000HZ  A_1000HZ[3] 
// 	#define A5_1000HZ  A_1000HZ[4] 
// 	#define A6_1000HZ  A_1000HZ[5]
// 	#define A7_1000HZ  A_1000HZ[6]
// 	#define A8_1000HZ  A_1000HZ[7]
// 	#define A9_1000HZ  A_1000HZ[8]
// 	#define A10_1000HZ A_1000HZ[9]
// 	
	float output;

	output =  B_1000HZ[0] * currentInput +  B_1000HZ[1] * filterParameters->inputTm1 + B_1000HZ[2] * filterParameters->inputTm2 + B_1000HZ[3]* filterParameters->inputTm3 
	+  B_1000HZ[4] * filterParameters->inputTm4 + B_1000HZ[5] * filterParameters->inputTm5 + B_1000HZ[6] * filterParameters->inputTm6 + B_1000HZ[7] * filterParameters->inputTm7 + B_1000HZ[8] * filterParameters->inputTm8 + B_1000HZ[9] * filterParameters->inputTm9 + B_1000HZ[10] * filterParameters->inputTm10
	- A_1000HZ[0] * filterParameters->outputTm1 -  A_1000HZ[1] * filterParameters->outputTm2 - A_1000HZ[2] * filterParameters->outputTm3 - A_1000HZ[3]  * filterParameters->outputTm4  - A_1000HZ[4] * filterParameters->outputTm5 - A_1000HZ[5] * filterParameters->outputTm6- A_1000HZ[6] * filterParameters->outputTm7- A_1000HZ[7]* filterParameters->outputTm8- A_1000HZ[8] * filterParameters->outputTm9- A_1000HZ[9] * filterParameters->outputTm10;


filterParameters->inputTm10 = filterParameters->inputTm9;
filterParameters->inputTm9 = filterParameters->inputTm8;
filterParameters->inputTm8 = filterParameters->inputTm7;
filterParameters->inputTm7 = filterParameters->inputTm6;
filterParameters->inputTm6 = filterParameters->inputTm5;
filterParameters->inputTm5 = filterParameters->inputTm4;
	filterParameters->inputTm4 = filterParameters->inputTm3;
	filterParameters->inputTm3 = filterParameters->inputTm2;
	filterParameters->inputTm2 = filterParameters->inputTm1;
	filterParameters->inputTm1 = currentInput;



	
	filterParameters->outputTm10 = filterParameters->outputTm9;
	filterParameters->outputTm9 = filterParameters->outputTm8;
	filterParameters->outputTm8 = filterParameters->outputTm7;
	filterParameters->outputTm7 = filterParameters->outputTm6;
	filterParameters->outputTm6 = filterParameters->outputTm5;
	filterParameters->outputTm5 = filterParameters->outputTm4;

	filterParameters->outputTm4 = filterParameters->outputTm3;
	filterParameters->outputTm3 = filterParameters->outputTm2;
	filterParameters->outputTm2 = filterParameters->outputTm1;
	filterParameters->outputTm1 = output;

	return output;
}


/************************************************************************/
/* cut off 100HZ                                                        */
/************************************************************************/
// float LPF_Fs1000Hz_40CutOff(float currentInput, fourthOrderData_t * filterParameters)
// {
// 
// // cheby2(4,60,12.5/100)
// #define B0_1000HZ_40CO  B_1000HZ_40CO[0]
// #define B1_1000HZ_40CO  B_1000HZ_40CO[1]
// #define B2_1000HZ_40CO  B_1000HZ_40CO[2]
// #define B3_1000HZ_40CO  B_1000HZ_40CO[3]
// #define B4_1000HZ_40CO  B_1000HZ_40CO[4]
// #define B5_1000HZ_40CO  B_1000HZ_40CO[5]
// #define B6_1000HZ_40CO  B_1000HZ_40CO[6]
// #define B7_1000HZ_40CO  B_1000HZ_40CO[7]
// #define B8_1000HZ_40CO  B_1000HZ_40CO[8]
// #define B9_1000HZ_40CO  B_1000HZ_40CO[9]
// #define B10_1000HZ_40CO B_1000HZ_40CO[10]
// 
// #define A1_1000HZ_40CO  A_1000HZ_40CO[0]
// #define A2_1000HZ_40CO  A_1000HZ_40CO[1]
// #define A3_1000HZ_40CO  A_1000HZ_40CO[2]
// #define A4_1000HZ_40CO  A_1000HZ_40CO[3]
// #define A5_1000HZ_40CO  A_1000HZ_40CO[4]
// #define A6_1000HZ_40CO  A_1000HZ_40CO[5]
// #define A7_1000HZ_40CO  A_1000HZ_40CO[6]
// #define A8_1000HZ_40CO  A_1000HZ_40CO[7]
// #define A9_1000HZ_40CO  A_1000HZ_40CO[8]
// #define A10_1000HZ_40CO A_1000HZ_40CO[9]
// 
// float output;
// 
// output = B0_1000HZ_40CO * currentInput + B1_1000HZ_40CO * filterParameters->inputTm1 + B2_1000HZ_40CO * filterParameters->inputTm2 + B3_1000HZ_40CO * filterParameters->inputTm3 + B4_1000HZ_40CO * filterParameters->inputTm4
// 		+ B5_1000HZ_40CO * filterParameters->inputTm5 +  B6_1000HZ_40CO * filterParameters->inputTm6 + B7_1000HZ_40CO * filterParameters->inputTm7 + B8_1000HZ_40CO * filterParameters->inputTm8 + B9_1000HZ_40CO * filterParameters->inputTm9 + B10_1000HZ_40CO * filterParameters->inputTm10
// 
// 		- A1_1000HZ_40CO * filterParameters->outputTm1 - A2_1000HZ_40CO * filterParameters->outputTm2 - A3_1000HZ_40CO * filterParameters->outputTm3 - A4_1000HZ_40CO * filterParameters->outputTm4
// 		- A5_1000HZ_40CO * filterParameters->outputTm5 - A6_1000HZ_40CO * filterParameters->outputTm6- A7_1000HZ_40CO * filterParameters->outputTm7- A8_1000HZ_40CO * filterParameters->outputTm8- A9_1000HZ_40CO * filterParameters->outputTm9- A10_1000HZ_40CO * filterParameters->outputTm10;
// 
// 
// 		filterParameters->inputTm10 = filterParameters->inputTm9;
// 		filterParameters->inputTm9 = filterParameters->inputTm8;
// 		filterParameters->inputTm8 = filterParameters->inputTm7;
// 		filterParameters->inputTm7 = filterParameters->inputTm6;
// 		filterParameters->inputTm6 = filterParameters->inputTm5;
// 		filterParameters->inputTm5 = filterParameters->inputTm4;
// 		filterParameters->inputTm4 = filterParameters->inputTm3;
// 		filterParameters->inputTm3 = filterParameters->inputTm2;
// 		filterParameters->inputTm2 = filterParameters->inputTm1;
// 		filterParameters->inputTm1 = currentInput;
// 
// 
// 
// 
// 		filterParameters->outputTm10 = filterParameters->outputTm9;
// 		filterParameters->outputTm9 = filterParameters->outputTm8;
// 		filterParameters->outputTm8 = filterParameters->outputTm7;
// 		filterParameters->outputTm7 = filterParameters->outputTm6;
// 		filterParameters->outputTm6 = filterParameters->outputTm5;
// 		filterParameters->outputTm5 = filterParameters->outputTm4;
// 
// 		filterParameters->outputTm4 = filterParameters->outputTm3;
// 		filterParameters->outputTm3 = filterParameters->outputTm2;
// 		filterParameters->outputTm2 = filterParameters->outputTm1;
// 		filterParameters->outputTm1 = output;
// 
// 	return output;
// }
