/*
  June 2012

  BaseFlightPlus Rev -

  An Open Source STM32 Based Multicopter

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick

  Designed to run on Naze32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

///////////////////////////////////////////////////////////////////////////////

#define AX_FILTER 0
#define AY_FILTER 1
#define AZ_FILTER 2

typedef struct {
    float inputTm1, inputTm2, inputTm3, inputTm4, inputTm5, inputTm6, inputTm7, inputTm8, inputTm9, inputTm10, inputTm11;
    float outputTm1, outputTm2, outputTm3, outputTm4, outputTm5, outputTm6, outputTm7, outputTm8, outputTm9, outputTm10, outputTm11;
} fourthOrderData_t;


///////////////////////////////////////////////////////////////////////////////

float computeFourthOrder100Hz(float currentInput, fourthOrderData_t * filterParameters);

///////////////////////////////////////////////////////////////////////////////

void setupFourthOrder100Hz(void);

///////////////////////////////////////////////////////////////////////////////

float LPF_computeFourthOrder200Hz(float currentInput, fourthOrderData_t * filterParameters);

///////////////////////////////////////////////////////////////////////////////

void LPF_setupFourthOrder200Hz(void);

///////////////////////////////////////////////////////////////////////////////

float LPF_Fs1000Hz(float currentInput, fourthOrderData_t * filterParameters);

///////////////////////////////////////////////////////////////////////////////

float LPF_Fs1000Hz_50CutOff(float currentInput, fourthOrderData_t * filterParameters);


void HPF_setupFourthOrder200Hz(void);