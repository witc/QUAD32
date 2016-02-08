//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"

// Kalman filter parameters
float beta = 0.151149947f; // Sqrt(3.0 / 4.0) * (PI * (10 / 180.0));
float zeta = 0.001511499f; // Sqrt(3.0 / 4.0) * (PI * (0.1 / 180.0));

// gyroscope biases error (part of filter state)
float w_bx = 0, w_by = 0, w_bz = 0;

// state & output
// reference direction of earth's magnetic field aka heading and inclination
float b_x = 1, b_z = 0;

// corrected gyro result
float w_x, w_y, w_z;
// computed flux in the earth frame
float h_x, h_y, h_z;
//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	76.0f		// sample frequency in Hz	76 HZ - 13 ms
#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations


//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float samplePeriod, EulerAngles *Angle)
{
	
		Quaternion quat;
		
		float recipNorm;
		float s0, s1, s2, s3;
		float qDot1, qDot2, qDot3, qDot4;
		float hx, hy;
		float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz,_8bx, _8bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
		//Heading uhly;
		// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
// 		if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
// 			//	MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
// 			return;
// 		}

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Normalise magnetometer measurement
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0f * q0 * mx;
			_2q0my = 2.0f * q0 * my;
			_2q0mz = 2.0f * q0 * mz;
			_2q1mx = 2.0f * q1 * mx;
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_2q0q2 = 2.0f * q0 * q2;
			_2q2q3 = 2.0f * q2 * q3;
			q0q0 = q0 * q0;
			q0q1 = q0 * q1;
			q0q2 = q0 * q2;
			q0q3 = q0 * q3;
			q1q1 = q1 * q1;
			q1q2 = q1 * q2;
			q1q3 = q1 * q3;
			q2q2 = q2 * q2;
			q2q3 = q2 * q3;
			q3q3 = q3 * q3;

			// Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
			_2bx = sqrt(hx * hx + hy * hy);
			_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
			_4bx = 2.0f * _2bx;
			_4bz = 2.0f * _2bz;
			_8bx = 2.0f * _4bx;
			_8bz = 2.0f * _4bz;

			// Gradient decent algorithm corrective step
			s0= -_2q2*(2.0f*(q1q3 - q0q2) - ax)    +   _2q1*(2.0f*(q0q1 + q2q3) - ay)   +  -_4bz*q2*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   +   (-_4bx*q3+_4bz*q1)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)    +   _4bx*q2*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
			s1= _2q3*(2.0f*(q1q3 - q0q2) - ax) +   _2q0*(2.0f*(q0q1 + q2q3) - ay) +   -4.0f*q1*(2.0f*(0.5 - q1q1 - q2q2) - az)    +   _4bz*q3*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   + (_4bx*q2+_4bz*q0)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)   +   (_4bx*q3-_8bz*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
			s2= -_2q0*(2.0f*(q1q3 - q0q2) - ax)    +     _2q3*(2.0f*(q0q1 + q2q3) - ay)   +   (-4.0f*q2)*(2.0f*(0.5 - q1q1 - q2q2) - az) +   (-_8bx*q2-_4bz*q0)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(_4bx*q1+_4bz*q3)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q0-_8bz*q2)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
			s3= _2q1*(2.0f*(q1q3 - q0q2) - ax) +   _2q2*(2.0f*(q0q1 + q2q3) - ay)+(-_8bx*q3+_4bz*q1)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(-_4bx*q0+_4bz*q2)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
			
			
			
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		q0 += qDot1 * (1.0f* samplePeriod);
		q1 += qDot2 * (1.0f * samplePeriod);
		q2 += qDot3 * (1.0f * samplePeriod);
		q3 += qDot4 * (1.0f * samplePeriod);

		// Normalise quaternion
		recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
		/*
	Quaternion quat;
	
// 	float a_x=a[0];
// 	float a_y=a[1];
// 	float a_z=a[2];
// 	float mx=m[0];
// 	float my=m[1];
// 	float mz=m[2];
	
	// earth relative to sensor quaternion elements with initial conditions (part of filter state)
	#define SEq_1 q0
	#define SEq_2 q1
	#define SEq_3 q2
	#define SEq_4 q3

	// local system variables
	float norm;                                                            // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;  // quaternion rate from gyroscopes elements
	float f_1, f_2, f_3, f_4, f_5, f_6;                                    // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33,              // objective function Jacobian elements
	J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;              // estimated direction of the gyroscope error (quaternion derrivative)
	float w_err_x, w_err_y, w_err_z;                                       // estimated direction of the gyroscope error (angular)

	// axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;
	float twoSEq_4 = 2.0f * SEq_4;
	float twob_xSEq_1 = 2.0f * b_x * SEq_1;
	float twob_xSEq_2 = 2.0f * b_x * SEq_2;
	float twob_xSEq_3 = 2.0f * b_x * SEq_3;
	float twob_xSEq_4 = 2.0f * b_x * SEq_4;
	float twob_zSEq_1 = 2.0f * b_z * SEq_1;
	float twob_zSEq_2 = 2.0f * b_z * SEq_2;
	float twob_zSEq_3 = 2.0f * b_z * SEq_3;
	float twob_zSEq_4 = 2.0f * b_z * SEq_4;

	// normalise the accelerometer measurement
	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= norm;
	ay *= norm;
	az *= norm;

	// normalise the magnetometer measurement
	norm = invSqrt(mx * mx + my * my + mz * mz);
	mx *= norm;
	my *= norm;
	mz *= norm;

	// compute the objective function and Jacobian
	f_1 = 2.0f * (SEq_2 * SEq_4 - SEq_1 * SEq_3) - ax;
	f_2 = 2.0f * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - ay;
	f_3 = 2.0f * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - az;
	f_4 = 2.0f * b_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + 2.0f * b_z * (SEq_2 * SEq_4 - SEq_1 * SEq_3) - mx;
	f_5 = 2.0f * b_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + 2.0f * b_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - my;
	f_6 = 2.0f * b_x * (SEq_1 * SEq_3 + SEq_2 * SEq_4) + 2.0f * b_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - mz;
	J_11or24 = twoSEq_3;                                                    // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1;                                                    // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21;                                                    // negated in matrix multiplication
	J_33 = 2.0f * J_11or24;                                                    // negated in matrix multiplication
	J_41 = twob_zSEq_3;                                                     // negated in matrix multiplication
	J_42 = twob_zSEq_4;
	J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1;                                   // negated in matrix multiplication
	J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;                                   // negated in matrix multiplication
	J_51 = twob_xSEq_4 - twob_zSEq_2;                                       // negated in matrix multiplication
	J_52 = twob_xSEq_3 + twob_zSEq_1;
	J_53 = twob_xSEq_2 + twob_zSEq_4;
	J_54 = twob_xSEq_1 - twob_zSEq_3;                                       // negated in matrix multiplication
	J_61 = twob_xSEq_3;
	J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
	J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
	J_64 = twob_xSEq_2;

	// compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

	// normalise the gradient to estimate direction of the gyroscope error
	norm = invSqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 = SEqHatDot_1 * norm;
	SEqHatDot_2 = SEqHatDot_2 * norm;
	SEqHatDot_3 = SEqHatDot_3 * norm;
	SEqHatDot_4 = SEqHatDot_4 * norm;

	// compute angular estimated direction of the gyroscope error
	w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
	w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
	w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;

	// compute gyroscope baises
	w_bx += w_err_x * samplePeriod * zeta;
	w_by += w_err_y * samplePeriod * zeta;
	w_bz += w_err_z * samplePeriod * zeta;

	// correct gyroscope result
	w_x = gx - w_bx;
	w_y = gy - w_by;
	w_z = gz - w_bz;

	// compute the quaternion rate measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

	// compute then integrate the estimated quaternion rate
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * samplePeriod;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * samplePeriod;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * samplePeriod;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * samplePeriod;

	// normalise quaternion
	norm = invSqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 *= norm;
	SEq_2 *= norm;
	SEq_3 *= norm;
	SEq_4 *= norm;

	// compute flux in the earth frame
	h_x = 2.0f * mx * (0.5f - (-SEq_3) * (-SEq_3) - (-SEq_4) * (-SEq_4)) + 2.0f * my * (SEq_1 * (-SEq_4) + (-SEq_2) * (-SEq_3)) + 2.0f * mz * ((-SEq_2) * (-SEq_4) - SEq_1 * (-SEq_3));
	h_y = 2.0f * mx * ((-SEq_2) * (-SEq_3) - SEq_1 * (-SEq_4)) + 2.0f * my * (0.5f - (-SEq_2) * (-SEq_2) - (-SEq_4) * (-SEq_4)) + 2.0f * mz * (SEq_1 * (-SEq_2) + (-SEq_3) * (-SEq_4));
	h_z = 2.0f * mx * (SEq_1 * (-SEq_3) + (-SEq_2) * (-SEq_4)) + 2.0f * my * ((-SEq_3) * (-SEq_4) - SEq_1 * (-SEq_2)) + 2.0f * mz * (0.5f - (-SEq_2) * (-SEq_2) - (-SEq_3) * (-SEq_3));

	// normalise the flux vector to have only components in the x and z
	b_x = sqrtf((h_x * h_x) + (h_y * h_y));
	b_z = h_z;
	*/
    const float qwqw = q0 * q0; // calculate common terms to avoid repetition
	Angle->roll = RADIANS_TO_DEGREES(atan2f(2.0f * (q2 * q3 - q0 * q1), 2.0f * (qwqw - 0.5f + q3 * q3)));
    Angle->pitch = RADIANS_TO_DEGREES(-asinf(2.0f * (q1 * q3 + q1 * q2)));
    Angle->yaw = RADIANS_TO_DEGREES(atan2f(2.0f * (q1 * q2 - q0 * q3), 2.0f * (qwqw - 0.5f + q1 * q1)));
   
 // QuaternionToEulerAngles(quat,&Angle);
   
	/* WIKI */
		
//     uhly->roll = (double)(atan2((double)((2 * (q0 * q1 + q2 * q3))),(double)(( 1 - 2 * (q1 * q1 + q2 * q2)))));
//    	uhly->pitch =(double) (asin((2 * (q0 * q2 - q1 * q3))));
//    	uhly->yaw = (double )(atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)));
// 	
// 	uhly->roll=uhly->roll*180/M_PI;
// 	uhly->pitch=uhly->pitch*180/M_PI;
// 	uhly->yaw=uhly->yaw*180/M_PI;
// 	float q[4];
// 	
//
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float samplePeriod, EulerAngles *Angle)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f* samplePeriod);
	q1 += qDot2 * (1.0f * samplePeriod);
	q2 += qDot3 * (1.0f * samplePeriod);
	q3 += qDot4 * (1.0f* samplePeriod);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	   const float qwqw = q0 * q0; // calculate common terms to avoid repetition
	   Angle->roll = RADIANS_TO_DEGREES(atan2f(2.0f * (q2 * q3 - q0 * q1), 2.0f * (qwqw - 0.5f + q3 * q3)));
	   Angle->pitch = RADIANS_TO_DEGREES(-asinf(2.0f * (q1 * q3 + q1 * q2)));
	   Angle->yaw = RADIANS_TO_DEGREES(atan2f(2.0f * (q1 * q2 - q0 * q3), 2.0f * (qwqw - 0.5f + q1 * q1)));
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
// 	float halfx = 0.5f * x;
// 	float y = x;
// 	long i = *(long*)&y;
// 	i = 0x5f3759df - (i>>1);
// 	y = *(float*)&i;
// 	y = y * (1.5f - (halfx * y * y));
// 	return y;
	
//  	unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
//  	float tmp = *(float*)&i;
//  	return (tmp * (1.69000231f - 0.714158168f * x * tmp * tmp));

	return 1/sqrtf(x);
}



/**
 * @brief Converts a quaternion to Euler angles in degrees.
 * @param quaternion Quaternion to be converted.
 * @return Euler angles in degrees.
 */
void QuaternionToEulerAngles(Quaternion Q,EulerAngles *eulerAngles) 
{
    const float qwqw = Q.w * Q.w; // calculate common terms to avoid repetition
	eulerAngles->roll = RADIANS_TO_DEGREES(atan2f(2.0f * (Q.y * Q.z - Q.w * Q.x), 2.0f * (qwqw - 0.5f + Q.z * Q.z)));
    eulerAngles->pitch = RADIANS_TO_DEGREES(-asinf(2.0f * (Q.x * Q.z + Q.w * Q.y)));
    eulerAngles->yaw = RADIANS_TO_DEGREES(atan2f(2.0f * (Q.x * Q.y - Q.w * Q.z), 2.0f * (qwqw - 0.5f + Q.x * Q.x)));
   

}

//====================================================================================================
// END OF CODE
//====================================================================================================
