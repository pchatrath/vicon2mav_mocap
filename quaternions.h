/* quaternions.h
 * author: Julian L. Nicklas, julian.nicklas@posteo.de
 *
 * Use at your own risk, I guarantee nothing.
 *
 * A quaternion is defined as an float array {w, x, y, z}.
 * Sources for the equations:
 *      hhtp://mathworld.wolfram.com/Quaternion.html
 *		http://j3d.org/matrix_faq/matrfaq_latest.html#Q60
 *		https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 */

#pragma once

#include <math.h>

void quat_from_axisrad(float *quat, float *radians);

void quat_rotate(float *position, float *rotation);
// result will be saved in position
// value of rotation will not be changed
void quat_normalize(float *quat);
void quat_inverse(float *quat);
void quat_multiply(float* lhs, float* rhs);
// result will be saved in lhs
// value of rhs will not be changed


/**
 * Converts a one-axis-rotation to a quaternion
 * ONLY ONE OF X_RAD, Y_RAD AND Z_RAD IS ALLOWED TO CARRY A VALUE !!!
 * THE OTHERS NEEDS TO BE 0 !!
 *
 * *quat points to an float array {w, x, y, z}, which will be filled
 * *radians points to an float array {x_rad, y_rad, z_rad} 
 * 
 * ONLY ONE OF X_RAD, Y_RAD AND Z_RAD IS ALLOWED TO CARRY A VALUE !!!
 * THE OTHERS NEEDS TO BE 0 !!
 * otherwise: there will be a result with no use at all
 */
void quat_from_axisrad(float *quat, float *radians)
{
	float x_rad = *((float*)(radians    ));
	float y_rad = *((float*)(radians + 1));
	float z_rad = *((float*)(radians + 2));

    *((float*)(quat))       = cos((x_rad + y_rad + z_rad) / 2);
	*((float*)(quat + 1))	= sin(x_rad / 2);
	*((float*)(quat + 2))	= sin(y_rad / 2);
	*((float*)(quat + 3))	= sin(z_rad / 2);
}
 
 
/**
 * Converts the quaternion position from one coordinate system to the other.
 * The quaternion rotation defines the transformation from one system to the other.
 *
 * The variable rotation won't be changed.
 * The result will be stored at *position.
 */
void quat_rotate(float *position, float *rotation)
{
	float temproation_inverse[4];
	temproation_inverse[0] = *((float*)(rotation    ));
	temproation_inverse[1] = *((float*)(rotation + 1));
	temproation_inverse[2] = *((float*)(rotation + 2));
	temproation_inverse[3] = *((float*)(rotation + 3));

	float temproation[4];
	temproation[0] = *((float*)(rotation    ));
	temproation[1] = *((float*)(rotation + 1));
	temproation[2] = *((float*)(rotation + 2));
	temproation[3] = *((float*)(rotation + 3));

	quat_normalize(temproation);
	quat_inverse(temproation_inverse);

	quat_multiply(temproation, position);
	
	*position 	= temproation[0];
	*(position + 1)	= temproation[1];
	*(position + 2)	= temproation[2];
	*(position + 3)	= temproation[3];

	quat_multiply(position, temproation_inverse);	
}

/**
 *
 */
void quat_normalize(float *quat)
{
	float norm = sqrt(pow( *((float*)(quat    )), 2)
			+ pow( *((float*)(quat + 1)), 2) 
			+ pow( *((float*)(quat + 2)), 2) 
			+ pow( *((float*)(quat + 3)), 2));

	*quat 		= *quat       / norm;
	*(quat + 1) 	= *(quat + 1) / norm;
	*(quat + 2) 	= *(quat + 2) / norm;
	*(quat + 3) 	= *(quat + 3) / norm;
}

/**
 *
 */
void quat_inverse(float *quat)
{
	quat_normalize(quat);

	*(quat + 1) = -*((float*)(quat + 1));
	*(quat + 2) = -*((float*)(quat + 2));
	*(quat + 3) = -*((float*)(quat + 3));
}

/**
 *
 */
void quat_multiply(float* lhs, float* rhs)
{
	float result[4];

	result[0] = (*(lhs + 0) * *(rhs + 0) - *(lhs + 1) * *(rhs + 1) 
		   - *(lhs + 2) * *(rhs + 2) - *(lhs + 3) * *(rhs + 3));

	result[1] = (*(lhs + 0) * *(rhs + 1) + *(lhs + 1) * *(rhs + 0) 
		   + *(lhs + 2) * *(rhs + 3) - *(lhs + 3) * *(rhs + 2));

	result[2] = (*(lhs + 0) * *(rhs + 2) - *(lhs + 1) * *(rhs + 3) 
		   + *(lhs + 2) * *(rhs + 0) + *(lhs + 3) * *(rhs + 1));

	result[3] = (*(lhs + 0) * *(rhs + 3) + *(lhs + 1) * *(rhs + 2) 
		   - *(lhs + 2) * *(rhs + 1) + *(lhs + 3) * *(rhs + 0));

	*lhs		= result[0];
	*(lhs + 1) 	= result[1];
	*(lhs + 2) 	= result[2];
	*(lhs + 3) 	= result[3];
}
