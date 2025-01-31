#ifndef _AHRS_H
#define _AHRS_H
#include <math.h>
#include <stdio.h>
#include "sys.h"


/* Private define------------------------------------------------------------*/
#define Kp 2.0f                       // proportional gain governs rate of convergence toaccelerometer/magnetometer
	 //Kp比例增益 决定了加速度计和磁力计的收敛速度
#define Ki 0.005f          // integral gain governs rate of convergenceof gyroscope biases
		//Ki积分增益 决定了陀螺仪偏差的收敛速度
#define halfT 0.0025f      // half the sample period  
		//halfT采样周期的一半
#define dt 0.005f		

#endif
