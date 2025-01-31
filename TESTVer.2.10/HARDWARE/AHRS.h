#ifndef _AHRS_H
#define _AHRS_H
#include <math.h>
#include <stdio.h>
#include "sys.h"


/* Private define------------------------------------------------------------*/
#define Kp 2.0f                       // proportional gain governs rate of convergence toaccelerometer/magnetometer
	 //Kp�������� �����˼��ٶȼƺʹ����Ƶ������ٶ�
#define Ki 0.005f          // integral gain governs rate of convergenceof gyroscope biases
		//Ki�������� ������������ƫ��������ٶ�
#define halfT 0.0025f      // half the sample period  
		//halfT�������ڵ�һ��
#define dt 0.005f		

#endif
