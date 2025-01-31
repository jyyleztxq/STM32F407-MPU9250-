   /*
    *AHRS
   */
   // AHRS.c
   // Quaternion implementation of the 'DCM filter' [Mayhony et al]. Incorporates the magnetic distortion
   // compensation algorithms from my filter [Madgwick] which eliminatesthe need for a reference
   // direction of flux (bx bz) to be predefined and limits the effect ofmagnetic distortions to yaw
   // axis only.
   // User must define 'halfT' as the (sample period / 2), and the filtergains 'Kp' and 'Ki'.
   // Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elementsrepresenting the estimated
   // orientation.  See my report foran overview of the use of quaternions in this application.
   // User must call 'AHRSupdate()' every sample period and parsecalibrated gyroscope ('gx', 'gy', 'gz'),
   // accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz')data.  Gyroscope units are
   // radians/second, accelerometer and magnetometer units are irrelevantas the vector is normalised.                                

   #include "AHRS.h"


   #define ACCEL_1G 1000    //theacceleration of gravity is: 1000 mg
	 //重力加速度为1000mg
/* Private variables---------------------------------------------------------*/
   static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;        // quaternion elements representing theestimated orientation
	 //四元数
   static float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
	 //标量的积分误差
/* Public variables----------------------------------------------------------*/
   EulerAngle_Type EulerAngle;       //unit: radian
	 //欧拉角 单位弧度
    u8InitEulerAngle_Finished = 0;		
		//用于表示欧拉角初始化完毕
   float Magnetoresistor_mGauss_X = 0, Magnetoresistor_mGauss_Y = 0,Magnetoresistor_mGauss_Z = 0;//unit: milli-Gauss    
	 //磁力计 单位毫高斯
   float Accelerate_mg_X, Accelerate_mg_Y, Accelerate_mg_Z;//unit: mg     
		//加速度计 单位毫克
   float AngularRate_dps_X, AngularRate_dps_Y, AngularRate_dps_Z;//unit:dps: degree per second    
		//角速度 单位度每秒
   int16_t Magnetoresistor_X, Magnetoresistor_Y, Magnetoresistor_Z; 
		//磁阻 （原数据）
   uint16_t Accelerate_X = 0, Accelerate_Y = 0, Accelerate_Z = 0;    
		//加速度计（原数据）
   uint16_t AngularRate_X = 0, AngularRate_Y = 0, AngularRate_Z = 0;
	 //陀螺仪 （原数据）
   u8 Quaternion_Calibration_ok = 0;
	 //四元数校准完成标志位
   /* Private macro-------------------------------------------------------------*/
   /* Private typedef-----------------------------------------------------------*/
   /* Private function prototypes -----------------------------------------------*/
/******************************************************************************
    *Function Name  : AHRSupdate
    *Description    : None
    *Input          : None
    *Output         : None
    *Return         : None
*******************************************************************************/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float *roll,float *pitch,float *yaw)
{
           float norm;									//用于单位化
           float hx, hy, hz, bx, bz;		//
           float vx, vy, vz, wx, wy, wz; 
           float ex, ey, ez;
 
           // auxiliary variables to reduce number of repeated operations  辅助变量减少重复操作次数
           float q0q0 = q0*q0;
           float q0q1 = q0*q1;
           float q0q2 = q0*q2;
           float q0q3 = q0*q3;
           float q1q1 = q1*q1;
           float q1q2 = q1*q2;
           float q1q3 = q1*q3;
           float q2q2 = q2*q2;
           float q2q3 = q2*q3;
           float q3q3 = q3*q3;
          
           // normalise the measurements  对加速度计和磁力计数据进行规范化
           norm = sqrt(ax*ax + ay*ay + az*az);
           ax = ax / norm;
           ay = ay / norm;
           az = az / norm;
           norm = sqrt(mx*mx + my*my + mz*mz);
           mx = mx / norm;
           my = my / norm;
           mz = mz / norm;
           
           // compute reference direction of magnetic field  计算磁场的参考方向
					 //hx,hy,hz是mx,my,mz在参考坐标系的表示
           hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
           hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
           hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 -q2q2);    
						//bx,by,bz是地球磁场在参考坐标系的表示
           bx = sqrt((hx*hx) + (hy*hy));
           bz = hz;
          
// estimated direction of gravity and magnetic field (v and w)  //估计重力和磁场的方向
//vx,vy,vz是重力加速度在物体坐标系的表示
           vx = 2*(q1q3 - q0q2);
           vy = 2*(q0q1 + q2q3);
           vz = q0q0 - q1q1 - q2q2 + q3q3;
					 //wx,wy,wz是地磁场在物体坐标系的表示
           wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
           wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
           wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2); 
          
// error is sum ofcross product between reference direction of fields and directionmeasured by sensors 
//ex,ey,ez是加速度计与磁力计测量出的方向与实际重力加速度与地磁场方向的误差，误差用叉积来表示，且加速度计与磁力计的权重是一样的
           ex = (ay*vz - az*vy) + (my*wz - mz*wy);
           ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
           ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
          
           // integral error scaled integral gain
					 //积分误差
           exInt = exInt + ex*Ki* (1.0f / sampleFreq);
           eyInt = eyInt + ey*Ki* (1.0f / sampleFreq);
           ezInt = ezInt + ez*Ki* (1.0f / sampleFreq);
           // adjusted gyroscope measurements
					 //PI调节陀螺仪数据
           gx = gx + Kp*ex + exInt;
           gy = gy + Kp*ey + eyInt;
           gz = gz + Kp*ez + ezInt;
          
           // integrate quaernion rate aafnd normalaizle
					 //欧拉法解微分方程
//           q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//           q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//           q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//           q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT; 
//RUNGE_KUTTA 法解微分方程
					  k10=0.5 * (-gx*q1 - gy*q2 - gz*q3);
						k11=0.5 * ( gx*q0 + gz*q2 - gy*q3);
						k12=0.5 * ( gy*q0 - gz*q1 + gx*q3);
						k13=0.5 * ( gz*q0 + gy*q1 - gx*q2);
						
						k20=0.5 * (halfT*(q0+halfT*k10) + (halfT-gx)*(q1+halfT*k11) + (halfT-gy)*(q2+halfT*k12) + (halfT-gz)*(q3+halfT*k13));
						k21=0.5 * ((halfT+gx)*(q0+halfT*k10) + halfT*(q1+halfT*k11) + (halfT+gz)*(q2+halfT*k12) + (halfT-gy)*(q3+halfT*k13));
						k22=0.5 * ((halfT+gy)*(q0+halfT*k10) + (halfT-gz)*(q1+halfT*k11) + halfT*(q2+halfT*k12) + (halfT+gx)*(q3+halfT*k13));
						k23=0.5 * ((halfT+gz)*(q0+halfT*k10) + (halfT+gy)*(q1+halfT*k11) + (halfT-gx)*(q2+halfT*k12) + halfT*(q3+halfT*k13));
						
						k30=0.5 * (halfT*(q0+halfT*k20) + (halfT-gx)*(q1+halfT*k21) + (halfT-gy)*(q2+halfT*k22) + (halfT-gz)*(q3+halfT*k23));
						k31=0.5 * ((halfT+gx)*(q0+halfT*k20) + halfT*(q1+halfT*k21) + (halfT+gz)*(q2+halfT*k22) + (halfT-gy)*(q3+halfT*k23));
						k32=0.5 * ((halfT+gy)*(q0+halfT*k20) + (halfT-gz)*(q1+halfT*k21) + halfT*(q2+halfT*k22) + (halfT+gx)*(q3+halfT*k23));
						k33=0.5 * ((halfT+gz)*(q0+halfT*k20) + (halfT+gy)*(q1+halfT*k21) + (halfT-gx)*(q2+halfT*k22) + halfT*(q3+halfT*k23));
						
						k40=0.5 * (dt*(q0+dt*k30) + (dt-gx)*(q1+dt*k31) + (dt-gy)*(q2+dt*k32) + (dt-gz)*(q3+dt*k33));
						k41=0.5 * ((dt+gx)*(q0+dt*k30) + dt*(q1+dt*k31) + (dt+gz)*(q2+dt*k32) + (dt-gy)*(q3+dt*k33));
						k42=0.5 * ((dt+gy)*(q0+dt*k30) + (dt-gz)*(q1+dt*k31) + dt*(q2+dt*k32) + (dt+gx)*(q3+dt*k33));
						k43=0.5 * ((dt+gz)*(q0+dt*k30) + (dt+gy)*(q1+dt*k31) + (dt-gx)*(q2+dt*k32) + dt*(q3+dt*k33));	
						
						q0=q0 + dt/6.0 * (k10+2*k20+2*k30+k40);
						q1=q1 + dt/6.0 * (k11+2*k21+2*k31+k41);
						q2=q2 + dt/6.0 * (k12+2*k22+2*k32+k42);
						q3=q3 + dt/6.0 * (k13+2*k23+2*k33+k43);
						
           // normalise quaternion
           norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
           q0 = q0 / norm;
           q1 = q1 / norm;
           q2 = q2 / norm;
           q3 = q3 / norm;
					 
					 *pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
					 *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
					 *yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
}