#ifndef __TASK_CANMOTOR_H__
#define __TASK_CANMOTOR_H__

#include "CAN_receive.h"

//chassis 3508 max motor control current
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 300.0f

//m3508 rmp change to chassis speed,
//m3508ת���ɵ����ٶ�(m/s)�ı�����
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//chassis motor speed PID
//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 150.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 40.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

/*typedef struct
{
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��, the point to remote control
  const gimbal_motor_t *chassis_yaw_motor;   //will use the relative angle of yaw gimbal motor to calculate the euler angle.����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����.
  const gimbal_motor_t *chassis_pitch_motor; //will use the relative angle of pitch gimbal motor to calculate the euler angle.����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;               //state machine. ���̿���״̬��
  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��
  chassis_motor_t motor_chassis[4];          //chassis motor data.���̵������
  pid_type_def motor_speed_pid[4];             //motor speed PID.���̵���ٶ�pid
  pid_type_def chassis_angle_pid;              //follow angle PID.���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ

  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. �����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.��������̨����ԽǶȣ���λ rad
  fp32 chassis_relative_angle_set;  //the set relative angle.���������̨���ƽǶ�
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  //max forward speed, unit m/s.ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.���˷�������ٶ� ��λm/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.��������ٶ� ��λm/s
  fp32 vy_min_speed;  //max right speed, unit m/s.�ҷ�������ٶ� ��λm/s
  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�yaw�Ƕ�
  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�pitch�Ƕ�
  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�roll�Ƕ�

} chassis_move_t;*/

extern void can_filter_init(void);
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern int16_t getSpeedpoint(void);

#endif
