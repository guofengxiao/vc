vcMotionTarget
"vcMotionTarget provides properties and methods for defining how a robot moves. Such movements may include joint and tool behavior as well as speed and configuration settings for orientation solutions��see Articulated Kinematics for more information."

�ܽ᣺
01. vcMotionTargetû�м��ٶ����ѡ��
��������������˶�
�˶����������ٶȺ�״̬�趨�Ĺؽ���Ϊ�͹�����Ϊ

����1��vcRobotController.move()ʱ�����������Teachʱ�䲻һ��
����2����statement��ȡ��vcMotionTarget ����move���ˡ�



01. createTarget ������
  vcRobotController.createTarget()
  vcRobotController.createTarget( vcMatrix )
    ����: ��Ե�ǰ������base������λ������
    
 02. ��Ϊ����
   a. addTarget
     vcRobotController.addTarget( vcMotionTarget )
     vcRobotController.addTarget( vcMatrix )