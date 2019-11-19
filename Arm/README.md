This folder includes codes of theia II's arm

HOW TO CONTROL ARM MANUALLY
- Run 'arm_m.launch' file in tx2 via ssh. It runs 
  * j0_manuel.ino at 76800 baud
  * j1_manuel.ino at 76800 baud
  * j2_manuel.ino at 76800 baud
  * step_cont.ino at 9600 vaud
  * dxl.py

- Enter terminal 'rosrun joy joy_node' to activate joystick on dell.
- Run 'xboxs_m.py' file in dell to publish degrees of joints on dell.
- Run 'dxl_m_control.py' to control gripper on dell.
