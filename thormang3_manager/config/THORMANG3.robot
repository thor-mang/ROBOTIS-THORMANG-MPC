[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0 | 3000000  | r_arm_sh_p1
/dev/ttyUSB1 | 3000000  | l_arm_sh_p1
/dev/ttyUSB2 | 3000000  | r_leg_hip_y
/dev/ttyUSB3 | 3000000  | l_leg_hip_y

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME     | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1   | H54-200-S500-R | 2.0      | r_arm_sh_p1  | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 2   | H54-200-S500-R | 2.0      | l_arm_sh_p1  | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 3   | H54-100-S500-R | 2.0      | r_arm_sh_r   | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 4   | H54-100-S500-R | 2.0      | l_arm_sh_r   | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 5   | H54-100-S500-R | 2.0      | r_arm_sh_p2  | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 6   | H54-100-S500-R | 2.0      | l_arm_sh_p2  | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 7   | H54-100-S500-R | 2.0      | r_arm_el_y   | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 8   | H54-100-S500-R | 2.0      | l_arm_el_y   | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 9   | H42-20-S300-R  | 2.0      | r_arm_wr_r   | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 10  | H42-20-S300-R  | 2.0      | l_arm_wr_r   | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 11  | H42-20-S300-R  | 2.0      | r_arm_wr_y   | present_position, present_voltage, external_port_data_1, external_port_data_2
dynamixel | /dev/ttyUSB1 | 12  | H42-20-S300-R  | 2.0      | l_arm_wr_y   | present_position, present_voltage, external_port_data_1, external_port_data_2
dynamixel | /dev/ttyUSB0 | 13  | H42-20-S300-R  | 2.0      | r_arm_wr_p   | present_position, present_voltage, external_port_data_1, external_port_data_2, external_port_data_3, external_port_data_4
dynamixel | /dev/ttyUSB1 | 14  | H42-20-S300-R  | 2.0      | l_arm_wr_p   | present_position, present_voltage, external_port_data_1, external_port_data_2, external_port_data_3, external_port_data_4
dynamixel | /dev/ttyUSB2 | 15  | H54-100-S500-R | 2.0      | r_leg_hip_y  | present_position, present_voltage
dynamixel | /dev/ttyUSB3 | 16  | H54-100-S500-R | 2.0      | l_leg_hip_y  | present_position, present_voltage
dynamixel | /dev/ttyUSB2 | 17  | H54-200-S500-R | 2.0      | r_leg_hip_r  | present_position, present_voltage
dynamixel | /dev/ttyUSB3 | 18  | H54-200-S500-R | 2.0      | l_leg_hip_r  | present_position, present_voltage
dynamixel | /dev/ttyUSB2 | 19  | H54-200-B500-R | 2.0      | r_leg_hip_p  | present_position, present_voltage
dynamixel | /dev/ttyUSB3 | 20  | H54-200-B500-R | 2.0      | l_leg_hip_p  | present_position, present_voltage
dynamixel | /dev/ttyUSB2 | 21  | H54-200-S500-R | 2.0      | r_leg_kn_p   | present_position, present_voltage
dynamixel | /dev/ttyUSB3 | 22  | H54-200-S500-R | 2.0      | l_leg_kn_p   | present_position, present_voltage
dynamixel | /dev/ttyUSB2 | 23  | H54-200-B500-R | 2.0      | r_leg_an_p   | present_position, present_voltage, external_port_data_1, external_port_data_2
dynamixel | /dev/ttyUSB3 | 24  | H54-200-B500-R | 2.0      | l_leg_an_p   | present_position, present_voltage, external_port_data_1, external_port_data_2
dynamixel | /dev/ttyUSB2 | 25  | H54-200-S500-R | 2.0      | r_leg_an_r   | present_position, present_voltage, external_port_data_1, external_port_data_2, external_port_data_3, external_port_data_4
dynamixel | /dev/ttyUSB3 | 26  | H54-200-S500-R | 2.0      | l_leg_an_r   | present_position, present_voltage, external_port_data_1, external_port_data_2, external_port_data_3, external_port_data_4
dynamixel | /dev/ttyUSB0 | 27  | H54-100-S500-R | 2.0      | torso_y      | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 28  | H42-20-S300-R  | 2.0      | head_y       | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 29  | H42-20-S300-R  | 2.0      | head_p       | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 30  | MX-106         | 2.0      | l_f0_j0      | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 31  | MX-106         | 2.0      | r_f0_j0      | present_position, present_voltage
dynamixel | /dev/ttyUSB1 | 32  | MX-106         | 2.0      | l_f1_j0      | present_position, present_voltage
dynamixel | /dev/ttyUSB0 | 33  | MX-106         | 2.0      | r_f1_j0      | present_position, present_voltage
