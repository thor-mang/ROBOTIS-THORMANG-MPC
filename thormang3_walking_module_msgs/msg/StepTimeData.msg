#### StepTimeData
## WalkingStateFlag
# InWalkingStarting	= 0;
# InWalking 		= 1;
# InWalkingEnding	= 2;
int16   walking_state

## Unit [sec]
float32 abs_step_time

# dsp_ratio >= 0.0 dsp_ratio < 1.0, recommand value is 0.2 
float32 dsp_ratio

float32 start_time_delay_ratio_x
float32 start_time_delay_ratio_y
float32 start_time_delay_ratio_z
float32 start_time_delay_ratio_roll
float32 start_time_delay_ratio_pitch
float32 start_time_delay_ratio_yaw
float32 finish_time_advance_ratio_x
float32 finish_time_advance_ratio_y
float32 finish_time_advance_ratio_z
float32 finish_time_advance_ratio_roll
float32 finish_time_advance_ratio_pitch
float32 finish_time_advance_ratio_yaw