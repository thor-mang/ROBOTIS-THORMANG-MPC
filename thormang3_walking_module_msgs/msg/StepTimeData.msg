#### StepTimeData

## WalkingStateFlag
int16   walking_state

int16 InWalkingStarting=0 #WalkingStarting
int16 InWalking= 1        #Walking
int16 InWalkingEnding=2   #WalkingEnding

## Unit [sec]
float32 abs_step_time #global time

# dsp_ratio >= 0.0 dsp_ratio < 1.0, recommand value is 0.2 
float32 dsp_ratio

# 0.0 <= start_time_delay_ratio    < 1.0
# 0.0 <= finish_time_advance_ratio < 1.0
# 0.0 <= start_time_delay_ratio +  finish_time_advance_ratio < 1.0
# default value is 0.0
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