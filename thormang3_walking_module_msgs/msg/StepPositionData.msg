#### Step Position Data

## MovingFootFlag
int16 moving_foot

int16 LEFT_FOOT_SWING  = 1 # Left foot constant
int16 RIGHT_FOOT_SWING = 2 # Right foot constant
int16 STANDING         = 3 # No foot constant

## Unit [meter, rad]
float32     foot_z_swap
float32     body_z_swap
float32     torso_yaw_angle_rad
PoseXYZRPY  left_foot_pose
PoseXYZRPY  right_foot_pose
PoseZRPY    body_pose