#define ENCODER_FR_PIN 21
#define ENCODER_FL_PIN 22
#define COUNTS_PER_REV 20

#define MAX_RPM 400
#define MOTOR_MAX_RPM 200
#define MAX_RPM_RATIO 0.85
#define MOTOR_OPERATING_VOLTAGE 9
#define MOTOR_POWER_MAX_VOLTAGE 9
#define MOTOR_POWER_MEASURED_VOLTAGE 9                      
#define WHEEL_DIAMETER 0.0652              
#define LR_WHEELS_DISTANCE 0.145 
#define FR_WHEELS_DISTANCE 0.115

#define PWM_MAX 100
#define PWM_MIN -100

#define K_P 50                             // P constant
#define K_I 40                             // I constant
#define K_D 20                             // D constant

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define RPM_TO_RPS 1/60
#define PI 3.14159
