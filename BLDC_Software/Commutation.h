#define PWM_MAX_DUTY   255
#define PWM_MIN_DUTY   48
#define PWM_START_DUTY 64

void initialize_controller(void);

void motor_start(unsigned char);
void motor_stop(void);

unsigned char open_loop_startup(unsigned char);

unsigned char commutation_step_set(unsigned char);
unsigned char motor_speed_set(unsigned char);
void phase_output_set(unsigned char);
void pwm_duty_set(unsigned char);