#define VOLTAGE_IN        9

#define PWM_MAX_DUTY      255
#define PWM_START_DUTY    30

#define ALIGN_MS          100
#define OPEN_LOOP_DELAY   5000

#define DEBOUNCE_PERIOD   10

void initialize_controller(void);

unsigned char motor_start(unsigned char, unsigned char, unsigned char );
void motor_stop(void);

unsigned char commutation_step_set(unsigned char);
unsigned char motor_speed_set(unsigned char);
void pwm_duty_set(unsigned char);