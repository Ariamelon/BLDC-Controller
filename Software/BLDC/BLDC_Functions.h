void motor_phase_state(void);
void motor_start(void);
void motor_stop(void);
void pwm_duty_set(uint8_t duty);

void High_Phase_A(void);
void High_Phase_B(void);
void High_Phase_C(void);

void Low_Phase_A(void);
void Low_Phase_B(void);
void Low_Phase_C(void);

void 

extern volatile uint8_t commutation_step;
extern uint8_t motor_speed;