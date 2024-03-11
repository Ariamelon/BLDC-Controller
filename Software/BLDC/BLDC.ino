#define PWM_MAX_DUTY      254
#define PWM_MIN_DUTY      32
#define PWM_START_DUTY    50
#define DEBOUNCE_PERIOD   5
#define BTN_SPEED_UP      PD3
#define BTN_SPEED_DOWN    PD2
#define SERIAL_DEBUG      FALSE

void pwm_duty_set(uint8_t duty);
void bldc_comm();
 
uint8_t bldc_state = 0, motor_speed;
uint16_t i;
bool motor_direction = 1;

void setup() {
  // If serial debug is enabled, start serial comms and initialize timer 0.
  #if SERIAL_DEBUG
  Serial.begin(9600);
  TCCR0A = 0;
  TCCR0B = 0b00000010; // Set clock source to clkIO / 2 (62.5 * 2).
  #endif

  // Enable input with pullups for the speed control buttons.
  pinMode(BTN_SPEED_UP, INPUT_PULLUP);
  pinMode(BTN_SPEED_DOWN, INPUT_PULLUP);

  DDRB  |= 0b00001110; // Configure HI_A (PB3), HI_B (PB2) and HI_C (PB1) (high side) as outputs.
  PORTB &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
  DDRC  |= 0b00000111; // Configure LI_A (PC2), LI_B (PC1) and LI_C (PC0) (low side) as outputs.
  PORTC &= 0b11111000; // Disable LI_A (PC2), LI_B (PC1) and LI_C (PC0).

  // Initialize timers 1 and 2.
  TCCR1A = 0;
  TCCR2A = 0;

  // Set clock source to clkIO / 1 (no prescaling) for maximum PWM frequency.
  TCCR1B = 0b00000001;
  TCCR2B = 0b00000001;

  ACSR   = 0b00010000; // Disable and clear ACI flag (analog comparator interrupt).

  // Setup starting PWM with duty cycle = PWM_START_DUTY.
  motor_speed = PWM_START_DUTY;
  pwm_duty_set(motor_speed);
  
  // Initial open loop commutation to enable motor to start properly.
  i = 5000;
  while(i > 100) {
    delayMicroseconds(i);
    bldc_comm();
    i = i - 20;
  }

  ADCSRA = 0;          // Disable ADC.
  ADCSRB = 0b01000000; // Enable analog comparator mux.
  ACSR  |= 0b00001000; // Enable analog comparator interrupt.
}

void loop() {
  // Speed control via on-board buttons.
  while(!(digitalRead(BTN_SPEED_UP)) && motor_speed < PWM_MAX_DUTY){
    motor_speed++;
    pwm_duty_set(motor_speed);
    delay(10);
  }
  while(!(digitalRead(BTN_SPEED_DOWN)) && motor_speed > PWM_MIN_DUTY){
    motor_speed--;
    pwm_duty_set(motor_speed);
    delay(10);
  }
}

void pwm_duty_set(uint8_t duty){
  // Constrain duty cycle to max and min values
  if(duty < PWM_MIN_DUTY){
    duty  = PWM_MIN_DUTY;
  }
  if(duty > PWM_MAX_DUTY){
    duty  = PWM_MAX_DUTY;
  }
  OCR1A = OCR1B = OCR2A = duty; // Set PWM duty cycle for all outputs.
}

void bldc_comm(){ // BLDC motor commutation function.
  // If serial debug is enabled, show current commutation state and print time spend on last state.
  #if SERIAL_DEBUG
  Serial.print("Current commutation state: ");
  Serial.print(bldc_state);
  Serial.print(". Time taken for last state: ");
  Serial.println(TCNT0);
  TCNT0 = 0;
  #endif

  switch(bldc_state){
    case 0:
      PORTC &= 0b11111010; // Disable LI_A (PC2) and LI_C (PC0).
      PORTB &= 0b11111001; // Disable HI_B (PB2) and HI_C (PB1).
      PORTC |= 0b00000010; // Enable LI_B (PC1).
      PORTB |= 0b00001000; // Enable HI_A (PB3).
      TCCR1A = 0;          // Disable HI_C (0C1A) and HI_B (OC1B).
      TCCR2A = 0b10000001; // Enable HI_A (OC2A) in PWM, phase correct, 8-bit mode.
      ADMUX  = 0b00000101; // Select ADC5 (PC5) as comparator negative input.
      ACSR  |= 0b00000011; // Set interrupt on rising edge.
      break;
    case 1:
      PORTC &= 0b11111001; // Disable LI_A (PC2) and LI_B (PC1).
      PORTB &= 0b11111001; // Disable HI_B (PB2) and HI_C (PB1).
      PORTC |= 0b00000001; // Enable LI_C (PC0).
      PORTB |= 0b00001000; // Enable HI_A (PB3).
      TCCR1A = 0;          // Disable HI_C (0C1A) and HI_B (OC1B).
      TCCR2A = 0b10000001; // Enable HI_A (OC2A) in PWM, phase correct, 8-bit mode.
      ADMUX  = 0b00000100; // Select ADC4 (PC4) as comparator negative input.
      ACSR  &= 0b11111110; // Set interrupt on falling edge.
      break;
    case 2:
      PORTC &= 0b11111001; // Disable LI_A (PC2) and LI_B (PC1).
      PORTB &= 0b11110101; // Disable HI_A (PB3) and HI_C (PB1).
      PORTC |= 0b00000001; // Enable LI_C (PC0).
      PORTB |= 0b00000100; // Enable HI_B (PB2).
      TCCR1A = 0b00100001; // Enable HI_B (OC1B) in PWM, phase correct, 8-bit mode and disable HI_C (0C1A).
      TCCR2A = 0;          // Disable HI_A (OC2A).
      ADMUX  = 0b00000101; // Select ADC5 (PC5) as comparator negative input.
      ACSR  |= 0b00000011; // Set interrupt on rising edge.
      break;
    case 3:
      PORTC &= 0b11111100; // Disable LI_B (PC1) and LI_C (PC0).
      PORTB &= 0b11110101; // Disable HI_A (PB3) and HI_C (PB1).
      PORTC |= 0b00000100; // Enable LI_A (PC2).
      PORTB |= 0b00000100; // Enable HI_B (PB2).
      TCCR1A = 0b00100001; // Enable HI_B (OC1B) in PWM, phase correct, 8-bit mode and disable HI_C (0C1A).
      TCCR2A = 0;          // Disable HI_A (OC2A).
      ADMUX  = 0b00000011; // Select ADC3 (PC3) as comparator negative input.
      ACSR  &= 0b11111110; // Set interrupt on falling edge.
      break;
    case 4:
      PORTC &= 0b11111100; // Disable LI_B (PC1) and LI_C (PC0).
      PORTB &= 0b11111001; // Disable HI_A (PB3) and HI_B (PB2).
      PORTC |= 0b00000100; // Enable LI_A (PC2).
      PORTB |= 0b00000010; // Enable HI_C (PB1).
      TCCR1A = 0b10000001; // Enable HI_C (0C1A) in PWM, phase correct, 8-bit mode and disable HI_B (OC1B).
      TCCR2A = 0;          // Disable HI_A (OC2A).
      ADMUX  = 0b00000100; // Select ADC4 (PC4) as comparator negative input.
      ACSR  |= 0b00000011; // Set interrupt on rising edge.
      break;
    case 5:
      PORTC &= 0b11111010; // Disable LI_A (PC2) and LI_C (PC0).
      PORTB &= 0b11111001; // Disable HI_A (PB3) and HI_B (PB2).
      PORTC |= 0b00000010; // Enable LI_B (PC1).
      PORTB |= 0b00000010; // Enable HI_C (PB1).
      TCCR1A = 0b10000001; // Enable HI_C (0C1A) in PWM, phase correct, 8-bit mode and disable HI_B (OC1B).
      TCCR2A = 0;          // Disable HI_A (OC2A).
      ADMUX  = 0b00000101; // Select ADC5 (PC5) as comparator negative input.
      ACSR  &= 0b11111110; // Set interrupt on falling edge.
      break;
  }
  bldc_state ++; // Advance commutation state by one.
  bldc_state %= 6;
}

// Analog comparator ISR
ISR (ANALOG_COMP_vect) {
  // BEMF debounce, reads values DEBOUNCE_PERIOD times before processing result.
  for(i = 0; i < DEBOUNCE_PERIOD; i++) {
    if(bldc_state & 1){                // If BLDC state is odd (001, 011 or 101).
      if(!(ACSR & 0b00100000)) i -= 1; // If 0 detected on ACO (comparator output), delay debouncing.
    }
    else{                              // If BLDC state is even (000, 010 or 100).
      if((ACSR & 0b00100000)) i -= 1;  // If 1 detected on ACO (comparator output), delay debouncing.
    }
  }
  
  #if SERIAL_DEBUG
  Serial.print("Interrupt detected. ");
  #endif

  bldc_comm();
}