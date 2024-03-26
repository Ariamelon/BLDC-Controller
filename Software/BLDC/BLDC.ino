#define PWM_MAX_DUTY      248
#define PWM_MIN_DUTY      32
#define PWM_START_DUTY    64
#define DEBOUNCE_PERIOD   10

void pwm_duty_set(uint8_t duty);
void bldc_comm_state(void);
void bldc_comm_startup(void);
void bldc_comm_shutdown(void);

uint8_t bldc_state = 0, motor_speed;

void setup() {
  DDRB  |= 0b00001110; // Configure HI_A (PB3), HI_B (PB2) and HI_C (PB1) (high side) as outputs.
  PORTB &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
  PORTB |= 0b00110000; // Enable speed+ (PB5) and speed- (PB4) pullups.
  DDRC  |= 0b00000111; // Configure LI_A (PC2), LI_B (PC1) and LI_C (PC0) (low side) as outputs.
  PORTC &= 0b11111000; // Disable LI_A (PC2), LI_B (PC1) and LI_C (PC0).
  DDRD  |= 0b00011100; // Configure red LED (PD2), yellow LED (PD3) and green LED (PD4) as outputs.
  PORTD &= 0b11100111; // Disable yellow LED (PD3) and green LED (PD4).
  PORTD |= 0b00000100; // Enable red LED (PD2).

  TCCR1A = 0;          // Initialize timers 1 and 2.
  TCCR1B = 0b00000001; // Clock source set to clkIO / 1 (no prescaling).
  TCCR2A = 0;          // Allows for highest PWM frequency.
  TCCR2B = 0b00000001;

  ACSR   = 0b00010000; // Disable and clear ACI flag (analog comparator interrupt).
  ADCSRA = 0;          // Disable ADC.
  
  bldc_comm_shutdown();
}

void loop() {
  // Speed control via on-board buttons.
  while(!(PINB & (1 << PINB5))){
    if ((motor_speed < PWM_MAX_DUTY) && (motor_speed >= PWM_MIN_DUTY)){
      motor_speed++;
      pwm_duty_set(motor_speed);
    }
    else if (motor_speed < PWM_MIN_DUTY){
      bldc_comm_startup();
    }
    delay(10);
  }
  while(!(PINB & (1 << PINB4))){
    if (motor_speed > PWM_MIN_DUTY){
      motor_speed--;
      pwm_duty_set(motor_speed);
    }
    else if ((motor_speed <= PWM_MIN_DUTY) && (motor_speed > 0)){
      bldc_comm_shutdown();
    }
    delay(10);
  }
}

void pwm_duty_set(uint8_t duty){
  OCR1A = OCR1B = OCR2A = duty; // Set PWM duty cycle for all outputs.
}

void bldc_comm_startup(void){
  // Setup starting PWM with duty cycle = PWM_START_DUTY.
  motor_speed = PWM_START_DUTY;
  pwm_duty_set(motor_speed);

  bldc_state = 0;     // Enable BLDC commutation.

  // Initial open loop commutation to enable motor to start properly.
  uint16_t i = 5000;
  while(i > 100) {
    bldc_comm_state();
    delayMicroseconds(i);
    i -= 100;
  }

  ADCSRB = 0b01000000; // Enable analog comparator mux.
  ACSR   = 0b00001000; // Enable analog comparator interrupt and clear ACI flag (analog comparator interrupt).

  PORTD &= 0b11110111; // Disable yellow LED (PD3).
  PORTD |= 0b00010000; // Enable green LED (PD4).
}

void bldc_comm_shutdown(void){
  // Turn off PWM.
  motor_speed = 0;
  pwm_duty_set(motor_speed);

  PORTB &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
  PORTC &= 0b11111000; // Disable LI_A (PC2), LI_B (PC1) and LI_C (PC0).

  TCCR1A = 0b00000000; // Disable HI_C (0C1A) and HI_B (OC1B).
  TCCR2A = 0b10000001; // Enable HI_A (OC2A).

  ADCSRB = 0b00000000; // Disable analog comparator mux.
  ACSR   = 0b00010000; // Disable analog comparator interrupt and clear analog comparator interrupt flag.

  PORTD &= 0b11101111; // Disable green LED (PD4).
  PORTD |= 0b00001000; // Enable yellow LED (PD3).
}

void bldc_comm_state(void){ // BLDC motor commutation function.
  switch(bldc_state){
    case 0:
      PORTC &= 0b11111010; // Disable LI_A (PC2) and LI_C (PC0).
      PORTC |= 0b00000010; // Enable LI_B (PC1).

      TCCR1A = 0;          // Disable HI_C (0C1A) and HI_B (OC1B).
      TCCR2A = 0b10000001; // Enable HI_A (OC2A) in PWM, phase correct, 8-bit mode.

      ADMUX  = 0b00000101; // Select ADC5 (PC5) as comparator negative input.
      ACSR  |= 0b00000011; // Set interrupt on rising edge and enable comparator interrupt.
      break;
    case 1:
      PORTC &= 0b11111001; // Disable LI_A (PC2) and LI_B (PC1).
      PORTC |= 0b00000001; // Enable LI_C (PC0).

      TCCR1A = 0;          // Disable HI_C (0C1A) and HI_B (OC1B).
      TCCR2A = 0b10000001; // Enable HI_A (OC2A) in PWM, phase correct, 8-bit mode.

      ADMUX  = 0b00000100; // Select ADC4 (PC4) as comparator negative input.
      ACSR  &= 0b11111110; // Set interrupt on falling edge.
      break;
    case 2:
      PORTC &= 0b11111001; // Disable LI_A (PC2) and LI_B (PC1).
      PORTC |= 0b00000001; // Enable LI_C (PC0).

      TCCR1A = 0b00100001; // Enable HI_B (OC1B) in PWM, phase correct, 8-bit mode and disable HI_C (0C1A).
      TCCR2A = 0;          // Disable HI_A (OC2A).

      ADMUX  = 0b00000101; // Select ADC5 (PC5) as comparator negative input.
      ACSR  |= 0b00000011; // Set interrupt on rising edge.
      break;
    case 3:
      PORTC &= 0b11111100; // Disable LI_B (PC1) and LI_C (PC0).
      PORTC |= 0b00000100; // Enable LI_A (PC2).

      TCCR1A = 0b00100001; // Enable HI_B (OC1B) in PWM, phase correct, 8-bit mode and disable HI_C (0C1A).
      TCCR2A = 0;          // Disable HI_A (OC2A).

      ADMUX  = 0b00000011; // Select ADC3 (PC3) as comparator negative input.
      ACSR  &= 0b11111110; // Set interrupt on falling edge.
      break;
    case 4:
      PORTC &= 0b11111100; // Disable LI_B (PC1) and LI_C (PC0).
      PORTC |= 0b00000100; // Enable LI_A (PC2).

      TCCR1A = 0b10000001; // Enable HI_C (0C1A) in PWM, phase correct, 8-bit mode and disable HI_B (OC1B).
      TCCR2A = 0;          // Disable HI_A (OC2A).

      ADMUX  = 0b00000100; // Select ADC4 (PC4) as comparator negative input.
      ACSR  |= 0b00000011; // Set interrupt on rising edge.
      break;
    case 5:
      PORTC &= 0b11111010; // Disable LI_A (PC2) and LI_C (PC0).
      PORTC |= 0b00000010; // Enable LI_B (PC1).

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
  uint8_t i;
  // BEMF debounce, reads values DEBOUNCE_PERIOD times before processing result.
  for(i = 0; i < DEBOUNCE_PERIOD; i++) {
    if(bldc_state & 1){         // If BLDC state is odd (001, 011 or 101).
      if(!(ACSR & 0b00100000)){ // If 0 detected on ACO (comparator output), delay debouncing.
        i -= 1;
      }
    }
    else{                       // If BLDC state is even (000, 010 or 100).
      if((ACSR & 0b00100000)){  // If 1 detected on ACO (comparator output), delay debouncing.
        i -= 1;
      }
    }
  }

  bldc_comm_state();
}