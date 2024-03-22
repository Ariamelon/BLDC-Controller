#define PWM_MAX_DUTY      254
#define PWM_MIN_DUTY      32
#define PWM_START_DUTY    100
#define DEBOUNCE_PERIOD   5

void pwm_duty_set(uint8_t duty);
void bldc_comm_state(void);
void bldc_comm_startup(void);
void bldc_comm_shutdown(void);
 
uint8_t bldc_state = 0, motor_speed;
uint16_t i;
bool motor_direction = 1;

void setup() {
  Serial.begin(9600);

  DDRB  |= 0b00001110; // Configure HI_A (PB3), HI_B (PB2) and HI_C (PB1) (high side) as outputs.
  PORTB |= 0b00110000; // Enable speed+ (PB5) and speed- (PB4) pullups.
  PORTB &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
  DDRC  |= 0b00000111; // Configure LI_A (PC2), LI_B (PC1) and LI_C (PC0) (low side) as outputs.
  PORTC &= 0b11111000; // Disable LI_A (PC2), LI_B (PC1) and LI_C (PC0).
  DDRD  |= 0b00011100; // Configure red LED (PD2), yellow LED (PD3) and green LED (PD4) as outputs.
  PORTD &= 0b11100011; // Disable red LED (PD2), yellow LED (PD3) and green LED (PD4).
  PORTD |= 0b00000100; // Enable red LED (PD2).

  // Initialize timers 1 and 2.
  TCCR1A = 0;
  TCCR2A = 0;

  // Set clock source to clkIO / 1 (no prescaling) for maximum PWM frequency.
  TCCR1B = 0b00000001;
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
      Serial.println("Starting up motor.");
      bldc_comm_startup();
      Serial.println("Start-up complete.");
    }
    Serial.print("Motor speed: ");
    Serial.println(motor_speed);
    delay(100);
  }
  while(!(PINB & (1 << PINB4))){
    if (motor_speed > PWM_MIN_DUTY){
      motor_speed--;
      pwm_duty_set(motor_speed);
    }
    else if ((motor_speed <= PWM_MIN_DUTY) && (motor_speed > 0)){
      Serial.println("Shutting down motor.");
      bldc_comm_shutdown();
    }
    Serial.print("Motor speed: ");
    Serial.println(motor_speed);
    delay(100);
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

void bldc_comm_startup(void){
  // Setup starting PWM with duty cycle = PWM_START_DUTY.
  motor_speed = PWM_START_DUTY;
  pwm_duty_set(motor_speed);

  bldc_state = 0;     // Enable BLDC commutation.

  // Initial open loop commutation to enable motor to start properly.
  i = 5000;
  while(i > 100) {
    bldc_comm_state();
    delayMicroseconds(i);
    i = i - 20;
  }

  ADCSRB = 0b01000000; // Enable analog comparator mux.
  ACSR  |= 0b00001000; // Enable analog comparator interrupt.

  PORTD &= 0b11110111; // Disable yellow LED (PD3).
  PORTD |= 0b00010000; // Enable green LED (PD4).
}

void bldc_comm_shutdown(void){
  // Turn off PWM.
  motor_speed = 0;
  pwm_duty_set(motor_speed);

  bldc_state = 6;      // Shutdown BLDC commutation.
  bldc_comm_state();

  ADCSRB = 0b00000000; // Disable analog comparator mux.
  ACSR  &= 0b11110111; // Disable analog comparator interrupt.

  PORTD &= 0b11101111; // Disable green LED (PD4).
  PORTD |= 0b00001000; // Enable yellow LED (PD3).
}

void bldc_comm_state(void){ // BLDC motor commutation function.
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
    case 6:
      // Shut down motor controller.
      PORTB &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
      PORTC &= 0b11111000; // Disable LI_A (PC2), LI_B (PC1) and LI_C (PC0).
      return;
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

  bldc_comm_state();
}