#define PWM_MAX_DUTY      255
#define PWM_MIN_DUTY      48
#define PWM_START_DUTY    64
#define FILTER_PERIOD     3

void pwm_duty_set(uint8_t duty);
void motor_phase_state(void);
void motor_start(void);
void motor_stop(void);

uint8_t motor_step = 0, motor_speed;

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing BLDC motor controller.");

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
  TCCR2B = 0b00000001; // Also disable PWM outputs.

  ACSR   = 0b00010000; // Disable and clear ACI flag (analog comparator interrupt).
  ADCSRA = 0;          // Disable ADC.

  Serial.println("Initialization complete.");
}

void loop() {
  // Speed control via on-board buttons. Gets overridden by analog comparator ISR.
  // Some Arduino functions used here as speed is not critical.
  
  while(!(PINB & (1 << PINB5))){                                        // Speed up button pressed.
    if ((motor_speed < PWM_MAX_DUTY) && (motor_speed >= PWM_MIN_DUTY)){ // If speed is below max and motor is already moving.
      motor_speed++;
      pwm_duty_set(motor_speed);
    }
    else if (motor_speed < PWM_MIN_DUTY){                               // If motor is stopped.
      Serial.println("Starting up motor.");                             // Start up motor.
      motor_start();
      Serial.println("Start-up complete.");
    }
    Serial.print("Motor speed: ");
    Serial.print(motor_speed);
    Serial.println("/255");
    delay(10);
  }
  while(!(PINB & (1 << PINB4))){                                        // Speed down button pressed
    if (motor_speed > PWM_MIN_DUTY){                                    // If speed is above min.
      motor_speed--;
      pwm_duty_set(motor_speed);
      Serial.print("Motor speed: ");
      Serial.print(motor_speed);
      Serial.println("/255");
    }
    else if ((motor_speed <= PWM_MIN_DUTY) && (motor_speed > 0)){       // If speed is at or below min.
      Serial.println("Shutting down motor.");
      motor_stop();
    }
    delay(10);
  }
}

void pwm_duty_set(uint8_t duty){
  OCR1A = OCR1B = OCR2A = duty; // Set PWM duty cycle for all outputs.
}

void motor_start(void){
  PORTD |= 0b00001000;          // Enable yellow LED (PD3).

  motor_speed = PWM_START_DUTY; // Setup motor starting PWM with duty cycle = PWM_START_DUTY.
  pwm_duty_set(motor_speed);

  uint16_t i = 5000;
  while(i > 100) {              // Initial open loop commutation to enable motor to start properly.
    motor_phase_state();
    delayMicroseconds(i);
    i -= 100;
  }

  ADCSRB = 0b01000000;          // Enable analog comparator mux.
  ACSR   = 0b00011000;          // Enable analog comparator interrupt and clear ACI flag (analog comparator interrupt).

  PORTD &= 0b11110111;          // Disable yellow LED (PD3).
  PORTD |= 0b00010000;          // Enable green LED (PD4).
}

void motor_stop(void){
  motor_speed = 0;
  pwm_duty_set(motor_speed);  // Turn off PWM.

  PORTC &= 0b11111000;        // Disable LI_A (PC2), LI_B (PC1) and LI_C (PC0).

  TCCR1A = 0;                 // Disable HI_C (0C1A) and HI_B (OC1B).
  TCCR2A = 0;                 // Disable HI_A (OC2A).

  ADCSRB = 0;                 // Disable analog comparator mux.
  ACSR   = 0b00010000;        // Disable analog comparator interrupt and clear analog comparator interrupt flag.

  PORTD &= 0b11101111;        // Disable green LED (PD4).
}

void motor_phase_state(void){ // BLDC motor commutation function.
  switch(motor_step){
    case 0:
      PORTC &= 0b11111010;    // Disable LI_A (PC2) and LI_C (PC0).
      PORTC |= 0b00000010;    // Enable LI_B (PC1).

      TCCR1A = 0;             // Disable HI_C (0C1A) and HI_B (OC1B).
      TCCR2A = 0b10000001;    // Enable HI_A (OC2A) in PWM, phase correct, 8-bit mode.

      ADMUX  = 0b00000011;    // Select V_Sen_C (ADC3) as comparator negative input.
      ACSR  |= 0b00000011;    // Set interrupt on rising edge and enable comparator interrupt.
      break;
    case 1:
      PORTC &= 0b11111001;    // Disable LI_A (PC2) and LI_B (PC1).
      PORTC |= 0b00000001;    // Enable LI_C (PC0).

      TCCR1A = 0;             // Disable HI_C (0C1A) and HI_B (OC1B).
      TCCR2A = 0b10000001;    // Enable HI_A (OC2A) in PWM, phase correct, 8-bit mode.

      ADMUX  = 0b00000100;    // Select V_Sen_B (ADC4) as comparator negative input.
      ACSR  &= 0b11111110;    // Set interrupt on falling edge.
      break;
    case 2:
      PORTC &= 0b11111001;    // Disable LI_A (PC2) and LI_B (PC1).
      PORTC |= 0b00000001;    // Enable LI_C (PC0).

      TCCR1A = 0b00100001;    // Enable HI_B (OC1B) in PWM, phase correct, 8-bit mode and disable HI_C (0C1A).
      TCCR2A = 0;             // Disable HI_A (OC2A).

      ADMUX  = 0b00000101;    // Select V_Sen_A (PC5) as comparator negative input.
      ACSR  |= 0b00000011;    // Set interrupt on rising edge.
      break;
    case 3:
      PORTC &= 0b11111100;    // Disable LI_B (PC1) and LI_C (PC0).
      PORTC |= 0b00000100;    // Enable LI_A (PC2).

      TCCR1A = 0b00100001;    // Enable HI_B (OC1B) in PWM, phase correct, 8-bit mode and disable HI_C (0C1A).
      TCCR2A = 0;             // Disable HI_A (OC2A).

      ADMUX  = 0b00000011;    // Select V_Sen_C (ADC3) as comparator negative input.
      ACSR  &= 0b11111110;    // Set interrupt on falling edge.
      break;
    case 4:
      PORTC &= 0b11111100;    // Disable LI_B (PC1) and LI_C (PC0).
      PORTC |= 0b00000100;    // Enable LI_A (PC2).

      TCCR1A = 0b10000001;    // Enable HI_C (0C1A) in PWM, phase correct, 8-bit mode and disable HI_B (OC1B).
      TCCR2A = 0;             // Disable HI_A (OC2A).

      ADMUX  = 0b00000100;    // Select V_Sen_B (ADC4) as comparator negative input.
      ACSR  |= 0b00000011;    // Set interrupt on rising edge.
      break;
    case 5:
      PORTC &= 0b11111010;    // Disable LI_A (PC2) and LI_C (PC0).
      PORTC |= 0b00000010;    // Enable LI_B (PC1).

      TCCR1A = 0b10000001;    // Enable HI_C (0C1A) in PWM, phase correct, 8-bit mode and disable HI_B (OC1B).
      TCCR2A = 0;             // Disable HI_A (OC2A).
      
      ADMUX  = 0b00000101;    // Select V_Sen_A (PC5) as comparator negative input.
      ACSR  &= 0b11111110;    // Set interrupt on falling edge.
      break;
  }
  motor_step ++;              // Advance commutation step by one.
  motor_step %= 6;
}

// Analog comparator ISR
ISR (ANALOG_COMP_vect) {
  uint8_t i;
  // BEMF filter, reads values FILTER_PERIOD times before processing result.
  for(i = 0; i < FILTER_PERIOD; i++) {
    if(motor_step & 1){         // If BLDC step is odd (001, 011 or 101).
      if(!(ACSR & 0b00100000)){ // If 0 detected on ACO (comparator output), delay debouncing.
        i -= 1;
      }
    }
    else{                       // If BLDC step is even (000, 010 or 100).
      if((ACSR & 0b00100000)){  // If 1 detected on ACO (comparator output), delay debouncing.
        i -= 1;
      }
    }
  }

  motor_phase_state();
}