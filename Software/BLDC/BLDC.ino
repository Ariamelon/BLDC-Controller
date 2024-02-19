#define PWM_MAX_DUTY      255
#define PWM_MIN_DUTY      24
#define PWM_START_DUTY    100
#define DEBOUNCE_PERIOD   10
#define BTN_SPEED_UP      D2
#define BTN_SPEED_DOWN    D3
 
uint8_t bldc_state = 0; motor_speed;
uint32_t i;

void setup() {
  // Enable input with pullups for the speed control buttons.
  pinMode(BTN_SPEED_UP, INPUT_PULLUP);
  pinMode(BTN_SPEED_DOWN, INPUT_PULLUP);

  DDRB  |= 0b00001110;          // Configure pins B1, B2 and B3 (high side) as outputs.
  PORTB &= 0b11110001;          // Set pins B1, B2 and B3 to 0.

  DDRC  |= 0b00000111;          // Configure pins C0, C1 and C2 (low side) as outputs.
  PORTC &= 0b11111000;          // Set pins C0, C1 and C2 to 0.

  // Disable timers 1 and 2.
  TCCR1A = 0;
  TCCR2A = 0;

  // Set clock source to clkIO / 1 (no prescaling) for maximum PWM frequency. 
  TCCR1B = 0b00000001;
  TCCR2B = 0b00000001;

  ACSR   = 0b00010000;          // Disable and clear ACI flag (analog comparator interrupt).

  motor_speed = PWM_START_DUTY;
  PWM_DUTY_SET(motor_speed); // Setup starting PWM with duty cycle = PWM_START_DUTY.

  // Initial open loop commutation with delay from 10000us down to 100us to enable motor to start properly.
  i = 10000;
  while(i > 100) {
    delayMicroseconds(i);
    bldc_comm();
    i = i - 20;
  }

  ADCSRB = 0b01000000;          // Enable analog comparator mux
  ACSR  |= 0b00001000;          // Enable analog comparator interrupt.
}

void loop() {
  // Speed control via on-board buttons.
  while(!(digitalRead(SPEED_UP)) && motor_speed < PWM_MAX_DUTY){
    motor_speed++;
    PWM_DUTY_SET(motor_speed);
    delay(100);
  }
  while(!(digitalRead(SPEED_DOWN)) && motor_speed > PWM_MIN_DUTY){
    motor_speed--;
    PWM_DUTY_SET(motor_speed);
    delay(100);
  }
}

void BEMF_A_RISING(){
  ADMUX = 0b00000101;           // Select ADC5 (PC5) as comparator negative input.
  ACSR |= 0b00000011;           // Set interrupt on rising edge.
}
void BEMF_A_FALLING(){
  ADMUX = 0b00000101;           // Select ADC5 (PC5) as comparator negative input.
  ACSR &= 0b11111110;           // Set interrupt on falling edge.
}
void BEMF_B_RISING(){
  ADMUX = 0b00000100;           // Select ADC4 (PC4) as comparator negative input.
  ACSR |= 0b00000011;           // Set interrupt on rising edge.
}
void BEMF_B_FALLING(){
  ADMUX = 0b00000100;           // Select ADC4 (PC4) as comparator negative input.
  ACSR &= 0b11111110;           // Set interrupt on falling edge.
}
void BEMF_C_RISING(){
  ADMUX = 0b00000011;           // Select ADC3 (PC3) as comparator negative input.
  ACSR |= 0b00000011;           // Set interrupt on rising edge.
}
void BEMF_C_FALLING(){
  ADMUX = 0b00000011;           // Select ADC3 (PC3) as comparator negative input.
  ACSR &= 0b11111110;           // Set interrupt on falling edge.
}

void HI_A(){
  TCCR1A = 0;                   // Disable HI_C (0C1A) and HI_B (OC1B).
  TCCR2A = 0b10000001;          // Enable OC2A in PWM, phase correct, 8-bit mode.
}

void HI_B(){
  TCCR1A = 0b00100001;          // Enable HI_B (OC1B) in PWM, phase correct, 8-bit mode and disable HI_C (0C1A).
  TCCR2A = 0;                   // Disable HI_A (OC2A).
}

void HI_C(){
  TCCR1A = 0b10000001;          // Enable HI_C (0C1A) in PWM, phase correct, 8-bit mode and disable HI_B (OC1B).
  TCCR2A = 0;                   // Disable HI_A (OC2A).
}

void LI_A(){
  PORTC  = 0b00000100;          // Enable LI_A (PC2) (low side) output.
}

void LI_B(){
  PORTC  = 0b00000010;          // Enable LI_B (PC1) (low side) output.
}

void LI_C(){
  PORTC  = 0b00000001;          // Enable LI_C (PC0) (low side) output.
}
 
void PWM_DUTY_SET(uint8_t duty){
  // Constrain duty cycle to max and min values
  if(duty < PWM_MIN_DUTY)
    duty  = PWM_MIN_DUTY;
  if(duty > PWM_MAX_DUTY)
    duty  = PWM_MAX_DUTY;

  OCR1A = duty;                 // Set B1 PWM duty cycle.
  OCR1B = duty;                 // Set B2 PWM duty cycle.
  OCR2A = duty;                 // Set B3 PWM duty cycle.
}

void bldc_comm(){               // BLDC motor commutation function.
  switch(bldc_state){
    case 0:
      HI_A();
      LI_B();
      BEMF_C_RISING();
      break;
    case 1:
      HI_A();
      LI_C();
      BEMF_B_FALLING();
      break;
    case 2:
      HI_B();
      LI_C();
      BEMF_A_RISING();
      break;
    case 3:
      HI_B();
      LI_A();
      BEMF_C_FALLING();
      break;
    case 4:
      HI_C();
      LI_A();
      BEMF_B_RISING();
      break;
    case 5:
      HI_C();
      LI_B();
      BEMF_A_FALLING();
      break;
  }

  bldc_state++;                 // Advance commutation state by one.
  bldc_state %= 6;              // To prevent overflow. If step > 5, step = 0.
}

// Analog comparator ISR
ISR (ANALOG_COMP_vect) {

  // BEMF debounce, reads values DEBOUNCE_PERIOD times before processing result.
  for(i = 0; i < DEBOUNCE_PERIOD; i++) {
    if(bldc_state & 1){
      if(!(ACSR & 0x20)) i -= 1;
    }
    else {
      if((ACSR & 0x20))  i -= 1;
    }
  }

  bldc_comm();
}