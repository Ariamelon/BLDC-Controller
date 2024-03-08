#include "BLDC_Functions.h"

#define PWM_MAX_DUTY      255
#define PWM_MIN_DUTY      32
#define PWM_START_DUTY    100
#define DEBOUNCE_PERIOD   5
#define BTN_SPEED_UP      PD2
#define BTN_SPEED_DOWN    PD3

void disable_output();
void H_phase_out(uint8_t);
void L_phase_out(uint8_t);

void comp_phase_set(uint8_t);
void comp_edge_set(bool);

void pwm_duty_set(uint8_t duty);
void bldc_comm();
 
uint8_t bldc_state = 0, motor_speed;
uint16_t i;
bool motor_direction = 1;

void setup() {
  // Serial.begin(9600);

  // Enable input with pullups for the speed control buttons.
  pinMode(BTN_SPEED_UP, INPUT_PULLUP);
  pinMode(BTN_SPEED_DOWN, INPUT_PULLUP);

  DDRB  |= 0b00001110; // Configure HI_A (PB3), HI_B (PB2) and HI_C (PB1) (high side) as outputs.
  DDRC  |= 0b00000111; // Configure LI_A (PC2), LI_B (PC1) and LI_C (PC0) (low side) as outputs.
  disable_output();

  // Disable timers 1 and 2.
  TCCR1A = 0;
  TCCR2A = 0;

  // Set clock source to clkIO / 1 (no prescaling) for maximum PWM frequency. 
  TCCR1B = 0b00000001;
  TCCR2B = 0b00000001;

  ACSR   = 0b00010000; // Disable and clear ACI flag (analog comparator interrupt).

  motor_speed = PWM_START_DUTY;
  pwm_duty_set(motor_speed); // Setup starting PWM with duty cycle = PWM_START_DUTY.

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

// Sets all phase outputs to 0.
void disable_output(){
  PORTB &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
  PORTC &= 0b11111000; // Disable LI_A (PC2), LI_B (PC1) and LI_C (PC0).
}

// Determines which high-side phase will be activated.
void H_phase_out(uint8_t high_phase){
  switch (high_phase){
    case 0:
      PORTB |= 0b00001000; // Enable HI_A (PB3).
      TCCR1A = 0;          // Disable HI_C (0C1A) and HI_B (OC1B).
      TCCR2A = 0b10000001; // Enable HI_A (OC2A) in PWM, phase correct, 8-bit mode.
      break;
    case 1:
      PORTB |= 0b00000100; // Enable HI_B (PB2).
      TCCR1A = 0b00100001; // Enable HI_B (OC1B) in PWM, phase correct, 8-bit mode and disable HI_C (0C1A).
      TCCR2A = 0;          // Disable HI_A (OC2A).
      break;
    case 2:
      PORTB |= 0b00000010; // Enable HI_C (PB1).
      TCCR1A = 0b10000001; // Enable HI_C (0C1A) in PWM, phase correct, 8-bit mode and disable HI_B (OC1B).
      TCCR2A = 0;          // Disable HI_A (OC2A).
      break;
    default:
      Serial.println("Erroneous input detected. Phase outputs disabled.");
      disable_output();
      break;
  }
}

// Determines which low-side phase will be activated.
void L_phase_out(uint8_t low_phase){
  switch (low_phase){
    case 0:
      PORTC |= 0b00000100; // Enable LI_A (PC2).
      break;
    case 1:
      PORTC |= 0b00000010; // Enable LI_B (PC1).
      break;
    case 2:
      PORTC |= 0b00000001; // Enable LI_C (PC0).
      break;
    default:
      Serial.println("Erroneous input detected. Phase outputs disabled.");
      disable_output();
      break;
  }
}

// Determines which pin should be MUXED to the comparator negative input and whether it triggers on a rising or falling edge.
void comp_phase_set(uint8_t comp_phase_detect){
  switch (comp_phase_detect){
    case 0:
      ADMUX = 0b00000101; // Select ADC5 (PC5) as comparator negative input.
      break;
    case 1:
      ADMUX = 0b00000100; // Select ADC4 (PC4) as comparator negative input.
      break;
    case 2:
      ADMUX = 0b00000011; // Select ADC3 (PC3) as comparator negative input.
      break;
    default:
      Serial.println("Erroneous input detected. Phase outputs disabled.");
      disable_output();
      break;
  }
}

void comp_edge_set(bool comp_edge_direction){
  switch (comp_edge_direction){
    case 0:
      ACSR &= 0b11111110; // Set interrupt on falling edge.
      break;
    case 1:
      ACSR |= 0b00000011; // Set interrupt on rising edge.
      break;
    default:
      Serial.println("Erroneous input detected. Phase outputs disabled.");
      disable_output();
      break;
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
  //Serial.print("Commutation state: ");
  //Serial.println(bldc_state);

  disable_output();

  switch(bldc_state){
    case 0:
      H_phase_out(0);
      L_phase_out(1);
      comp_phase_set(2);
      comp_edge_set(motor_direction);
      break;
    case 1:
      H_phase_out(0);
      L_phase_out(2);
      comp_phase_set(1);
      comp_edge_set(!motor_direction);
      break;
    case 2:
      H_phase_out(1);
      L_phase_out(2);
      comp_phase_set(0);
      comp_edge_set(motor_direction);
      break;
    case 3:
      H_phase_out(1);
      L_phase_out(0);
      comp_phase_set(2);
      comp_edge_set(!motor_direction);
      break;
    case 4:
      H_phase_out(2);
      L_phase_out(0);
      comp_phase_set(1);
      comp_edge_set(motor_direction);
      break;
    case 5:
      H_phase_out(2);
      L_phase_out(1);
      comp_phase_set(0);
      comp_edge_set(!motor_direction);
      break;
    default:
      Serial.println("Erroneous input detected. Phase outputs disabled.");
      disable_output();
      break;
  }

  switch(motor_direction){
    case 0:
      bldc_state --;
      if (bldc_state > 5){
        bldc_state = 5;
      }
      break;
    case 1:
      bldc_state ++; // Advance commutation state by one.
      if (bldc_state > 5){
        bldc_state = 0;
      }
      break;
  }
}

// Analog comparator ISR
ISR (ANALOG_COMP_vect) {
  // BEMF debounce, reads values DEBOUNCE_PERIOD times before processing result.
  for(i = 0; i < DEBOUNCE_PERIOD; i++) {
    if(ACSR & 0b00000011){
      if(!(ACSR & 0b00100000)) i -= 1; // If 0 detected on ACO (comparator output), delay debouncing.
    }
    else if(ACSR & 0b00000010){
      if((ACSR & 0b00100000))  i -= 1; // If 1 detected on ACO (comparator output), delay debouncing.
    }
  }

  bldc_comm();
}