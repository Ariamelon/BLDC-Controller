#include "Arduino.h"
#include "Commutation.h"

void initialize_controller(void){
  DDRB   |= 0b00001110; // Configure HI_A (PB3), HI_B (PB2) and HI_C (PB1) (high side) as outputs.
  PORTB  &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
  PORTB  |= 0b00110000; // Enable speed+ (PB5) and speed- (PB4) pullups.

  DDRC   |= 0b00000111; // Configure LI_A (PC2), LI_B (PC1) and LI_C (PC0) (low side) as outputs.
  PORTC  &= 0b11111000; // Disable LI_A (PC2), LI_B (PC1) and LI_C (PC0).
  
  DDRD   |= 0b00011100; // Configure red LED (PD2), yellow LED (PD3) and green LED (PD4) as outputs.
  PORTD  &= 0b11100111; // Disable yellow LED (PD3) and green LED (PD4).
  PORTD  |= 0b00000100; // Enable red LED (PD2).

  TCCR1A  = 0;          // Initialize timers 1 and 2.
  TCCR1B  = 0b00000001; // Clock source set to clkIO / 1 (no prescaling).
  TCCR2A  = 0;          // Allows for highest PWM frequency.
  TCCR2B  = 0b00000001; // Also disable PWM outputs.

  ACSR    = 0b00010000; // Disable and clear ACI flag (analog comparator interrupt).
  ADCSRA  = 0;          // Disable ADC.
}

void motor_start(unsigned char current_speed){
  Serial.print("Starting up motor. ");
  PORTD |= 0b00010000;  // Enable green LED (PD4).

  ADCSRB = 0b01000000;  // Enable analog comparator mux.
  ACSR   = 0b00011000;  // Enable analog comparator interrupt and clear ACI flag (analog comparator interrupt).
}

void motor_stop(void){
  Serial.print("Shutting down motor. ");
  PORTD &= 0b11101111;  // Disable green LED (PD4).

  PORTC &= 0b11111000;  // Disable LI_A (PC2), LI_B (PC1) and LI_C (PC0).

  TCCR1A = 0;           // Disable HI_C (0C1A) and HI_B (OC1B).
  TCCR2A = 0;           // Disable HI_A (OC2A).

  ADCSRB = 0;           // Disable analog comparator mux.
  ACSR   = 0b00010000;  // Disable analog comparator interrupt and clear analog comparator interrupt flag.
}

unsigned char open_loop_startup(unsigned char next_step){
  unsigned int i = 2500;
  while(i > 250) {
    phase_output_set(next_step);
    next_step = commutation_step_set(next_step);
    delayMicroseconds(i);
    i -= 250;
  }
  return next_step;
}

unsigned char commutation_step_set (unsigned char old_step){
  unsigned char next_step = old_step;
  next_step++;
  next_step %= 6;
  return next_step;
}

void phase_output_set(unsigned char current_step){ // BLDC motor commutation function.
  switch(current_step){
    case 0:
      PORTC &= 0b11111010;  // Disable LI_A (PC2) and LI_C (PC0).
      PORTC |= 0b00000010;  // Enable LI_B (PC1).

      TCCR1A = 0;           // Disable HI_C (0C1A) and HI_B (OC1B).
      TCCR2A = 0b10000001;  // Enable HI_A (OC2A) in PWM, phase correct, 8-bit mode.

      ADMUX  = 0b00000011;  // Select V_Sen_C (ADC3) as comparator negative input.
      ACSR  |= 0b00000011;  // Set interrupt on rising edge and enable comparator interrupt.
      break;
    case 1:
      PORTC &= 0b11111001;  // Disable LI_A (PC2) and LI_B (PC1).
      PORTC |= 0b00000001;  // Enable LI_C (PC0).

      TCCR1A = 0;           // Disable HI_C (0C1A) and HI_B (OC1B).
      TCCR2A = 0b10000001;  // Enable HI_A (OC2A) in PWM, phase correct, 8-bit mode.

      ADMUX  = 0b00000100;  // Select V_Sen_B (ADC4) as comparator negative input.
      ACSR  &= 0b11111110;  // Set interrupt on falling edge.
      break;
    case 2:
      PORTC &= 0b11111001;  // Disable LI_A (PC2) and LI_B (PC1).
      PORTC |= 0b00000001;  // Enable LI_C (PC0).

      TCCR1A = 0b00100001;  // Enable HI_B (OC1B) in PWM, phase correct, 8-bit mode and disable HI_C (0C1A).
      TCCR2A = 0;           // Disable HI_A (OC2A).

      ADMUX  = 0b00000101;  // Select V_Sen_A (PC5) as comparator negative input.
      ACSR  |= 0b00000011;  // Set interrupt on rising edge.
      break;
    case 3:
      PORTC &= 0b11111100;  // Disable LI_B (PC1) and LI_C (PC0).
      PORTC |= 0b00000100;  // Enable LI_A (PC2).

      TCCR1A = 0b00100001;  // Enable HI_B (OC1B) in PWM, phase correct, 8-bit mode and disable HI_C (0C1A).
      TCCR2A = 0;           // Disable HI_A (OC2A).

      ADMUX  = 0b00000011;  // Select V_Sen_C (ADC3) as comparator negative input.
      ACSR  &= 0b11111110;  // Set interrupt on falling edge.
      break;
    case 4:
      PORTC &= 0b11111100;  // Disable LI_B (PC1) and LI_C (PC0).
      PORTC |= 0b00000100;  // Enable LI_A (PC2).

      TCCR1A = 0b10000001;  // Enable HI_C (0C1A) in PWM, phase correct, 8-bit mode and disable HI_B (OC1B).
      TCCR2A = 0;           // Disable HI_A (OC2A).

      ADMUX  = 0b00000100;  // Select V_Sen_B (ADC4) as comparator negative input.
      ACSR  |= 0b00000011;  // Set interrupt on rising edge.
      break;
    case 5:
      PORTC &= 0b11111010;  // Disable LI_A (PC2) and LI_C (PC0).
      PORTC |= 0b00000010;  // Enable LI_B (PC1).

      TCCR1A = 0b10000001;  // Enable HI_C (0C1A) in PWM, phase correct, 8-bit mode and disable HI_B (OC1B).
      TCCR2A = 0;           // Disable HI_A (OC2A).
      
      ADMUX  = 0b00000101;  // Select V_Sen_A (PC5) as comparator negative input.
      ACSR  &= 0b11111110;  // Set interrupt on falling edge.
      break;
  }
}

void pwm_duty_set(unsigned char duty){
  OCR1A = OCR1B = OCR2A = duty; // Set PWM duty cycle for all outputs.
}