#include <stdlib.h>
#include "Commutation.h"
#include "UART_Comms.h"

#define FILTER_PERIOD 10

static volatile unsigned char commutation_step = 0;

// Analog comparator ISR
ISR (ANALOG_COMP_vect) {
  unsigned char i;
  // Debouncing filter.
  // Reads values FILTER_PERIOD times before processing result.
  for(i = 0; i < FILTER_PERIOD; i++) {
    if(commutation_step & 1){   // If BLDC step is odd (001, 011 or 101).
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

  phase_output_set(commutation_step);
  commutation_step = commutation_step_set(commutation_step);
}

// Sets motor speed depending on value of old speed.
unsigned char motor_speed_set(unsigned char new_speed){
  static unsigned char old_speed = 0;
  if (new_speed == old_speed){      // If speed hasn't changed, just return the old speed value.
    return old_speed;
  }

  if (new_speed < PWM_MIN_DUTY && new_speed < old_speed){
    new_speed = 0;                  // Stop motor if speed has been decreased below minimum.
    pwm_duty_set(new_speed);
    motor_stop();
  }
  else {
    if (new_speed < PWM_MIN_DUTY){  // If speed has been increased but is below min, set to min.
      new_speed = PWM_MIN_DUTY;
    }
    pwm_duty_set(new_speed);
    if (old_speed == 0){            // Start motor if motor was stopped.
      commutation_step = open_loop_startup(commutation_step);
      motor_start(new_speed);
    }
  }

  Serial.print("Speed: ");
  Serial.print(new_speed);
  Serial.println("/255.");

  old_speed = new_speed;
  return new_speed;
}

int main(void){
  init();
  
  Serial.begin(BAUD);
  Serial.println("Initializing BLDC motor controller.");
  initialize_controller();
  Serial.println("Initialization complete.");

  // Speed control via on-board buttons or serial input.
  // Some Arduino functions used here as speed is not critical.

  unsigned char motor_speed = 0, old_speed = 0;
  char uart_buffer[UART_BUFFER_SIZE];
  bool motor_spinning = 0;

  while(1){
    while(Serial.available() > 0){
      if(!UART_process_data(uart_buffer, UART_BUFFER_SIZE)){
        motor_speed = strtoul(uart_buffer, NULL, 0);
      }
    }

    if(!(PINB & (1 << PINB5))){         // Speed up button pressed.
      if (motor_speed < PWM_MAX_DUTY){
        motor_speed++;
      }
      delay(10);
    }

    else if(!(PINB & (1 << PINB4))){    // Speed down button pressed.
      if (motor_speed > 0){
        motor_speed--;
      }
      delay(10);
    }

    motor_speed = motor_speed_set(motor_speed);
  }

  return 1;
}