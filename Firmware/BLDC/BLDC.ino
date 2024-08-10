#include <stdlib.h>
#include "Commutation.h"
#include "UART_Comms.h"

static volatile unsigned char commutation_step = 0;

// Analog comparator ISR
ISR (ANALOG_COMP_vect) {
  unsigned char i;

  // Debouncing filter. Reads values DEBOUNCE_PERIOD times before processing result.
  // Increases debounce period if BLDC step is odd & 0 detected on ACO (comparator output).
  // Or if BLDC step is even & 1 detected on ACO (comparator output).
  for(i = 0; i < DEBOUNCE_PERIOD; i++) {
    if(((commutation_step & 1) && !(ACSR & 0b00100000)) 
   || (!(commutation_step & 1) && (ACSR & 0b00100000))){
      i -= 1;
    }
  }

  commutation_step = commutation_step_set(commutation_step);
}

int main(void){
  init();
  
  Serial.begin(BAUD);
  while(Serial.available() > 0){
    Serial.read();
  }
  Serial.flush();
  
  Serial.println("Initializing BLDC motor controller.");
  initialize_controller();
  Serial.println("Initialization complete.");

  // Speed control via on-board buttons or serial input.
  // Some Arduino functions used here as speed is not critical.

  unsigned char pwm_old_duty = 0;
  unsigned char pwm_min_duty = 65 - VOLTAGE_IN * 2;
  int pwm_new_duty = 0;
  char uart_buffer[UART_BUFFER_SIZE];
  bool motor_spinning = 0;

  while(1){
    while(Serial.available() > 0){
      if(!UART_process_data(uart_buffer, UART_BUFFER_SIZE)){
        pwm_new_duty = strtoul(uart_buffer, NULL, 0);
      }
    }

    if(!(PINB & (1 << PINB5))){             // Speed up button pressed.
      pwm_new_duty++;
      delay(10);
    }

    else if(!(PINB & (1 << PINB4))){        // Speed down button pressed.
      pwm_new_duty--;
      delay(10);
    }

    if (pwm_new_duty < pwm_min_duty){
      if (pwm_new_duty > pwm_old_duty){     // Constrain lower limit.
        pwm_new_duty = pwm_min_duty;        // Interpret as motor start command.
      }
      else{
        pwm_new_duty = 0;                   // Interpret as motor stop command.
      }
    }
    else if (pwm_new_duty > PWM_MAX_DUTY){  // Constrain upper limit.
      pwm_new_duty = PWM_MAX_DUTY;
    }

    if (pwm_new_duty != pwm_old_duty){      // Ignore command if value is unchanged.
      if (pwm_new_duty == 0){
        motor_stop();
      }
      else if (pwm_old_duty == 0){          // If motor is stationary, any number starts the motor.
        pwm_new_duty = pwm_min_duty;
        commutation_step = motor_start(pwm_new_duty, commutation_step, VOLTAGE_IN);
      }
      Serial.print("Duty cycle: ");
      Serial.print(pwm_new_duty);
      Serial.println("/255.");

      pwm_duty_set(pwm_new_duty);
      pwm_old_duty = pwm_new_duty;
    }
  }  
}