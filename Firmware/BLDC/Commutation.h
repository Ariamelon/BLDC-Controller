/*
This file is part of BLDC control firmware.

BLDC control firmware is free software: you can redistribute it and/or modify it
under the terms of the GNUGeneral Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.

BLDC control firmware is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with BLDC control firmware.
If not, see <https://www.gnu.org/licenses/>. 
*/

#define VOLTAGE_IN        9

#define PWM_MAX_DUTY      255
#define PWM_START_DUTY    30

#define ALIGN_MS          100
#define OPEN_LOOP_DELAY   5000

#define DEBOUNCE_PERIOD   10

void initialize_controller(void);

unsigned char motor_start(unsigned char, unsigned char, unsigned char );
void motor_stop(void);

unsigned char commutation_step_set(unsigned char);
unsigned char motor_speed_set(unsigned char);
void pwm_duty_set(unsigned char);