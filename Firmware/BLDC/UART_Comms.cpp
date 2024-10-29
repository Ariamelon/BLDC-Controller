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

#include "Arduino.h"
#include "UART_Comms.h"

bool UART_process_data(char * in_buffer, size_t in_buffer_size){
  static unsigned int buffer_pos = 0;         // Buffer pointer variable.
  unsigned char in_byte = Serial.read();      // Read in next byte in serial buffer.
  switch (in_byte){
    case '\n':                                // Detects end of text.
      if (buffer_pos == 0){                   // If no text was entered, keep previous output.
        return 1;
      }
      in_buffer[buffer_pos] = 0;              // Terminates string.
      buffer_pos = 0;                         // Reset buffer location.
      return 0;
    case '\r':                                // Discard carriage return.
      break;
    default:
      if (buffer_pos < (in_buffer_size - 1)){ // Only appends to string if buffer is not full.
        in_buffer[buffer_pos++] = in_byte;    // Appends new byte to buffer and advances buffer pointer.
      }
      break;
  }
  return 1;
}