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

#define BAUD              9600
#define UART_BUFFER_SIZE  64

bool UART_process_data(char *, size_t);