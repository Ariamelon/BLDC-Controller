void test_outputs(int);
int i = 1;

void setup() {
  Serial.begin(9600);
  DDRB   |= 0b00001110; // Configure HI_A (PB3), HI_B (PB2) and HI_C (PB1) (high side) as outputs.
  PORTB  &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
  PORTB  |= 0b00110000; // Enable speed+ (PB5) and speed- (PB4) pullups.

  DDRC   |= 0b00000111; // Configure LO_A (PC2), LO_B (PC1) and LO_C (PC0) (low side) as outputs.
  PORTC  &= 0b11111000; // Disable LO_A (PC2), LO_B (PC1) and LO_C (PC0).

  DDRD   |= 0b00011100; // Configure red LED (PD2), yellow LED (PD3) and green LED (PD4) as outputs.
  PORTD  &= 0b11100011; // Disable red LED (PD2), yellow LED (PD3) and green LED (PD4).
  
  test_outputs(i);
}

void loop() {
  while(!(PINB & (1 << PINB5))){  // Speed up button pressed.
    test_outputs(i);
    i++;
    if (i > 6){
      i = 1;
    }
    delay(500);
  }
  while(!(PINB & (1 << PINB4))){  // Speed down button pressed.
    test_outputs(i);
    i--;
    if (i < 1){
      i = 6;
    }
    delay(500);
  }
}

void test_outputs(int j){
  Serial.println(j);

  switch(j){
    case 1:
      PORTC  &= 0b11111000; // Disable LO_A (PC2), LO_B (PC1) and LO_C (PC0).
      PORTB  &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
      PORTC  |= 0b00000100; // Enable LO_A (PC2).
      PORTD  &= 0b11100011; // Disable red LED (PD2), yellow LED (PD3) and green LED (PD4).
      PORTD  |= 0b00000100; // Enable red LED (PD2).
      break;
    case 2:
      PORTC  &= 0b11111000; // Disable LO_A (PC2), LO_B (PC1) and LO_C (PC0).
      PORTB  &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
      PORTC  |= 0b00000010; // Enable LO_B (PC1).
      PORTD  &= 0b11100011; // Disable red LED (PD2), yellow LED (PD3) and green LED (PD4).
      PORTD  |= 0b00001000; // Enable yellow LED (PD3).
      break;
    case 3:
      PORTC  &= 0b11111000; // Disable LO_A (PC2), LO_B (PC1) and LO_C (PC0).
      PORTB  &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
      PORTC  |= 0b00000001; // Enable LO_C (PC0).
      PORTD  &= 0b11100011; // Disable red LED (PD2), yellow LED (PD3) and green LED (PD4).
      PORTD  |= 0b00001100; // Enable red LED (PD2) and yellow LED (PD3).
      break;
    case 4:
      PORTC  &= 0b11111000; // Disable LO_A (PC2), LO_B (PC1) and LO_C (PC0).
      PORTB  &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
      PORTB  |= 0b00001000; // Enable HI_A (PB3).
      PORTD  &= 0b11100011; // Disable red LED (PD2), yellow LED (PD3) and green LED (PD4).
      PORTD  |= 0b00010000; // Enable green LED (PD4).
      break;
    case 5:
      PORTC  &= 0b11111000; // Disable LO_A (PC2), LO_B (PC1) and LO_C (PC0).
      PORTB  &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
      PORTB  |= 0b00000100; // Enable HI_B (PB2).
      PORTD  &= 0b11100011; // Disable red LED (PD2), yellow LED (PD3) and green LED (PD4).
      PORTD  |= 0b00010100; // Enable red LED (PD2) and green LED (PD4).
      break;
    case 6:
      PORTC  &= 0b11111000; // Disable LO_A (PC2), LO_B (PC1) and LO_C (PC0).
      PORTB  &= 0b11110001; // Disable HI_A (PB3), HI_B (PB2) and HI_C (PB1).
      PORTB  |= 0b00000010; // Enable HI_C (PB1).
      PORTD  &= 0b11100011; // Disable red LED (PD2), yellow LED (PD3) and green LED (PD4).
      PORTD  |= 0b00011000; // Enable yellow LED (PD3) and green LED (PD4).
      break;
  }
}
