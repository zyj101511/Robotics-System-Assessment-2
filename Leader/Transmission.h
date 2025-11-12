byte data_to_send = 0b10110010;
volatile byte bit_index = 0;
volatile boolean sending = false; 
volatile boolean first_half_cycle = true;
//static boolean last_emit_state = true;

// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the compiler.
// It actually associates with Timer3 in CTC mode (Clear Timer on Compare Match).
// so this ISR will run every time Timer3 reaches the value in OCR3A.

ISR(TIMER3_COMPA_vect)
{
  // 发完所有位
  if (bit_index >= 8)
  {
    bit_index = 0;
    sending = false;
  }

  
  if (sending)
  {
    boolean current_bit = (data_to_send >> (7 - bit_index)) & 1;

    if (first_half_cycle)
    {
      // bit 周期上半段
      if (current_bit == 1)
      {
        // bit=1: 上半段亮
        DDRB |= (1 << PB7);
        PORTB &= ~(1 << PB7);
      }
      else
      {
        // bit=0: 上半段灭
        DDRB &= ~(1 << PB7);
      }
      first_half_cycle = false; // 进入下半段
    }
    else
    {
      // bit 周期下半段 → 翻转（保证中间跳变）
      if (current_bit == 1)
      {
        // bit=1: 下半段灭
        DDRB &= ~(1 << PB7);
      }
      else
      {
        // bit=0: 下半段亮
        DDRB |= (1 << PB7);
        PORTB &= ~(1 << PB7);
      }
      first_half_cycle = true;
      bit_index++;
    }
  }
  else
  {
    //DDRB |= (1 << PB7);
    //PORTB &= ~(1 << PB7);
  }
}

void setupTimer3(){

  // disable global interrupts
  cli();

  // Reset timer3 to a blank condition
  // TCCR = Timer/Counter Contol Register
  TCCR3A = 0;
  TCCR3B = 0;

  // First turn on CTC mode. Timer3 will counts up
  // and create an interrupt on a match to a value.
  TCCR3B = TCCR3B | (1 << WGM32);  // Waveform Generation Mode

  // For a cpu clock prescaler of 256:
  // Shift a 1 up to bit CS32(clock select, timer3, bit2)
  TCCR3B = TCCR3B | (1 << CS31);
  TCCR3B = TCCR3B | (1 << CS30);

  // set compare match register to desired timer count
  // CPU Clock = 16000000
  // Prescaler = 256
  // Timer freq = 16000000/256 = 62500
  // We can think of this as timer3 counting upto 62500 in 1 second.
  // compare match value = 62500 / 2
  OCR3A = 250;

  //enanle timer compare interrupt
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  //enable global interrupts
  sei();  
}
