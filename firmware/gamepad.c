/* Fischertechnik Joystick Interface
 * Copyright (C) 2015 Till Harbaum
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usb_gamepad.h"

#include "adctab.h"

#define LED_CONFIG	(DDRC |= (1<<7))
#define LED_OFF	        (PORTC &= ~(1<<7))
#define LED_ON	        (PORTC |= (1<<7))

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

void io_init() {
  // buttons
  DDRD  = 0x00; PORTD  = 0x00;   // Port D = Input, no pullups

  // direction
  DDRB &= 0x0f; PORTB &= 0x0f;   // Port B4-7 = Input, no pullups
}

#define GAMEPAD_BUT0         ((PIND & (1<<0))?0:0x01)  // Port D0
#define GAMEPAD_BUT1         ((PIND & (1<<1))?0:0x02)  // Port D1
#define GAMEPAD_BUT2         ((PIND & (1<<2))?0:0x04)  // Port D2
#define GAMEPAD_BUT3         ((PIND & (1<<3))?0:0x08)  // Port D3

#define GAMEPAD_BUT4         ((PIND & (1<<4))?0:0x10)  // Port D4
#define GAMEPAD_BUT5         ((PIND & (1<<5))?0:0x20)  // Port D5
#define GAMEPAD_BUT6         ((PIND & (1<<6))?0:0x40)  // Port D6
#define GAMEPAD_BUT7         ((PIND & (1<<7))?0:0x80)  // Port D7

#define GAMEPAD_LEFT_ON      ((PINB & (1<<6))?0:0x01)  // Port B6
#define GAMEPAD_RIGHT_ON     ((PINB & (1<<7))?0:0x01)  // Port B7
#define GAMEPAD_UP_ON        ((PINB & (1<<4))?0:0x01)  // Port B4
#define GAMEPAD_DOWN_ON      ((PINB & (1<<5))?0:0x01)  // Port B5

volatile uint8_t axis;
volatile uint16_t val[4];

/* adc interrupt service routine */
ISR(ADC_vect) {
  // read ADC value and store it
  if(axis < 4) 
    val[axis++] = ADC;

  // restart with next channel until all four channels
  // have been read
  if(axis < 4) {
    ADMUX = (ADMUX & ~((1<<MUX0) | (1<<MUX1)) ) | (axis << MUX0);
    _delay_us(10);
    ADCSRA |= (1<<ADSC);
  }
}

void adc_init() {
  /* 10 bit, AVcc reference */
  axis = 0;
  ADMUX = (1 << MUX2) | (0 << REFS1) | (1 << REFS0);

  /* conversion rate prescaler /64 */
  ADCSRA = (1<<ADEN) | (1<<ADIE) | (1<<ADPS1) | (1<<ADPS2);
  
  /* free running */
  ADCSRB = 0;
  
  /* start first conversion */
  ADCSRA |= (1<<ADSC);
}

int8_t adc_conv(uint16_t in) {
  // the useful input range only works in the lower 50% of the
  // adc range. The 32 values above the range return "max" 255, 
  // otherwise "out of range" 127. This makes sure that we can
  // handle potis which exceed the 4k7 limit a little bit but
  // can still detect open inputs correctly

  if(in > ADC_LIMIT) {
    if(in < ADC_LIMIT+32)
      return 255;
    else
      return 127;
  }

  return pgm_read_byte_near(adc2r + in);
}

int main(void) {
  // set for 16 MHz clock
  CPU_PRESCALE(0);
  
  LED_CONFIG;
  LED_OFF;

  usb_gamepad_reset_state();
    
  //  char i;  
  // for(i=0;i<10;i++)
    _delay_ms(10);
  
  // Initialize the USB, and then wait for the host to set configuration.
  // If the device is powered without a PC connected to the USB port,
  // this will wait forever.
  usb_init();
  while (!usb_configured()) /* wait */ ;
  
  // start adc conversion
  adc_init();
  io_init();
  
  LED_ON;    // everything is fine -> light led
  
  while (1) {
    // read gamepad data
    //    gamepad_read();

    // set "hat" values
    gamepad_state.direction = 8;
    if (GAMEPAD_UP_ON) {
      gamepad_state.direction = 0;
      if (GAMEPAD_LEFT_ON) {
	gamepad_state.direction = 7;
      } else if (GAMEPAD_RIGHT_ON) {
	gamepad_state.direction = 1;
      }
    } else {
      if (GAMEPAD_DOWN_ON) {
	gamepad_state.direction = 4;
	if (GAMEPAD_LEFT_ON) {
	  gamepad_state.direction = 5;
	} else if (GAMEPAD_RIGHT_ON) {
	  gamepad_state.direction = 3;
	}
      } else {
	if (GAMEPAD_LEFT_ON) {
	  gamepad_state.direction = 6;
	} else if (GAMEPAD_RIGHT_ON) {
	  gamepad_state.direction = 2;
	}
      }
    }

    // set button values
    gamepad_state.buttons =
      GAMEPAD_BUT0 | GAMEPAD_BUT1 |  GAMEPAD_BUT2 |  GAMEPAD_BUT3 | 
      GAMEPAD_BUT4 | GAMEPAD_BUT5 |  GAMEPAD_BUT6 |  GAMEPAD_BUT7;
    
    // wait for USB to become ready. This gives the ADC the time
    // to finish the conversion
    if(usb_gamepad_wait4tx() == 0) {

      if(axis == 4) {
	// set analogue axes values
	gamepad_state.l_x_axis = adc_conv(val[0]);
	gamepad_state.l_y_axis = adc_conv(val[1]);
	gamepad_state.r_x_axis = adc_conv(val[2]);
	gamepad_state.r_y_axis = adc_conv(val[3]);
      
	axis = 0;             // restart with axis 0 on channel 4
	ADMUX &= ~(3<<MUX0);  // restart with ADC input 4
	_delay_us(10);
	ADCSRA |= (1<<ADSC);  // start conversion
      }
	
      usb_gamepad_send();
    }
  }
}

