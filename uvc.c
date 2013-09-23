#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include "uvc.h"

// write a string to the uart
#define uart_print(s) uart_print_P(PSTR(s))
void uart_print_P(const char *str)
{
	char c;
	while (1) {
		c = pgm_read_byte(str++);
		if (!c) break;
			uart_putchar(c);
	}
}

// A very basic example...
// when the user types a character, print it back
int main(void)
{
	uint8_t c;
	uint32_t cnt = 0;

	CPU_PRESCALE(0);  // run at 16 MHz
	LED_CONFIG;

	uart_init(BAUD_RATE);
	
	// Initialize the USB, and then wait for the host to set configuration.
	usb_init();
	while (!usb_configured()) /* wait */ ;

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	while (1) {
		
	}
}
