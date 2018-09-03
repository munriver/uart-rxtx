/*
 * Receiver   
 */


#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 8000000
#endif

#define BAUDRATE 2400
#define UBRRVAL ((F_CPU/(BAUDRATE*16UL))-1)
    
#define SYNC 0x5F

// Global variables
static volatile uint8_t xstick, ystick, throttle;

void uartInit(void)
{
	// Set baud rate
	UBRRL = (uint8_t)UBRRVAL;  // low byte
	UBRRH = (UBRRVAL>>8) & 0x0F;  // high byte
	// Set data frame format: asynchronous mode,no parity, 1 stop bit, 8 bit size
	UCSRC = (0<<UMSEL) | (0<<UPM1) | (0<<UPM0) |
            (1<<URSEL) |  // Bit specific to atmega8
		    (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0);	
	// Enable Receiver and Interrupt on receive complete
	UCSRB = (1<<RXEN); //| (1<<RXCIE); 
}

uint8_t uartRecvByte(void)
{
    // Wait until a byte has been received
    while((UCSRA & (1<<RXC)) == 0);
    // Return received data
    return UDR;
}

void recv_packet(void)
{
    uint8_t inbyte;

    while((inbyte = uartRecvByte()) != SYNC);
    xstick = uartRecvByte();
    ystick = uartRecvByte();
    throttle = uartRecvByte();
}

void ioInit(void)
{
    DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3);
    DDRD |= (1<<PD7);
}

void setupPwm1(void)
{
   TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);  // NON Inverted PWM
   TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS12);  // PRESCALER=256 MODE 14(FAST PWM)

   ICR1 = 624;  // fPWM=50Hz (Period = 20ms Standard).
}

void setupPwm2(void)
{
    TCCR2 |= (1<<WGM20) | (1<<CS20) | (1<<COM21);
}

int main(void)
{
    ioInit();
    sei();
    setupPwm1();
    setupPwm2();
    uartInit();

    while(1)
    {
        recv_packet();
        PORTB ^= (1<<PB0);
        OCR1A = 16 + (xstick * 0.24); 
        OCR1B = 16 + (ystick * 0.24);
        OCR2 =  throttle;
    }
    return 0;
}

