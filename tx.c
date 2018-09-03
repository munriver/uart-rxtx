/*
 * Transmitter  
 */


#include<avr/io.h>
#include<util/delay.h>

#ifndef F_CPU
#define F_CPU 8000000
#endif

#define BAUDRATE 2400
#define UBRRVAL ((F_CPU/(BAUDRATE*16UL))-1)

#define SYNC 0x5F

void wait(uint8_t mode)
{
    uint8_t i;
    
    if(mode == 1)
    {
        for(i = 0; i < 3; i++)
        {
            _delay_ms(100);
        }
    }
    else 
    {
        for(i = 0; i < 5; i++)
        {
            _delay_ms(100);
        }
    }
}

void beep(void)
{
    TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);  // non inverted PWM
    TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS10);  // no prescaler, fast PWM mode
    
    ICR1 = 2665;  // 3khz
    OCR1A = 1332;
    wait(1);

    ICR1 = 1999;  // 4khz
    OCR1A = 999;
    wait(1);

    TCCR1A = 0;
    TCCR1B = 0;
}

void adcSetup(void) 
{ 
   ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0);  // Set ADC prescalar to 64 - 125KHz sample rate @ 8MHz 
   ADMUX |= (1<<REFS0);  // Set ADC reference to AVCC 
   ADMUX |= (1<<ADLAR);  // Left adjust ADC result to allow easy 8 bit reading 
   //ADCSRA |= (1<<ADFR);  // Set ADC to Free-Running Mode 
   ADCSRA |= (1<<ADEN);  // Enable ADC 
}

uint8_t adcSwitch(uint8_t channel)
{
    ADMUX &= ~(1<<MUX0);
    ADMUX &= ~(1<<MUX1);
    ADMUX |= channel;
    ADCSRA |= (1<<ADSC);
    while(!(ADCSRA & (1<<ADIF)));
    ADCSRA |= (1<<ADIF);
    
    return ADCH;
}

void uartInit(void)
{
	// Set baud rate
	UBRRL = (uint8_t)UBRRVAL;  // low byte
	UBRRH = (UBRRVAL>>8) & 0x0F;  // high byte
	// Set data frame format: asynchronous mode,no parity, 1 stop bit, 8 bit size
	UCSRC = (1<<URSEL) | (0<<UMSEL) | (0<<UPM1) | (0<<UPM0) |
		    (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0);	
	// Enable Transmitter 
	UCSRB = (1<<TXEN);
}

void uartSendByte(uint8_t u8Data)
{
    // Wait if a byte is being transmitted
    while((UCSRA & (1<<UDRE)) == 0);
    // Transmit data
    UDR = u8Data; 
}

void ioInit(void)
{
    DDRD |= (1<<PD6) | (1<<PD7);
    DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB3) | (1<<PB2);
}

int main(void)
{
    ioInit();
    beep();
    adcSetup();
    uartInit();
    
    while(1)
    {
        uartSendByte(SYNC);
        uartSendByte(adcSwitch(0));        
        uartSendByte(adcSwitch(1));
        uartSendByte(adcSwitch(2));

        PORTB ^= (1<<PB0);
        _delay_ms(100);
    }

    return 0;
}


