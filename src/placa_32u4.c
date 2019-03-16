/*
 * 	
 *  Author: Lucas Tamborrino
 *  Alterado: Magoo
 */ 


#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "lcd_usb.h"
#include "defines.h"
#include <math.h>

//************************************************************************

#define F_OSC	8000000
#define PWM_F_OSC	10000
#define SIN_F_OSC	60
#define LUT_SIZE	PWM_F_OSC/SIN_F_OSC //(166)
#define MAX_ADC_VALUE	1023
//************************************************************************

uint16_t adcValue = 0;
uint16_t adcNorm = 0;

uint16_t ICR1_value = 0;
uint16_t LUT_size = 0;

double lookUpTable[LUT_SIZE];


//************************************************************************
uint16_t calculate_sin_values()
{	
	double sinVal = 0;
	// Calcula valor ICR1
	ICR1_value = (uint16_t) F_OSC/PWM_F_OSC;


	for(uint16_t i = 0; i<LUT_SIZE; i++)
	{
		sinVal = sin(i*2*M_PI/LUT_SIZE);
		lookUpTable[i] = sinVal;
	}

	return ICR1_value;
	
}

void Adc_init(void){

    //ADC Multiplexer Selection Register = ADMUX
	ADMUX = (1<<MUX0)| (1<<REFS0);  // Set reference to AVcc,set chanel 1

	// ADCSRA - ADC Control and Status Register A
	ADCSRA = (1<<ADEN);  // Enable ADC
	ADCSRA = (1<<ADATE)|(1<<ADEN)|(1<<ADIF);  // Auto Trigger Enable  (default auto-trigger is free-running mode ADCSRB)
	ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADEN)|(1<<ADSC)|(1<<ADIE);
	// Disable digital input buffer on the ADC pin (reduces power consumption)
	DIDR0=(1<<5)|(1<<4)|(1<<1)|(1<<0);
}

//Interrupcao de leitura analogica
ISR(ADC_vect)
{
	adcValue = ADC;

	adcNorm = (uint16_t)((float)adcValue * (float)(ICR1_value/MAX_ADC_VALUE);

	//Nova leitura 
	ADCSRA|= (1<<ADSC);

}



void Timer0_init(void) // Aciona contador externo
{

	TCNT0 = 0;
	TCCR0A =0;
	TCCR0B = (1<< CS02)| (1<< CS01)|(1<< CS00); /// subida em T0 === D2 === PD7
	//TIMSK1 = (1<<TOIE1);
}



void Timer1_init(void)
{

	//TCNT1 = 217;
	TCCR1A = 0;
	TCCR1B =  (1<< CS11)|(1<< CS10);
	TIMSK1 = (1<<TOIE1);
}

// Interrupção Timer 1
ISR(TIMER1_OVF_vect)
{

	

}

// Interrupção Timer 0
ISR(TIMER0_OVF_vect)
{

	

}

void init_pwm1C(void)
{
   
   TCCR1A |= (1<<COM1C1);
   TCCR1B = (1<< WGM13)|(1<< CS10); //NO preescaler
   TCCR1C = 0;
}

void initIO(void)
{
	MCUCR |= (1<<JTD);
	MCUCR |= (1<<JTD);
    MCUCR |= (1<<JTD); // Desabilita JTAG wr 2x
	// Inicializa portas de IO do processador
	DDRB = (1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<1); // Port B Bits 7 ao 1 como sa�da
	DDRC = (1<<6)|(1<<7); // Port C Bits 7 e 6 como sa�da
	DDRD = (1<<4)|(1<<6)|(1<<7); // Port D Bits 4, 6 e 7 como sa�da
	DDRE = 0; // Port E entrada
	DDRF = 0; // Port F entrada
	
	PORTB = 0; 
	PORTC = 0;
	PORTD = (1<<5)|(1<<1)|(1<<0); // Liga pullup
	PORTE = (1<<2); // Liga pullup
	PORTF = (1<<7)|(1<<6); // Liga pullup

}

int main(void)
{
	char buffer[20];

	initIO();

	Timer0_init();
	
	Adc_init();

	//init LCD
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_puts_p(PSTR("sPWM Generator"));

	init_pwm1C();  // inicia pwm
	OCR1C = calculate_sin_values();  // define valor

	sei(); // interrupcao geral
    
	//TODO - mudança do valor de OCR1C na interrupção do timer1
	
    while(1)
    {
       


     
	    
    }
}

