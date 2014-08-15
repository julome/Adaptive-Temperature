/********************************************************
	    Temperature Adaptive Predictive Control with Atmega328P	
    Copyright (C) 2014  Juan Lopez Medina

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

* You can contact me by mail : julome21@gmail.com
*********************************************************/

#define F_CPU			16000000UL	// Frequency XTAL 16MHz
#include <avr/io.h>
#include "usart.h"
//#include "util/delay.h"
//#include <stdio.h>
#include "stdlib.h"
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "Adaptive.h"

uint32_t T_SAMPLE = 200;					// Time sample Calculate attitude T = T_SAMPLE * 1ms
uint16_t T_CONTROL = 5;						// CP = T_SAMPLE * T_CONTROL in ms
uint16_t T_CNT;								// Number count temp1.  1 Count Temp1 64us => T_sample = Value_CNT1 = ms/64us 

//#define T_SAMPLE		100					// Time sample Calculate attitude T = T_SAMPLE * 1ms (max 16 ms).
//#define T_CONTROL		10					// CP = T_SAMPLE * T_CONTROL in ms
//#define T_CNT			T_SAMPLE * 1000L/64	// Number count temp1.  1 Count Temp1 64us => T_sample = Value_CNT1 = ms/64us 


// Define functions
void adc_Setup(void);
unsigned int adc_read(char channel);	
int16_t lpf(int16_t pv_sf, int16_t pv_fil_ant, double kf);
void timer1_init();
void timer2_init();						// PWM FAN control
void timer0_init();						// System timer in ms 
void clearbuffer1(void);					// Clear buffer for Get char
void clearbuffer2(void);				// Reset index vector Get char USART and Reset command

volatile unsigned long millis = 0;		// Count milliseconds
//float EEMEM eeprom_float = 10.5;
//int EEMEM eeprom_int;

volatile uint8_t m = 0;					// Index vector Get char USART
volatile char data[15];					// Vector buffer for Get char
volatile uint8_t command;
int16_t sp_temp_get = 0;

int main(void)
{
	int temp = 0;					// Read temperature
	//int sp_temp_get;
	float temp_real = 0;			// Real format temperature		
	//int t_sample = 100;				// Time period samples read ms
	//int t_control = 1000;			// Time period control system ms
	
	uint16_t t_sample = 0;				// Time period samples read ms
	int t_control = 0;			// Time period control system ms
	
	//long millis_ant1;				// Aux timer
	//long millis_ant2;				// Aux timer
	uint16_t out_pwm;				// Out PWM control
	double y_temp[PmA+2] = {0};								// Array out incremental y(k), y(k-1), y(k-2), y(k-3)
	double u_temp[PmB+3] = {0};								// Array process input incremental u(k), u(k-1), u(k-2), u(k-3), u(k-4)
	double t_temp[5] = {1.0, -0.3, 0.1, 0.1, 0.1};			// Array parameters adaptive mechanism a1k, a2k, b1k, b2k, b3k		
	double sp_temp[2] = {0};								// Set point process sp(k)
	double yp_temp[2] = {0};								// Array process out yp(k)		
	
	cli();							// Disable interrupts
	conductor_block();				// Calculate parameters conductor block
	usart_init();					// Initialize USART
	UCSR0B |= (1 << RXCIE0);		// Enable interrupt RX (If enabled do not use functions get_xx)
	adc_Setup();					// Initialize ADC
	timer1_init();					// Timer system without interrupts
	//timer0_init();				// Initialize timer0 system ms
	timer2_init();					// Initialize timer2 PWM
	sei();
	
	DDRD |= (1 << PIND3);			// Out PWM OC2B PIND3 Digital PIN 3
	
	temp = adc_read(0);				// Initialize temperature readers
	//millis_ant1 = millis;			// Initialize period samples
	//millis_ant2 = millis;			// Initialize period control	
	
	//eeprom_update_float(&eeprom_float,f);
	
	//float eeprom_float_read;
	
	//eeprom_float_read = eeprom_read_float(&eeprom_float);
	//put_float(eeprom_float_read);

	
	//put_string("\n Input SP temperature: ");
	//sp_temp_get = get_int();
	
    while(1)
    {		
        
		// Samples read
		//if ((millis - millis_ant1) >= t_sample){
			//millis_ant1 = millis;
		t_sample = TCNT1;									// Catch sample time for integer angle gyro		
		T_CNT = T_SAMPLE * 1000L/64;						// Number count temp1.  1 Count Temp1 64us => T_sample = Value_CNT1 = ms/64us 
		if (t_sample >= T_CNT){								// Attitude calculates Read IMU (Accel and Gyro)
			TCNT1 = 0;										// Restart sample time
			t_control++;									// Increment time Period control
			temp = lpf(adc_read(0), temp, 0.1);						
		}
		
		//if ((millis - millis_ant2) >= t_control){		
			//millis_ant2 = millis;
			
		if (t_control >= T_CONTROL){						//Control action balancer
			t_control = 0;	
			temp_real = temp * 110.0 / 1023.0;	// solo en el control
	
			//Adaptive predictive balancer process			
			sp_temp[0] = sp_temp_get;											// Set point
			yp_temp[0] = temp_real;												// Process out y(k).
									
			adaptive(sp_temp, t_temp, y_temp, u_temp, yp_temp, UP_PWM);	// Call adaptive function
			out_pwm = u_temp[0];									// Out Controller adaptive
			if (out_pwm > UP_PWM) out_pwm = UP_PWM;							// Upper limit out
				else if (out_pwm < 0) out_pwm = 0;
			//out_pwm = 0;														// Off control			
			OCR2B = 255 - out_pwm;												// Out PWM
			
			put_float(temp_real);
			put_string(" ");
			put_int(OCR2B);
			put_string(" ");
			put_int(sp_temp_get);
			//put_string(" ");
			//put_float(NL);
			//put_string(" ");
			//put_float(GainA);
			//put_string(" ");
			//put_float(GainB);
			//put_string(" ");
			//put_int(T_SAMPLE);
			//put_string(" ");
			//put_int(T_CONTROL);			
			put_string("\n");
			
		}		
		//TODO:: Please write your application code    
    }
}

void adc_Setup(){
	DIDR0 = 0x3F;									// Disable input digital in ADC PINS
	ADMUX = 0x00;									// Reset ADMUX
	ADMUX |= (1 << REFS1) | (0 << REFS0);			// Select reference internal +Vref = 1.1 V
	ADCSRA |= (1 << ADEN) | (3 << ADPS0);			// Enable ADC and Prescaler at 128 => 125KHz	
}

unsigned int adc_read(char channel){	
	ADMUX &= 0xF8;					// Reset channel
	ADMUX |= channel;				// Select channel	
	ADCSRA |= (1 << ADSC);			// Start conversion	
	while(ADCSRA & (1 << ADSC));    // While for conversion done	
	return ADC;						// Return conversion result
}

//Low Pass Filter (kf = 0.0 - 1.0)
int16_t lpf(int16_t pv_sf, int16_t pv_fil_ant, double kf){
	double pv_fil;
	pv_fil = (kf * (double)pv_sf + (1.0 - kf) * (double)pv_fil_ant);		//pv_fil = kf * pv_sf + (1 - kf) * pv_fil_ant;
	return (int16_t)pv_fil;
}
/*
// Routine initialize ICP timer1 for catch Signal FAN RPM
void timer1_init(void){
	TCCR1A = 0;									// Normal Mode
	TCCR1B |= (3 << CS10);						// Timer prescaler = 64 (tick = 4us)
	TCCR1B |= (1 << ICNC1) | (1 << ICES1);						// Noise Canceler enabled Falling edge enabled ICP1 D_PIN = 8
	TIMSK1 |= (1 << ICIE1);						// Active interrupt ICP1
}

// Interrupt Input Capture Pulse for catch RPM
ISR(TIMER1_CAPT_vect){
	poles++;
	if (poles == 5){					// Five interrupts one fan revolution
		time_cycle = ICR1;				// Save time cycle in us/revolution			
		TCNT1 = 0;						// Reset counter
		poles = 0;						// Reset interrupt counter
	}
}
*/
// Initialize Timer2 PWM Fast Mode for control PWM FAN
void timer2_init(){
	//Fast PWM Mode Fpwm = F_cpu / (N * (End + 1))
	TCCR2B = (2 << CS20);			// Configuration prescaler to 8	
	TCCR2A = (2 << COM2B0);			// No inverter mode PIN OC2B	
	// Enable FAST PWM Mode
	TCCR2A |= (1 << WGM20) | (1 << WGM21);	
	OCR2B = 60;					// Initialize 	
	return;
}

void timer0_init(){		// Initialize timer0 for system timer (T = 1ms)	
	TCCR0A = (1 << WGM01);
	TCCR0B = (3 << CS00);					// Prescaler = 64 CTC Mode
	OCR0A = 249;							// Config end count for T = 1ms
	TIMSK0 = (1 << OCIE0A);					// Enable CTC interrupt
}
/*
// Interrupt CTC TIMER0 for system timer in ms
ISR(TIMER0_COMPA_vect){
	millis++;		// Increment count ms
}	
*/
// Timer count for samples. 1 count = 64us. Max = 256 * 64us = 16ms
void timer1_init(){
	TCCR1A = 0;						// Normal Mode
	TCCR1B = (5 << CS10);			// Prescaler = 1024 Normal Mode
}

ISR(USART_RX_vect){			
	data[m++] = UDR0;
	//UDR0 = data[m-1];												// Get char
	if ((data[0] == 'S') & (data[1] == 'P')) {command = 1;			// Command for Set point
		clearbuffer1();}						
	if ((data[0] == 'N') & (data[1] == 'L')) {command = 2;			// Command for Noise Level
		clearbuffer1();}
	if ((data[0] == 'G') & (data[1] == 'A')) {command = 3;			// Command for GainA
		clearbuffer1();}
	if ((data[0] == 'G') & (data[1] == 'B')) {command = 4;			// Command for GainB
		clearbuffer1();}
	if ((data[0] == 'T') & (data[1] == 'S')) {command = 5;			// Command for T_SAMPLE
		clearbuffer1();}
	if ((data[0] == 'T') & (data[1] == 'C')) {command = 6;			// Command for T_CONTROL
		clearbuffer1();}
	if (command == 0 & m > 1) clearbuffer1();						// Reset if wrong command													
	if (command == 1 & data[m-1] == '\n' & atoi(data) != 0) {sp_temp_get = atoi(data);	// Get Data SP																			
		clearbuffer2();}
	if (command == 2 & data[m-1] == '\n' & atof(data) != 0.0) {NL = atof(data);			// Get Data NL			
		clearbuffer2();}	
	if (command == 3 & data[m-1] == '\n' & atof(data) != 0.0) {GainA = atof(data);		// Get Data GainA						
		clearbuffer2();}			
	if (command == 4 & data[m-1] == '\n' & atof(data) != 0.0) {GainB = atof(data);		// Get Data GainB							
		clearbuffer2();}		
	if (command == 5 & data[m-1] == '\n' & atoi(data) != 0) {T_SAMPLE = atoi(data);		// Get Data T_SAMPLE						
		clearbuffer2();}
	if (command == 6 & data[m-1] == '\n' & atoi(data) != 0) {T_CONTROL = atoi(data);	// Get Data T_CONTROL			
		clearbuffer2();}
}

void clearbuffer1(void){			// Reset index vector Get char USART
	m = 0;								
	for (int i = 0; i < 16; i++) data[i] = 0;	
}

void clearbuffer2(void){			// Reset index vector Get char USART and Reset command
	m = 0;
	for (int i = 0; i < 16; i++) data[i] = 0;
	command = 0;
}

