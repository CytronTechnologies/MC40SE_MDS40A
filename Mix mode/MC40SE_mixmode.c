/*******************************************************************************
* This is the main program to sanity test the MC40SE Controller using PIC16F887
* This sample code is compiled under MPLAB IDE v8.63 with Hi-Tech C Pro Compiler (Lite Mode) V9.80
* Author: Ober Choo Sui Hong @ Cytron Technologies Sdn. Bhd.
* Date: 20 Jan 2012
*******************************************************************************/
#include <htc.h>
#include "system.h"
#include "timer1.h"
#include "uart.h"
#include "adc.h"
#include "pwm.h"
#include "lcd.h"
#include "skps.h"

/*******************************************************************************
* DEVICE CONFIGURATION WORDS FOR PIC16F887                                     *
*******************************************************************************/

__CONFIG( //INTIO &		// Internal Clock, pins as I/O.
		 HS &			// External Crystal at High Speed
		 WDTDIS &		// Disable Watchdog Timer.
		 PWRTEN &		// Enable Power Up Timer.
		 IESOEN &			// Enabled Internal External Clock Switch Over
		 //IESODIS &		// Disabled Internal External Clock Switch Over
		 BORDIS &		// Disable Brown Out Reset.
		 //FCMDIS	&		// Disable monitor clock fail safe
		 FCMEN &		// Enable monitor clock fail safe
		 MCLREN &		// MCLR function is enabled
		 LVPDIS);		// Disable Low Voltage Programming.

// For details connection, please refer to schematic of MC40SE
//================================== MC40SE PIN ASSIGNMENT ===========================
/*;PIN Des 	func			PIN Des 	func
;1	MCLR	Reset button	40	RB7		LED/BUZZER
;2	RA0		ADC				39	RB6		GPIO
;3	RA1		SEN7			38	RB5		SEN6
;4	RA2		SW1/GPIO 		37	RB4		SEN5
;5	RA3		SW2/GPIO		36	RB3		SEN4
;6	RA4		GPIO			35	RB2		SEN3
;7	RA5		SEN8			34	RB1		SEN2
;8	RE0		RUN1			33	RB0		SEN1
;9	RE1		DIR1			32	VDD		5V
;10	RE2		LCD_E			31	VSS		GND
;11	VDD		5V				30	RD7		LCD_DB7
;12	VSS		GND				29	RD6		LCD_DB6
;13	OSC1	BL_R/Crystal 	28	RD5		LCD_DB5
;14	OSC2	SK_R/Crystal 	27	RD4		LCD_DB4
;15	RC0		ENCODER			26	RC7		RX
;16	RC1		PWM1			25	RC6		TX
;17	RC2		PWM2			24	RC5		DIR2
;18	RC3		LATCH			23	RC4		RUN2
;19	RD0		Relay 1/LCD_RS	22	RD3		Relay 4
;20	RD1		Relay 2			21	RD2		Relay 3
*/
/*******************************************************************************
* PRIVATE CONSTANT DEFINE                                                  *
*******************************************************************************/
#define	PORT1 	1
#define	PORT2	2
#define	RUN		0
#define	BRAKE	1
#define	CW		0
#define	CCW		1


/*******************************************************************************
* PRIVATE FUNCTION PROTOTYPES                                                  *
*******************************************************************************/

void delay_ms(unsigned int ui_value);
void beep(unsigned char uc_count);
void mc40se_init(void);

void brushless(unsigned char uc_port_number, unsigned char uc_motor_status, unsigned char uc_motor_dir, unsigned int ui_speed);
void brush(unsigned char uc_port_number, unsigned char uc_motor_status, unsigned char uc_motor_dir, unsigned int ui_speed);
void relay_on(unsigned char uc_relay_number);
void relay_off(unsigned char uc_relay_number);
void relay_off_all(void);


/*******************************************************************************
* Global Variables                                                             *
*******************************************************************************/
char string1[] = "Passed!";
int i,j,k,l;
/*******************************************************************************
* MAIN FUNCTION                                                                *
*******************************************************************************/
int main(void)
{
	unsigned char temp = 0;	//declare local variable here

	// Initialize PIC16F887 to correct Input/Output based on MC40SE on board interface
	mc40se_init();	
	
	// Initialize PWM. //must set first to prevent motor to run
	pwm_init();
	set_pwm1(512);
	set_pwm2(512);

	// Initialize ADC.
	adc_init();
	
	// off all relays
	relay_off_all();
	
	// Initialize UART.
	uart_init();

	// Initialize Timer 1 for encoder.
	timer1_init();
	
	// Initialize the LCD.
	lcd_init();		

	// Initialize bruhsless motor port
//	brushless(PORT1, BRAKE, CW, 0);
//	rushless(PORT2, BRAKE, CCW, 0);
		
	// Display the messages and beep twice.
	lcd_clear_msg(" Cytron \n  Tech");
	beep(2);
////	delay_ms(1000);	
//	while(SW1 == 1);	
//	set_pwm1(300);
//	set_pwm2(700);
////	delay_ms(1000);
//
//	while(SW2 == 1);
//	set_pwm1(512);
//	set_pwm2(512);
	// ### Start Your Program Here ###
	
	// Here is a sample program with that wait for SW1 to be press
	// start LED blinking		
	lcd_clear_msg(" PRESS \n START");
	while(uc_skps(3) == 1) continue;	//wait for SW1 to be press
	lcd_clear_msg(" use \nJoyLJoyR");
	i=0;

	while (1) 
	{
		if(uc_skps(19)<50)   //control speed using joystik 2
		{	lcd_clear_msg("+++ ");
			while(uc_skps(19)<50);
			if(i==500)
			{i=500;}
			else
			{i=i+100;}
		}
		else if(uc_skps(19)>200)
		{	lcd_clear_msg("--- ");
			while(uc_skps(19)>200);
			if(i==0)
			{i=0;}
			else
			{i=i-100;}
		}
		else if(uc_skps(4)==0)
		{lcd_clear_msg("forw ");							//forward
		set_pwm1(512);
		set_pwm2(512+i);										
		}
		else if(uc_skps(6)==0)
		{lcd_clear_msg("down");								//reverse
		set_pwm1(512);
		set_pwm2(512-i);										
		}

		else if(uc_skps(5)==0)
		{lcd_clear_msg("right ");							//right
		set_pwm1(512+i/2);
		set_pwm2(512+i);										
		}

		else if(uc_skps(7)==0)
		{lcd_clear_msg("left");								//left
		set_pwm1(512-i/2);
		set_pwm2(512+i);										
		}

		else  												//stop	
		{lcd_clear_msg("stop ");
		set_pwm1(512);
		set_pwm2(512);
		}	
		
	} 
	
	while(1);	// infinite loop
}



/*******************************************************************************
* PRIVATE FUNCTION: delay_ms
*
* PARAMETERS:
* ~ ui_value	- The period for the delay in miliseconds.
*
* RETURN:
* ~ void
*
* DESCRIPTIONS:
* Delay in miliseconds.
*
*******************************************************************************/
void delay_ms(unsigned int ui_value)
{
	while (ui_value-- > 0) {
		__delay_ms(1);
	}	
}


/*******************************************************************************
* PRIVATE FUNCTION: beep
*
* PARAMETERS:
* ~ uc_count	- How many times we want to beep.
*
* RETURN:
* ~ void
*
* DESCRIPTIONS:
* Beep for the specified number of times.
*
*******************************************************************************/
void beep(unsigned char uc_count)
{
	while (uc_count-- > 0) {
		BUZZER = 1;
		delay_ms(50);
		BUZZER = 0;
		delay_ms(50);
	}
}
/*******************************************************************************
* PRIVATE FUNCTION: mc40se_init
*
* PARAMETERS:
* ~ void
*
* RETURN:
* ~ void
*
* DESCRIPTIONS:
* Initialize the PIC16F887 for MC40SE on board device.
*
*******************************************************************************/
void mc40se_init(void)
{
	// Initialize the Internal Osc, under OSCCON register
	IRCF2 = 1;		// IRCF<2:0> = 111 => 8MHz
	IRCF1 = 1;		// IRCF<2:0> = 110 => 4MHz, default
	IRCF0 = 1;		// IRCF<2:0> = 101 => 2MHz, 
		
	// clear port value
	PORTA = 0;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;
	PORTE = 0;
	
	// Initialize the I/O port direction.
	TRISA = 0b10111111;
	TRISB = 0b00111111;
	TRISC = 0b10000001;
	TRISD = 0;
	TRISE = 0b00000000;	
}

/*******************************************************************************
* PRIVATE FUNCTION: brushless
*
* PARAMETERS:
* ~ unsigned char uc_port_number - refer to brushless port in MC40SE, PORT1 or PORT2
* ~ unsigned char uc_motor_status - refer to either RUN or BRAKE brushless motor
* ~ unsigned char uc_motor_dir - to control the direction of brushless motor, CW or CCW
* ~ unsigned int ui_speed - speed for brushless motor, from 0 to 1024
*
* RETURN:
* ~ void
*
* DESCRIPTIONS:
* control brushless motor, suitable for Vexta or LINIX brushless motor connect to MC40SE
*
*******************************************************************************/
void brushless(unsigned char uc_port_number, unsigned char uc_motor_status, unsigned char uc_motor_dir, unsigned int ui_speed)
{
	if(uc_port_number == PORT1)
	{
		if(uc_motor_status == RUN)
		{		
			RUN1 = 0;	// active low		
		}
		else if(uc_motor_status == BRAKE)
		{
			RUN1 = 1;	// activate low				
		}
		
		if(uc_motor_dir == CW)
		{
			DIR1 = 1;
		}
		else if (uc_motor_dir == CCW)
		{
			DIR1 = 0;
		}
		set_pwm1(ui_speed);
	}
	else if(uc_port_number == PORT2)
	{
		if(uc_motor_status == RUN)
		{		
			RUN2 = 0;	// active low		
		}
		else if(uc_motor_status == BRAKE)
		{
			RUN2 = 1;	// activate low				
		}
		
		if(uc_motor_dir == CW)
		{
			DIR2 = 1;
		}
		else if (uc_motor_dir == CCW)
		{
			DIR2 = 0;
		}
		set_pwm2(ui_speed);
	}
}	

/*******************************************************************************
* PRIVATE FUNCTION: brush
*
* PARAMETERS:
* ~ unsigned char uc_port_number - refer to brush port in MC40SE, PORT1 or PORT2
* ~ unsigned char uc_motor_status - refer to either RUN or BRAKE brush motor
* ~ unsigned char uc_motor_dir - to control the direction of brush motor, CW or CCW
* ~ unsigned int ui_speed - speed for brush motor, from 0 to 1024
*
* RETURN:
* ~ void
*
* DESCRIPTIONS:
* control brush motor, require MD10X or MD30X as motor driver
*
*******************************************************************************/
void brush(unsigned char uc_port_number, unsigned char uc_motor_status, unsigned char uc_motor_dir, unsigned int ui_speed)
{
	if(uc_port_number == PORT1)
	{
		if(uc_motor_status == RUN)
		{
			if(uc_motor_dir == CW)
			{
				RUN1 = 0;
				DIR1 = 1;
			}
			else if (uc_motor_dir == CCW)
			{
				RUN1 = 1;
				DIR1 = 0;
			}			
		}
		else if(uc_motor_status == BRAKE)
		{
			RUN1 = 0;	// brake to gnd
			DIR1 = 0;	
		}
		set_pwm1(ui_speed);
	}
	else if(uc_port_number == PORT2)
	{
		if(uc_motor_status == RUN)
		{
			if(uc_motor_dir == CW)
			{
				RUN2 = 0;
				DIR2 = 1;
			}
			else if (uc_motor_dir == CCW)
			{
				RUN2 = 1;
				DIR2 = 0;
			}			
		}
		else if(uc_motor_status == BRAKE)
		{
			RUN2 = 0;	// brake to gnd
			DIR2 = 0;	
		}
		set_pwm2(ui_speed);
	}	
}	
/*******************************************************************************
* PRIVATE FUNCTION: relay_on
*
* PARAMETERS:
* ~ unsigned char uc_relay_number - relay that needed to be activate, from 1-8
*
* RETURN:
* ~ void
*
* DESCRIPTIONS:
* activate relay.
*
*******************************************************************************/
void relay_on(unsigned char uc_relay_number)
{
	LATCH = 0;	// hold the state of latch
	PORTD = PORTD | (0b00000001<< (uc_relay_number - 1));
	LATCH = 1;	// transfer the new output to relay
	delay_ms(1);
	LATCH = 0;	// hold the output of latch
}	
/*******************************************************************************
* PRIVATE FUNCTION: relay_off
*
* PARAMETERS:
* ~ unsigned char uc_relay_number - relay that needed to be deactivate, from 1-8
*
* RETURN:
* ~ void
*
* DESCRIPTIONS:
* Deactivate relay
*
*******************************************************************************/
void relay_off(unsigned char uc_relay_number)
{
	LATCH = 0;	// hold the state of latch
	PORTD = PORTD & (~(0b00000001<< (uc_relay_number -1)));
	LATCH = 1;	// transfer the new output to relay
	delay_ms(1);
	LATCH = 0;	// hold the output of latch
}	
/*******************************************************************************
* PRIVATE FUNCTION: relay_off_all
*
* PARAMETERS:
* ~ void
*
* RETURN:
* ~ void
*
* DESCRIPTIONS:
* deactivate all relays.
*
*******************************************************************************/
void relay_off_all(void)
{
	LATCH = 0;	// hold the output of latch
	PORTD = 0; 	// Off all relays
	LATCH = 1;	// enable latch to read input and transfer to output
	delay_ms(5);
	LATCH = 0;	// hold the output of latch
}


