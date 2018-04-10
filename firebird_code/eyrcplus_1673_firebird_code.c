/* 
 * Team ID: e-YRC+ #1673
 * Author List: Heethesh Vhavle
 *
 * Filename: eyrcplus_1673_firebird_code.c
 * Theme: eYRC-Plus
 * 
 * Global Variables: data, ShaftCountLeft, ShaftCountRight, Degrees
 */

#define F_CPU 14745600
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

//Global variables
unsigned char data = 0; //to store received data from UDR0

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning

//Function to initialise buzzer
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to initialise LEDs
void led_pin_config (void)
{
 DDRJ = DDRJ | 0xFF;		
 PORTJ = PORTJ & 0x00;		
}

//Function to initialise motors
void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

//Function to initialize ports
void port_init()
{
	motion_pin_config();
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	led_pin_config();
	buzzer_pin_config();
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

//LED 1
void led1_red()
{
 unsigned char port_restore = 0;
 port_restore = PINJ;
 port_restore = port_restore | 0x04;
 PORTJ = port_restore;
}

void led1_yel()
{
 unsigned char port_restore = 0;
 port_restore = PINJ;
 port_restore = port_restore | 0x0C;
 PORTJ = port_restore;
}

void led1_blu()
{
 unsigned char port_restore = 0;
 port_restore = PINJ;
 port_restore = port_restore | 0x10;
 PORTJ = port_restore;
}

void led1_off()
{
 PORTJ &= 0xE0;
}

//LED 2
void led2_red()
{
 unsigned char port_restore = 0;
 port_restore = PINJ;
 port_restore = port_restore | 0x20;
 PORTJ = port_restore;
}

void led2_yel()
{
 unsigned char port_restore = 0;
 port_restore = PINJ;
 port_restore = port_restore | 0x60;
 PORTJ = port_restore;
}

void led2_blu()
{
 unsigned char port_restore = 0;
 port_restore = PINJ;
 port_restore = port_restore | 0x80;
 PORTJ = port_restore;
}

void led2_off()
{
 PORTJ &= 0x1C;
}

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}


//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

//Function to initialize all the devices
void init_devices()
{
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the ports
	timer5_init();
	uart0_init(); //Initailize UART1 for serial communiaction
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   // Enables the global interrupt
}

SIGNAL(SIG_USART0_RECV)
{
	data = UDR0;
}

void xbee_write(unsigned char data_send)
{	
   UDR0 = data_send;
}

//Function to check all LEDs
void check_leds()
{	
	_delay_ms(50);
	led1_red();
	led2_red();
	_delay_ms(100);
	led1_off();
	led2_off();
	
	_delay_ms(50);
	led1_yel();
	led2_yel();
	_delay_ms(100);
	led1_off();
	led2_off();

	_delay_ms(50);
	led1_blu();
	led2_blu();
	_delay_ms(100);
	led1_off();
	led2_off();
}

//Main Function
int main(void)
{	
	init_devices();

	unsigned int distance;
	unsigned int distanceINT;
	unsigned int rotationINT;
	unsigned int repetition;

	check_leds();
	
	// SYNC COMPUTER AND ROBOT
	while(data != 255)
	{
		xbee_write(0x30);
		_delay_ms(500);
	}
	
	xbee_write(0x31);

	while(1)
	{		
			velocity(250,255);
			
			// FORWARD
			if(data>100 && data<125)
			{	
			
				velocity(255,253);
				if(data != 121)
				{
					repetition = data-100;
					distance = 48*repetition;
				}

				else distance = 24;
		
				forward_mm(distance); //Moves robot forward
				stop();

				data = 0x00;
				_delay_ms(100);
				xbee_write(0x31);
			}
			
			// FEEDBACK FORWARD
			if(data>=130 && data<150)
			{	
				velocity(255,253);
				
				distanceINT = (unsigned int)data;
				distance = ((distanceINT-130)+3)*4;
				forward_mm(distance); //Moves robot forward
				stop();

				data = 0x00;
				_delay_ms(100);
				xbee_write(0x31);
			}
			
			// BACKWARD
			if(data>200 && data<225)
			{	
				if(data != 221)
				{
					repetition = data-200;
					distance = 48*repetition;
				}

				else distance = 24;

				back_mm(distance);   //Moves robot backward
				stop();

				data = 0x00;
				_delay_ms(100);
				xbee_write(0x31);				
			}

			// FEEDBACK BACKWARD
			if(data>=230 && data<250)
			{	
				velocity(255,253);
				
				distanceINT = (unsigned int)data;
				distance = ((distanceINT-230)+3)*4;
				back_mm(distance); //Moves robot backward
				stop();

				data = 0x00;
				_delay_ms(100);
				xbee_write(0x31);
			}
			
			// LEFT TURN
			if(data == 2)
			{	
				velocity(255,255);
				left_degrees(95); //Rotate robot left by 90 degrees
				stop();

				data = 0x00;
				xbee_write(0x32);
				_delay_ms(500);				
			}
			
			// RIGHT TURN
			if(data == 4) //ASCII value of 4
			{
				right_degrees(91); //Rotate robot right by 90 degrees
				stop();

				data = 0x00;
				xbee_write(0x32);
				_delay_ms(500);				
			}
			
			// FEEDBACK LEFT
			if(data>=30 && data<=46)
			{	
				if(data != 30)
				{
					if(data>=31 && data<=37)
					{
						rotationINT = (unsigned int)data;
						rotationINT = ((rotationINT-30)*2)+4;
						left_degrees(rotationINT);
						stop();
					}

					else if(data>=38 && data<=46)
					{
						rotationINT = (unsigned int)data;
						rotationINT = ((rotationINT-38)*20)+20;
						left_degrees(rotationINT);
						stop();
					} 
					

					if(data == 46)
					{
						xbee_write(0x32);
						_delay_ms(500);
					}
					else 
					{
						xbee_write(0x31);
						_delay_ms(100);
					}
					data = 0x00;
					
				}
				
				else 
				{
					xbee_write(0x31);					
					data = 0x00;
				}
			}

			// FEEDBACK RIGHT
			if(data>=60 && data<=76)
			{
				if(data != 60)
				{
					if(data>=61 && data<=67)
					{
						rotationINT = (unsigned int)data;
						rotationINT = ((rotationINT-60)*2)+4;
						right_degrees(rotationINT);
						stop();
					}

					else if(data>=68 && data<=76)
					{
						rotationINT = (unsigned int)data;
						rotationINT = ((rotationINT-68)*20)+20;
						right_degrees(rotationINT);
						stop();
					} 

					if (data == 76)
					{
						xbee_write(0x32);
						_delay_ms(500);
					}
					else 
					{
						xbee_write(0x31);
						_delay_ms(100);
					}
					data = 0x00;
				}
				
				else 
				{
					xbee_write(0x31);
					data = 0x00;
				}
			}
			
			// SEND FEEDBACK COMMAND
			if(data == 100)
			{	
				data = 0x00;						
				xbee_write(0x32);
				_delay_ms(500);
			}

			if(data == 99)
			{	
				data = 0x00;						
				xbee_write(0x33);
				_delay_ms(500);
			}

			if(data == 98)
			{	
				data = 0x00;						
				xbee_write(0x34);
				_delay_ms(500);
			}
			
			// STOP	
			if(data == 0)
			{
				stop();				
			}
			
			// LED 1
			if(data == 0x0B)
			{	
				led1_red();
				data = 0x00;						
				xbee_write(0x31);
			}

			else if(data == 0x0C)
			{	
				led1_yel();
				data = 0x00;					
				xbee_write(0x31);
			}

			else if(data == 0x0D)
			{
				led1_blu();
				data = 0x00;
				xbee_write(0x31);
			}
			
			else if(data == 0x0A)
			{;
				led1_off();
				data = 0x00;					
				xbee_write(0x31);
			}

			// LED 2
			if(data == 0x15)
			{
				led2_red();
				data = 0x00;						
				xbee_write(0x31);
			}

			else if(data == 0x16)
			{
				led2_yel();
				data = 0x00;					
				xbee_write(0x31);
			}

			else if(data == 0x17)
			{
				led2_blu();
				data = 0x00;						
				xbee_write(0x31);
			}

			else if(data == 0x14)
			{
				led2_off();
				data = 0x00;						
				xbee_write(0x31);
			}
			
			// LONG BUZZER
			if(data == 5)
			{
				buzzer_on();
				_delay_ms(6000);
				buzzer_off();

				data = 0x00;					
				xbee_write(0x31);
			}
			
			// SHORT BUZZER
			if(data == 6)
			{
				buzzer_on();
				_delay_ms(500);
				buzzer_off();

				_delay_ms(2000);
				data = 0x00;						
				xbee_write(0x31);
			}
	}
}
