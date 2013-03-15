#include<avr/io.h>
#include<util/delay.h>
#include <compat/deprecated.h> 
#include "lcd.h"
#define WHITE 1
#define BLACK 0

#define DEBUG 1
#define LINE BLACK		//WHITE : white line , BLACK : black line

unsigned int s1, s2, s3, s4, s5, s6, s7;		//7 SENSORS
int leftpulse, rightpulse; 
float error=0.0, perror=0.0;
unsigned int basespeed = 210, Kp = 45, Ki = 0, Kd = 1170;		//PID CONSTANTS
float P, I1=0, I, D, correction;		//PID CORRECTIONS
char lastreading = 'r';		//LAST SENSOR ACTIVATED BEFORE BOT LEAVES THE LINE

void InitPorts()
{
	DDRB=0XFF;		//MOTORS: PORT B AS OUTPUT
	DDRD=0XFF;		//PWM: PORT D AS OUTPUT
	DDRA=0;			//SENSORS: PORT A AS INPUT 

	PORTB=0;

	PORTD |= (1<<PD5) | (1<<PD4);		//TIMER1 PWM AVAILABLE ON PD4 AND PD5-ATMEGA16
	//PD4 OC1B
	//PD5 OC1A

	//8 BIT PHASE CORRECT PWM, NON INVERTED MODE
	TCCR1A |= (1<<WGM10) | (1<<COM1A1) | (1<<COM1B1);
	//NO PRESCALAR
	TCCR1B |= (1<<CS10);
	
	#ifdef DEBUG
		DDRD &= ~((1<<PD2) | (1<<PD3));		//Set PD2 AND PD3 AS INPUT FOR BUTTONS
		PORTD |= (1<<PD2) | (1<<PD3);		// ENABLE PULL UP RESISTOR
		LCDInit(LS_NONE);
		LCDClear();
	#endif
}

#ifdef DEBUG
	void updateLCD()
	{	
		//int pressnum = 0;
		//UPDATE Kp Kd
		while(bit_is_clear(PIND, PD2))
		{
			/*Kp++;
			if(pressnum == 0)
			{
				_delay_ms(1000);
				pressnum++;
			}
			else
				_delay_ms(50);
			*/
			//DISPLAY Kp Ki Kd values
			LCDWriteStringXY(0,0,"Kp");
			LCDWriteIntXY(2, 0, Kp, 2);
			LCDWriteStringXY(4,0," ");
			/*
			LCDWriteStringXY(5,0,"Ki");
			LCDWriteIntXY(7, 0, Ki, 2);
			LCDWriteStringXY(9,0," ");
			*/	
			LCDWriteStringXY(5,0,"Kd");
			LCDWriteIntXY(7, 0, Kd, 3);
			LCDWriteStringXY(10,0," ");
		}
		//pressnum = 0;

		
		/*if(LINE == WHITE)	
			LCDWriteStringXY(11,0,"W");	
		else
			LCDWriteStringXY(11,0,"B");	*/

		//DISPLAY sensor inputs, 1 for line 0 for no line
		LCDWriteStringXY(0,1,"L");
		LCDWriteIntXY(1, 1, s1, 1);	//1 left
		LCDWriteIntXY(2, 1, s2, 1);	//2
		LCDWriteIntXY(3, 1, s3, 1);	//3
		LCDWriteIntXY(4, 1, s4, 1);	//4
		LCDWriteIntXY(5, 1, s5, 1);	//5
		LCDWriteIntXY(6, 1, s6, 1);	//6
		LCDWriteIntXY(7, 1, s7, 1);	//7 right
		LCDWriteStringXY(8,1," ");
		LCDWriteStringXY(9,1,"E");
	}
#endif

void LeftMF()		//LEFT MOTOR FORWARD
{
	sbi(PORTB,PB1);
	cbi(PORTB, PB0);
}	
void RightMF()		//RIGHT MOTOR FORWARD
{
	sbi(PORTB, PB3);
	cbi(PORTB, PB2);
}
void LeftMS()		//LEFT MOTOR STOP
{
	sbi(PORTB, PB1);
	sbi(PORTB, PB0);
}
void RightMS()		//RIGHT MOTOR STOP
{
	sbi(PORTB, PB3);
	sbi(PORTB, PB2);
}
void LeftMB()		//LEFT MOTOR REVERSE
{
	sbi(PORTB, PB0);
	cbi(PORTB, PB1);
}
void RightMB()		//RIGHT MOTOR REVERSE
{
	sbi(PORTB, PB2);
	cbi(PORTB, PB3);
}


void CalcError()		//CALUCLATE ERROR
{
	
	// Sensor gives logic 1 FOR WHITE, 0 FOR BLACK. CALCULATES ERROR FOR FOLLOWING WHITE LINE
	s1=0;
	s2=0;
	s3=0;
	s4=0;
	s5=0;
	s6=0;
	s7=0;

	#if LINE == WHITE						//For WHITE line tracing
		if(!bit_is_clear(PINA, PA0))		//IF LEFTMOST SENSOR IS ACTIVATED
		{
			s1=1;
			lastreading='l';
		}
		if(!bit_is_clear(PINA, PA1))
		{
			s2=1;
		}
		if(!bit_is_clear(PINA, PA2))		
		{
			s3=1;
		}
		
		if(!bit_is_clear(PINA, PA4))
		{
			s5=1;
		}
		if(!bit_is_clear(PINA, PA5))
		{
			s6=1;
		}
		if(!bit_is_clear(PINA, PA6))		//IF RIGHTMOST SENSOR IS ACTIVATED
		{
			s7=1;
			lastreading='r';
		}
		if(!bit_is_clear(PINA, PA3))		//IF MIDDLE SENSOR IS ACTIVATED
		{
			s4=1;
			//lastreading = 'm';
		}	
	#elif LINE == BLACK					//For BLACK line tracing
		if(bit_is_clear(PINA, PA0))		//IF LEFTMOST SENSOR IS ACTIVATED
		{
			s1=1;
			lastreading='l';
		}
		if(bit_is_clear(PINA, PA1))
		{
			s2=1;
		}
		if(bit_is_clear(PINA, PA2))		
		{
			s3=1;
		}
		
		if(bit_is_clear(PINA, PA4))
		{
			s5=1;
		}
		if(bit_is_clear(PINA, PA5))
		{
			s6=1;
		}
		if(bit_is_clear(PINA, PA6))		//IF RIGHTMOST SENSOR IS ACTIVATED
		{
			s7=1;
			lastreading='r';
		}
		if(bit_is_clear(PINA, PA3))		//IF MIDDLE SENSOR IS ACTIVATED
		{
			s4=1;
			//lastreading = 'm';
		}	
	#endif
			
	if( (s1+s2+s3+s4+s5+s6+s7) != 0 )
	{
		perror=error;
		// CALCULATE ERROR USING WEIGHTED AVERAGE METHOD
		error = (s1 * 1) + (s2 * 2) + (s3 * 3) + (s4 * 4) + (s5 * 5) + (s6*6) + (s7*7);
		error = (error)/(s1+s2+s3+s4+s5+s6+s7);	
		error = error - 4;
	}

}


int main()
{
	InitPorts();
		
	while(1)
	{
		
		CalcError();

		#ifdef DEBUG
			updateLCD();
		#endif

		if((s1+s2+s3+s4+s5+s6+s7) == 0)		//IF NO SENSORS ARE ON THE LINE
		{
			if(lastreading == 'r')				//checks if the last sensor to the activated was right
			{
				OCR1A = 170;
				OCR1B = 170;
			//	LeftMB();
			//	RightMB();
			//	_delay_ms(500);
				RightMB();					//turn right at full speed
				LeftMF();
			}
			else if(lastreading == 'l')			//checks if the last sensor to the activated was left
			{
				OCR1A = 170;
				OCR1B = 170;
			//	LeftMB();
			//	RightMB();
			//	_delay_ms(1000);
				RightMF();						//turn left at full speed
				LeftMB();	
			}
		}
		else if(s1 == 1 &&(s3+s4+s5+s6+s7) == 0)
		{
		
				OCR1A = 170;
				OCR1B = 170;
				RightMF();					//turn right at full speed
				LeftMB();
		}
		else if((s1+s2+s3+s4+s5) == 0 && s7 == 1)
		{
		
				OCR1A = 170;
				OCR1B = 170;
				RightMB();					//turn right at full speed
				LeftMF();
		}
		else  									//IF ROBOT ON LINE
		{
			P = error * Kp;
			I1 += error;				//SUM OF ALL PREVIOUS ERRORS
			I = I1 * Ki;				//cALCULATE THE I TERM
			D = error - perror;
			D = D * Kd;

			correction = P + I + D;
			rightpulse =  basespeed - correction;
			leftpulse = basespeed + correction;				
			
			LeftMF();
			RightMF();
			if(leftpulse > 255)
				leftpulse = 255;
			if(rightpulse > 255)
				rightpulse = 255;
			if(leftpulse < 0)
				leftpulse = 0;
			if(rightpulse < 0)
				rightpulse = 0;
			OCR1A = rightpulse;			//RIGHT MOTOR ON OC1A
			OCR1B = leftpulse;			//LEFT MOTOR ON OC1B
		}
	}
	return 0;
}


// code by Devaj Mitra, feel free to contribute
