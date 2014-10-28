/*
 * GccApplication1.c
 *
 * Created: 7/8/2013 4:44:35 PM
 *  Author: Milad
 */

#include <asf.h>
#include <avr/io.h>
#include <stdio.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include "Initialize.h"
#define _Freq (1.0/(2.0*3.14*2.0))
#define setpoint ((M.RPM_setpointA<<8)& 0xff00) | (M.RPM_setpointB & 0x0ff)//1500//

unsigned char adc;
float adc_I,adc_I_1;
int flg_ask=0,current_ov=0;
int t,t_10ms=0;

char slave_address=0;
char send_buff;
char str[100];
int hall_flag=0,hall_dir=0;
uint16_t counter=0,count=0;;
int PWM;
int usart_change;
int Transmission_Data_1,Transmission_Data_2,Transmission_Data_3,Transmission_Data_4;
int Motor_Free;
float ctrl_time=0.001;//0.020;
int tmp_setpoint,tmp_rpmA,tmp_rpmB;
unsigned char pck_num = 0;
float RPM,kp,ki,kd;
uint8_t Motor_Direction;
char test_driver=0b11;
struct Motor_Param 
{
	int Encoder;
	signed long Err,d,d_last,i,p;
	float kp,ki,kd;
	int Direction;
	int RPM;
	int RPM_last;
	int PWM;
	int HSpeed;
	int8_t RPM_setpointB;
	int8_t RPM_setpointA;
	signed int Setpoint , Setpoint_last ;
	int PID , PID_last , PID_Err , PID_Err_last ;
	int Feed_Back,Feed_Back_last;
	int psin;
}M;

#define ADC_VREF_TYPE 0x00

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
	ADMUX=adc_input | (ADC_VREF_TYPE & 0xff);
	// Delay needed for the stabilization of the ADC input voltage
	_delay_us(10);
	// Start the AD conversion
	ADCSRA|=0x40;
	// Wait for the AD conversion to complete
	while ((ADCSRA & 0x10)==0);
	ADCSRA|=0x10;
	return ADCW;
}

int main(void)
{  
	M.PWM=120;
	slave_address=ADD0|(ADD1<<1);
// Input/Output Ports initialization
// Port B initialization
// Func7=In Func6=In Func5=Out Func4=Out Func3=Out Func2=Out Func1=Out Func0=Out
// State7=T State6=T State5=0 State4=0 State3=0 State2=0 State1=0 State0=0
PORTB=0x00;
DDRB=0x3F;

// Port C initialization
// Func6=In Func5=Out Func4=Out Func3=In Func2=In Func1=In Func0=In
// State6=T State5=0 State4=0 State3=T State2=T State1=T State0=T
PORTC=0x00;
DDRC=0x30;

// Port D initialization
// Func7=In Func6=Out Func5=In Func4=Out Func3=Out Func2=In Func1=Out Func0=In
// State7=T State6=0 State5=T State4=0 State3=0 State2=T State1=0 State0=T
PORTD=0x00;
DDRD=0x5A;

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 8000.000 kHz
// Mode: Fast PWM top=0xFF
// OC0A output: Disconnected
// OC0B output: Disconnected
TCCR0A=0x03;
TCCR0B=0x01;
TCNT0=0x00;
OCR0A=0x00;
OCR0B=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 1000.000 kHz
// Mode: Normal top=0xFFFF
// OC1A output: Discon.
// OC1B output: Discon.
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: On
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=0x00;
TCCR1B=0x02;
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 8000.000 kHz
// Mode: Fast PWM top=0xFF
// OC2A output: Disconnected
// OC2B output: Disconnected
ASSR=0x00;
TCCR2A=0x03;
TCCR2B=0x01;
TCNT2=0x00;
OCR2A=0x00;
OCR2B=0x00;

// External Interrupt(s) initialization
// INT0: On
// INT0 Mode: Rising Edge
// INT1: Off
// Interrupt on any change on pins PCINT0-7: Off
// Interrupt on any change on pins PCINT8-14: Off
// Interrupt on any change on pins PCINT16-23: On
EICRA=0x03;
EIMSK=0x01;
EIFR=0x01;
PCICR=0x04;
PCMSK2=0xA4;
PCIFR=0x04;

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=0x00;

// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=0x01;

// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=0x00;

//USART initialization
//Communication Parameters: 8 Data, 1 Stop, No Parity
//USART Receiver: On
//USART Transmitter: On
//USART0 Mode: Asynchronous
//USART Baud Rate: 9600
UCSR0A=0x00;
UCSR0B=0x98;
UCSR0C=0x06;
UBRR0H=0x00;
UBRR0L=0x33;

// Analog Comparator initialization
// Analog Comparator: Off
// Analog Comparator Input Capture by Timer/Counter 1: Off
ACSR=0x80;
ADCSRB=0x00;
DIDR1=0x00;

//// ADC initialization
//// ADC disabled
//ADCSRA=0x00;

// ADC initialization
// ADC Clock frequency: 62.500 kHz
// ADC Voltage Reference: AREF pin
// ADC Auto Trigger Source: ADC Stopped
// Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On
// ADC4: On, ADC5: On
DIDR0=0x00;
ADMUX=ADC_VREF_TYPE & 0xff;
ADCSRA=0x87;

// SPI initialization
// SPI disabled
SPCR=0x00;

// TWI initialization
// TWI disabled
TWCR=0x00;

// Watchdog Timer initialization
// Watchdog Timer Prescaler: OSC/8k
// Watchdog Timer interrupt: Off
//#pragma optsize-
asm("wdr");
WDTCSR=0x18;
WDTCSR=0x08;
//#ifdef _OPTIMIZE_SIZE_
//#pragma optsize+
//#endif
slave_address=ADD0|(ADD1<<1);
// Global enable interrupts
asm("sei");
DDRC|=(1<<PINC5);

    while(1)
    {
		asm("wdr");
		
		if (t_10ms)
		{
			t_10ms=0;
			adc_I_1=adc_I;
			adc= (char)(read_adc(7)&0x00ff);
			adc_I=adc*8.65;//*34.732;	
			adc_I = adc_I_1 + /*((0.01/(f+0.01))*/ (0.02*(float)(adc_I-adc_I_1));
	
			if (adc_I>=1500)
			{
				count++;
				send_reply();
				if (count>5)// && count<200)
				{
					Motor_Free ='%';
					current_ov=1;
					WRITE_PORT(PORTC,4, current_ov);
					count=0;
					count++;
				}
			}
			else
			count=0;
		
		}
       

					if( test_driver == slave_address )
					{
						if (usart_change == 0)
						{
							//USART initialization
							//Communication Parameters: 8 Data, 1 Stop, No Parity
							//USART Receiver: On
							//USART Transmitter: On
							//USART0 Mode: Asynchronous
							//USART Baud Rate: 9600
							UCSR0A=0x00;
							UCSR0B=0x98;
							UCSR0C=0x06;
							UBRR0H=0x00;
							UBRR0L=0x33;
							usart_change=1;
						}
						send_reply () ;
					}
					else
					{
						if (usart_change == 0)
						{
							// USART initialization
							// Communication Parameters: 8 Data, 1 Stop, No Parity
							// USART Receiver: On
							// USART Transmitter: Off
							// USART0 Mode: Asynchronous
							// USART Baud Rate: 9600
							UCSR0A=0x00;
							UCSR0B=0x90;
							UCSR0C=0x06;
							UBRR0H=0x00;
							UBRR0L=0x33;
							usart_change=1;
						}
					}
		
    }
}



void Motor_Update(uint8_t Speed, uint8_t Direction)
{
	 unsigned char Hall_State;
	 int Hall_Condition;
	 asm("wdr");
	 Hall_State = (HALL3<<2)|(HALL2<<1)|(HALL1);
	 LED_1  (HALL1);
	 LED_2  (HALL2);
	 LED_3  (HALL3);
	 
	 if (Motor_Free == '%')
	 {
		 Hall_Condition = 7 ;
		 if (counter<200)
		 {
			Hall_Condition = Hall_State | ((Direction<<3)&0x8); 
		 }
	 }
	 else
	 {
		 Hall_Condition = Hall_State | ((Direction<<3)&0x8);
	 }

	switch(Hall_Condition)
	{		
		case 1:
		case (6|0x8):
		M2p (OFF);
		M3p (OFF);
		M2n_PWM = 0; M2_TCCR = (M2_TCCR & (~M2_TCCR_gm));
		M1n_PWM = 0; M1_TCCR = (M1_TCCR & (~M1_TCCR_gm));
		M1p (ON);
		M3n_PWM = Speed; M3_TCCR = (M3_TCCR & (~M3_TCCR_gm)) | (M3_TCCR_gm);
		break;
		
		case 3:
		case (4|0x8):
		M1p (OFF);
		M3p (OFF);
		M1n_PWM = 0; M1_TCCR = (M1_TCCR & (~M1_TCCR_gm));
		M2n_PWM = 0; M2_TCCR = (M2_TCCR & (~M2_TCCR_gm));
		M2p (ON);
		M3n_PWM = Speed; M3_TCCR = (M3_TCCR & (~M3_TCCR_gm)) | (M3_TCCR_gm);
		break;
		
		case 2:
		case (5|0x8):
		M1p (OFF);
		M3p (OFF);
		M2n_PWM = 0; M2_TCCR = (M2_TCCR & (~M2_TCCR_gm));
		M3n_PWM = 0; M3_TCCR = (M3_TCCR & (~M3_TCCR_gm));
		M2p (ON);
		M1n_PWM = Speed; M1_TCCR = (M1_TCCR & (~M1_TCCR_gm)) | (M1_TCCR_gm);
		break;
		
		case 6:
		case (1|0x8):
		M1p (OFF);
		M2p (OFF);
		M2n_PWM = 0; M2_TCCR = (M2_TCCR & (~M2_TCCR_gm));
		M3n_PWM = 0; M3_TCCR = (M3_TCCR & (~M3_TCCR_gm));
		M3p (ON);
		M1n_PWM = Speed; M1_TCCR = (M1_TCCR & (~M1_TCCR_gm)) | (M1_TCCR_gm);
		break;
		
		case 4:
		case (3|0x8):
		M1p (OFF);
		M2p (OFF);
		M1n_PWM = 0; M1_TCCR = (M1_TCCR & (~M1_TCCR_gm));
		M3n_PWM = 0; M3_TCCR = (M3_TCCR & (~M3_TCCR_gm));
		M3p (ON);
		M2n_PWM = Speed; M2_TCCR = (M2_TCCR & (~M2_TCCR_gm)) | (M2_TCCR_gm);
		break;
		
		case 5:
		case (2|0x8):
		M2p (OFF);
		M3p (OFF);
		M1n_PWM = 0; M1_TCCR = (M1_TCCR & (~M1_TCCR_gm));
		M3n_PWM = 0; M3_TCCR = (M3_TCCR & (~M3_TCCR_gm));
		M1p (ON);
		M2n_PWM = Speed; M2_TCCR = (M2_TCCR & (~M2_TCCR_gm)) | (M2_TCCR_gm);
		break;
		case 7:
		default:
		M1p (OFF);
		M2p (OFF);
		M3p (OFF);
		M1n_PWM = 0; M1_TCCR= (M1_TCCR & (~M1_TCCR_gm));
		M2n_PWM = 0; M2_TCCR= (M2_TCCR & (~M2_TCCR_gm));
		M3n_PWM = 0; M3_TCCR= (M3_TCCR & (~M3_TCCR_gm));
		break;

	}
}

inline int PID_CTRL()
{
	kp=.20;
	ki=0;
	kd=0.07;
	M.Setpoint = setpoint ;
	M.PID_Err = (setpoint)- M.RPM ;
	
	
	
	if (abs(M.PID_Err - M.PID_Err_last) < 20 && abs(M.PID_Err) < 700 && abs(M.PID_Err) > 20 && (M.kp<2.6 || (abs(M.RPM)>1900 && M.kp < 3.2)) &&  abs(M.RPM)>10) M.kp+=.001;
	if (abs(M.PID_Err - M.PID_Err_last) < 20 && abs(M.PID_Err) > 700 && (M.kp<2.6 || abs(M.RPM)>1900) &&  abs(M.RPM)>10) M.kp+=.003;
	//if (abs(M.PID_Err - M.PID_Err_last) < 20 && abs(M.PID_Err) > 30 && (M.kp<2.6 || (abs(M.RPM)>1900 && M.kp < 3.6)) &&  abs(M.RPM)>10 ) M.kp+=pow(log(M.PID_Err),3)/10000.0;

	//if (M.psin > 80) M.kp-=.05;
	if (abs (M.Setpoint_last - (setpoint)) > 5 ) 
	{ 
		if ((setpoint)>0 && M.Setpoint_last>(setpoint))M.kp = kp;
		if ((setpoint)<0 && M.Setpoint_last<(setpoint))M.kp = kp;
	}
	if (abs(M.RPM)<50) M.kp = kp;
	
	M.p = (M.PID_Err) * M.kp;
	M.i += M.PID_Err * ki * 0.1 ;
	
	
	M.p=(M.p>127)?(127):M.p;
	M.p=(M.p<-127)?(-127):M.p;
	
	
	M.i=(M.i>120)?(120):M.i;
	M.i=(M.i<-120)?(-120):M.i;
	
	M.d=(M.d>2400)?(2400):M.d;
	M.d=(M.d<-2400)?(2400):M.d;
	
	M.PID = M.i  + M.p - M.d * kd ;
	
	M.PID_last = M.PID_last ;
	
	if(M.PID>127)
	M.PID=127;
	if( M.PID<-127)
	M.PID=-127;

	M.PID_Err_last = M.PID_Err ;
	M.Feed_Back_last = M.Feed_Back ;
	M.Setpoint_last = setpoint ;
	
	if((setpoint)==0 && abs(M.RPM-(setpoint))<10)
	return 0;
		
	return M.PID;//
	//return setpoint*255/4000;  // PWM whithout PID controller
	
}


ISR(USART_RX_vect)
{
char status,data;
status=UCSR0A;
data=UDR0;

if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
{
	Motor_Update ( PWM , Motor_Direction ) ;
	switch (pck_num)
	{
		case 0:
		if(data == '*')
		pck_num++;
		break;
		
		case 1:
		if(data == '~')
		pck_num++;
		else
		pck_num=0;
		break;
		
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		if(slave_address * 2 + 2 == pck_num)
		tmp_rpmA=data;
		if(slave_address * 2 + 3 == pck_num)
		tmp_rpmB=data;
		
		pck_num++;
		break;
		
		case 10:
		kp = (float)data/100.0; //Robot_D[RobotID].P
		pck_num++;
		break;
		
		case 11:
		ki = (float)data/100.0; //Robot_D[RobotID].I
		pck_num++;
		break;
		
		case 12:
		kd = (float)data/100.0; //Robot_D[RobotID].D
		pck_num++;
		break;
		
		case 13:
		if (test_driver != data)
		{
			usart_change=0;
		}
		test_driver = data;
		pck_num++;
		break;
		
		case 14:
		if(data == '%' || data == '^')// free wheel : %
		{																		 
			asm("wdr");
			M.RPM_setpointA=tmp_rpmA;
			M.RPM_setpointB=tmp_rpmB;
			Motor_Free = data;

		}
		pck_num=0;
		break;
	}
}
}

ISR(INT0_vect)
{
	if ( HALL1 == 1 )
	{
		hall_dir = HALL2 ;
	}
}

ISR(PCINT2_vect)
{
	hall_flag ++ ;
	Motor_Update ( PWM , Motor_Direction ) ;
}


ISR(TIMER1_OVF_vect)
{
	TCNT1H=0xfc;
	TCNT1L=0x17;
	t++;
	if (t>=10)
	{
		t_10ms=1;
		t=0;
	}
	if (counter<200)
	{
		counter++;
		count=0;
	}
	
	T_20ms() ;
	
	M.RPM_last = M.RPM ; M.RPM=M.HSpeed;
	M.d_last=M.d; M.d= M.RPM - M.RPM_last ;
	M.RPM = M.RPM_last + _FILTER_CONST *( M.d ) ;
	M.d= M.d_last + _FILTER_PID_CONST *( M.d - M.d_last ) ;
	if (counter>199)
	{
		M.PWM =  PID_CTRL();//PD_CTRL ( (M.RPM_setpointB & 0x0ff)|((M.RPM_setpointA<<8) & 0xff00), M.RPM , &M.Err , &M.d , &M.i ) ;//
	}
	
				
	if ( M.PWM<0)
	{
		Motor_Direction = 1 ;
		PWM = -M.PWM*2;
	}
				
	else if (M.PWM >=0)
	{
		PWM = M.PWM * 2 ;
		Motor_Direction = 0 ;
	}	
	Motor_Update ( PWM , Motor_Direction ) ;
	
	
}


void T_20ms(void)
{
	M.HSpeed = (float) ( hall_flag * 1250 ) ;	//62.50=60s/(20ms*48)	48 = 3(number of hall sensors) * 8(number of pair poles) * 2
	M.HSpeed = ( hall_dir ) ? - M.HSpeed : M.HSpeed ;
	hall_flag = 0 ;
	
}

void send_reply(void)
{   
	
	Transmission_Data_1 = adc_I;
	Transmission_Data_2 = slave_address;
	Transmission_Data_3 = M.kp;
	Transmission_Data_4 = M.d;
		
	USART_send ('*');
	
	send_buff = ( ( ( (int) Transmission_Data_1 ) & 0x0ff00 ) >>8 ) ;
	USART_send ( send_buff ) ;
	
	send_buff = (((int)Transmission_Data_1) & 0x0ff) ;
	USART_send ( send_buff ) ;
	
	send_buff = ( ( ( (int) Transmission_Data_2 ) & 0x0ff00 ) >>8 ) ;//HALL2;
	USART_send ( send_buff ) ;
	
	send_buff = (((int)Transmission_Data_2) & 0x0ff) ;//HALL1;
	USART_send ( send_buff ) ;
	
	//send_buff = ( ( ( (int) Transmission_Data_3 ) & 0x0ff00 ) >>8 ) ;//HALL2;
	//USART_send ( send_buff ) ;
	//
	//send_buff = (((int)Transmission_Data_3) & 0x0ff) ;//HALL1;
	//USART_send ( send_buff ) ;
	//
	//send_buff = ( ( ( (int) Transmission_Data_4 ) & 0x0ff00 ) >>8 ) ;//HALL2;
	//USART_send ( send_buff ) ;
	//
	//send_buff = (((int)Transmission_Data_4) & 0x0ff) ;//HALL1;
	//USART_send ( send_buff ) ;
		
	USART_send ('#') ;
}


//	usart functions

unsigned char USART_receive(void){
	
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
	
}

void USART_send( unsigned char data){
	
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
	
}

void USART_putstring(char* StringPtr){
	
	while(*StringPtr != 0x00){
		USART_send(*StringPtr);
	StringPtr++;}
	
}