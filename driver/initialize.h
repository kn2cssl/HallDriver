/*
 * Initialize.h
 *
 * Created: 3/3/2014 7:13:57 PM
 *  Author: Milad
 */ 


#ifndef INITIALIZE_H_
#define INITIALIZE_H_

#define READ_PIN(_PIN_,_num_) ((_PIN_ & (0x01<<_num_))>>_num_)
#define WRITE_PORT(_PORT_,_num_,_Status_) _PORT_ = (_PORT_ & (~(0x01<<_num_))) | ((_Status_)<<_num_)

#define M1p(_Status_) WRITE_PORT(PORTD,4,_Status_) 
#define M2p(_Status_) WRITE_PORT(PORTB,1,_Status_) 
#define M3p(_Status_) WRITE_PORT(PORTB,2,_Status_) 
#define M1n(_Status_) WRITE_PORT(PORTD,3,_Status_) 
#define M2n(_Status_) WRITE_PORT(PORTD,6,_Status_) 
#define M3n(_Status_) WRITE_PORT(PORTB,3,_Status_) 

#define ADD0 READ_PIN(PINC,3)  
#define ADD1 READ_PIN(PINC,2)

#define HALL1 READ_PIN(PIND,2) 
#define HALL2 READ_PIN(PIND,5)
#define HALL3 READ_PIN(PIND,7)

#define ENC1 READ_PIN(PINC,0)
#define ENC2 READ_PIN(PINC,1)

#define M1n_PWM OCR2B
#define M2n_PWM OCR0A
#define M3n_PWM OCR2A

#define M1_TCCR TCCR2A
#define M2_TCCR TCCR0A
#define M3_TCCR TCCR2A

#define M1_TCCR_gm 0x20
#define M2_TCCR_gm 0x80
#define M3_TCCR_gm 0x80

#define OFF 0
#define ON 1

#define LED_1(_Status_) WRITE_PORT(PORTB,0,_Status_)
#define LED_2(_Status_) WRITE_PORT(PORTB,4,_Status_)
#define LED_3(_Status_) WRITE_PORT(PORTB,5,_Status_)




#ifndef UPE
#define UPE 2
#endif

#ifndef DOR
#define DOR 3
#endif

#ifndef FE
#define FE 4
#endif

#ifndef UDRE
#define UDRE 5
#endif

#ifndef RXC
#define RXC 7
#endif

#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<DOR)

#define _FILTER_FREQ 1.0 //
#define _FILTER_CONST 0.006 //(0.001/((1.0/(2.0*3.14*_FILTER_FREQ))+0.001))
#define _FILTER_PID_CONST 0.8 //(0.001/((1.0/(2.0*3.14*_FILTER_FREQ))+0.001))

void Motor_Update(unsigned char Speed, unsigned char Direction);
inline int PID_CTRL();
void T_20ms(void);
unsigned char USART_receive(void);
void USART_send( unsigned char data);
void USART_putstring(char* StringPtr);
void send_reply(void);

#endif /* INITIALIZE_H_ */