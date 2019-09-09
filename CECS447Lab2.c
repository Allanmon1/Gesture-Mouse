// CECS447Lab2.c
// Runs on LM4F120/TM4C123
// Test the switch initialization functions by setting the LED
// color according to the status of the switches.
// Daniel and Jonathan Valvano
// May 23, 2014

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Section 4.2    Program 4.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// negative logic switches connected to PF0 and PF4 on the Launchpad
// red LED connected to PF1 on the Launchpad
// blue LED connected to PF2 on the Launchpad
// green LED connected to PF3 on the Launchpad
// NOTE: The NMI (non-maskable interrupt) is on PF0.  That means that
// the Alternate Function Select, Pull-Up Resistor, Pull-Down Resistor,
// and Digital Enable are all locked for PF0 until a value of 0x4C4F434B
// is written to the Port F GPIO Lock Register.  After Port F is
// unlocked, bit 0 of the Port F GPIO Commit Register must be set to
// allow access to PF0's control registers.  On the LM4F120, the other
// bits of the Port F GPIO Commit Register are hard-wired to 1, meaning
// that the rest of Port F can always be freely re-configured at any
// time.  Requiring this procedure makes it unlikely to accidentally
// re-configure the JTAG pins as GPIO, which can lock the debugger out
// of the processor and make it permanently unable to be debugged or
// re-programmed.

#include <stdint.h>
#include "tm4c123gh6pm.h"
#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define SWITCHES  (*((volatile uint32_t *)0x40025044))
#define SW1       0x01                      // on the left side of the Launchpad board
#define SW2       0x10                      // on the right side of the Launchpad board
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define SYSCTL_RCGC2_GPIOB			0x00000002	// port B Clock Gating Control
#define SYSCTL_TCGC2_FPIOA			0x00000001	// port A Clock Gating Control
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08

#define BNO055_ID 0xA0
#define BNO055_CHIP_ID_ADDR  0x00
#define BNO055_ACCEL_REV_ID_ADDR  0x01
#define BNO055_GYRO_REV_ID_ADDR  0x03

#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_ACCEL_DATA_X_MSB_ADDR 0x09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR 0x0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR 0x0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR 0x0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR 0x0D
#define BNO055_GYRO_DATA_X_LSB_ADDR  0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR  0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR  0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR  0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR  0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR  0x19
#define BNO055_EULER_H_LSB_ADDR  0x1A
#define BNO055_EULER_H_MSB_ADDR  0x1B
#define BNO055_EULER_R_LSB_ADDR  0x1C
#define BNO055_EULER_R_MSB_ADDR  0x1D
#define BNO055_EULER_P_LSB_ADDR  0x1E
#define BNO055_EULER_P_MSB_ADDR  0x1F
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR  0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR  0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR  0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR  0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR  0x2D

// initialize global variables

//---------------------------------------------------------------//
// Functions
//---------------------------------------------------------------//

void UART_Init(void){
	SYSCTL_RCGCUART_R |= 0x0001; 			// activate UART0
	SYSCTL_RCGCGPIO_R |= 0x0001;			// activate GPIOA
	UART0_CTL_R &= ~0x0001;						// disable UART0
	UART0_IBRD_R = 104;								// IBRD, 16MHz, 9600 baud rate
	UART0_FBRD_R = 11;								// FBRD, (0.1667 * 64 + .05)
	UART0_LCRH_R = 0x0060;						// 8 bit (no parity, one stop, no FIFOs)
	UART0_CTL_R |= 0x0001; 						// enable UART
	GPIO_PORTA_AFSEL_R |= 0x03;				// enable alt funct on PA0,1
	GPIO_PORTA_PCTL_R &= ~0x000000FF; // configure PA0 and PA1 as U0Rx and U0Tx
	GPIO_PORTA_PCTL_R |= 0x00000011;
	GPIO_PORTA_AMSEL_R &= ~0x03;			// disable analog funct on PA0, PA1
	GPIO_PORTA_DEN_R |= 0x03;					// enable digital I/O on PA0, PA1
	}

// wait for new input, then return ASCII code
char UART_Rx(void){
	if ((UART0_FR_R &0x0010) == 0)	// non-blocking receive 
		return ((unsigned char) (UART0_DR_R & 0xFF));
	return ((unsigned char) 0x00);
}

void UART_Tx(unsigned char data){
	while((UART0_FR_R & 0x0020) != 0);	// wait until TXFF is 0
	UART0_DR_R = data;
}

void UART_transmit_String( const uint8_t *MessageString){
	while( *MessageString ){
		UART_Tx(*MessageString);
		MessageString++;
	}
}

void Timer0_Init(void){
	unsigned volatile delay;
	SYSCTL_RCGCTIMER_R |= 0x01;				// activate timer 0
	delay = SYSCTL_RCGCTIMER_R;
	TIMER0_CTL_R = 0;									// disable the timer for setup
	TIMER0_CFG_R = 0x04;							// configure for a 16-bit timer mode
	TIMER0_TAMR_R |= 0x01;						// configure for a one shot, count down
	TIMER0_CTL_R |= 0x01;							// enable timer0
}

void Timer0_DelayMs(uint16_t time){
	uint16_t x;
	unsigned long volatile delay;
	SYSCTL_RCGCTIMER_R |= 1;	// enable Timer Block 0
	delay =SYSCTL_RCGCTIMER_R;
	for(x=0; x < time; x++){
		TIMER0_TAILR_R = 16000 - 1;	// interval load value of every 1ms
		TIMER0_ICR_R = 0x01;				// clear TimerA timeout flag
		TIMER0_CTL_R = 0x01;			// enable timer A
		while((TIMER0_RIS_R & 0x01) == 0) {
			if((TIMER0_RIS_R & 0x01) == 0)
				break;	// wait for timeout
		}
	}
	TIMER0_ICR_R = 0x1;				// clear TimerA timeout flag	
}

	
#define TPR (500/63 - 1)
void I2C_Init(void){
	SYSCTL_RCGCI2C_R |= 0x0001;					// activate I2C0
	SYSCTL_RCGCGPIO_R |= 0x0002;				// activate port B
	while((SYSCTL_PRGPIO_R&0x0002) == 0); // ready?
	GPIO_PORTB_AFSEL_R |= 0x0C;					// enable alt function on PB2,3
	GPIO_PORTB_ODR_R |= 0x0C; 					// enable open drain on PB2,3
	GPIO_PORTB_PCTL_R &= ~0x0000FF00;		// clearing port control of PB2,3 to zero
	GPIO_PORTB_PCTL_R |= 0x00003300;		// setting port control of PB2,3 for I2C
	GPIO_PORTB_DEN_R |= 0x0C;						// enable digital I/O on PB2,3
	I2C0_MCR_R = 0x00000010;						// master function enable
	I2C0_MTPR_R = TPR;									// configure for 100kbps
}

uint32_t I2C_Send2(uint8_t slave, uint8_t data1, uint8_t data2){
	while(I2C0_MCS_R & 0x00000001);     // wait for I2C ready
	I2C0_MSA_R = (slave<<1)&0xFE;       // MSA[7:1] is slave address
	I2C0_MSA_R &= ~0x01;								// MSA[0] IS 0 for send
	I2C0_MDR_R = data1&0xFF;            // prepare first byte
	I2C0_MCS_R = (I2C_MCS_START					// generate start/restart
								| I2C_MCS_RUN);				// no ack, no stop, master enable
	while(I2C0_MCS_R & 0x00000001);			// wait for ftransmission done
		// chech error bits
	if((I2C0_MCS_R&
		 (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0){
			 I2C0_MCS_R = I2C_MCS_STOP;     // stop, no ack, disable
			 return (I2C0_MCS_R&             // return error bits if nonzero
				       (I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
		 }
	I2C0_MDR_R = data2&0xFF;						// prepare second byte
	I2C0_MCS_R = (I2C_MCS_STOP					// no ack, stop, no start
								 | I2C_MCS_RUN);			// master enable
	while(I2C0_MCS_R & 0x00000001);			// wait for transmission done
	return (I2C0_MCS_R&									// return error bits
					(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
}

#define MAXRETRIES 5                 // number of receive attempts before giving up

uint8_t I2C_Recv(uint8_t slave){
	uint8_t data1,data2;
	int retryCounter = 1;
	do{
		while(I2C0_MCS_R&0x00000001);     // wait for I2C ready
		I2C0_MSA_R = (slave << 1) & 0xFE; // MSA[7:1] is slave address
		I2C0_MSA_R |= 0x01;								// MSA[0] is 1 for receive
		I2C0_MCS_R = (I2C_MCS_ACK					// positive data ack
									| I2C_MCS_START     // no stop, yes start/restart
									| I2C_MCS_RUN);     // master enable
		while(I2C0_MCS_R & 0x00000001);   // wait for transmission done
		data1=(I2C0_MDR_R&0xFF);					// MSB data sent first
	}																		// repeat if error
	while(((I2C0_MCS_R&(I2C_MCS_ADRACK|I2C_MCS_ERROR)) != 0)
				 && (retryCounter <= MAXRETRIES));
	return(data1<<8)+data2;             // usually returns 0xFFFF on error
}

void start_up_config(){
	uint8_t addr = 0x3D;							// config Register
	uint8_t data = 0x00;              // Config Mode
	uint8_t len	= 1;									// data length 1
	I2C_Send2(addr, data, len);				// write to the config register
	Timer0_DelayMs(15);								// 15ms delay
	
	//Initialize power_on mode
	addr = 0x3E;											// PowerMode Register
	data = 0x00;											// Turn on IMU set Power Mode on
	len = 1;													// length 1
	I2C_Send2(addr, data, len);				// write to the pwer on register
	Timer0_DelayMs(15);								// 15ms delay
	
	//Initialize External ClockSource
	addr = 0x3F;											// OP_Mode
	data = 0x80;											// external clockSource
	len = 1;													// length 1
	I2C_Send2(addr, data, len);				// write to the register
	Timer0_DelayMs(17);								// 17ms delay
	
	// Configer
	addr = 0x3D;											// Config Register
	data = 0x0C;											// NDOF write mode
	len = 1;													// data length 1
	I2C_Send2(addr, data, len);				// write to the config Register
	Timer0_DelayMs(19);								// 19ms delay
}

int main(void){
	uint16_t x;
	uint32_t delay;
	UART_Init();
	/* 
		After initialization of UART, a delay is required before transmitting any data.
		Not including a delay will cause the TM4C to transmit incorrect data.
	*/
	I2C_Init();
	for(delay=0; delay<1000000; delay++);
	UART_transmit_String("Hello\n\r");
	/*
	Making sure the UART is connected correctly to the TM4C
	*/
	
	start_up_config();
	/*uint8_t id = I2C_Recv(BNO055_CHIP_ID_ADDR);
	if( id != BNO055_ID){
		Timer0_DelayMs(1);
		id = I2C_Recv(BNO055_CHIP_ID_ADDR);
		if(id != BNO055_ID){
			UART_transmit_String("Hello\n\r");
			while(1);
		}
	}*/
	while(1){
		//x = I2C_Recv2(0x28);
		//UART_Tx(x);
		for(delay=0; delay<1000000; delay++);
		UART_transmit_String("Hello\n\r");
	}
}

