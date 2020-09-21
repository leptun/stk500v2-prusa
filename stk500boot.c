/*****************************************************************************
Title:     STK500v2 compatible bootloader
           Modified for Wiring board ATMega128-16MHz
Author:    Peter Fleury <pfleury@gmx.ch>   http://jump.to/fleury
Compiler:  avr-gcc 3.4.5 or 4.1 / avr-libc 1.4.3
Hardware:  All AVRs with bootloader support, tested with ATmega8
License:   GNU General Public License

Modified:  Worapoht Kornkaewwattanakul <dev@avride.com>   http://www.avride.com
Date:      17 October 2007
Update:    1st, 29 Dec 2007 : Enable CMD_SPI_MULTI but ignore unused command by return 0x00 byte response..
Compiler:  WINAVR20060421
Description: add timeout feature like previous Wiring bootloader

DESCRIPTION:
    This program allows an AVR with bootloader capabilities to
    read/write its own Flash/EEprom. To enter Programming mode
    an input pin is checked. If this pin is pulled low, programming mode
    is entered. If not, normal execution is done from $0000
    "reset" vector in Application area.
    Size fits into a 1024 word bootloader section
	when compiled with avr-gcc 4.1
	(direct replace on Wiring Board without fuse setting changed)

USAGE:
    - Set AVR MCU type and clock-frequency (F_CPU) in the Makefile.
    - Set baud rate below (AVRISP only works with 115200 bps)
    - compile/link the bootloader with the supplied Makefile
    - program the "Boot Flash section size" (BOOTSZ fuses),
      for boot-size 1024 words:  program BOOTSZ01
    - enable the BOOT Reset Vector (program BOOTRST)
    - Upload the hex file to the AVR using any ISP programmer
    - Program Boot Lock Mode 3 (program BootLock 11 and BootLock 12 lock bits) // (leave them)
    - Reset your AVR while keeping PROG_PIN pulled low // (for enter bootloader by switch)
    - Start AVRISP Programmer (AVRStudio/Tools/Program AVR)
    - AVRISP will detect the bootloader
    - Program your application FLASH file and optional EEPROM file using AVRISP

Note:
    Erasing the device without flashing, through AVRISP GUI button "Erase Device"
    is not implemented, due to AVRStudio limitations.
    Flash is always erased before programming.

	AVRdude:
	Please uncomment #define REMOVE_CMD_SPI_MULTI when using AVRdude.
	Comment #define REMOVE_PROGRAM_LOCK_BIT_SUPPORT to reduce code size
	Read Fuse Bits and Read/Write Lock Bits is not supported

NOTES:
    Based on Atmel Application Note AVR109 - Self-programming
    Based on Atmel Application Note AVR068 - STK500v2 Protocol

LICENSE:
    Copyright (C) 2006 Peter Fleury

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*****************************************************************************/

//************************************************************************
//*	Edit History
//************************************************************************
//*	Jul  7,	2010	<MLS> = Mark Sproul msproul@skycharoit.com
//*	Jul  7,	2010	<MLS> Working on mega2560. No Auto-restart
//*	Jul  7,	2010	<MLS> Switched to 8K bytes (4K words) so that we have room for the monitor
//*	Jul  8,	2010	<MLS> Found older version of source that had auto restart, put that code back in
//*	Jul  8,	2010	<MLS> Adding monitor code
//*	Jul 11,	2010	<MLS> Added blinking LED while waiting for download to start
//*	Jul 11,	2010	<MLS> Added EEPROM test
//*	Jul 29,	2010	<MLS> Added recchar_timeout for timing out on bootloading
//*	Aug 23,	2010	<MLS> Added support for atmega2561
//*	Aug 26,	2010	<MLS> Removed support for BOOT_BY_SWITCH
//*	Sep  8,	2010	<MLS> Added support for atmega16
//*	Nov  9,	2010	<MLS> Issue 392:Fixed bug that 3 !!! in code would cause it to jump to monitor
//*	Jun 24,	2011	<MLS> Removed analogRead (was not used)
//*	Dec 29,	2011	<MLS> Issue 181: added watch dog timmer support
//*	Dec 29,	2011	<MLS> Issue 505:  bootloader is comparing the seqNum to 1 or the current sequence 
//*	Jan  1,	2012	<MLS> Issue 543: CMD_CHIP_ERASE_ISP now returns STATUS_CMD_FAILED instead of STATUS_CMD_OK
//*	Jan  1,	2012	<MLS> Issue 543: Write EEPROM now does something (NOT TESTED)
//*	Jan  1,	2012	<MLS> Issue 544: stk500v2 bootloader doesn't support reading fuses
//************************************************************************

// LCD startup screen and boot animation
#define LCD_HD44780
#define LCD_HD44780_ANIMATION
#define LCD_HD44780_COUNTER
// Dual serial support
#define DUALSERIAL
// EINSY board
#define EINSYBOARD

//************************************************************************

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/common.h>
#include "command.h"

#ifdef LCD_HD44780
#include "lcd.h"
#endif


#define BLINK_LED_WHILE_WAITING
#define UART_BAUDRATE_DOUBLE_SPEED 1


#ifndef BAUDRATE
	#define BAUDRATE 115200
#endif


// Uncomment the following lines to save code space
//#define	REMOVE_PROGRAM_LOCK_BIT_SUPPORT // disable program lock bits
//#define	REMOVE_BOOTLOADER_LED // no LED to show active bootloader
//#define	REMOVE_CMD_SPI_MULTI // disable processing of SPI_MULTI commands, Remark this line for AVRDUDE <Worapoht>

//************************************************************************
//*	LED on pin "PROGLED_PIN" on port "PROGLED_PORT"
//*	indicates that bootloader is active
//************************************************************************
#define PROGLED_PORT PORTB
#define PROGLED_RPORT PINB
#define PROGLED_DDR DDRB
#define PROGLED_PIN PINB7


#define _BLINK_LOOP_COUNT_ (F_CPU / 2250)

// HW and SW version, reported to AVRISP, must match version of AVRStudio
#define CONFIG_PARAM_BUILD_NUMBER_LOW	0
#define CONFIG_PARAM_BUILD_NUMBER_HIGH	0
#define CONFIG_PARAM_HW_VER				0x0F
#define CONFIG_PARAM_SW_MAJOR			2
#define CONFIG_PARAM_SW_MINOR			0x0A


#define	UART_BAUD_RATE_LOW			UBRR0L
#define	UART_STATUS_REG				UCSR0A
#define	UART_CONTROL_REG			UCSR0B
#define	UART_ENABLE_TRANSMITTER		TXEN0
#define	UART_ENABLE_RECEIVER		RXEN0
#define	UART_TRANSMIT_COMPLETE		TXC0
#define	UART_RECEIVE_COMPLETE		RXC0
#define	UART_DATA_REG				UDR0
#define	UART_DOUBLE_SPEED			U2X0

#ifdef DUALSERIAL
// Secondary UART defines
#define	UART_BAUD_RATE_LOW1			UBRR1L
#define	UART_STATUS_REG1			UCSR1A
#define	UART_CONTROL_REG1			UCSR1B
#define	UART_ENABLE_TRANSMITTER1	TXEN1
#define	UART_ENABLE_RECEIVER1		RXEN1
#define	UART_TRANSMIT_COMPLETE1		TXC1
#define	UART_RECEIVE_COMPLETE1		RXC1
#define	UART_DATA_REG1				UDR1
#define	UART_DOUBLE_SPEED1			U2X1
#endif //DUALSERIAL

// Macro to calculate UBBR from XTAL and baudrate
#if UART_BAUDRATE_DOUBLE_SPEED
	#define UART_BAUD_SELECT(baudRate,xtalCpu) (((float)(xtalCpu))/(((float)(baudRate))*8.0)-1.0+0.5)
#else
	#define UART_BAUD_SELECT(baudRate,xtalCpu) (((float)(xtalCpu))/(((float)(baudRate))*16.0)-1.0+0.5)
#endif


// States used in the receive state machine
#define	ST_START		0
#define	ST_GET_SEQ_NUM	1
#define ST_MSG_SIZE_1	2
#define ST_MSG_SIZE_2	3
#define ST_GET_TOKEN	4
#define ST_GET_DATA		5
#define	ST_GET_CHECK	6
#define	ST_PROCESS		7


// Use 16bit address variable for ATmegas with <= 64K flash
#if (FLASHEND > 0x10000)
	typedef uint32_t address_t;
#else
	typedef uint16_t address_t;
#endif


static void sendchar(char c);

uint8_t selectedSerial = 0;


// since this bootloader is not linked against the avr-gcc crt1 functions,
// to reduce the code size, we need to provide our own initialization
void __jumpMain	(void) __attribute__ ((naked)) __attribute__ ((section (".init9")));
#include <avr/sfr_defs.h>


#define STACK_TOP (RAMEND - 16)

//*****************************************************************************
void __jumpMain(void)
{
	asm volatile (".set __stack, %0" :: "i" (STACK_TOP));

	asm volatile ("ldi 16, %0" :: "i" (STACK_TOP >> 8)); // set stack pointer to top of RAM
	asm volatile ("out %0,16" :: "i" (AVR_STACK_POINTER_HI_ADDR));

	asm volatile ("ldi 16, %0" :: "i" (STACK_TOP & 0x0ff));
	asm volatile ("out %0,16" :: "i" (AVR_STACK_POINTER_LO_ADDR));

	asm volatile ("clr __zero_reg__" ); // GCC depends on register r1 set to 0
	asm volatile ("out %0, __zero_reg__" :: "I" (_SFR_IO_ADDR(SREG))); // set SREG to 0
	asm volatile ("jmp main"); // jump to main()
}

void jumpToUserSpace(void)
{
	asm volatile(
		"clr r30 \n\t"
		"clr r31 \n\t"
		"ijmp \n\t"
		);
}


//*****************************************************************************
void delay_ms(uint16_t timedelay)
{
	for (uint16_t i = 0; i < timedelay; i++)
	{
		_delay_ms(0.5);
	}
}

//*****************************************************************************
// send single byte to USART, wait until transmission is completed
static void sendchar(char c)
{
	if (selectedSerial == 0)
	{
		UART_DATA_REG = c; // prepare transmission
		while (!(UART_STATUS_REG & _BV(UART_TRANSMIT_COMPLETE))); // wait until byte sent
		UART_STATUS_REG |= _BV(UART_TRANSMIT_COMPLETE); // delete TXCflag
	}
#ifdef DUALSERIAL
	else if (selectedSerial == 1)
	{
		UART_DATA_REG1 = c; // prepare transmission
		while (!(UART_STATUS_REG1 & _BV(UART_TRANSMIT_COMPLETE1))); // wait until byte sent
		UART_STATUS_REG1 |= _BV(UART_TRANSMIT_COMPLETE1); // delete TXCflag
	}
#endif //DUALSERIAL
}

//************************************************************************

static uint8_t Serial_Available(uint8_t serial)
{
	if (serial == 0)
		return (UART_STATUS_REG & _BV(UART_RECEIVE_COMPLETE)); // wait for data
#ifdef DUALSERIAL
	else if (serial == 1)
		return (UART_STATUS_REG1 & _BV(UART_RECEIVE_COMPLETE1)); // wait for data
#endif //DUALSERIAL
	return 0;
}

#define	MAX_TIME_COUNT (F_CPU >> 1)
//*****************************************************************************
static uint8_t recchar_timeout(void)
{
	uint32_t count = 0;
	while (1)
	{
		if ((selectedSerial == 0) && Serial_Available(0))
			break;
#ifdef DUALSERIAL
		else if ((selectedSerial == 1) && Serial_Available(1))
			break;
#endif
		count++;
		if (count > MAX_TIME_COUNT)
		{
		uint16_t data;
		#if (FLASHEND > 0x10000)
			data = pgm_read_word_far(0); // get the first word of the user program
		#else
			data = pgm_read_word_near(0); // get the first word of the user program
		#endif
			if (data != 0xffff) // make sure its valid before jumping to it.
			{
				jumpToUserSpace();
			}
			count = 0;
		}
	}
	if (selectedSerial == 0)
		return UART_DATA_REG;
#ifdef DUALSERIAL
	else if (selectedSerial == 1) 
		return UART_DATA_REG1;
#endif
	return 0;
}

void initUart(void)
{
	// init uart0
#if UART_BAUDRATE_DOUBLE_SPEED
	UART_STATUS_REG |= _BV(UART_DOUBLE_SPEED);
#endif
	UART_BAUD_RATE_LOW = UART_BAUD_SELECT(BAUDRATE,F_CPU);
	UART_CONTROL_REG = _BV(UART_ENABLE_RECEIVER) | _BV(UART_ENABLE_TRANSMITTER);

#ifdef DUALSERIAL
	// init uart1
#if UART_BAUDRATE_DOUBLE_SPEED
	UART_STATUS_REG1 |= _BV(UART_DOUBLE_SPEED1);
#endif
	UART_BAUD_RATE_LOW1 = UART_BAUD_SELECT(BAUDRATE,F_CPU);
	UART_CONTROL_REG1 = _BV(UART_ENABLE_RECEIVER1) | _BV(UART_ENABLE_TRANSMITTER1);
#endif //DUALSERIAL
}


#ifdef EINSYBOARD
//Heaters off (PG5=0, PE5=0)
//Fans on (PH5=1, PH3=1)
//Motors off (PA4..7=1)
void pinsToDefaultState(void)
{
	DDRA |= 0b11110000; //PA4..7 out
	PORTA |= 0b11110000; //PA4..7 = 1
	DDRE |= 0b00100000; //PE5 out
	PORTE &= 0b11011111; //PE5 = 0
	DDRG |= 0b00100000; //PG5 out
	PORTG &= 0b11011111; //PG5 = 0
	DDRH |= 0b00101000; //PH5, PH3 out
	PORTH |= 0b00101000; //PH5, PH3 = 1
}
#endif //EINSYBOARD

uint32_t flashSize = 0; //flash data size in bytes
uint32_t flashCounter = 0; //flash counter (readed / written bytes)
address_t flashAddressLast = 0; //last written flash address
uint8_t flashOperation = 0; //current flash operation (0-nothing, 1-write, 2-verify)

#define RAMSIZE        (RAMEND - RAMSTART + 1)
#define boot_src_addr  (*((uint32_t*)(RAMSIZE - 16)))
#define boot_dst_addr  (*((uint32_t*)(RAMSIZE - 12)))
#define boot_copy_size (*((uint16_t*)(RAMSIZE - 8)))
#define boot_reserved  (*((uint8_t*)(RAMSIZE - 6)))
#define boot_app_flags (*((uint8_t*)(RAMSIZE - 5)))
#define boot_app_magic (*((uint32_t*)(RAMSIZE - 4)))
#define BOOT_APP_FLG_ERASE 0x01
#define BOOT_APP_FLG_COPY  0x02
#define BOOT_APP_FLG_FLASH 0x04
#define BOOT_APP_FLG_RUN   0x08 //!< Do not jump to application immediately


//*****************************************************************************
int main(void)
{
	address_t address = 0;
	address_t eraseAddress = 0;
	uint8_t msgParseState;
	uint16_t ii = 0;
	uint8_t checksum = 0;
	uint8_t seqNum = 0;
	uint16_t msgLength = 0;
	uint8_t msgBuffer[285];
	uint8_t c;
	uint8_t* p;
	uint8_t isLeave = 0;

	uint16_t boot_timeout = 20000ul; // should be about 1 second
	uint16_t boot_timer = 0;
	uint8_t boot_state = 0;

	uint8_t mcuStatusReg;
	mcuStatusReg = MCUSR;
	MCUSR = 0;
	wdt_disable();
	// check if WDT generated the reset, if so, go straight to app
	if (mcuStatusReg & _BV(WDRF))
	{
		if (boot_app_magic == 0x55aa55aa)
		{
			if (boot_app_flags & BOOT_APP_FLG_RUN)
				goto start;
			
			address = boot_dst_addr;
			address_t pageAddress = address;
			while (boot_copy_size)
			{
				if (boot_app_flags & BOOT_APP_FLG_ERASE)
				{
					boot_page_erase(pageAddress);
					boot_spm_busy_wait();
				}
				pageAddress += SPM_PAGESIZE;
				if ((boot_app_flags & BOOT_APP_FLG_COPY))
				{
					while (boot_copy_size && (address < pageAddress))
					{
						uint16_t word = 0x0000;
						if (boot_app_flags & BOOT_APP_FLG_FLASH)
							word = pgm_read_word_far(boot_src_addr); //from FLASH
						else
							word = *((uint16_t*)((uint16_t)boot_src_addr)); //from RAM
						boot_page_fill(address, word);
						address += 2;
						boot_src_addr += 2;
						if (boot_copy_size > 2)
							boot_copy_size -= 2;
						else
							boot_copy_size = 0;
					}
					boot_page_write(pageAddress - SPM_PAGESIZE);
					boot_spm_busy_wait();
					boot_rww_enable();
				}
				else
				{
					address += SPM_PAGESIZE;
					if (boot_copy_size > SPM_PAGESIZE)
						boot_copy_size -= SPM_PAGESIZE;
					else
						boot_copy_size = 0;
				}
			}

		}
		goto exit;
	}
	start:


#ifndef REMOVE_BOOTLOADER_LED
	// PROGLED_PIN pulled high, indicate with LED that bootloader is active
	PROGLED_DDR |= _BV(PROGLED_PIN);
//	PROGLED_PORT &= ~_BV(PROGLED_PIN); // active low LED ON
	PROGLED_PORT |= _BV(PROGLED_PIN); // active high LED ON
#endif

	initUart(); // Set baudrate and enable USART receiver and transmiter without interrupts

#ifdef EINSYBOARD
	pinsToDefaultState();
#endif //EINSYBOARD

#ifdef LCD_HD44780
	lcd_init();
	lcd_clrscr();
	lcd_goto(65);
	lcd_puts("Original Prusa i3");
	lcd_goto(23);
	lcd_puts("Prusa Research");
	lcd_goto(101);
	lcd_puts("...");

#endif //LCD_HD44780

	uint8_t animationTimer = 0;
	uint8_t animationFrame = 0;


	while (boot_state==0)
	{
		while ((!(Serial_Available(0))) && (!(Serial_Available(1))) && (boot_state == 0)) // wait for data
		{
			_delay_ms(0.001);
			boot_timer++;
			if (boot_timer > boot_timeout)
			{
				boot_state = 1; // (after ++ -> boot_state=2 bootloader timeout, jump to main 0x00000 )
			}
		#if defined(BLINK_LED_WHILE_WAITING) && !defined(REMOVE_BOOTLOADER_LED)
			if ((boot_timer % _BLINK_LOOP_COUNT_) == 0)
			{
				PROGLED_RPORT |= _BV(PROGLED_PIN); // toggle the LED
			}
		#endif
		}
		if (Serial_Available(1))
			selectedSerial = 1;
		boot_state++; // ( if boot_state=1 bootloader received byte from UART, enter bootloader mode)
	}

	uint8_t messageShown = 0;

	if (boot_state==1)
	{
		// main loop
		while (!isLeave)
		{
			// Collect received bytes to a complete message
			msgParseState = ST_START;
			while (msgParseState != ST_PROCESS)
			{
				if (boot_state == 1)
				{
					boot_state = 0;
					c = UART_DATA_REG;
				}
				else
				{
					c = recchar_timeout();
				}


				switch (msgParseState)
				{
				case ST_START:
					if (c == MESSAGE_START)
					{
						msgParseState = ST_GET_SEQ_NUM;
						checksum = MESSAGE_START;
					}
					break;

				case ST_GET_SEQ_NUM:
					seqNum = c;
					msgParseState = ST_MSG_SIZE_1;
					checksum ^= c;
					break;

				case ST_MSG_SIZE_1:
					msgLength = c << 8;
					msgParseState = ST_MSG_SIZE_2;
					checksum ^= c;
					break;

				case ST_MSG_SIZE_2:
					msgLength |= c;
					msgParseState = ST_GET_TOKEN;
					checksum ^= c;
					break;

				case ST_GET_TOKEN:
					if (c == TOKEN)
					{
						msgParseState = ST_GET_DATA;
						checksum ^= c;
						ii = 0;
					}
					else
					{
						msgParseState = ST_START;
					}
					break;

				case ST_GET_DATA:
					msgBuffer[ii++] = c;
					checksum ^= c;
					if (ii == msgLength)
					{
						msgParseState = ST_GET_CHECK;
					}
					break;

				case ST_GET_CHECK:
					if (c == checksum)
					{
						msgParseState = ST_PROCESS;
					}
					else
					{
						msgParseState = ST_START;
					}
					break;
				}
			}

#ifdef LCD_HD44780
			if (messageShown == 0)
			{
				lcd_clrscr();
				lcd_goto(20);
				lcd_puts(" Do not disconnect!");
				lcd_goto(45);
				lcd_puts(" Upgrading firmware");
				messageShown = 1;
			}
#endif //LCD_HD44780

#ifdef LCD_HD44780_ANIMATION
			if (flashSize == 0)
			{
				animationTimer++;
				if (animationTimer > 10)
				{
					animationTimer = 0;
					animationFrame++;
					if (animationFrame > 5)
						animationFrame = 0;
					lcd_goto(91);
					lcd_puts("|    |");
					lcd_goto((animationFrame <= 3) ? (92 + animationFrame) : (98 - animationFrame));
					lcd_putc('*');
				}
			}
#endif //LCD_HD44780_ANIMATION

#ifdef LCD_HD44780_COUNTER
			if ((flashSize != 0) && flashOperation)
			{
				if (flashOperation == 1) //write
				{
					lcd_goto(88);
					lcd_puts("write ");
				}
				if (flashOperation == 2) //verify
				{
					lcd_goto(87);
					lcd_puts("verify ");
				}
				uint8_t progress = 100 * flashCounter / flashSize;
				char text[4] = "   ";
				for (int i = 2; i >= 0; i--)
					if (progress > 0)
					{
						text[i] = '0' + (progress % 10);
						progress /= 10;
					}
					else
						text[i] = ' ';
				lcd_puts(text);
				lcd_putc('%');
			}
#endif //LCD_HD44780_COUNTER

			// Now process the STK500 commands, see Atmel Appnote AVR068

			switch (msgBuffer[0])
			{
	#ifndef REMOVE_CMD_SPI_MULTI
				case CMD_SPI_MULTI:
					{
						uint8_t answerByte;

						if (msgBuffer[4] == 0x30)
						{
							uint8_t signatureIndex = msgBuffer[6];

							if (signatureIndex == 0)
							{
								answerByte = SIGNATURE_0;
							}
							else if (signatureIndex == 1)
							{
								answerByte = SIGNATURE_1;
							}
							else
							{
								answerByte = SIGNATURE_2;
							}
						}
						else if ( msgBuffer[4] & 0x50 )
						{
						//*	Issue 544: 	stk500v2 bootloader doesn't support reading fuses
						//*	I cant find the docs that say what these are supposed to be but this was figured out by trial and error
						//	answerByte	=	boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
						//	answerByte	=	boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
						//	answerByte	=	boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
							if (msgBuffer[4] == 0x50)
							{
								answerByte = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
							}
							else if (msgBuffer[4] == 0x58)
							{
								answerByte = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
							}
							else
							{
								answerByte = 0;
							}
						}
						else
						{
							answerByte = 0; // for all others command are not implemented, return dummy value for AVRDUDE happy <Worapoht>
						}

						msgLength = 7;
						msgBuffer[1] = STATUS_CMD_OK;
						msgBuffer[2] = 0;
						msgBuffer[3] = msgBuffer[4];
						msgBuffer[4] = 0;
						msgBuffer[5] = answerByte;
						msgBuffer[6] = STATUS_CMD_OK;
					}
					break;
	#endif
				case CMD_SIGN_ON:
					msgLength = 11;
					msgBuffer[1] = STATUS_CMD_OK;
					msgBuffer[2] = 8;
					msgBuffer[3] = 'A';
					msgBuffer[4] = 'V';
					msgBuffer[5] = 'R';
					msgBuffer[6] = 'I';
					msgBuffer[7] = 'S';
					msgBuffer[8] = 'P';
					msgBuffer[9] = '_';
					msgBuffer[10] = '2';
					break;

				case CMD_GET_PARAMETER:
					{
						uint8_t value;

						switch(msgBuffer[1])
						{
						case PARAM_BUILD_NUMBER_LOW:
							value = CONFIG_PARAM_BUILD_NUMBER_LOW;
							break;
						case PARAM_BUILD_NUMBER_HIGH:
							value = CONFIG_PARAM_BUILD_NUMBER_HIGH;
							break;
						case PARAM_HW_VER:
							value = CONFIG_PARAM_HW_VER;
							break;
						case PARAM_SW_MAJOR:
							value = CONFIG_PARAM_SW_MAJOR;
							break;
						case PARAM_SW_MINOR:
							value = CONFIG_PARAM_SW_MINOR;
							break;
						default:
							value = 0;
							break;
						}
						msgLength = 3;
						msgBuffer[1] = STATUS_CMD_OK;
						msgBuffer[2] = value;
					}
					break;

				case CMD_LEAVE_PROGMODE_ISP:
					isLeave	=	1;
					//*	fall thru

				case CMD_SET_PARAMETER:
				case CMD_ENTER_PROGMODE_ISP:
					msgLength = 2;
					msgBuffer[1] = STATUS_CMD_OK;
					break;

				case CMD_READ_SIGNATURE_ISP:
					{
						uint8_t signatureIndex = msgBuffer[4];
						uint8_t signature;

						if (signatureIndex == 0)
							signature = SIGNATURE_0;
						else if (signatureIndex == 1)
							signature = SIGNATURE_1;
						else
							signature = SIGNATURE_2;

						msgLength = 4;
						msgBuffer[1] = STATUS_CMD_OK;
						msgBuffer[2] = signature;
						msgBuffer[3] = STATUS_CMD_OK;
					}
					break;

				case CMD_READ_LOCK_ISP:
					msgLength = 4;
					msgBuffer[1] = STATUS_CMD_OK;
					msgBuffer[2] = boot_lock_fuse_bits_get(GET_LOCK_BITS);
					msgBuffer[3] = STATUS_CMD_OK;
					break;

				case CMD_READ_FUSE_ISP:
					{
						uint8_t fuseBits;

						if (msgBuffer[2] == 0x50)
						{
							if (msgBuffer[3] == 0x08)
								fuseBits = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
							else
								fuseBits = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
						}
						else
						{
							fuseBits = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
						}
						msgLength = 4;
						msgBuffer[1] = STATUS_CMD_OK;
						msgBuffer[2] = fuseBits;
						msgBuffer[3] = STATUS_CMD_OK;
					}
					break;

	#ifndef REMOVE_PROGRAM_LOCK_BIT_SUPPORT
				case CMD_PROGRAM_LOCK_ISP:
					{
						uint8_t lockBits = msgBuffer[4];

						lockBits = (~lockBits) & 0x3C; // mask BLBxx bits
						boot_lock_bits_set(lockBits); // and program it
						boot_spm_busy_wait();

						msgLength = 3;
						msgBuffer[1] = STATUS_CMD_OK;
						msgBuffer[2] = STATUS_CMD_OK;
					}
					break;
	#endif
				case CMD_CHIP_ERASE_ISP: //clear both RWW flash and EEPROM
					{
						eraseAddress = 0;
						msgLength = 2;
						while (eraseAddress < BOOTLOADER_ADDRESS)
						{
							boot_page_erase(eraseAddress); // Perform page erase
							boot_spm_busy_wait(); // Wait until the memory is erased.
							eraseAddress += SPM_PAGESIZE; // point to next page to be erase
						}
						boot_rww_enable(); // Re-enable the RWW section
						for (uint16_t i = 0; i <= E2END; i++)
							eeprom_update_byte((uint8_t*)i, 0xFF);
						eeprom_busy_wait(); //wait for eeprom operations to complete
						msgBuffer[1] = STATUS_CMD_OK;
					}
					break;

				case CMD_LOAD_ADDRESS:
	#if (FLASHEND > 0x10000)
					address	= (((address_t)(msgBuffer[1]) << 24) | ((address_t)(msgBuffer[2]) << 16) | ((address_t)(msgBuffer[3]) << 8) | msgBuffer[4]) << 1;
	#else
					address	= ((msgBuffer[3] << 8) | msgBuffer[4]) << 1; //convert word to byte address
	#endif
					msgLength = 2;
					msgBuffer[1] = STATUS_CMD_OK;
					break;

				case CMD_SET_UPLOAD_SIZE_PRUSA3D:
					((uint8_t*)&flashSize)[0] = msgBuffer[1];
					((uint8_t*)&flashSize)[1] = msgBuffer[2];
					((uint8_t*)&flashSize)[2] = msgBuffer[3];
					((uint8_t*)&flashSize)[3] = 0;
					msgLength = 2;
					msgBuffer[1] = STATUS_CMD_OK;
					break;

				case CMD_PROGRAM_FLASH_ISP:
				case CMD_PROGRAM_EEPROM_ISP:
					{
						uint16_t size = (msgBuffer[1] << 8) | msgBuffer[2];
						uint8_t *p = msgBuffer + 10;
						uint16_t data;
						uint8_t highByte, lowByte;
						address_t tempaddress = address;

						if (msgBuffer[0] == CMD_PROGRAM_FLASH_ISP)
						{
							// Write FLASH
							if (flashSize != 0)
							{
								if (address == 0) //first page
								{
									flashCounter = size; //initial value = size
									flashAddressLast = 0; //last 
									flashOperation = 1; //write
								}
								else if (address != flashAddressLast)
									flashCounter += size; //add size to counter
								flashAddressLast = address;
							}
							
							if (address < BOOTLOADER_ADDRESS)
							{
								uint8_t skipFlash = 1;
								uint16_t existingData;
								uint8_t* _p = p;
								for (uint16_t i = 0; i < size; i += 2)
								{
#if (FLASHEND > 0x10000)
									existingData = pgm_read_word_far(address + i);
#else
									existingData = pgm_read_word_near(address + i);
#endif
									lowByte = *_p++;
									highByte = *_p++;

									data = (highByte << 8) | lowByte;
									if (existingData != data)
									{
										skipFlash = 0;
										if (address + SPM_PAGESIZE > eraseAddress)
										{
											boot_page_erase(address); // Perform page erase
											boot_spm_busy_wait(); // Wait until the memory is erased.
											eraseAddress = address + SPM_PAGESIZE;
										}
										break;
									}
								}
								
								if (!skipFlash)
								{
									do
									{
										lowByte = *p++;
										highByte = *p++;

										data = (highByte << 8) | lowByte;
										boot_page_fill(address,data);

										address += 2; // Select next word in memory
										size -= 2; // Reduce number of bytes to write by two
									} while (size); // Loop until all bytes written

									boot_page_write(tempaddress);
									boot_spm_busy_wait();
									boot_rww_enable(); // Re-enable the RWW section
								}
							}
						}
						else
						{
							// write EEPROM
							eeprom_update_block(p, (uint8_t*)((uint16_t)(address) >> 1), size);
							address += size * 2;
							p += size;
							eeprom_busy_wait(); //wait for eeprom operations to complete
						}
						msgLength = 2;
						msgBuffer[1] = STATUS_CMD_OK;

					}
					break;

				case CMD_READ_FLASH_ISP:
				case CMD_READ_EEPROM_ISP:
					{
						uint16_t size = ((msgBuffer[1])<<8) | msgBuffer[2];
						uint8_t *p = msgBuffer + 1;
						msgLength = size + 3;

						*p++ = STATUS_CMD_OK;
						if (msgBuffer[0] == CMD_READ_FLASH_ISP )
						{
							if (flashSize != 0)
							{
								if ((address == 0x00000) && (flashOperation == 1))
								{
									flashOperation = 2; //verify
									flashCounter = size; //initial value = size
								}
								else
									flashCounter += size; //add size to counter
							}

							uint16_t data;

							// Read FLASH
							do
							{
						#if (FLASHEND > 0x10000)
								data = pgm_read_word_far(address);
						#else
								data = pgm_read_word_near(address);
						#endif
								*p++ = (uint8_t)data; // LSB
								*p++ = (uint8_t)(data >> 8); // MSB
								address += 2; // Select next word in memory
								size -= 2;
							} while (size);
						}
						else
						{
							// Read EEPROM
							eeprom_read_block((uint8_t*)p, (uint8_t*)((uint16_t)(address) >> 1), size);
							address += size * 2;
							p += size;
						}
						*p++ = STATUS_CMD_OK;
					}
					break;

				default:
					msgLength = 2;
					msgBuffer[1] = STATUS_CMD_FAILED;
					break;
			}

			// Now send answer message back
			sendchar(MESSAGE_START);
			checksum = MESSAGE_START;

			sendchar(seqNum);
			checksum ^= seqNum;

			c = ((msgLength >> 8) & 0xFF);
			sendchar(c);
			checksum ^= c;

			c = msgLength & 0x00FF;
			sendchar(c);
			checksum ^= c;

			sendchar(TOKEN);
			checksum ^= TOKEN;

			p = msgBuffer;
			while (msgLength)
			{
				c = *p++;
				sendchar(c);
				checksum ^= c;
				msgLength--;
			}
			sendchar(checksum);
			seqNum++;
	
		#ifndef REMOVE_BOOTLOADER_LED
			PROGLED_RPORT |= _BV(PROGLED_PIN); // toggle the LED
		#endif
		}
	}


	exit: // Now leave bootloader

#ifndef REMOVE_BOOTLOADER_LED
	PROGLED_DDR &= ~_BV(PROGLED_PIN); // set to default
	PROGLED_PORT &= ~_BV(PROGLED_PIN); // set to default
	delay_ms(100); // delay after exit
#endif

#if UART_BAUDRATE_DOUBLE_SPEED
	UART_STATUS_REG &= ~_BV(UART_DOUBLE_SPEED);
#ifdef DUALSERIAL
	UART_STATUS_REG1 &= ~_BV(UART_DOUBLE_SPEED1);
#endif
#endif

	boot_rww_enable(); // enable application section
	jumpToUserSpace();

	// Never return to stop GCC to generate exit return code
	// Actually we will never reach this point, but the compiler doesn't
	// understand this
	
	for(;;);
}
