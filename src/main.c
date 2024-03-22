/***********************************************************************
 * 
 * Use USART unit and transmit data between ATmega328P and computer.
 * 
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2018 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/


/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
# define F_CPU 16000000  // CPU frequency in Hz required for UART_BAUD_SELECT
#endif


/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <uart.h>           // Peter Fleury's UART library
#include <stdlib.h>         // C library. Needed for number conversions
#include <twi.h>            // I2C/TWI library for AVR-GCC
#include <gpio.h>       // GPIO library for AVR-GCC
#include <util/delay.h> // Functions for busy-wait delay loops

#define SENSOR_ADR 0x18


/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Use Timer/Counter1 and transmit UART data four times 
 *           per second.
 * Returns:  none
 **********************************************************************/
int main(void)
{

    // Initialize USART to asynchronous, 8N1, 9600
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
    
    // Configure 16-bit Timer/Counter1 to transmit UART data
    // Set prescaler to 262 ms and enable overflow interrupt
    //TIM1_OVF_262MS;
    //TIM1_OVF_ENABLE;

    // Enables interrupts by setting the global interrupt mask
    sei();

    // Put strings to ringbuffer for transmitting via UART
    

    GPIO_mode_output(&DDRB, 0);
    GPIO_write_low(&PORTB, 0);
    _delay_ms(1);
    GPIO_write_high(&PORTB, 0);

    uart_puts("Print one line... ");
    uart_puts("done\r\n");

    twi_init();

    if (twi_test_address(SENSOR_ADR) == 0)
        uart_puts("yes\r\n");
    else {
        uart_puts("nooooooooooooooooooooooooo\r\n");        
    }

    twi_start();
    twi_write((SENSOR_ADR<<1) | TWI_WRITE);
    twi_write(0x05);
    twi_stop();
    // Read data from internal memory
    twi_start();
    twi_write((SENSOR_ADR<<1) | TWI_READ);
    uint8_t ret = twi_read(TWI_ACK);
    uint8_t ret2 = twi_read(TWI_NACK);
    twi_stop();
/*
    _delay_ms(1);

    twi_start();
    twi_write((SENSOR_ADR<<1) | TWI_WRITE);
    twi_write(0x05);
    twi_stop();
    // Read data from internal memory
    twi_start();
    twi_write((SENSOR_ADR<<1) | TWI_READ);
    uint8_t ret2 = twi_read(TWI_NACK);
    
    twi_stop();
*/
    char string[8];  // String for converted numbers by itoa()
    itoa(ret,string,10);
    uart_puts("fuck:");
    uart_puts(string);
    uart_puts("\r\n");
    itoa(ret2,string,10);
    uart_puts("fuck2:");
    uart_puts(string);
    uart_puts("\r\n");
    

    
    // Infinite loop
    while (1)
    {
        /* Empty loop. All subsequent operations are performed exclusively 
         * inside interrupt service routines ISRs */
    }

    // Will never reach this
    return 0;
}

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Transmit UART data four times per second.
 **********************************************************************/
ISR(TIMER1_OVF_vect)
{
  
    uint8_t value;
    char string[8];  // String for converted numbers by itoa()

    value = uart_getc();
    if (value != '\0') {  // Data available from UART
        // Display ASCII code of received character
        // WRITE YOUR CODE HERE
        itoa(value,string,10);
        uart_puts("\x1b[4;32m");  // 4: underline style; 32: green foreground
        uart_puts(string);
        uart_puts("\r\n");
        itoa(value,string,2);
        uart_puts(string);
        uart_puts("\r\n");
        itoa(value,string,16);
        uart_puts(string);
        uart_puts("\r\n");
    }
}