/* 
Bachelor´s work - 7 Band Digital Equalizer
Filip Tuma (2024)
*/

/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
#define F_CPU 16000000 // CPU frequency in Hz required for UART_BAUD_SELECT
#endif

#define u8 uint8_t
#define string uint8_t *

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>        // AVR device-specific IO definitions
#include <avr/interrupt.h> // Interrupts standard C library for AVR-GCC
#include "timer.h"         // Timer library for AVR-GCC
#include <uart.h>          // Peter Fleury's UART library
#include <stdlib.h>        // C library. Needed for number conversions
#include <twi.h>           // I2C/TWI library for AVR-GCC
#include <gpio.h>          // GPIO library for AVR-GCC
#include <util/delay.h>    // Functions for busy-wait delay loops
#include <avr/pgmspace.h>

// Sampling Rate 44.1 kHz
//#include "equalizer_registers.h"
//#include "pp_1kz.h"
//#include "transport.h"
//#include "agc.h"

// Sampling Rate 48 kHz
//#include "equalizer_pop.h"
//#include "equalizer_rap.h"
//#include "equalizer_rock.h"
#include "vstup_vystup.h"

// Hexadecimal I2C Address of TLV320AIC3254
#define SENSOR_ADR 0x18

// Function for sending register via I2C
void equ_write_full(reg_value const *reg_values, size_t size)
{
    /*
    // Reading of register (testing) 
    reg_values++;
    char str[8]; // str for converted numbers by itoa()
    itoa(pgm_read_byte_near(&reg_values->reg_off), str, 10);
    uart_puts("Testing:");
    uart_puts(str);
    uart_puts("\r\n");
    */

    for (size_t i = 0; i < size; i++)
    {
        twi_start();
        twi_write((SENSOR_ADR<<1) | TWI_WRITE);

        // Write into defined register        
        twi_write(pgm_read_byte_near(&reg_values->reg_off));
        // Write into register defined value
        twi_write(pgm_read_byte_near(&reg_values->reg_val));
        
        twi_stop();
        reg_values++;
    }
}

int main(void)
{

    // Initialize USART to asynchronous
    uart_init(UART_BAUD_SELECT(9600, F_CPU));

    // Enables interrupts by setting the global interrupt mask
    sei();

    // Set low pin D8 on Arduino UNO (RESET of TLV320AIC3254)
    GPIO_mode_output(&DDRB, 0);
    GPIO_write_low(&PORTB, 0);
    _delay_ms(1);
    GPIO_write_high(&PORTB, 0);

    // Control of connection 
    uart_puts("Starting... ");
    uart_puts("Is AIC3254 connected?\r\n");

    twi_init();

    if (twi_test_address(SENSOR_ADR) == 0)
        uart_puts("Yes, device is ready\r\n");
    else
    {
        uart_puts("No, device not found\r\n");
    }
/*
    twi_start();
    twi_write((SENSOR_ADR << 1) | TWI_WRITE);
    twi_write(0x00);
    twi_write(0x01);
    twi_stop();

    twi_start();
    twi_write((SENSOR_ADR << 1) | TWI_WRITE);
    twi_write(0x12);
    
    twi_stop();
    // Read data from internal memory
    twi_start();
    twi_write((SENSOR_ADR << 1) | TWI_READ);
    uint8_t ret2 = twi_read(TWI_ACK);
    twi_stop();

    char str2[8]; // str for converted numbers by itoa()
    itoa(ret2, str2, 10);
    uart_puts("First reg :");
    uart_puts(str2);
    uart_puts("\r\n");
    
    twi_start();
    twi_write((SENSOR_ADR << 1) | TWI_WRITE);
    twi_write(0x05);
    twi_stop();
    // Read data from internal memory
    twi_start();
    twi_write((SENSOR_ADR << 1) | TWI_READ);
    uint8_t ret = twi_read(TWI_ACK);
    uint8_t ret2 = twi_read(TWI_NACK);
    twi_stop();

    char str[8]; // str for converted numbers by itoa()
    itoa(ret, str, 10);
    uart_puts("test of reg:");
    uart_puts(str);
    uart_puts("\r\n");
    itoa(ret2, str, 10);
    uart_puts("another test:");
    uart_puts(str);
    uart_puts("\r\n");
*/

    // Send registers into TLV320AIC3254
    equ_write_full(REG_Section_program, sizeof(REG_Section_program) / sizeof(REG_Section_program[0]));
    equ_write_full(miniDSP_A_reg_values, miniDSP_A_reg_values_COEFF_SIZE + miniDSP_A_reg_values_INST_SIZE);
    equ_write_full(miniDSP_D_reg_values, miniDSP_D_reg_values_COEFF_SIZE + miniDSP_D_reg_values_INST_SIZE);

    uart_puts("Upload of registers is done!\r\n");
    
/*
    twi_start();
    twi_write((SENSOR_ADR << 1) | TWI_WRITE);
    twi_write(0x00);
    twi_write(0x01);
    twi_stop();

    twi_start();
    twi_write((SENSOR_ADR << 1) | TWI_WRITE);
    twi_write(0x12);
    
    twi_stop();
    // Read data from internal memory
    twi_start();
    twi_write((SENSOR_ADR << 1) | TWI_READ);
    uint8_t ret = twi_read(TWI_ACK);
    twi_stop();

    char str[8]; // str for converted numbers by itoa()
    itoa(ret, str, 10);
    uart_puts("Page reg (161?):");
    uart_puts(str);
    uart_puts("\r\n");
*/
    // Infinite loop
    while (1)
    {   
    }

    // Will never reach this
    return 0;
}