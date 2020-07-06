/*
 * uart.h
 *
 *  Created on: Jul 6, 2020
 *      Author: michael
 */

#ifndef UART_H_
#define UART_H_


// ASCII COMMAND SEQUENCES
#define CLEAR         0x1B5B324A
#define CURSOR_HOME   0x1B5B3B48
#define TAB           0x20202020
#define ESC           0x1B


void init_uart(void);
void send_characters(uint32_t long_d_word);
void cursor_pos(uint8_t row, uint8_t col);
void down_line(uint8_t lines);


#endif /* UART_H_ */
