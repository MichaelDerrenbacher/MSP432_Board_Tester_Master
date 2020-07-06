
#include "msp.h"
#include "uart.h"

/*
 * uart.c
 *
 *  Created on: Jul 6, 2020
 *      Author: michael
 */


/*
 *  Configure UART for communication
 */
void init_uart(void)
{
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST;       // software reset

    EUSCI_A0->CTLW0 |= (EUSCI_A_CTLW0_PEN        // odd parity
                    | EUSCI_A_CTLW0_SPB          // 2 stop bits
                    | EUSCI_A_CTLW0_MODE_0       // Uart A
                    | EUSCI_A_CTLW0_UCSSEL_2);   // SMCLK

    EUSCI_A0->BRW = 0x01;

    EUSCI_A0->MCTLW = ((10 << EUSCI_A_MCTLW_BRF_OFS) | EUSCI_A_MCTLW_OS16);

    P1->SEL0 |=  (BIT2 | BIT3);     // pins P1.2, P1.3 used as RX and TX
    P1->SEL1 &= ~(BIT2 | BIT3);

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;     // end reset
}

/*
 *  Send 4 ASCII values in DWORD
 */
void send_characters(uint32_t d_word)
{
    int i;
    uint8_t tx_data;

    for(i = 3; i >= 0; i--)     // send in 1 byte at a time, starting at MSB
    {
        tx_data = (0xFF & (d_word >> i*8));
        EUSCI_A0->TXBUF = tx_data;
        while(!((EUSCI_A0->IFG) & EUSCI_A_IFG_TXIFG));
    }
}


/*
 *  Change cursor position to [row, col]
 */
void cursor_pos(uint8_t row, uint8_t col)
{
    uint64_t command;
    // send 2 4 bit command sequences to move cursor to location

    command = (ESC << 24) | ('[' << 16) | ((row + '0') << 8) | (';');
    send_characters(command);
    while(!((EUSCI_A0->IFG) & EUSCI_A_IFG_TXIFG));

    command = ((col + '0') << 24) | ('H' << 16);
    send_characters(command);
}


void down_line(uint8_t lines)
{
    uint8_t counter;
    uint64_t command;
    // send 2 4 bit command sequences to move cursor to location

    command = (ESC << 24) | ('[' << 16) | ('1' << 8) | ('B');
    for(counter = 0; counter < lines; counter++)
    {
        send_characters(command);
        while(!((EUSCI_A0->IFG) & EUSCI_A_IFG_TXIFG));
    }


}
