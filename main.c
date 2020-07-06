
#include "msp.h"
#include "uart.h"


/**
 * main.c
 */

// list of pins exposed on Lanchpad
#define PORT_1_PINS    (BIT5 | BIT6 | BIT7)
#define PORT_2_PINS    (BIT3 | BIT4 | BIT5 | BIT6 | BIT7)
#define PORT_3_PINS    (BIT0 | BIT2 | BIT3 | BIT5 | BIT6 | BIT7)
#define PORT_4_PINS    (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7)
#define PORT_5_PINS    (BIT0 | BIT1 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7)
#define PORT_6_PINS    (BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7)

const uint8_t test_pins[] = {PORT_1_PINS, PORT_2_PINS, PORT_3_PINS, PORT_4_PINS, PORT_5_PINS, PORT_6_PINS};


// Ports are separated into odd/even
DIO_PORT_Odd_Interruptable_Type* test_ports_odd[] = {P1, P3, P5};
DIO_PORT_Even_Interruptable_Type* test_ports_even[] = {P2, P4, P6};


// flags from interrupt
uint8_t test_start;
uint8_t output_test_type;


void init_test_pins(void);
void init_irq(void);

void test_ports(void);

void report_results(uint8_t port, uint8_t check);


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	init_test_pins();

	init_irq();

	init_uart();

	send_characters(CLEAR);  // clear UART

	while(1)
    {
	    while(!test_start);

	    send_characters(CLEAR);

	    if(output_test_type)
	    {
            //enable_pulldown(); // TODO if want to alternate if pullup/pulldown is active
	                             //       would make DUT have to drive harder, and disconnected
	                             //       master always reads pins stuck

	        cursor_pos(1, 2);   // set cursor to 2nd line

	        send_characters('Pins'); // I only send 4 chars at a time don't judge pls
	        send_characters(' stu');
	        send_characters('ck l');
	        send_characters('ow:');

	    }
	    else
	    {
            //enable_pullup();

            cursor_pos(1, 2);   // set cursor to 2nd line

            send_characters('Pins');
            send_characters(' stu');
            send_characters('ck h');
            send_characters('igh:');
	    }

        test_ports();

	}
}


void test_ports(void)
{
    uint8_t check;
    uint8_t index;
    uint8_t port_num;
    uint8_t pin_reading;

    for(port_num = 1; port_num <= 10; port_num++)
    {

        if(port_num & 0x01) // odd port number
        {
            // Read from odd port
            index = (port_num-1) / 2;  // 1 -> 0,  3 -> 1,  5 -> 2
            pin_reading = (test_ports_odd[index] -> IN  & test_pins[port_num-1]);
        }
        else
        {
            // Read from even port
            index = (port_num-2) / 2;  // 2 -> 0,  4 -> 1,  6 -> 2
            pin_reading = (test_ports_even[index] -> IN & test_pins[port_num-1]);
        }

        // XOR to find if results different than expected
        if(output_test_type)
        {
            check = (pin_reading ^ test_pins[port_num-1]);
        }
        else
        {
            // flip readings for output lows
            check = (pin_reading ^ ~(test_pins[port_num-1]));
        }

        report_results(port_num, check);
    }

    test_start = 0;
}



void report_results(uint8_t port, uint8_t check)
{
    uint8_t tested_pins;
    uint8_t pin_number;
    uint8_t pin_selector;

    cursor_pos(3, 2);   // reset cursor to 3rd line
    down_line(port);    // move cursor down based on port number

    // I keep ports as 1,2,3,4,..., but arrays are 0 indexed, so have to subtract 1
    tested_pins = test_pins[port-1];

    for(pin_number = 0; pin_number < 8; pin_number++)
    {
        // convert decimal into binary decoded number
        pin_selector = 1 << pin_number;

        // check if pin number is in tested array
        if(pin_selector == (tested_pins & pin_selector))
        {
            // check if tested pin didn't match what it should have been (aka bad pin)
            if(check & pin_selector)
            {
                send_characters('   P');
                send_characters(port + '0');
                send_characters('.');
                send_characters(pin_number + '0');

            }
        }
    }
}


/*
 *   Set all exposed Lanchpad pins to inputs to inputs
 *   with internal pull up resistors enabled
 *
 */
void init_test_pins(void)
{

    P1->DIR &= ~PORT_1_PINS;  // Port is all inputs
    P1->REN |=  PORT_1_PINS;  // enable internal resistor
    P1->OUT |=  PORT_1_PINS;  // resistor is pullup

    P2->DIR &= ~PORT_2_PINS;
    P2->REN |=  PORT_2_PINS;
    P2->OUT |=  PORT_2_PINS;

    P3->DIR &= ~PORT_3_PINS;
    P3->REN |=  PORT_3_PINS;
    P3->OUT |=  PORT_3_PINS;

    P4->DIR &= ~PORT_4_PINS;
    P4->REN |=  PORT_4_PINS;
    P4->OUT |=  PORT_4_PINS;

    P5->DIR &= ~PORT_5_PINS;
    P5->REN |=  PORT_5_PINS;
    P5->OUT |=  PORT_5_PINS;

    P6->DIR &= ~PORT_6_PINS;
    P6->REN |=  PORT_6_PINS;
    P6->OUT |=  PORT_6_PINS;

}

/*
 *  Enable interrupts for Lanchpad button press and UART RX
 */
void init_irq()
{

    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;

    P1->DIR &= ~(BIT1 | BIT4);         // Lanchpad buttons are inputs
    P1->REN |=  (BIT1 | BIT4);         // enable internal resistor
    P1->OUT |=  (BIT1 | BIT4);         // resistor is pullup


    P1->IES |=  (BIT1 | BIT4);         // interrupts trigger on falling edge for button press
    P1->IFG &= ~(BIT1 | BIT4);         // clear flags
    P1->IE  |=  (BIT1 | BIT4);         // enable interrupts for Lanchpad buttons

    NVIC->ISER[1] = 1 << (PORT1_IRQn & 31);      // NVIC interrupt enable for P1
    NVIC->ISER[0] = (1 << (EUSCIA0_IRQn & 31));  // NVIC interrupt enable for UART

    __enable_irq();                            // global interrupt enable

}


/*
 *  Lanchpad external buttons trigger read
 */
void PORT1_IRQHandler(void)
{
    if(P1->IFG & BIT1) // check for output low
    {
        P1->IFG &= ~(BIT1);  // clear interrupt flag

        output_test_type = 0; // testing that outputs are low
        test_start = 1;       // flag to start test

    }
    else if(P1->IFG & BIT4) // check for output high
    {
        P1->IFG &= ~(BIT4);  // clear interrupt flag

        output_test_type = 1; // testing that outputs are high
        test_start = 1;       // flag to start test

    }
    else
    {
        while(1);

        // some crazy error occurred, infinite loop for debugging purposes
        // how did you mess up this bad??? Think about whatever you've done

    }
}




