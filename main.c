// vim: ts=4 sts=4 sw=4 expandtab
//******************************************************************************
//   MSP430G2xx3 - USCI_A0, 9600 UART Echo ISR, DCO SMCLK
//
//   USCI_A0 RX interrupt triggers TX Echo.
//   Baud rate divider with 1MHz = 1MHz/9600 = ~104.2
//   ACLK = n/a, MCLK = SMCLK = CALxxx_1MHZ = 1MHz
//
//                MSP430G2xx3
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |     P1.1/UCA0RXD|<------------ green (blue jumper)
//            |                 | 9600 - 8N1
//            |     P1.2/UCA0TXD|------------> white
//            |                 |
//            |             P1.0|--> RCLK_F
//            |             P1.3|--> SER_F
//            |             P1.4|--> SRCLK_F
//            |                 |
//            |             P1.5|--> RCLK_DA
//            |             P1.6|--> SER_DA
//            |             P1.7|--> SRCLK_DA
//            |                 |
//
// Launchpad silkscreen is mis-labeled, TXD/RXD are backwards.
//
//   Output format:
//
//    Data/Address shift regs
//    |2222 1111|1111 11  |         |
//    |3210 9876|5432 1098|7654 3210|
//    |data (8b)|   address (16b)   |
//
//    Flags shift reg
//    |7654 3210|
//    |flag (8b)|
//         
//     flag[0] = ~OE
//     flag[1] = R/~W
//     flag[2] = ~CE
//
//
//******************************************************************************

/*** Protocol ***
  Idle state: MCU sends "ready>"

  Update echo state: "echo <on|off>"
    * Default is on: incoming characters are echoed, except when receiving data

  Read from EEPROM: "read <start-addr> <end-addr>"
    * Addresses are in hexadecimal, e.g. "read 0x0000 0x7fff"
    * The end address is inclusive, so to read a single byte, "read 0x5830 0x5830"
    * MCU will send the bytes immediately with no delimiter.  When transfer
      is complete, will return to "ready>"

  Write to EEPROM: "write <start-addr> <end-addr> <[no]page>"
    * Addresses are in hexadecimal, e.g. "read 0x0000 0x7fff"
    * MCU will go into programming mode. In paged mode, up to 64 bytes are read
      from the serial interface, and then written in a burst, followed by a
      10ms pause, and then another 64 bytes are read.  In non-paged mode,
      one byte is written at a time, with a 10ms pause after each byte.
*/

#include <msp430.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define RCLK_DA  BIT5
#define SER_DA   BIT6
#define SRCLK_DA BIT7

#define RCLK_F   BIT0
#define SRCLK_F  BIT3
#define SER_F    BIT4

#define _OE      BIT0
#define R_W      BIT1
#define _CE      BIT2
#define data_OE  BIT3

#define true    1
#define false   0

// Make 16-bit and 8-bit unsigned ints explicit
typedef unsigned int uint16_t;
typedef unsigned char uint8_t;

// serial routines
void send_str(char *);
void echo(char);
// global vars
char echo_mode = true;
char cmd[128];
char write_buf[64];
int write_buf_size;
char write_page_mode = true;
#define MODE_CMD    0
#define MODE_READ   1
#define MODE_WRITE  2
char run_mode = MODE_CMD;
uint16_t cur_write_addr;
uint16_t end_write_addr;

// shift register routines
void send(uint8_t *, uint8_t);
void send_flags(uint8_t);
void send_data(uint16_t, uint8_t);

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    if (CALBC1_16MHZ==0xFF) while(1);         // If calibration constant erased, do not load, trap CPU!!	
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_16MHZ;                   // Set DCO
    DCOCTL = CALDCO_16MHZ;
    P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2 ;                    // P1.1 = RXD, P1.2=TXD
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK

    /*
       For table of values see MSP430 user guide,
       table 15-4 on page 424
    */
    //UCA0BR0 = 104;                            // 1MHz 9600 (104)
    //UCA0BR1 = 0;                              // 1MHz 9600 (0)
    //UCA0MCTL = UCBRS0;                        // 1MHz Modulation UCBRSx = 1
    //UCA0BR0 = 65;                             // 8MHz 9600 = 833 [low=65]
    //UCA0BR1 = 3;                              // 8MHz 9600 = 833 [hi=3<<8 := 768]
    //UCA0MCTL = UCBRS1;                        // 8MHz Modulation UCBRSx = 2
    UCA0BR0 = 130;                            // 16MHz 9600 = 1666 [low=130]
    UCA0BR1 = 6;                              // 16MHz 9600 = 1666 [hi=6<<8 := 1536]
    UCA0MCTL = UCBRS2 + UCBRS1;               // 16MHz Modulation UCBRSx = 6

    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt

    send_str("\r\nready>");

    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
}

// Serial data RX interrupt
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    char buf[64];
    char rx_char = UCA0RXBUF;
    uint16_t start_addr;
    uint16_t end_addr;
    int len;
    int i;

    switch(run_mode) {
        case MODE_CMD:
            if(echo_mode) echo(rx_char);
            // If newline (return) key, execute command
            // as specified in cmd var
            if(rx_char == 0x0d) {
                send_str("\n\r");
                if(strncmp(cmd, "echo", 4) == 0) {
                    if(strcmp(cmd, "echo on") == 0) {
                        echo_mode = true;
                    }
                    else if(strcmp(cmd, "echo off") == 0) {
                        echo_mode = false;
                    }
                    else {
                        if(echo_mode)
                            sprintf(buf, "Current echo mode: %d (true)\r\n", echo_mode);
                        else
                            sprintf(buf, "Current echo mode: %d (false)\r\n", echo_mode);
                        send_str(buf);
                    }
                }
                else if(strncmp(cmd, "read", 4) == 0) {
                    if(strlen(cmd) != 18) {
                        sprintf(buf, "Invalid read command: wrong length: %d, expecting 18\r\n", strlen(cmd));
                        send_str(buf);
                        goto ready;
                    }
                    // read 0xabcd 0xef01
                    // 0      ^7     ^14
                    start_addr = strtoul(&cmd[7], 0, 16);
                    end_addr = strtoul(&cmd[14], 0, 16);
                    if(start_addr == 0 && strncmp(&cmd[7], "0000", 4) != 0) {
                        send_str("Invalid read command: cannot parse start addr\r\n");
                        goto ready;
                    }
                    if(end_addr == 0 && strncmp(&cmd[14], "0000", 4) != 0) {
                        send_str("Invalid read command: cannot parse end addr\r\n");
                        goto ready;
                    }
                    if(start_addr > end_addr) {
                        send_str("Invalid read command: start-addr > end-addr\r\n");
                        goto ready;
                    }
                    sprintf(buf, "Start addr: %04x (%d)\r\n", start_addr, start_addr);
                    send_str(buf);
                    sprintf(buf, "End addr: %04x (%d)\r\n", end_addr, end_addr);
                    send_str(buf);
                    sprintf(buf, "Sending %d bytes now...\r\n", end_addr - start_addr);
                    send_str(buf);
                    run_mode = MODE_READ;
                    for(i=start_addr; i<=end_addr; i++) {
                        sprintf(buf, "0x%04x\r\n", i);
                        send_str(buf);
                    }
                    run_mode = MODE_CMD;
                }
                else if(strncmp(cmd, "write", 5) == 0) {
                    len = strlen(cmd);
                    if(len != 19 && len != 24 && len != 26) {
                        sprintf(buf, "Invalid write command: wrong length: %d, expecting 19, 24, or 26\r\n", len);
                        send_str(buf);
                        goto ready;
                    }
                    // write 0xabcd 0xef01 nopage
                    // 0       ^8     ^15  ^20
                    start_addr = strtoul(&cmd[8], 0, 16);
                    end_addr = strtoul(&cmd[15], 0, 16);
                    if(start_addr == 0 && strncmp(&cmd[8], "0000", 4) != 0) {
                        send_str("Invalid write command: cannot parse start addr\r\n");
                        goto ready;
                    }
                    if(end_addr == 0 && strncmp(&cmd[15], "0000", 4) != 0) {
                        send_str("Invalid write command: cannot parse end addr\r\n");
                        goto ready;
                    }
                    if(start_addr > end_addr) {
                        send_str("Invalid write command: start-addr > end-addr\r\n");
                        goto ready;
                    }
                    if(len == 19 || strcmp(&cmd[20], "page") == 0) {
                        write_page_mode = true;
                    }
                    else if(strcmp(&cmd[20], "nopage") == 0) {
                        write_page_mode = false;
                    }
                    else {
                        send_str("Invalid write command: page/nopage not parsed\r\n");
                        goto ready;
                    }

                    sprintf(buf, "Start addr: %04x (%d)\r\n", start_addr, start_addr);
                    send_str(buf);
                    sprintf(buf, "End addr: %04x (%d)\r\n", end_addr, end_addr);
                    send_str(buf);
                    sprintf(buf, "Total bytes to write: %d\r\n", end_addr - start_addr + 1);
                    send_str(buf);
                    if(write_page_mode) {
                        send_str("Paging\r\n");
                        // Compute the size of the first page
                        len = 64 - (start_addr % 64);
                        if(len == 0) len = 64; // start addr is on a boundary
                        if(start_addr + len > end_addr)
                            sprintf(buf, "Send %d bytes, %d remaining...\r\n", end_addr - start_addr + 1, end_addr - start_addr + 1);
                        else
                            sprintf(buf, "Send %d bytes, %d remaining...\r\n", len, end_addr - start_addr + 1);
                        send_str(buf);
                    }
                    else {
                        send_str("No Paging\r\n");
                        sprintf(buf, "Send %d bytes, %d remaining...\r\n", 1, end_addr - start_addr + 1);
                        send_str(buf);
                    }
                    cur_write_addr = start_addr;
                    end_write_addr = end_addr;
                    write_buf_size = 0;
                    run_mode = MODE_WRITE;
                }
                else {
                    sprintf(buf, "Invalid Command: %s\r\n", cmd);
                    send_str(buf);
                }
                ready:
                strcpy(cmd, "");
                if(run_mode == MODE_CMD)
                    send_str("ready>");
            }
            // Otherwise, append to cmd
            else {
                strncat(cmd, &UCA0RXBUF, 1);
            }
            break;
        case MODE_READ:
            break;
        case MODE_WRITE:
            if(write_page_mode) {
                write_buf[write_buf_size] = UCA0RXBUF;
                //sprintf(buf, "Cached byte %02x for writing\r\n", write_buf[write_buf_size]);
                //send_str(buf);
                write_buf_size++;
                // Write out our buffered data if...
                // * the buffer is full, or
                // * we are at the end of our data stream, or
                // * a 64-byte boundary appears in the buffer (this aligns the following writes to pages)
                if(write_buf_size == 64 
                        || cur_write_addr + write_buf_size > end_write_addr
                        || (cur_write_addr + write_buf_size) % 64 == 0
                        ) {
                    sprintf(buf, "Writing page of %d bytes starting at 0x%04x\r\n", write_buf_size, cur_write_addr);
                    send_str(buf);
                    for(i=0; i<write_buf_size; i++) {
                        sprintf(buf, "%04x = %02x\r\n", cur_write_addr, write_buf[i]);
                        send_str(buf);
                        cur_write_addr++;
                    }
                    send_str("Pausing 10ms\r\n");
                    __delay_cycles(200000); // 200k cycles @ 16MHz => 12.5ms
                    write_buf_size = 0;

                    if(cur_write_addr <= end_write_addr) {
                        // Compute the size of the next page
                        len = 64 - (cur_write_addr % 64);
                        if(len == 0) len = 64; // cur_write_addr is on a boundary
                        if(cur_write_addr + len > end_write_addr)
                            sprintf(buf, "Send %d bytes, %d remaining...\r\n", end_write_addr - cur_write_addr + 1, end_write_addr - cur_write_addr + 1);
                        else
                            sprintf(buf, "Send %d bytes, %d remaining...\r\n", len, end_write_addr - cur_write_addr + 1);
                        send_str(buf);
                    }
                }
            }
            else {
                sprintf(buf, "Writing byte %02x to addr 0x%04x\r\n", UCA0RXBUF, cur_write_addr);
                send_str(buf);
                cur_write_addr++;
                send_str("Pausing 10ms\r\n");
                __delay_cycles(200000); // 200k cycles @ 16MHz => 12.5ms
                sprintf(buf, "Send %d bytes, %d remaining...\r\n", 1, end_write_addr - cur_write_addr + 1);
                send_str(buf);
            }
            if(cur_write_addr > end_write_addr) {
                run_mode = MODE_CMD;
                send_str("ready>");
            }
            break;
    } // end switch
}

// Echos the RXBUF back to the client
void echo(char chr) {
    char to_send[2];
    to_send[0] = chr;
    to_send[1] = 0x00;
    send_str(to_send);
}

// Sends a string
void send_str(char *str) {
    char *ptr = str;
    while(*ptr) {
        while (!(IFG2 & UCA0TXIFG));            // USCI_A0 TX buffer ready?
        UCA0TXBUF = *ptr;
        ptr++;
    }
}

void send(uint8_t *data_arr, uint8_t count) {
    uint8_t SER;
    uint8_t SRCLK;
    uint8_t RCLK;
    // If just one byte, the data must be
    // flags, so use the _F ports, otherwise
    // use the _DA ports
    if(count == 1) {
        SER = SER_F;
        SRCLK = SRCLK_F;
        RCLK = RCLK_F;
    }
    else {
        SER = SER_DA;
        SRCLK = SRCLK_DA;
        RCLK = RCLK_DA;
    }

    uint8_t mask;

    for(uint8_t idx=0; idx<count; idx++) {
        mask = 0x01;
        uint8_t this_byte = data_arr[idx];
        for(uint8_t bit = 0; bit <= 7; bit++) {
            // Put this bit on the serial input
            if(this_byte & mask)
                P1OUT |= SER;
            else
                P1OUT &= ~SER;

            // Strobe the SRCLK to shift
            // this bit into the shift register
            P1OUT |= SRCLK;
            P1OUT &= ~SRCLK;

            // Shift our mask to the next bit
            mask = mask << 1;
        }
    }

    // reset SER
    P1OUT &= ~SER;

    // Strobe the RCLK to load the shifted
    // data into the output register
    P1OUT |= RCLK;
    P1OUT &= ~RCLK;

    return;
}

void send_flags(uint8_t flags) {
    send(&flags, 1);
    return;
}

void send_data(uint16_t send_addr, uint8_t send_data) {
    uint8_t to_send[3];
    to_send[0] = (send_addr & 0x00ff);
    to_send[1] = ((send_addr & 0xff00) >> 8);
    to_send[2] = send_data;
    send(to_send, 3);
    return;
}

