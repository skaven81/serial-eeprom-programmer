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
// command processing routines
void cmd_echo();
void cmd_read();
void cmd_write();

// global vars
char echo_mode = true;
char cmd[32];
char write_buf[64];
int write_buf_idx = 0;
int write_buf_target_size = 0;
char write_mode = false;
uint16_t cur_write_addr;
uint16_t end_write_addr;

// shift register routines
void shiftreg_send(uint8_t *, uint8_t);
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

    send_str("\r\n");
    while(1) {
        // Send ready prompt
        send_str("ready>");

        // Enable USCI_A0 RX interrupt
        IE2 |= UCA0RXIE;

        // Go to sleep while RX interrupt routine
        // builds command
        __bis_SR_register(LPM0_bits + GIE);

        // Disable the serial interrupts while we
        // process the command
        IE2 &= ~UCA0RXIE;

        // Analyze the command and call the
        // appropriate routine
        if(strncmp(cmd, "echo", 4) == 0)
            cmd_echo();
        else if(strncmp(cmd, "read", 4) == 0)
            cmd_read();
        else if(strncmp(cmd, "write", 5) == 0)
            cmd_write();
        else if(strncmp(cmd, "help", 4) == 0)
            cmd_help();
        else
            send_str("Invalid command\r\n");
        strcpy(cmd, "");
    }
}

// Help command
void cmd_help() {
    send_str("help: this help information\r\n");
    send_str("echo {on,off}: display, enable, disable echo\r\n");
    send_str("read 0xabcd 0xef01: read bytes from start to end addr, inclusive\r\n");
    send_str("write 0xabcd 0xef01 {nopage}: write bytes from start to end addr.\r\n");
    send_str("  If nopage, bytes will be written individually with 10ms pauses\r\n");
    send_str("  between each byte. By default, will RX bytes up to next 64-byte\r\n");
    send_str("  boundary and write up to 64 bytes as a page.\r\n");
}

// Echo command: change echo_mode
void cmd_echo() {
    char buf[64];
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

// Read command: read from EEPROM
void cmd_read() {
    char buf[64];
    uint16_t start_addr;
    uint16_t end_addr;

    if(strlen(cmd) != 18) {
        sprintf(buf, "Invalid read command: wrong length: %d, expecting 18\r\n", strlen(cmd));
        send_str(buf);
        return;
    }
    // read 0xabcd 0xef01
    // 0      ^7     ^14
    start_addr = strtoul(&cmd[7], 0, 16);
    end_addr = strtoul(&cmd[14], 0, 16);
    if(start_addr == 0 && strncmp(&cmd[7], "0000", 4) != 0) {
        send_str("Invalid read command: cannot parse start addr\r\n");
        return;
    }
    if(end_addr == 0 && strncmp(&cmd[14], "0000", 4) != 0) {
        send_str("Invalid read command: cannot parse end addr\r\n");
        return;
    }
    if(start_addr > end_addr) {
        send_str("Invalid read command: start-addr > end-addr\r\n");
        return;
    }
    sprintf(buf, "Start addr: %04x (%d)\r\n", start_addr, start_addr);
    send_str(buf);
    sprintf(buf, "End addr: %04x (%d)\r\n", end_addr, end_addr);
    send_str(buf);
    sprintf(buf, "Sending %d bytes now...\r\n", end_addr - start_addr + 1);
    send_str(buf);
    for(int i=start_addr; i<=end_addr; i++) {
        sprintf(buf, "0x%04x %02x\r\n", i, i);
        send_str(buf);
    }
}

// Write command: write to EEPROM
void cmd_write() {
    char buf[128];
    char write_page_mode;
    int len;

    len = strlen(cmd);
    if(len != 19 && len != 24 && len != 26) {
        sprintf(buf, "Invalid write command: wrong length: %d, expecting 19, 24, or 26\r\n", len);
        send_str(buf);
        return;
    }
    // write 0xabcd 0xef01 nopage
    // 0       ^8     ^15  ^20
    cur_write_addr = strtoul(&cmd[8], 0, 16);
    end_write_addr = strtoul(&cmd[15], 0, 16);
    if(cur_write_addr == 0 && strncmp(&cmd[8], "0000", 4) != 0) {
        send_str("Invalid write command: cannot parse start addr\r\n");
        return;
    }
    if(end_write_addr == 0 && strncmp(&cmd[15], "0000", 4) != 0) {
        send_str("Invalid write command: cannot parse end addr\r\n");
        return;
    }
    if(cur_write_addr > end_write_addr) {
        send_str("Invalid write command: start-addr > end-addr\r\n");
        return;
    }
    if(len == 19 || strcmp(&cmd[20], "page") == 0) {
        write_page_mode = true;
    }
    else if(strcmp(&cmd[20], "nopage") == 0) {
        write_page_mode = false;
    }
    else {
        send_str("Invalid write command: page/nopage not parsed\r\n");
        return;
    }

    sprintf(buf, "Start addr: %04x (%d)\r\n", cur_write_addr, cur_write_addr);
    send_str(buf);
    sprintf(buf, "End addr: %04x (%d)\r\n", end_write_addr, end_write_addr);
    send_str(buf);
    sprintf(buf, "Total bytes to write: %d\r\n", end_write_addr - cur_write_addr + 1);
    send_str(buf);
    if(write_page_mode)
        send_str("Paging\r\n");
    else
        send_str("No Paging\r\n");

    // Enable write mode in the serial RX interrupt routine
    write_mode = true;

    while(cur_write_addr <= end_write_addr) {
        // Figure out how many bytes we want this round
        if(!write_page_mode) {
            write_buf_target_size = 1;
        }
        else {
            len = 64 - (cur_write_addr % 64);
            if(len == 0) len = 64; // start addr is on a boundary
            if(cur_write_addr + len > end_write_addr)
                write_buf_target_size = end_write_addr - cur_write_addr + 1;
            else
                write_buf_target_size = len;
        }
        
        // Prompt the client to send that much data
        sprintf(buf, "Send %d bytes, %d remaining...\r\n", write_buf_target_size, end_write_addr - cur_write_addr + 1);
        send_str(buf);

        // Reset write_buf_idx so new data arrives
        // at beginning of write_buf
        write_buf_idx = 0;

        // Enable USCI_A0 RX interrupt
        IE2 |= UCA0RXIE;

        // Go to sleep while RX interrupt routine
        // collects write_buf_target_size bytes
        __bis_SR_register(LPM0_bits + GIE);

        // Disable the serial interrupts while we
        // process the received data
        IE2 &= ~UCA0RXIE;

        sprintf(buf, "Writing %d bytes starting at 0x%04x\r\n", write_buf_target_size, cur_write_addr);
        send_str(buf);
        for(int i=0; i<write_buf_idx; i++) {
            sprintf(buf, "%04x = %02x\r\n", cur_write_addr, write_buf[i]);
            send_str(buf);
            cur_write_addr++;
        }
        send_str("Pausing 10ms\r\n");
        __delay_cycles(200000); // 200k cycles @ 16MHz => 12.5ms
    }

    // Disable write mode in the serial RX interrupt routine
    write_mode = false;
}

// Serial data RX interrupt
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
    if(write_mode) {
        write_buf[write_buf_idx] = UCA0RXBUF;
        write_buf_idx++;
        // once we have collected enough bytes, wake the CPU back up
        if(write_buf_idx >= write_buf_target_size) {
            __bic_SR_register_on_exit(LPM0_bits);
        }
    }
    else if(UCA0RXBUF == 0x0d) {
        if(echo_mode) send_str("\r\n");
        // when command is complete, wake the CPU back up
        __bic_SR_register_on_exit(LPM0_bits);
    }
    else {
        if(echo_mode) echo(UCA0RXBUF);
        strncat(cmd, &UCA0RXBUF, 1);
    }

}

// Echos a char back to the client
void echo(char chr) {
    char to_send[2];
    to_send[0] = chr;
    to_send[1] = 0x00;
    send_str(to_send);
}

// Sends a string to the client
void send_str(char *str) {
    char *ptr = str;
    while(*ptr) {
        while (!(IFG2 & UCA0TXIFG)); // USCI_A0 TX buffer ready?
        UCA0TXBUF = *ptr;
        ptr++;
    }
}

// Send a one-byte set of flags to the flags shift register
void send_flags(uint8_t flags) {
    shiftreg_send(&flags, 1);
    return;
}

// Send three bytes (2-byte addr + 1 byte data) to the
// address+data shift register
void send_data(uint16_t send_addr, uint8_t send_data) {
    uint8_t to_send[3];
    to_send[0] = (send_addr & 0x00ff);
    to_send[1] = ((send_addr & 0xff00) >> 8);
    to_send[2] = send_data;
    shiftreg_send(to_send, 3);
    return;
}

// Generic shift register interface
void shiftreg_send(uint8_t *data_arr, uint8_t count) {
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


