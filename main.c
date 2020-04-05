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
//            |             P1.6|--> RCLK_F
//            |             P1.0|--> SER_F
//            |             P1.7|--> SRCLK_F
//            |                 |
//            |             P1.6|--> RCLK_A
//            |             P1.5|--> SER_A
//            |             P1.7|--> SRCLK_A
//            |                 |
//            |             P2.3|--> SER_DOUT
//            |             P2.4|--> RCLK_DOUT
//            |             P2.5|--> SRCLK_DOUT
//            |             P2.6|--> OE_DOUT
//            |                 |
//            |             P2.0|<-- DIN_QH
//            |             P2.1|--> DIN_CLK
//            |             P2.2|--> DIN_SHLD
//            |                 |
//
// Launchpad silkscreen is mis-labeled, TXD/RXD are backwards.
//
//   Output format:
//
//    Address shift reg
//    Qhgfe dcba hgfe dcba
//    |1111 11  |         |
//    |5432 1098|7654 3210|
//    |   address (16b)   |
//
//    Data out shift reg
//    Qhgfe dcba
//    |7654 3210|
//    |data (8b)|
//
//    Flags shift reg
//    Qhgfe dcba
//    |7654 3210|
//    |flag (8b)|
//         
//     flag[0]/Qc = ~CE _CE
//     flag[1]/Qb = ~OE _OE
//     flag[2]/Qa = ~WE R_W
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

// Port 1 - flags
#define RCLK_F   BIT3
#define SRCLK_F  BIT4
#define SER_F    BIT0
#define F_OUT    RCLK_F+SRCLK_F+SER_F
#define SENDMODE_FLAG 1
// Flag bits
#define _CE      BIT0
#define _OE      BIT1
#define R_W      BIT2
// Port 1 - address
#define RCLK_A   BIT6
#define SER_A    BIT5
#define SRCLK_A  BIT7
#define A_OUT    RCLK_A+SRCLK_A+SER_A
#define SENDMODE_ADDR 2

// Port 2 - data in
#define DIN_QH   BIT0
#define DIN_CLK  BIT1
#define DIN_SHLD BIT2
#define DIN_OUT  DIN_CLK+DIN_SHLD
#define DIN_IN   DIN_QH

// Port 2 - data out
#define SER_DOUT   BIT3
#define RCLK_DOUT  BIT4
#define SRCLK_DOUT BIT5
#define OE_DOUT    BIT6
#define DOUT_OUT   SER_DOUT+RCLK_DOUT+SRCLK_DOUT+OE_DOUT
#define SENDMODE_DATA 3

#define true    1
#define false   0
#define noop

// Make 16-bit and 8-bit unsigned ints explicit
typedef unsigned int uint16_t;
typedef unsigned char uint8_t;

// help routine
void cmd_help();
// serial routines
void send_str(char *);
void echo(char);
void pause_for_char();
// command processing routines
void cmd_echo();
void cmd_read();
void cmd_write();
void cmd_page_write();
void cmd_eeprom_lock();

// global vars
char echo_mode = true;
char page_write = true;
char eeprom_lock = true;
char cmd[32];
char write_buf[64];
char eeprom_flags = 0;
int write_buf_idx = 0;
int write_buf_target_size = 0;
#define SERMODE_WRITE 0
#define SERMODE_CMD   1
#define SERMODE_ECHO  2
char serial_mode = SERMODE_CMD;
uint16_t cur_write_addr;
uint16_t end_write_addr;

// software data protection sequences
uint16_t enable_data_protect[4][2] = {
    { 0x5555, 0x00aa },
    { 0x2aaa, 0x0055 },
    { 0x5555, 0x00a0 },
    { 0x0000, 0x0000 },
};
uint16_t disable_data_protect[7][2] = {
    { 0x5555, 0x00aa },
    { 0x2aaa, 0x0055 },
    { 0x5555, 0x0080 },
    { 0x5555, 0x00aa },
    { 0x2aaa, 0x0055 },
    { 0x5555, 0x0020 },
    { 0x0000, 0x0000 },
};

// shift register routines
void shiftreg_send(uint8_t *, uint8_t);
void send_flags(uint8_t);
void send_data(uint8_t);
void send_addr(uint16_t);

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    if (CALBC1_16MHZ==0xFF) while(1);         // If calibration constant erased, do not load, trap CPU!!	
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_16MHZ;                   // Set DCO
    DCOCTL = CALDCO_16MHZ;

    P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2 ;                    // P1.1 = RXD, P1.2=TXD
    P1DIR = (F_OUT+A_OUT);                    // Port 1 outputs

    P2SEL = 0;                                // Port 2 == GPIO
    P2SEL2 = 0;                               // Port 2 == GPIO
    P2DIR = (DIN_OUT+DOUT_OUT);               // Port 2 outputs
    P2DIR &= ~(DIN_IN);                       // Port 2 inputs

    // Start in safe state
    P1OUT &= ~(SER_F + RCLK_F + SRCLK_F);
    P1OUT &= ~(SER_A + RCLK_A + SRCLK_A);
    P2OUT &= ~(DIN_CLK + DIN_SHLD);
    P2OUT &= ~(SER_DOUT + RCLK_DOUT + SRCLK_DOUT);
    P2OUT |= OE_DOUT;

    /*
       For table of values see MSP430 user guide,
       table 15-4 on page 424
    */
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
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

    // Set ~OE on the data-out shift register,
    // as the ~OE line on the EEPROM is in an
    // undefined state.
    P2OUT |= OE_DOUT;

    serial_mode = SERMODE_CMD;
    send_str("\r\n");
    while(1) {
        // Send ready prompt
        send_str("ready>");

        pause_for_char();

        // Analyze the command and call the
        // appropriate routine
        if(strlen(cmd) == 0)
            // do nothing; just loop back 
            // to a command prompt
            noop;
        else if(strncmp(cmd, "echo", 4) == 0)
            cmd_echo();
        else if(strncmp(cmd, "read", 4) == 0)
            cmd_read();
        else if(strncmp(cmd, "write", 5) == 0)
            cmd_write();
        else if(strncmp(cmd, "page_write", 10) == 0)
            cmd_page_write();
        else if(strncmp(cmd, "eeprom_lock", 11) == 0)
            cmd_eeprom_lock();
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
    send_str("page_write {on,off}: display, enable, disable page write mode\r\n");
    send_str("eeprom_lock {on,off}: display, enable, disable EEPROM lock mode\r\n");
    send_str("read 0xabcd 0xef01: read bytes from start to end addr, inclusive\r\n");
    send_str("write 0xabcd 0xef01: write bytes from start to end addr.\r\n");
    send_str("- If page_write enabled, 64 byte pages will be written with\r\n");
    send_str("  10ms pauses between each page.  Otherwise, each byte will\r\r");
    send_str("  be written individually with 10ms pauses in between.\r\n");
    send_str("- If eeprom_lock enabled, the Atmel software write protection\r\n");
    send_str("  routine will be executed before and after writing\r\n");
}

// Stops the CPU and waits for a character to
// arrive on the serial interface.  USCI0RX_ISR
// is executed when the byte arrives.
void pause_for_char() {
    // Enable USCI_A0 RX interrupt
    IE2 |= UCA0RXIE;

    // Go to sleep while RX interrupt routine
    // builds command
    __bis_SR_register(LPM0_bits + GIE);

    // Disable the serial interrupts while we
    // process the command
    IE2 &= ~UCA0RXIE;
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
            sprintf(buf, "Current echo setting: %d (enabled)\r\n", echo_mode);
        else
            sprintf(buf, "Current echo setting: %d (disabled)\r\n", echo_mode);
        send_str(buf);
    }
}

// page_write command: change page_write mode
void cmd_page_write() {
    char buf[64];
    if(strcmp(cmd, "page_write on") == 0) {
        page_write = true;
    }
    else if(strcmp(cmd, "page_write off") == 0) {
        page_write = false;
    }
    else {
        if(page_write)
            sprintf(buf, "Current page_write setting: %d (enabled)\r\n", page_write);
        else
            sprintf(buf, "Current page_write setting: %d (disabled)\r\n", page_write);
        send_str(buf);
    }
}

// eeprom_lock command: change eeprom_lock mode
void cmd_eeprom_lock() {
    char buf[64];
    if(strcmp(cmd, "eeprom_lock on") == 0) {
        eeprom_lock = true;
    }
    else if(strcmp(cmd, "eeprom_lock off") == 0) {
        eeprom_lock = false;
    }
    else {
        if(eeprom_lock)
            sprintf(buf, "Current eeprom_lock setting: %d (enabled)\r\n", eeprom_lock);
        else
            sprintf(buf, "Current eeprom_lock setting: %d (disabled)\r\n", eeprom_lock);
        send_str(buf);
    }
}

// Read command: read from EEPROM
void cmd_read() {
    uint16_t start_addr;
    uint16_t end_addr;
    uint8_t read_byte;
    uint8_t mask;
    char buf[64];

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
    sprintf(buf, "Start addr: %04x (%u)\r\n", start_addr, start_addr);
    send_str(buf);
    sprintf(buf, "End addr: %04x (%u)\r\n", end_addr, end_addr);
    send_str(buf);
    sprintf(buf, "Requesting %u bytes now...\r\n", end_addr - start_addr + 1);
    send_str(buf);

    // Disable the data write shift register's outputs
    // by pulling its _OE pin high
    P2OUT |= OE_DOUT;
    // Set the EEPROM flags to a known state
    // R_W high (strobe low to write)
    // _OE low  (data pins are outputs)
    // _CE low  (chip enabled)
    eeprom_flags = R_W;
    send_flags(eeprom_flags);

    // Set the shift register's pins to known states
    P2OUT &= ~DIN_CLK;
    P2OUT |= DIN_SHLD;

    // Iterate through each address
    for(uint16_t i=start_addr; i<=end_addr; i++) {
        // Set the address
        send_addr(i);
        // Strobe the SH/~LD pin and strobe the clock to load the byte
        P2OUT &= ~DIN_SHLD;
        P2OUT |= DIN_CLK;
        P2OUT &= ~DIN_CLK;
        P2OUT |= DIN_SHLD;

        // the LSB of the EEPROM data is now on QH
        read_byte = 0x00;
        mask = 0x01;
        while(1) {
            if(P2IN & DIN_QH)
                read_byte |= mask;
            if(mask >= 0x80)
                break;
            mask = (mask << 1);
            // Shift bits on rising clock edge
            P2OUT |= DIN_CLK;
            P2OUT &= ~DIN_CLK;
        }

        if(echo_mode) {
            sprintf(buf, "%02x", read_byte);
            send_str(buf);
        }
    }

    // Set the EEPROM flags to a known state
    // R_W high (strobe low to write)
    // _OE high (data pins are inputs)
    // _CE low  (chip enabled)
    eeprom_flags = R_W + _OE;
    send_flags(eeprom_flags);
}

// Write command: write to EEPROM
void cmd_write() {
    char buf[128];
    int len;

    len = strlen(cmd);
    if(len != 19) {
        sprintf(buf, "Invalid write command: wrong length: %d, expecting 19\r\n", len);
        send_str(buf);
        return;
    }
    // write 0xabcd 0xef01
    // 0       ^8     ^15 
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

    sprintf(buf, "Start addr: %04x (%u)\r\n", cur_write_addr, cur_write_addr);
    send_str(buf);
    sprintf(buf, "End addr: %04x (%u)\r\n", end_write_addr, end_write_addr);
    send_str(buf);
    sprintf(buf, "Total bytes to write: %u\r\n", end_write_addr - cur_write_addr + 1);
    send_str(buf);
    if(page_write)
        send_str("Paging\r\n");
    else
        send_str("No Paging\r\n");
    if(eeprom_lock)
        send_str("EEPROM Lock Enabled\r\n");
    else
        send_str("EEPROM Lock Disabled\n");

    // Set the EEPROM flags to a known state
    // R_W high (strobe low to write)
    // _OE high (data pins are inputs)
    // _CE low  (chip enabled)
    eeprom_flags = R_W + _OE;
    send_flags(eeprom_flags);
    // Enable the data shift register's outputs
    // by pulling its _OE pin low
    P2OUT &= ~OE_DOUT;

    if(eeprom_lock) {
        // disable software data protection
        for(uint16_t i=0; disable_data_protect[i][0] > 0; i++) {
            // Set address and data
            send_addr(disable_data_protect[i][0]);
            send_data((char)(disable_data_protect[i][1] & 0xff));
            // Strobe R_W pin
            eeprom_flags &= ~R_W;
            send_flags(eeprom_flags);
            eeprom_flags |= R_W;
            send_flags(eeprom_flags);
        }
    }

    // Enable write mode in the serial RX interrupt routine
    serial_mode = SERMODE_WRITE;

    while(cur_write_addr <= end_write_addr) {
        // Figure out how many bytes we want this round
        if(!page_write) {
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
        sprintf(buf, "Send %d bytes, %u remaining...\r\n", write_buf_target_size, end_write_addr - cur_write_addr + 1);
        send_str(buf);

        // Reset write_buf_idx so new data arrives
        // at beginning of write_buf
        write_buf_idx = 0;

        pause_for_char();

        sprintf(buf, "Writing %d bytes starting at 0x%04x\r\n", write_buf_target_size, cur_write_addr);
        send_str(buf);
        for(int i=0; i<write_buf_idx; i++) {
            // Set address and data
            send_addr(cur_write_addr);
            send_data(write_buf[i]);
            // Strobe R_W pin
            eeprom_flags &= ~R_W;
            send_flags(eeprom_flags);
            eeprom_flags |= R_W;
            send_flags(eeprom_flags);

            cur_write_addr++;
        }
        __delay_cycles(200000); // 200k cycles @ 16MHz => 12.5ms
    }

    // Disable write mode in the serial RX interrupt routine
    serial_mode = SERMODE_CMD;

    if(eeprom_lock) {
        // enable software dsata protection
        for(uint16_t i=0; enable_data_protect[i][0] > 0; i++) {
            // Set address and data
            send_addr(enable_data_protect[i][0]);
            send_data((char)(enable_data_protect[i][1] & 0xff));
            // Strobe R_W pin
            eeprom_flags &= ~R_W;
            send_flags(eeprom_flags);
            eeprom_flags |= R_W;
            send_flags(eeprom_flags);
        }
    }

    // Disable the data shift register's outputs
    // by pulling its _OE pin high
    P2OUT |= OE_DOUT;

    // Set the EEPROM flags to a known state
    // R_W high (strobe low to write)
    // _OE high (data pins are inputs)
    // _CE low  (chip enabled)
    eeprom_flags = R_W + _OE;
    send_flags(eeprom_flags);
}

// Serial data RX interrupt
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
    if(serial_mode == SERMODE_WRITE) {
        write_buf[write_buf_idx] = UCA0RXBUF;
        write_buf_idx++;
        // once we have collected enough bytes, wake the CPU back up
        if(write_buf_idx >= write_buf_target_size) {
            __bic_SR_register_on_exit(LPM0_bits);
        }
    }
    else if(serial_mode == SERMODE_ECHO) {
        if(echo_mode) echo(UCA0RXBUF);
        __bic_SR_register_on_exit(LPM0_bits);
    }
    else if(serial_mode == SERMODE_CMD) {
        if(UCA0RXBUF == 0x0d) {
            if(echo_mode) send_str("\r\n");
            // when command is complete, wake the CPU back up
            __bic_SR_register_on_exit(LPM0_bits);
        }
        else {
            if(echo_mode) echo(UCA0RXBUF);
            strncat(cmd, &UCA0RXBUF, 1);
        }
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
    shiftreg_send(&flags, SENDMODE_FLAG);
    return;
}

// Send two bytes to the address shift register
void send_addr(uint16_t addr) {
    uint8_t to_send[2];
    to_send[0] = (addr & 0x00ff);
    to_send[1] = ((addr & 0xff00) >> 8);
    shiftreg_send(to_send, SENDMODE_ADDR);
    return;
}

// Send one byte to the data-out shift register
void send_data(uint8_t data) {
    shiftreg_send(&data, SENDMODE_DATA);
    return;
}

// Generic shift register interface
void shiftreg_send(uint8_t *data_arr, uint8_t sendmode) {
    uint8_t SER = 0;
    uint8_t SRCLK = 0;
    uint8_t RCLK = 0;
    uint8_t count = 1;
    uint8_t max_bit = 7;
    uint8_t port = 1;
    switch(sendmode) {
        case SENDMODE_FLAG:
            SER = SER_F;
            SRCLK = SRCLK_F;
            RCLK = RCLK_F;
            max_bit = 2;
            break;
        case SENDMODE_DATA:
            SER = SER_DOUT;
            SRCLK = SRCLK_DOUT;
            RCLK = RCLK_DOUT;
            port = 2;
            break;
        case SENDMODE_ADDR:
            SER = SER_A;
            SRCLK = SRCLK_A;
            RCLK = RCLK_A;
            count = 2;
            break;
    }

    uint8_t mask;

    for(uint8_t idx=0; idx<count; idx++) {
        mask = 0x01;
        uint8_t this_byte = data_arr[idx];
        for(uint8_t bit = 0; bit <= max_bit; bit++) {
            // Put this bit on the serial input
            if(this_byte & mask) {
                if(port == 1)
                    P1OUT |= SER;
                else
                    P2OUT |= SER;
            }
            else {
                if(port == 1)
                    P1OUT &= ~SER;
                else
                    P2OUT &= ~SER;
            }

            // Strobe the SRCLK to shift
            // this bit into the shift register
            if(port == 1) {
                P1OUT |= SRCLK;
                P1OUT &= ~SRCLK;
            }
            else {
                P2OUT |= SRCLK;
                P2OUT &= ~SRCLK;
            }

            // Shift our mask to the next bit
            mask = (mask << 1);
        }
    }

    // reset SER
    if(port == 1) {
        P1OUT &= ~SER;
    }
    else {
        P2OUT &= ~SER;
    }

    // Strobe the RCLK to load the shifted
    // data into the output register
    if(port == 1) {
        P1OUT |= RCLK;
        P1OUT &= ~RCLK;
    }
    else {
        P2OUT |= RCLK;
        P2OUT &= ~RCLK;
    }

    return;
}


