/*
    AUTHOR: Shyam
    Domain: Embedded and Electronics
    Sub Domain: Embedded Systems
    Functions: initI2C, sendDataToRTC, receiveDataFromRTC
    Macros: PINMODEINPUT, PINMODEOUTPUT, PINHIGH, PINLOW, PINPULLUP
    Global variables: pin_scl, pin_sda
*/

#include<avr/interrupt.h>
#include<avr/io.h>

#define PINMODEOUTPUT(port, pin) DDR##port |= (1 << pin)
#define PINMODEINPUT(port, pin) DDR##port &= !(1 << pin)
#define PINHIGH(port, pin) PORT##port |= (1 << pin)
#define PINLOW(port, pin) PORT##port &= !(1 << pin)
#define PINPULLUP(port, pin) PORT##port |= (1 << pin)

//DS3231 ADDRESS
#define DS3231_ADDRESS 0b1101000

//TWI STATUS CODES MT -> master transmission mode MR - master receiving mode
#define TW_MT_START 0x08
#define TW_MT_REPEATEDSTART 0x10
#define TW_MT_SL_ACK 0x18
#define TW_MT_DA_ACK 0x28
#define TW_MR_SL_ACK 0x40
#define TW_MR_DA_ACK 0x50
#define TW_MR_DA_NACK 0x58

const int pin_scl = 5; //PORTC
const int pin_sda = 4; //PORTC

struct Time {
    uint8_t seconds, hours, minutes;
    bool am;
};

struct Date {
    uint8_t date, month, year;
    char day[4];
};

//////////////////
// UART methods //
//////////////////
/*
    Function name: initUSART
    Input: none
    Output: none
    Logic: Initialize serial communication
    Example call: initUSART();
*/
void initUSART() {
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); //asynchronous, 8 bit data
    UBRR0 = 103; //set baud rate = 9600 bps
    UCSR0B = (1 << RXEN0) | (1 << TXEN0); //enable RX and TX
}

/*
    Function name: USART_sendByte
    Input:  uint8_t data: the data to be sent
    Output: none
    Logic: Send 1 byte of data to the serial monitor
    Example call: USART_sendByte('A');
*/
void USART_sendByte(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

/*
    Function name: USART_sendData
    Input:  char* data: the data (string) to be sent
    Output: none
    Logic: Send a string to the serial monitor
    Example call: USART_sendData("Hello world!\n");
*/
void USART_sendData(char* data) {
    while(*data) {
        USART_sendByte(*data);
        data++;
    }
}

/*
    Function name: USART_sendData
    Input:  float data: the data to be sent
    Output: none
    Logic: Send a float to the serial monitor
    Example call: USART_sendData(10.0);
*/
void USART_sendData(float data) {
    char temp[10];
    dtostrf(data, 8, 2, temp);
    USART_sendData(temp);
}

/*
    Function name: USART_sendData
    Input:  uint16_t data: the data to be sent
    Output: none
    Logic: Send a uint16_t to the serial monitor
    Example call: USART_sendData(10.0);
*/
void USART_sendData(uint16_t data) {
    char temp[10];
    sprintf(temp, "%d", data);
    USART_sendData(temp);
}

/*
    Function name: USART_getByte
    Input:  none
    Output: uint8_t : the byte received
    Logic: Send a float to the serial monitor
    Example call: USART_sendData(10.0);
*/
uint8_t USART_getByte() {
    uint8_t data;
    while(!(UCSR0A & (1 << RXC0)));
    data = UDR0;
    return data;
}

//////////////////////////////
// I2C interfacing with RTC //
//////////////////////////////
/*
    Function name: initI2C
    Input: none
    Output: none
    Logic:  
    Example call: initI2C();
 */
void initI2C() {
    PINMODEINPUT(C, pin_scl);
    PINMODEINPUT(C, pin_sda);

    PINPULLUP(C, pin_scl);
    PINPULLUP(C, pin_sda);

    //set I2C at 100 Khz
    TWBR = 72; 
    TWSR &= !((1 << TWPS0) | (1 << TWPS1));
}

/*
    Function name: sendDataToRTC
    Input:  uint8_t address: the address in RTC module to which data has to be put 
            uint8_t* data: an array of all data bytes 
            uint8_t dataBytes: number of bytes of data to be sent
    Output: uint8_t : error code 
            0 -> no error 
            1 -> start bit not sent
            2 -> error : no slave acknowledgement (module address is wrong)
            3 -> no address acknowledgement
            4..5+dataBytes-> no data byte acknowledgement
 */
uint8_t sendDataToRTC(uint8_t address, uint8_t* data = NULL, uint8_t dataBytes = 0) {
    TWCR = (1 << TWEN) // make sure TWI is enabled
            | (1 << TWSTA) //send Start I2C signal
            | (1 << TWINT) //clearing the int flag
    ;
    //wait for transmission
    while (!(TWCR & (1 << TWINT)));

    //check if START was sent 
    if (TWSR & 0xF8 != TW_MT_START) {
        return 1; 
    }

    //send slave address and write bit
    TWDR = (DS3231_ADDRESS << 1) + 0; 
    TWCR = (1 << TWEN)
            | ( 1 << TWINT)
    ;

    //wait for transmission
    while (!(TWCR & (1 << TWINT)));

    //check for ack
    if (TWSR & 0xF8 != TW_MT_SL_ACK) {
        return 2;
    }

    //now send address to RTC module to place the address pointer of DS3231
    TWDR = address;
    TWCR = (1 << TWEN)
            | ( 1 << TWINT)
    ;

    //wait for transmission
    while (!(TWCR & (1 << TWINT)));

    //check for ack
    if (TWSR & 0xF8 != TW_MT_DA_ACK) {
        return 3;
    }

    //send all databytes
    for (int i = 0; i < dataBytes; i++) {
        TWDR = data[i];
        TWCR = (1 << TWEN)
            | ( 1 << TWINT)
        ;

        //wait for transmission
        while (!(TWCR & (1 << TWINT)));

        //check for ack
        if (TWSR & 0xF8 != TW_MT_DA_ACK) {
            return 4 + i;
        }
    }

    TWCR = (1 << TWEN)
            | (1 << TWSTO) //send stop signal 
            | (1 << TWINT);

    return 0; //no error
}

/*
    Function name: receiveDataFromRTC
    Input:  uint8_t address: the address in RTC to start receiving from 
            uint8_t* data: pointer to store the data 
            uint8_t dataBytes: number of bytes to receive
    Output: uint8_t : errorcode
            0 -> no error
            1 -> start bit not sent
            2 -> error : no slave acknowledgement (module address is wrong)
            3 -> no address acknowledgement
            4 -> no slave ack for reading mode
 */
uint8_t receiveDataFromRTC(uint8_t address, uint8_t * data, uint8_t dataBytes) {
    //First set the memory pointer of RTC to required address
    sendDataToRTC(address);

    //receive mode 
    TWCR = (1 << TWEN) // make sure TWI is enabled
            | (1 << TWSTA) //send Start I2C signal
            | (1 << TWINT) //clearing the int flag
    ;

    //wait for transmission
    while (!(TWCR & (1 << TWINT)));

    //check if START was sent 
    if (TWSR & 0xF8 != TW_MT_START) {
        return 1; 
    }

    //send slave address and write bit
    TWDR = (DS3231_ADDRESS << 1) + 1; // read mode
    TWCR = (1 << TWEN)
            | ( 1 << TWINT)
    ;

    //wait for transmission
    while (!(TWCR & (1 << TWINT)));

    //check for ack
    if (TWSR & 0xF8 != TW_MR_SL_ACK) {
        return 4;
    }

    //receive dataBytes bytes of data and place it in data 
    for (int i = 0; i <dataBytes; i++) {
        if (i != dataBytes - 1) {
            //not the last byte
            TWCR = (1 << TWEN)
                    | (1 << TWEA) //send ack after receiving 
                    | (1 << TWINT);
            
            //wait for transmission
            while (!(TWCR & (1 << TWINT)));

            if (TWSR & 0xF8 != TW_MR_DA_ACK) {
                return 5 + i;
            }

            //set the data 
            data[i] = TWDR;
        }  
        else {
            //last byte 
            TWCR = (1 << TWEN)
                    |(1 << TWINT);
            
            //wait for transmission
            while (!(TWCR & (1 << TWINT)));

            if (TWSR & 0xF8 != TW_MR_DA_NACK) {
                return 5 + i;
            }

            //set the data 
            data[i] = TWDR;
        }
    }

    //send stop signal 
    TWCR = (1 << TWEN) 
            | (1 << TWINT) 
            | (1 << TWSTO);

    return 0; //no error
}


/////////////////////
// Utility methods //
/////////////////////
/*
    Function name: RTC_getTime
    Input: none 
    Output: Time : struct Time filled with the current time from the RTC in 12 hour format
    Logic: obtain current time from the address starting at 0x00 (3 bytes) corresponding to 
            byte 0: seconds:    <1 bit 0> <3 bits tens place> <4 bits units place>
            byte 1: minutes:    <1 bit 0> <3 bits tens place> <4 bits units place>
            byte 2: hours:      <0> <12/24!> <20/AM!-PM><10><4 bits units>
 */
Time RTC_getTime() {
    uint8_t data[3];
    Time time;
    receiveDataFromRTC(0x00, data, 3);
    
    time.seconds = (data[0] & 0x0F) + ((data[0]&0xF0)>>4) * 10;
    time.minutes = (data[1] & 0x0F) + ((data[1]&0xF0)>>4) * 10;
    if (data[2] & 0x40) {
        //12 hour mode 
        //set am-pm
        time.am = !(data[2] & 0x20);

        //get hours
        time.hours = (data[2] & 0x0F) + ((data[2] & 0x10)>>4) * 10;
    } 
    else {
        //24 hour mode 
        time.hours = (data[2] & 0x0F); //units place 
        if (data[2] & 0x20) time.hours += 20;
        if (data[2] & 0x10) time.hours += 10;

        if (time.hours == 0) {
            time.hours = 12;
        } 

        if (time.hours > 12) {
            time.hours -= 12;
            time.am = false;
        }
        else time.am = true;
    }
    return time;
}


int main() {
    initI2C();
    initUSART();

    uint8_t data = 2;
    uint8_t address = 0x04;
    uint8_t error = 0;
    Time time ;
    time.seconds = 0;
    time.minutes = 0;

    error = sendDataToRTC(address, &data, 1);
    data = 0;
    error = receiveDataFromRTC(address, &data, 1);
    while (1) {
        time = RTC_getTime();
        USART_sendData("Time: ");
        USART_sendData((uint16_t)time.hours);
        USART_sendData(" : ");
        USART_sendData((uint16_t)time.minutes);
        USART_sendData(" : ");
        USART_sendData((uint16_t)time.seconds);
        USART_sendData(" ");
        if (time.am) USART_sendData(" am\n");
        else USART_sendData(" pm\n");
    }
    
    return 0;
}