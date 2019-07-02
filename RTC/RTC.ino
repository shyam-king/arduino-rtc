/*
    AUTHOR: Shyam
    Domain: Embedded and Electronics
    Sub Domain: Embedded Systems
    Functions: initI2C, sendDataToRTC, receiveDataFromRTC, initUSART, USART_getByte, USART_sendByte, USART_sendData,
                RTC_getDate, RTC_getTime
    Macros: PINMODEINPUT, PINMODEOUTPUT, PINHIGH, PINLOW, PINPULLUP
    Global variables: pin_scl, pin_sda, map_day
    Structs: Time, Date
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
const char* map_day[] = {
    "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};
uint8_t data = 'a';

volatile char USART_DATA_BUFFER[32] = "";
volatile uint8_t USART_DATA_INDEX = 0;
volatile bool USART_BUFFER_READY = false;

struct Time {
    uint8_t seconds, hours, minutes;
    bool am;
};

struct Date {
    uint8_t date, month, year, day;
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
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); //enable RX and TX
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
void USART_sendData(const char* data) {
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
uint8_t USART_getByte(uint8_t *data) {
    if (UCSR0A & (1 << RXC0)) {
        *data = UDR0;
        return true;
    }
    return false;
}

/*
    TODO
 */
void USART_readBuffer(char *dest) {
    strcpy(dest, USART_DATA_BUFFER);
    USART_DATA_INDEX = 0;
    USART_BUFFER_READY = false;
}

/*
    TODO: 
 */
ISR(USART_RX_vect) {
    if (USART_BUFFER_READY == false) {
        char data = UDR0;
        if (data == '\n' || data == '\r' || USART_DATA_INDEX == 31) 
        {
            data = '\0';
            USART_BUFFER_READY = true;
        }
        USART_DATA_BUFFER[USART_DATA_INDEX] = data;
        USART_DATA_INDEX++;
    }
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
    Input: Time structure to be filled 
    Output: none
    Logic: obtain current time from the address starting at 0x00 (3 bytes) corresponding to 
            byte 0: seconds:    <1 bit 0> <3 bits tens place> <4 bits units place>
            byte 1: minutes:    <1 bit 0> <3 bits tens place> <4 bits units place>
            byte 2: hours:      <0> <12/24!> <20/AM!-PM><10><4 bits units>
 */
void RTC_getTime(Time* time) {
    uint8_t data[3];
    receiveDataFromRTC(0x00, data, 3);
    
    time->seconds = (data[0] & 0x0F) + ((data[0]&0xF0)>>4) * 10;
    time->minutes = (data[1] & 0x0F) + ((data[1]&0xF0)>>4) * 10;
    if (data[2] & 0x40) {
        //12 hour mode 
        //set am-pm
        time->am = !(data[2] & 0x20);

        //get hours
        time->hours = (data[2] & 0x0F) + ((data[2] & 0x10)>>4) * 10;
    } 
    else {
        //24 hour mode 
        time->hours = (data[2] & 0x0F); //units place 
        if (data[2] & 0x20) time->hours += 20;
        if (data[2] & 0x10) time->hours += 10;

        if (time->hours == 0) {
            time->hours = 12;
        } 

        if (time->hours > 12) {
            time->hours -= 12;
            time->am = false;
        }
        else time->am = true;
    }
}

/*
    Function name: RTC_getDate
    Input: Date structure to be filled 
    Output: None
    Logic: obtain current date from the address starting at 0x03 (4 bytes) corresponding to 
            byte 0: day:    <0 for 5 bits> <3 bits mapping to day>
            byte 1: date:   <2 bit 0> <2 bits tens place> <4 bits units place>
            byte 2: month:  <1 bit century> <2 bits 0> <1 bits tens place><4 bits units>
            byte 3: year:   <4bit tens place> <4bits units>
 */
void RTC_getDate(Date* date) {
    uint8_t data[4];
    receiveDataFromRTC(0x03, data, 4);

    date->day = data[0];
    date->date = (data[1] & 0x0F) + (((data[1]&0xF0)>>4)*10);
    date->month = (data[2] & 0x0F) + (((data[2]&0x1F)>>4)*10);
    date->year = (data[3] & 0x0F) + (((data[3] & 0xF0)>>4)*10) + (((data[2]&0x80)>>7) * 100);
}

/*
    Function name: RTC_sendTime
    Input: Time to be sent 
    Output: None 
    Logic : set the time as (address 0x00)
            byte 0: seconds:    <1 bit 0> <3 bits tens place> <4 bits units place>
            byte 1: minutes:    <1 bit 0> <3 bits tens place> <4 bits units place>
            byte 2: hours:      <0> <12/24!> <20/AM!-PM><10><4 bits units>
 */
void RTC_sendTime(Time time) {
    uint8_t data[3] = {0, 0, 0};

    //fill byte 0
    data[0] = (time.seconds % 10) | ((time.seconds/10) << 4); 

    //fill byte 1
    data[1] = (time.minutes % 10) | ((time.minutes)/10) << 4;

    //fill byte 2
    data[2] = (time.hours % 10) | ((time.hours / 10) << 4) 
                | ((!time.am) << 5) //am/pm
                | (1 << 6); //12 hour mode

    sendDataToRTC(0x00, data, 3);
}

/*
    Function name: RTC_sendDate
    Input: Date date: to be sent 
    Output: None 
    Logic : set the date as (address 0x03)
            byte 0: day:    <0 for 5 bits> <3 bits mapping to day>
            byte 1: date:   <2 bit 0> <2 bits tens place> <4 bits units place>
            byte 2: month:  <1 bit century> <2 bits 0> <1 bits tens place><4 bits units>
            byte 3: year:   <4bit tens place> <4bits units>
 */
void RTC_sendDate(Date date) {
    uint8_t data[4] = {0, 0, 0, 0};

    //fill byte 0
    data[0] = date.day;
    //fill byte 1
    data[1] = (date.date % 10) | ((date.date/10) << 4);
    //fill byte 2
    data[2] = (date.month % 10) | ((date.month / 10) << 4) | ((date.year / 100) << 7);
    //fill byte 3
    data[3] = (date.year % 10) | (((date.year / 10)%10) << 4);

    sendDataToRTC(0x03, data, 4);
}

/*
    TODO

    0x07
    byte 0: seconds:    <A1M1> <3 bits tens place> <4 bits units place>
    byte 1: minutes:    <A1M2> <3 bits tens place> <4 bits units place>
    byte 2: hours:      <A1M3> <12/24!> <20/AM!-PM><10><4 bits units>
    byte 3: if Date:        <A1M4> <DY/DT!> <2 bits tens place> <4 bits units place>
            if Day :                          <2 0> <4 bits day>
 */
void RTC_setAlarm0(Time time) {
    uint8_t data[4];

    data[0] = 0;
    data[1] = (time.minutes%10) | ((time.minutes)/10) << 4;
    data[2] = (time.hours % 10) | ((time.hours)/10) << 4 | ((!time.am)<<5) | (1 << 6);
    data[3] = 0x80;

    //make sure alarm-flag is not set 
    uint8_t flag;
    receiveDataFromRTC(0x0F, &flag, 1);
    flag &= !(1 << 0);
    sendDataToRTC(0x0F, &flag, 1);

    //send data
    sendDataToRTC(0x07, data, 4);
}

/*
TODO
 */
bool RTC_checkAlarm0() {
    uint8_t data;
    receiveDataFromRTC(0x0F, &data, 1);

    if (data & (1 << 0)) {
        data &= !(1 << 0);
        sendDataToRTC(0x0F, &data, 1);
        return true;
    }

    return false;
}

int main() {
    initI2C();
    initUSART();
    sei();

    Time time ;
    Date date;

    char data[32] = "nothing";

    date.day = 2;
    date.date = 2;
    date.month = 7;
    date.year = 19;
    RTC_sendDate(date);

    time.hours = 9;
    time.minutes = 11;

    while (1) {

        //PRINT CURRENT DATE-TIME
        RTC_getDate(&date);
        RTC_getTime(&time);
        USART_sendData("Now: ");
        USART_sendData((uint16_t)time.hours);
        USART_sendData(" : ");
        USART_sendData((uint16_t)time.minutes);
        USART_sendData(" : ");
        USART_sendData((uint16_t)time.seconds);
        USART_sendData(" ");
        if (time.am) USART_sendData(" am; ");
        else USART_sendData(" pm; ");

        USART_sendData(map_day[date.day]);
        USART_sendData(", ");
        USART_sendData((uint16_t)date.date);
        USART_sendData(" / ");
        USART_sendData((uint16_t)date.month);
        USART_sendData(" / ");
        USART_sendData((uint16_t)date.year);
        USART_sendByte('\n');

        //PRINT MENU
        USART_sendData("\nENTER \"time\" FOR changing TIME\n");
        USART_sendData("ENTER \"date\" FOR changing DATE\n");
        USART_sendData("ENTER \"alarm\" FOR setting alarm\n");
        USART_sendData("ENTER \"now\" FOR current time\n");

        //wait for user-input
        while (!USART_BUFFER_READY);
        USART_readBuffer(data);

        if (strcmp(data, "time") == 0) {
            USART_sendData("Enter hour:\n");
            while (!USART_BUFFER_READY);
            USART_readBuffer(data);
            if (strlen(data) == 2) {
                time.hours = data[1] - '0' + (data[0] - '0') * 10;
            }
            else {
                time.hours = data[0] - '0';
            }

            USART_sendData("Enter minutes:\n");
            while (!USART_BUFFER_READY);
            USART_readBuffer(data);
            if (strlen(data) == 2) {
                time.minutes = data[1] - '0' + (data[0] - '0') * 10;
            }
            else {
                time.minutes = data[0] - '0';
            }

            USART_sendData("Enter seconds:\n");
            while (!USART_BUFFER_READY);
            USART_readBuffer(data);
            if (strlen(data) == 2) {
                time.seconds = data[1] - '0' + (data[0] - '0') * 10;
            }
            else {
                time.seconds = data[0] - '0';
            }

            USART_sendData("If AM enter 1:\n");
            while (!USART_BUFFER_READY);
            USART_readBuffer(data);
            if (data[0] == '1') {
                time.am = true;
            }
            else {
                time.am = false;
            }

            RTC_sendTime(time);
        }
        else if (strcmp(data, "date") == 0) {
            USART_sendData("Enter day-of-the-week\n");
            for (int i =0; i< 7; i++) {
                USART_sendData((uint16_t)i);
                USART_sendData(" : ");
                USART_sendData(map_day[i]);
                USART_sendByte('\n');
            }
            while (!USART_BUFFER_READY);
            USART_readBuffer(data);
            date.day = data[0] - '0';

            USART_sendData("Enter date:\n");
            while (!USART_BUFFER_READY);
            USART_readBuffer(data);
            if (strlen(data) == 2) {
                date.date = data[1] - '0' + (data[0] - '0') * 10;
            }
            else {
                date.date = data[0] - '0';
            }

            USART_sendData("Enter month:\n");
            while (!USART_BUFFER_READY);
            USART_readBuffer(data);
            if (strlen(data) == 2) {
                date.month = data[1] - '0' + (data[0] - '0') * 10;
            }
            else {
                date.month = data[0] - '0';
            }

            USART_sendData("Enter year (0 - 99):\n");
            while (!USART_BUFFER_READY);
            USART_readBuffer(data);
            if (strlen(data) == 2) {
                date.year = data[1] - '0' + (data[0] - '0') * 10;
            }
            else {
                date.year = data[0] - '0';
            }

            RTC_sendDate(date);
        }
        
    }
    
    return 0;
}