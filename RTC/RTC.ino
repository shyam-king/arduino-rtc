/*
    AUTHOR: Shyam
    Domain: Embedded and Electronics
    Sub Domain: Embedded Systems
    Functions: 
        initUSART, USART_sendByte, USART_sendData, USART_getByte, USART_readBuffer
        init I2C, sendDataToRTC, receiveDataFromRTC
        initAlarm
        RTC_getTime, RTC_getDate, RTC_sendTime, RTC_sendDate, RTC_setAlarm0, RTC_setAlarm1, RTC_checkAlarm0
            RTC_checkAlarm1, RTC_getAlarm0
        addTime, TimeSeconds, TimeHours, TimeMinutes, TimeToSeconds, SecondsToTime
        startStopwatch, stopStopwatch
    Macros: PINMODEINPUT, PINMODEOUTPUT, PINHIGH, PINLOW, PINPULLUP, PINTOGGLE
    Global variables:   pin_scl, pin_sda, pin_speaker, map_day, USART_DATA_BUFFER, USART_DATA_INDEX, USART_BUFFER_READY,
                        alarm_ringing
    Structs: Time, Date
*/

//REQUIRED HEADERS
#include<avr/interrupt.h>
#include<avr/io.h>


//HELPER MACROS
#define PINMODEOUTPUT(port, pin) DDR##port |= (1 << pin)
#define PINMODEINPUT(port, pin) DDR##port &= !(1 << pin)
#define PINTOGGLE(port, pin) PORT##port ^= (1 << pin)
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


//helper structs
struct Date {
    uint8_t date, month, year, day;
};

struct Time {
    uint8_t seconds, hours, minutes;
    bool am;
};


//pins
const int pin_scl = 5; //PORTC
const int pin_sda = 4; //PORTC
const int pin_speaker = 0; //PORTB

//day mapping (0-6 -> day of week)
const char* map_day[] = {
    "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};

//for async USART
volatile char USART_DATA_BUFFER[32] = "";
volatile uint8_t USART_DATA_INDEX = 0;
volatile bool USART_BUFFER_READY = false;

//for alarm speaker
volatile bool alarm_ringing = false;

//for stopwatch 
bool STOPWATCH_COUNTING = false;
Time STOPWATCH_START;

//for timer 
bool TIMER_RINGING = false;
bool TIMER_SET = false;


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
    Example call: USART_sendData((uint16_t)10));
*/
void USART_sendData(uint16_t data) {
    char temp[10];
    sprintf(temp, "%d", data);
    USART_sendData(temp);
}

/*
    Function name: USART_getByte
    Input:  uint8_t* data : pointer to store received byte
    Output: bool : true if a byte was received and put in data
    Logic: By checking the UCSR0A.RXC0 flag availability of data in UDR0 is known,
            if data is available, it is stored and true is returned
    Example call: 
    uint8_t data;
    if (USART_getByte(&data)) {
        //do something with data
    } 
*/
bool USART_getByte(uint8_t *data) {
    if (UCSR0A & (1 << RXC0)) {
        *data = UDR0;
        return true;
    }
    return false;
}

/*
    Function name: USART_readBuffer 
    Input:  char *dest: memory to be filled with received data (capacity 32 bytes)
    Output: none 
    Logic:  read the received data that is stored in the buffer
    Example call: 
    char data[32];
    if (USART_BUFFER_READY) USART_readBuffer(data);
 */
void USART_readBuffer(char *dest) {
    strcpy(dest, USART_DATA_BUFFER);
    USART_DATA_INDEX = 0;
    USART_BUFFER_READY = false;
}

/*
    Function name: ISR(USART_RX_vect)
    Logic: Whenever data is available it is appended to the buffer if the buffer has already been read 
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
    Logic:  The SDA and SCL pins are configured for I2C usage and the tranmission frequency is set at 
            100 Khz
    Example call: initI2C();
 */
void initI2C() {
    //configure pins connected to the RTC Module
    PINMODEINPUT(C, pin_scl);
    PINMODEINPUT(C, pin_sda);

    //pull up sda and scl lines to VCC = 5V
    PINPULLUP(C, pin_scl);
    PINPULLUP(C, pin_sda);

    //set I2C at 100 Khz
    TWBR = 72; 
    TWSR &= !((1 << TWPS0) | (1 << TWPS1)); // reset prescalar to 1
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
    Logic:  employs I2C protocol to send dataBytes number of bytes to the RTC module 
    Example call: 
    uint8_t data[3]; 
    //set data bytes 
    sendDataToRTC(0x0F, data, 3);
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
    Logic:  employs I2C protocol receive dataBytes number of bytes from the RTC module
            The address is sent in master transmission mode to set the register pointer of the RTC module
            The data is received in master receiving mode
    Example call: 
    uint8_t data[2];
    receiveDataFromRTC(0x0F, data, 2);
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

////////////////////
// Alarm methods //
///////////////////

/*
    Function name:  initAlarm
    Input:  none 
    Output: none 
    Logic:  Set up pins for speaker and the Timer for producing sound
    Example call:   initAlarm();
 */
void initAlarm() {
    //set up pin
    PINMODEOUTPUT(B, pin_speaker);
    PINLOW(B, pin_speaker);

    //set up timer 400 Hz
    OCR1A = 625;
    TCCR1B = (1 << WGM12) // CTC mode 
            | (1 << CS11) | (1 << CS10); //64 prescalar
    TIMSK1 = (1 << 1); //interrupt enable 
}

/*
    Function name: ISR(TIMER1_COMPA_vect)
    Logic: toggle speaker pin to produce sound whenever alarm is ringing
 */
ISR(TIMER1_COMPA_vect) {
    if (alarm_ringing || (TIMER_RINGING && TIMER_SET)) {
        PINTOGGLE(B, pin_speaker);
    }
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
    Example call: 
    Time time;
    RTC_getTime(&time);
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
    Input:  Date structure to be filled 
    Output: None
    Logic:  obtain current date from the address starting at 0x03 (4 bytes) corresponding to 
            byte 0: day:    <0 for 5 bits> <3 bits mapping to day>
            byte 1: date:   <2 bit 0> <2 bits tens place> <4 bits units place>
            byte 2: month:  <1 bit century> <2 bits 0> <1 bits tens place><4 bits units>
            byte 3: year:   <4bit tens place> <4bits units>
    Example call:   
    Date date;
    RTC_getDate(&date);
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
    Example call:
    Time time;
    //set time 
    RTC_sendTime(time);
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
    Example call:
    Date date;
    //set date 
    RTC_sendDate(date);
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
    Function name: RTC_setAlarm1
    Input:  Time time: the time to set alarm (the seconds field is ignored)
    Output: none
    Logic:  set 4 bytes starting at 0x07
            byte 0: seconds:    <A1M1> <3 bits tens place> <4 bits units place>
            byte 1: minutes:    <A1M2> <3 bits tens place> <4 bits units place>
            byte 2: hours:      <A1M3> <12/24!> <20/AM!-PM><10><4 bits units>
            byte 3: if Date:        <A1M4> <DY/DT!> <2 bits tens place> <4 bits units place>
                    if Day :                          <2 0> <4 bits day>
    Example call: 
    Time time;
    time.hours = 12; time.minutes = 0; time.am = true;
    RTC_setAlarm0(time);
 */
void RTC_setAlarm1(Time time) {
    uint8_t data[4];

    data[0] = (time.seconds % 10) | (((time.seconds)/10) << 4);
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
    Function name: RTC_setAlarm0
    Input:  Time time: the time to set alarm (the seconds field is ignored)
    Output: none
    Logic:  set 3 bytes starting at 0x0B
            byte 0: minutes:    <A1M2> <3 bits tens place> <4 bits units place>
            byte 1: hours:      <A1M3> <12/24!> <20/AM!-PM><10><4 bits units>
            byte 2: if Date:        <A1M4> <DY/DT!> <2 bits tens place> <4 bits units place>
                    if Day :                          <2 0> <4 bits day>
    Example call: 
    Time time;
    time.hours = 12; time.minutes = 0; time.am = true;
    RTC_setAlarm1(time);
 */
void RTC_setAlarm0(Time time) {
    uint8_t data[3];
    data[0] = (time.minutes%10) | ((time.minutes)/10) << 4;
    data[1] = (time.hours % 10) | ((time.hours)/10) << 4 | ((!time.am)<<5) | (1 << 6);
    data[2] = 0x80;

    //make sure alarm-flag is not set 
    uint8_t flag;
    receiveDataFromRTC(0x0F, &flag, 1);
    flag &= !(1 << 0);
    sendDataToRTC(0x0F, &flag, 1);

    //send data
    sendDataToRTC(0x0B, data, 3);
}

/*
    Function name: RTC_checkAlarm1
    Input:  none
    Output: bool : returns true if alarm0 matches
    Logic:  Read LSB of status register @ 0x0F
    Example call: 
    if (RTC_checkAlarm0()) {
        //do something
    }
 */
bool RTC_checkAlarm1() {
    uint8_t data;
    receiveDataFromRTC(0x0F, &data, 1);

    if (data & (1 << 0)) { //check alarm 0 flag 
        data &= !(1 << 0);
        sendDataToRTC(0x0F, &data, 1);
        return true;
    }

    return false;
}

/*
    Function name: RTC_checkAlarm0
    Input:  none
    Output: bool : returns true if alarm1 matches
    Logic:  Read 2nd LSB of status register @ 0x0F
    Example call: 
    if (RTC_checkAlarm1()) {
        //do something
    }
 */
bool RTC_checkAlarm0() {
    uint8_t data;
    receiveDataFromRTC(0x0F, &data, 1);

    if (data & (1 << 1)) { //check alarm 1 flag 
        data &= !(1 << 1);
        sendDataToRTC(0x0F, &data, 1);
        return true;
    }

    return false;
}

/*
    Function name: RTC_getAlarm1
    Input:  none;
    Output: none 
    Logic: Read 2 bytes @ 0x08
        byte 0: minutes:    <A1M2> <3 bits tens place> <4 bits units place>
        byte 1: hours:      <A1M3> <12/24!> <20/AM!-PM><10><4 bits units>
    Example call:
    Time time = RTC_getAlarm0();
 */
Time RTC_getAlarm1() {
    uint8_t data[2]; 

    receiveDataFromRTC(0x08, data, 2);
    Time t;
    t.seconds = 0;
    t.minutes = (data[0] & 0x0F) + ((data[0] & 0x70)>>4) * 10;
    if (data[1] & 0x40) {
        //12 hour mode 
        //set am-pm
        t.am = !(data[1] & 0x20);

        //get hours
        t.hours = (data[1] & 0x0F) + ((data[1] & 0x10)>>4) * 10;
    } 
    else {
        //24 hour mode 
        t.hours = (data[1] & 0x0F); //units place 
        if (data[1] & 0x20) t.hours += 20;
        if (data[1] & 0x10) t.hours += 10;

        if (t.hours == 0) {
            t.hours = 12;
        } 

        if (t.hours > 12) {
            t.hours -= 12;
            t.am = false;
        }
        else t.am = true;
    } 

    return t;
}

/*
    Function name: RTC_getAlarm0
    Input:  none;
    Output: none 
    Logic: Read 2 bytes @ 0x0B
        byte 0: minutes:    <A2M2> <3 bits tens place> <4 bits units place>
        byte 1: hours:      <A2M3> <12/24!> <20/AM!-PM><10><4 bits units>
    Example call:
    Time time = RTC_getAlarm0();
 */
Time RTC_getAlarm0() {
    uint8_t data[2]; 

    receiveDataFromRTC(0x0B, data, 2);
    Time t;
    t.seconds = 0;
    t.minutes = (data[0] & 0x0F) + ((data[0] & 0x70)>>4) * 10;
    if (data[1] & 0x40) {
        //12 hour mode 
        //set am-pm
        t.am = !(data[1] & 0x20);

        //get hours
        t.hours = (data[1] & 0x0F) + ((data[1] & 0x10)>>4) * 10;
    } 
    else {
        //24 hour mode 
        t.hours = (data[1] & 0x0F); //units place 
        if (data[1] & 0x20) t.hours += 20;
        if (data[1] & 0x10) t.hours += 10;

        if (t.hours == 0) {
            t.hours = 12;
        } 

        if (t.hours > 12) {
            t.hours -= 12;
            t.am = false;
        }
        else t.am = true;
    } 

    return t;
}

/*
    Function name: addTime 
    Input:  Time *time : the time object to be added to 
            Time toAdd : the amount of time to be added (the am field is ignored)
    Output: None 
    Logic:  The time values from toAdd are added to object pointed by time. The overflow checks are done as
            well, and the am field of time is altered correspondingly.
    Example call:   Time original, add;
                    //fill original and add 
                    addTime(&original, add);
*/
void addTime(Time* time, Time toAdd) {
    time -> seconds += toAdd.seconds;
    if (time -> seconds > 59) {
        time -> seconds -= 60;
        time -> minutes ++;
    }

    time -> minutes += toAdd.minutes;
    if (time -> minutes > 59) {
        time -> minutes -= 60;
        time -> hours ++;
    }

    time -> hours += toAdd.hours;
    if (time -> hours > 12) {
        time -> hours -= 12;
        time -> am = ! (time -> am);
    }
}

/*
    Function name: TimeSeconds
    Input:  uint8_T seconds: number of seconds 
    Output: time : the time object representing seconds (s)
    Logic:  fill returning time with corresponding seconds
    Example call: Time time = TimeSeconds(50);
 */
Time TimeSeconds (uint8_t seconds) {
    Time t;
    t.seconds = seconds;
    t.hours = t.minutes = 0;
    t.am = false;
    return t;
}

/*
    Function name: TimeMinutes
    Input:  uint8_T minutes: number of minutes 
    Output: time : the time object representing minutes (m)
    Logic:  fill returning time with corresponding minutes
    Example call: Time time = TimeMinutes(50);
 */
Time TimeMinutes (uint8_t minutes) {
    Time t;
    t.minutes = minutes;
    t.hours = t.seconds = 0;
    t.am = false;
    return t;
}

/*
    Function name: TimeHours
    Input:  uint8_T hours: number of hours 
    Output: time : the time object representing hours (h)
    Logic:  fill returning time with corresponding hours
    Example call: Time time = TimeHours(11);
 */
Time TimeHours (uint8_t hours) {
    Time t;
    t.hours = hours;
    t.minutes = t.seconds = 0;
    t.am = false;
    return t;
}

/*
    Function name: TimeToSeconds
    Input:  Time t: to be converted to seconds 
    Output: uint32_t : seconds 
    Logic: Convert hours, minutes and seconds of given Time to seconds 
    Example call: 
    Time t;
    t.hours =  2;
    t.minutes = 32;
    t.seconds = 2;
    uint32_t s = TimeToSeconds(t);
 */
uint32_t TimeToSeconds(Time t) {
    uint32_t s = t.seconds;
    s += t.minutes * 60;
    s += t.hours * 3600;
    return s;
}

/*
    Function name: SecondsToTime
    Input:  uint32_t s: seconds to be converted 
            Time *t -> Time struct to be filled 
    Output: none
    Logic: Convert the seconds and fill it in the give Time structure 
    Example call: 
    Time t;
    uint32_t s = 34555;
    SecondsToTime(s, &t);
 */
void SecondsToTime(uint32_t s, Time  *t) {
    t -> hours = s / 3600;
    s %= 3600;
    t -> minutes = s / 60;
    s %= 60;
    t -> seconds = s;
}

/*
    Function name: TimeDifference
    Input:  Time t1, t2 : the time objects 
    Output: Time : t1 - t2 
    Logic: Convert into seconds, find difference, convert back to Time 
 */
Time TimeDifference(Time t1, Time t2) {
    uint32_t s1 = TimeToSeconds(t1);
    uint32_t s2 = TimeToSeconds(t2);

    Time t;
    SecondsToTime(abs(s1 - s2), &t);

    return t;
}

/*
    Function name: startStopwatch
    Input:  None
    Output: None
    Logic:  Get the current time and store it
    Example call: startStopwatch();
 */
void startStopwatch() {
    STOPWATCH_COUNTING = true;
    Time t;
    RTC_getTime(&t);
    STOPWATCH_START = t; 
}

/*
    Function name: stopStopwatch
    Input:  none
    Output: Time : time elapsed
    Logic:  Get the current time and return the difference between current time and 
            the previously stored one
    Example call: Time elapsed = stopStopwatch();
 */
Time stopStopwatch() {
    STOPWATCH_COUNTING = false;
    Time t;
    RTC_getTime(&t);
    return TimeDifference(t, STOPWATCH_START);
}

int main() {
    initI2C();
    initUSART();
    initAlarm();
    sei();

    Time time, alarm;
    Date date;
    char data[32] = "nothing";

    while (1) {
        USART_sendByte('\n');
        USART_sendData("INTERFACING ARDUINO WITH RTC MODULE DS3231SN\n");
        USART_sendData("--------------------------------------------\n\n");

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
        USART_sendData("ENTER \"alarm\" FOR setting ALARM  (current alarm @ ");
        alarm = RTC_getAlarm0();
        USART_sendData((uint16_t)alarm.hours);
        USART_sendData(": ");
        USART_sendData((uint16_t)alarm.minutes);
        USART_sendData(" ");
        if (alarm.am) USART_sendData("am)\n");
        else USART_sendData("pm)\n");
        if (!STOPWATCH_COUNTING)
            USART_sendData("ENTER \"stopwatch start\" TO START STOPWATCH\n");
        else 
            USART_sendData("ENTER \"stopwatch stop\" TO STOP STOPWATCH\n");
        USART_sendData("ENTER \"timer\" to SET TIMER\n");
        USART_sendData("\nAny other input will refresh\n");

        //wait for user-input
        while (!USART_BUFFER_READY) {
            if (RTC_checkAlarm0()) { 
                USART_sendData("\n\nALARM RINGING - Enter \"stop\" to stop ringing.\n");
                USART_sendData("Enter \"snooze\" to snooze for 10 minutes\n");
                alarm_ringing = true;
            } 

            if (RTC_checkAlarm1()) {
                TIMER_RINGING = true;
                USART_sendData("\n\nTIMER RINGING - Enter \"stop\" to stop ringing.\n");
            }

            if (alarm_ringing) {
                Time t, alarm;
                RTC_getTime(&t);
                alarm = RTC_getAlarm0();

                if (TimeToSeconds(TimeDifference(t, alarm)) > 60) {
                    alarm_ringing = false;
                }
            }

            if (TIMER_RINGING) {
                Time t, timer;
                RTC_getTime(&t);
                timer = RTC_getAlarm1();

                if (TimeToSeconds(TimeDifference(t, timer)) > 60) {
                    TIMER_RINGING = false;
                }
            }
        }

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
        else if (strcmp(data, "alarm") == 0) {
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

            USART_sendData("If AM enter 1:\n");
            while (!USART_BUFFER_READY);
            USART_readBuffer(data);
            if (data[0] == '1') {
                time.am = true;
            }
            else {
                time.am = false;
            }

            RTC_setAlarm0(time);
        } 
        else if (strcmp(data, "stop") == 0) {
            alarm_ringing = false;
            if (TIMER_RINGING) {
                TIMER_SET = false;
                TIMER_RINGING = false;
            }
        }
        else if (strcmp(data, "snooze") == 0) {
            if (alarm_ringing) {
                alarm_ringing = false;
                Time temp;
                RTC_getTime(&temp);
                addTime(&temp, TimeMinutes(10));
                RTC_setAlarm0(temp);
            }
        }
        else if (strcmp(data, "stopwatch start") == 0) {
            if (!STOPWATCH_COUNTING)
                startStopwatch();
        }
        else if (strcmp(data, "stopwatch stop") == 0) {
            if (STOPWATCH_COUNTING) {
                Time t = stopStopwatch();
                USART_sendData("\nSTOPWATCH: ");
                USART_sendData((uint16_t) t.hours);
                USART_sendData(" : ");
                USART_sendData((uint16_t) t.minutes);
                USART_sendData(" : ");
                USART_sendData((uint16_t) t.seconds);
                USART_sendData("\n");
            }  
        }
        else if (strcmp(data, "timer") == 0) {
            Time t;
            USART_sendData("Enter hours(0-12):");
            while (!USART_BUFFER_READY);
            USART_readBuffer(data);
            t.hours = atoi(data);
            
            USART_sendData("Enter minutes(0-59):");
            while (!USART_BUFFER_READY);
            USART_readBuffer(data);
            t.minutes = atoi(data);

            USART_sendData("Enter seconds(0-59):");
            while (!USART_BUFFER_READY);
            USART_readBuffer(data);
            t.seconds = atoi(data);

            Time now;
            RTC_getTime(&now);

            addTime(&t, now);

            RTC_setAlarm1(t);
            TIMER_SET = true;
        }
    }
    
    return 0;
}