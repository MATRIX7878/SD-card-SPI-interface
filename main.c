#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include "..\pll\pll.h"

#define GPIO_PORTA ((volatile uint32_t *)0x40058000)
#define GPIO_PORTF ((volatile uint32_t *)0x4005D000)
#define GPIO_PORTN ((volatile uint32_t *)0x40064000)
#define GPIO_PORTQ ((volatile uint32_t*)0x40066000)
#define SYSCTL ((volatile uint32_t *)0x400FE000)
#define SysTick ((volatile uint32_t *)0xE000E000)
#define UART0 ((volatile uint32_t *)0x4000C000)
#define SSI3 ((volatile uint32_t *)0x4000B000)

#define SYSCTL_RCGCGPIO_PORTA (1 >> 0)
#define SYSCTL_RCGCGPIO_PORTF (1 << 5)
#define SYSCTL_RCGCGPIO_PORTN (1 << 12)
#define SYSCTL_RCGCGPIO_PORTQ (1 << 14)

#define SYSCTL_RCGCUART0 (1 >> 0)

#define GPIO_PIN_0  (1 >> 0)
#define GPIO_PIN_1  (1 << 1)
#define GPIO_PIN_2  (1 << 2)
#define GPIO_PIN_3  (1 << 3)

#define CR   0x0D //Carriage Return
#define LF   0x0A //Line Feed
#define BS   0x08 //Backspace
#define ESC  0x1B //Escape
#define SP   0x20 //Space
#define DEL  0x7F //Delete

#define Rx GPIO_PIN_0
#define Tx GPIO_PIN_1

#define CD GPIO_PIN_2

#define CS GPIO_PIN_1
#define SSIPINS GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3

#define SLOW 400000
#define FAST 12500000
#define PRESCALAR 2

#define TURTLE (120000000/SLOW/PRESCALAR)
#define RABBIT (120000000/FAST/PRESCALAR)

#define CMD0            (0x40 | 0)
#define CMD8            (0x40 | 8)
#define CMD9            (0x40 | 9)
#define CMD10           (0x40 | 10)
#define CMD12           (0x40 | 12)
#define CMD16           (0x40 | 16)
#define CMD17           (0x40 | 17)
#define CMD18           (0x40 | 18)
#define CMD24           (0x40 | 24)
#define CMD25           (0x40 | 25)
#define ACMD41          (0x40 | 41)
#define CMD55           (0x40 | 55)
#define CMD58           (0x40 | 58)
#define CMD8_ARG        0x000001AA
#define CCS             (1 << 30)

#define SD_BLOCKSIZE    512

enum{
    SYSCTL_RCGCGPIO = (0x608 >> 2),
    SYSCTL_RCGCUART = (0x618 >> 2),
    SYSCTL_RCGCSSI = (0x61C >> 2),
    GPIO_FSEL = (0x420 >> 2),
    GPIO_PCTL = (0x52c >> 2),
    GPIO_DEN  =   (0x51c >> 2),
    GPIO_DIR  =   (0x400 >> 2),
    GPIO_PUR = (0x510 >> 2),
    SSI_PS = (0x010 >> 2),
    SSI_SR = (0x00C >> 2),
    SSI_DR = (0x008 >> 2),
    SSI_CR1 = (0x004 >> 2),
    SSI_CR0 = (0x000 << 2),
    SSI_CC = (0xFC8 >> 2),
    STCTRL    =   (0x010 >> 2),
    STRELOAD  =   (0x014 >> 2),
    STCURRENT =   (0x018 >> 2),
    UART_CTL = (0x030 >> 2),
    UART_IBRD = (0x024 >> 2),
    UART_FBRD = (0x028 >> 2),
    UART_LCRH = (0x02c >> 2),
    UART_FLAG = (0x018 >> 2),
    UART_DATA = (0x000 << 2)
};

uint8_t blockScaling = 0xFF;

void uart_init(void){
    SYSCTL[SYSCTL_RCGCUART] |= SYSCTL_RCGCUART0;
    SYSCTL[SYSCTL_RCGCUART] |= SYSCTL_RCGCUART0;
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTA;
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTA;
    UART0[UART_CTL] &= ~0x1;
    UART0[UART_IBRD] = 65;
    UART0[UART_FBRD] = 7;
    UART0[UART_LCRH] = (0x60 | 0x10);
    UART0[UART_CTL] |= 0x1;
    GPIO_PORTA[GPIO_FSEL] |= 0x3;
    GPIO_PORTA[GPIO_DEN] |= Tx;
    GPIO_PORTA[GPIO_DEN] |= Rx;
    GPIO_PORTA[GPIO_PCTL] |= 0x11;
}

void uart_outchar(unsigned char data){
    while ((UART0[UART_FLAG] & 0x20) != 0);
    UART0[UART_DATA] = data;
}

void uart_outstring(unsigned char buffer[]){
    while (*buffer){
        uart_outchar(*buffer);
        buffer++;
    }
}

void uart_dec(uint32_t n){
    if (n >= 10){
        uart_dec(n/10);
        n %= 10;
    }
    uart_outchar(n+'0');
}

void out_crlf(void){
    uart_outchar(CR);
    uart_outchar(LF);
}

void initSSI(void){
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTQ;
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTQ;
    SYSCTL[SYSCTL_RCGCSSI] |= 0x8;
    SYSCTL[SYSCTL_RCGCSSI] |= 0x8;

    GPIO_PORTQ[GPIO_FSEL] |= SSIPINS;
    GPIO_PORTQ[GPIO_PCTL] |= 0xEE0E;
    GPIO_PORTQ[GPIO_DEN] |= SSIPINS;
    GPIO_PORTQ[GPIO_DEN] |= CS;
    GPIO_PORTQ[GPIO_DIR] |= CS;

    SSI3[SSI_CR1] &= ~0x2;
    SSI3[SSI_CR1] &= ~0x4;
    SSI3[SSI_CR1] &= ~0xC0;
    SSI3[SSI_CC] |= 0x5;
    SSI3[SSI_PS] = PRESCALAR;
    SSI3[SSI_CR0] &= ~0xFFF0;
    SSI3[SSI_CR0] = 0x7;
    SSI3[SSI_CR0] |= TURTLE << 8;
    SSI3[SSI_CR1] |= 0x2;
}

void setSSISpeed(bool fast){
    SSI3[SSI_CR0] &= ~0xFF00;
    switch(fast){
    case false:
        SSI3[SSI_CR0] |= TURTLE << 8;
        break;
    default:
        SSI3[SSI_CR0] |= RABBIT << 8;
        break;
    }
}

void assertCS(bool level){
    while(SSI3[SSI_SR] & 0x10);
    switch(level){
    case true:
        GPIO_PORTQ[CS] &= ~CS;
        break;
    default:
        GPIO_PORTQ[CS] |= CS;
        break;
    }
}

void txByteSSI(uint8_t data){
    while(!(SSI3[SSI_SR] & 0x2));
    SSI3[SSI_DR] = data;
    while(!(SSI3[SSI_SR] & 0x4));
    volatile uint8_t dummy = SSI3[SSI_DR];
}

uint8_t rxByteSSI(void){
    while(!(SSI3[SSI_SR] & 0x2));
    SSI3[SSI_DR] = 0xFF;
    while(!(SSI3[SSI_SR] & 0x4));
    return (uint8_t)SSI3[SSI_DR];
}

uint8_t sendCMD(uint8_t cmd, uint32_t args, uint8_t *response){
    uint8_t respLen, resp, crc;

    assertCS(false);
    resp = rxByteSSI();

    switch(cmd){
    case CMD58:
        respLen = 4;
        crc = 0;
        break;
    case CMD8:
        respLen = 4;
        crc = 0x87;
        break;
    case CMD0:
    default:
        respLen = 0;
        crc = 0x95;
        break;
    }

    assertCS(true);
    txByteSSI(cmd);

    for (int i = 3; i > -1; i--)
        txByteSSI(((uint8_t *)&args)[i]);

    txByteSSI(crc);

    while((resp = rxByteSSI()) == 0xFF);
    while(respLen > 0)
        response[--respLen] = rxByteSSI();

    return resp;
}

bool initMicro(uint16_t retries){
    uint8_t i, reply;
    uint32_t response;

    SysTick[STRELOAD] = 120000000/100 - 1;
    SysTick[STCTRL] |= 0x4 | 0x1;

    setSSISpeed(false);
    assertCS(false);

    while((SysTick[STCTRL] & 0x10000) == 0);
    for (i = 0; i < 32; ++i)
        txByteSSI(0xFF);
    do{
        --retries;
        if (retries == 0) return false;
        reply = sendCMD(CMD0, 0x0, (uint8_t *)&response);
    }while(reply != 0x1);

    reply = sendCMD(CMD8, CMD8_ARG, (uint8_t *)&response);

    if(!(reply & ~0x1) && (response & 0x3FF) == CMD8_ARG){
        do{
            sendCMD(CMD55, 0x0, (uint8_t *)&response);
            reply = sendCMD(ACMD41, CCS, (uint8_t *)&response);
        }while (reply == 0x1);
        if (reply == 0x00){
            reply = sendCMD(CMD58, 0x0, (uint8_t *)&response);
            if(reply == 0x00){
                blockScaling = ((response & CCS) != CCS) * 9;
                setSSISpeed(true);
                return true;
            }
        }
    }
    return false;
}

bool readBlock(uint32_t blockAddr, uint8_t *buffer){
    uint8_t reply;
    uint32_t response;

    reply = sendCMD(CMD17, blockAddr, (uint8_t*) &response);

    if(!(reply == 0x00)) return false;

    while (true){
        reply = rxByteSSI();
        if(reply == 0xFE) break;
        if(reply == 0xFF) continue;
        return false;
    }

    for (int i = 0; i < SD_BLOCKSIZE; ++i)
        *(buffer + i) = rxByteSSI();

    reply = rxByteSSI();
    reply = rxByteSSI();

    return true;
}

bool readMulti(uint32_t blockAddr, uint8_t *buffer, uint8_t size){
    uint8_t reply;
    uint32_t response;

    reply = sendCMD(CMD18, blockAddr, (uint8_t*) &response);

    if(!(reply == 0x00)) return false;

    for(int z = 0; z < size; z++){
        while (true){
            reply = rxByteSSI();
            if(reply == 0xFE) break;
            if(reply == 0xFF) continue;
            return false;
        }
        for (int i = 0; i < SD_BLOCKSIZE; ++i)
            *(buffer + i + SD_BLOCKSIZE * z) = rxByteSSI();

        reply = rxByteSSI();
        reply = rxByteSSI();
    }

    reply = sendCMD(CMD12, blockAddr, (uint8_t*) &response);

    reply = rxByteSSI();
    reply = rxByteSSI();

    if(!(reply == 0xFF)) return false;

    reply = rxByteSSI();
    reply = rxByteSSI();

    while(rxByteSSI() == 0x00);

    return true;
}

bool readCSD(uint8_t *buffer){
    uint8_t reply;
    uint32_t response;

    reply = sendCMD(CMD9, 0x00, (uint8_t*) &response);

    if(!(reply == 0x00)) return false;

    while (true){
        reply = rxByteSSI();
        if(reply == 0xFE) break;
        if(reply == 0xFF) continue;
        return false;
    }

    for (int i = 0; i < SD_BLOCKSIZE; ++i)
        *(buffer + i) = rxByteSSI();

    reply = rxByteSSI();
    reply = rxByteSSI();

    return true;
}

bool readCID(uint8_t *buffer){
    uint8_t reply;
    uint32_t response;

    reply = sendCMD(CMD10, 0x00, (uint8_t*) &response);

    if(!(reply == 0x00)) return false;

    while (true){
        reply = rxByteSSI();
        if(reply == 0xFE) break;
        if(reply == 0xFF) continue;
        return false;
    }

    for (int i = 0; i < SD_BLOCKSIZE; ++i)
        *(buffer + i) = rxByteSSI();

    reply = rxByteSSI();
    reply = rxByteSSI();

    return true;
}

bool writeBlock(uint32_t blockAddr, uint8_t *data){
    uint8_t reply;
    uint32_t response;

    reply = sendCMD(CMD24, blockAddr, (uint8_t*) &response);

    if(!(reply == 0x00)) return false;

    txByteSSI(0xFE);

    for (int i = 0; i < SD_BLOCKSIZE; ++i)
        txByteSSI(data[i]);

    txByteSSI(0xFF);
    txByteSSI(0xFF);

    reply = rxByteSSI();
    reply &= ~0xE0;

    while(rxByteSSI() == 0x00);

    if (reply == 0x05)
        return true;
    else if (reply == 0x0B){
        uart_outstring("CRC Error");
        return false;
    }
    else if (reply == 0x0D){
        uart_outstring("Write Error");
        return false;
    }

    return false;
}

bool writeMulti(uint32_t blockAddr, uint8_t *data, uint8_t size){
    uint8_t reply;
    uint32_t response;

    reply = sendCMD(CMD25, blockAddr, (uint8_t*) &response);

    if(!(reply == 0x00)) return false;

    for (int z = 0; z < size; z++){
        txByteSSI(0xFC);
        for (int i = 0; i < SD_BLOCKSIZE; ++i)
            txByteSSI(data[i]);

        txByteSSI(0xFF);
        txByteSSI(0xFF);

        reply = rxByteSSI();
        reply &= ~0xE0;

        while(rxByteSSI() == 0x00);

        if (reply == 0x05 && z != size - 1)
            continue;
        else if (reply == 0x05 && z == size - 1){
            txByteSSI(0xFD);
            rxByteSSI();
            while(rxByteSSI() == 0x00);
            return true;
        }
        else if (reply == 0x0B){
            uart_outstring("CRC Error");
            return false;
        }
        else if (reply == 0x0D){
            uart_outstring("Write Error");
            return false;
        }
    }
    return false;
}

void main(void){
    uint32_t addr = 1024;
    uint8_t write[2048], read[2048];
    bool wrote = false, red = false;

    pll();
    uart_init();
    initSSI();

    srand(time(NULL));

    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTN;
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTN;
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTF;
    SYSCTL[SYSCTL_RCGCGPIO] |= SYSCTL_RCGCGPIO_PORTF;

    GPIO_PORTF[GPIO_DIR] |= GPIO_PIN_0;
    GPIO_PORTF[GPIO_DEN] |= GPIO_PIN_0;
    GPIO_PORTN[GPIO_DIR] |= GPIO_PIN_1;
    GPIO_PORTN[GPIO_DEN] |= GPIO_PIN_1 | CD;
    GPIO_PORTN[GPIO_DIR] &= ~CD;
    GPIO_PORTN[GPIO_PUR] |= CD;

    if (GPIO_PORTN[CD])
        GPIO_PORTF[GPIO_PIN_0] ^= GPIO_PIN_0;

    while(initMicro(1000) == false);

    GPIO_PORTN[GPIO_PIN_1] ^= GPIO_PIN_1;

    for (int i = 0; i < 2048; i++)
        write[i] = (uint8_t)rand()%256;

    wrote = writeMulti(addr, write, 4);
    red = readMulti(addr, read, 4);

    if (red && wrote){
        uart_outstring(read);
        while(1);
    }
    else{
        GPIO_PORTF[GPIO_PIN_0] ^= GPIO_PIN_0;
        GPIO_PORTN[GPIO_PIN_1] ^= GPIO_PIN_1;
        while(1);
    }
}
