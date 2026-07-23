#include "VOFA.h"
#include "uart.h"

static void VOFA_SendByte(uint8_t byte)
{
    UART0_SendChar((char)byte);
}

static void VOFA_SendFloat(float value)
{
    union {
        float f;
        uint8_t b[4];
    } data;

    data.f = value;

    VOFA_SendByte(data.b[0]);
    VOFA_SendByte(data.b[1]);
    VOFA_SendByte(data.b[2]);
    VOFA_SendByte(data.b[3]);
}

static void VOFA_SendTail(void)
{
    VOFA_SendByte(0x00);
    VOFA_SendByte(0x00);
    VOFA_SendByte(0x80);
    VOFA_SendByte(0x7F);
}

void VOFA_SendFrame(const float *data, uint8_t len)
{
    uint8_t i;

    if (data == 0) {
        return;
    }

    for (i = 0; i < len; i++) {
        VOFA_SendFloat(data[i]);
    }

    VOFA_SendTail();
}
