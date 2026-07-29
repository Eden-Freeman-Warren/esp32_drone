#ifndef SERIAL_TRANSPORT_H
#define SERIAL_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

#define SERIAL_PACKET_SIZE 64

typedef struct
{
    uint8_t size;
    uint8_t data[SERIAL_PACKET_SIZE];
} SerialPacket;

void serialInit(void);

bool serialGetDataBlocking(SerialPacket *pkt);

bool serialSendData(uint32_t size, uint8_t *data);

#endif