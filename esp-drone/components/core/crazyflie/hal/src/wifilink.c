/*
 * wifilink.c: ESP-NOW/serial CRTP link for ESP-Drone
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "wifilink.h"
#include "serial_transport_2.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"
#include "pm_esplane.h"
#include "system.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queuemonitor.h"
#include "semphr.h"
#include "stm32_legacy.h"

#define DEBUG_MODULE "WIFILINK"
#include "debug_cf.h"
#include "static_mem.h"

/*
 * Give the link more time before the firmware says "connection lost".
 * 1000 ms was too short while debugging.
 */
#define WIFI_ACTIVITY_TIMEOUT_MS (3000)

static bool isInit = false;

/*
 * IMPORTANT:
 * Setpoint commands should not pile up.
 * We only want the newest packet.
 */
static xQueueHandle crtpPacketDelivery;
STATIC_MEM_QUEUE_ALLOC(crtpPacketDelivery, 1, sizeof(CRTPPacket));

static uint8_t sendBuffer[64];

static SerialPacket serialIn;
static uint32_t lastPacketTick = 0;

static int wifilinkSendPacket(CRTPPacket *p);
static int wifilinkSetEnable(bool enable);
static int wifilinkReceiveCRTPPacket(CRTPPacket *p);

STATIC_MEM_TASK_ALLOC(wifilinkTask, 4096);

static bool wifilinkIsConnected(void)
{
    if (lastPacketTick == 0) {
        return false;
    }

    return (xTaskGetTickCount() - lastPacketTick) < M2T(WIFI_ACTIVITY_TIMEOUT_MS);
}

static struct crtpLinkOperations wifilinkOp = {
    .setEnable     = wifilinkSetEnable,
    .sendPacket    = wifilinkSendPacket,
    .receivePacket = wifilinkReceiveCRTPPacket,
    .isConnected   = wifilinkIsConnected,
};

static bool checkChecksum(const SerialPacket *packet)
{
    uint8_t checksum = 0;

    if (packet == NULL) {
        return false;
    }

    if (packet->size < 3 || packet->size > SERIAL_PACKET_SIZE) {
        return false;
    }

    for (int i = 0; i < packet->size - 1; i++) {
        checksum += packet->data[i];
    }

    return checksum == packet->data[packet->size - 1];
}

/*
 * Temporary bench-test arming for our custom Python packets.
 *
 * Expected packet from Python/XIAO/ESP-NOW:
 *
 * data[0]      = CRTP header, usually 0x30
 * data[1..4]   = roll float
 * data[5..8]   = pitch float
 * data[9..12]  = yaw rate float
 * data[13..14] = thrust uint16 little-endian
 * data[15]     = checksum
 *
 * If thrust > 0, arm.
 * If thrust == 0, disarm.
 *
 * Props OFF while testing this.
 */
static void updateArmingFromSetpoint(const SerialPacket *packet)
{
    static bool printedArmed = false;
    static bool printedDisarmed = false;

    if (packet == NULL) {
        return;
    }

    if (packet->size < 16) {
        return;
    }

    if (packet->data[0] != 0x30) {
        return;
    }

    uint16_t thrust = packet->data[13] | ((uint16_t)packet->data[14] << 8);

    if (thrust > 0) {
        systemSetArmed(true);

        if (!printedArmed) {
            DEBUG_PRINTI("AUTO ARM: thrust=%u\n", thrust);
            printedArmed = true;
        }
    } else {
        systemSetArmed(false);

        if (!printedDisarmed) {
            DEBUG_PRINTI("AUTO DISARM: thrust=0\n");
            printedDisarmed = true;
        }
    }
}

static void wifilinkTask(void *param)
{
    while (1) {
        if (!serialGetDataBlocking(&serialIn)) {
            continue;
        }

        /*
         * Expected ESP-NOW payload from XIAO:
         *
         * byte 0      = CRTP header
         * byte 1..N   = CRTP data
         * last byte   = checksum
         *
         * checksum = uint8_t sum of all previous bytes
         *
         * Example:
         * 0x30 + roll(float) + pitch(float) + yawrate(float) + thrust(uint16) + checksum
         */

        if (!checkChecksum(&serialIn)) {
            continue;
        }

        CRTPPacket packet = {0};

        /*
         * Remove only the checksum.
         *
         * CRTP raw layout:
         * raw[0] = CRTP header
         * raw[1...] = CRTP data
         *
         * CRTPPacket.size is data length only.
         * So:
         * received size - header - checksum
         */
        packet.size = serialIn.size - 2;
        memcpy(&packet.raw, serialIn.data, serialIn.size - 1);

        /*
         * As soon as a valid packet arrives, mark the link as alive.
         * Do not depend on the queue accepting it.
         */
        lastPacketTick = xTaskGetTickCount();

        /*
         * Temporary auto-arm for Python thrust packets.
         */
        updateArmingFromSetpoint(&serialIn);

        static uint32_t goodPackets = 0;
        goodPackets++;

        if ((goodPackets % 50) == 0) {
            DEBUG_PRINTI("Good CRTP packets: %lu\n", goodPackets);
        }

        /*
         * Keep only the newest setpoint.
         * This avoids old motor commands backing up.
         */
        xQueueOverwrite(crtpPacketDelivery, &packet);
    }
}

static int wifilinkReceiveCRTPPacket(CRTPPacket *p)
{
    /*
     * Wait up to 100 ms for the newest CRTP packet.
     * If none arrives, return -1 and the rest of firmware can handle timeout safely.
     */
    if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE) {
        ledseqRun(&seq_linkUp);
        return 0;
    }

    return -1;
}

static int wifilinkSendPacket(CRTPPacket *p)
{
    int dataSize;

    ASSERT(p->size < SYSLINK_MTU);

    sendBuffer[0] = p->header;

    if (p->size <= CRTP_MAX_DATA_SIZE) {
        memcpy(&sendBuffer[1], p->data, p->size);
    }

    dataSize = p->size + 1;

    return serialSendData(dataSize, sendBuffer);
}

static int wifilinkSetEnable(bool enable)
{
    return 0;
}

void wifilinkInit()
{
    if (isInit) {
        return;
    }

    serialInit();

    crtpPacketDelivery = STATIC_MEM_QUEUE_CREATE(crtpPacketDelivery);
    DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);

    STATIC_MEM_TASK_CREATE(wifilinkTask,
                           wifilinkTask,
                           WIFILINK_TASK_NAME,
                           NULL,
                           WIFILINK_TASK_PRI);

    isInit = true;
}

bool wifilinkTest()
{
    return isInit;
}

struct crtpLinkOperations *wifilinkGetLink()
{
    return &wifilinkOp;
}