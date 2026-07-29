/*
 * Minimal CRTP localization service for ESP-Drone.
 *
 * Supports:
 *   - cflib cf.extpos.send_extpos(x, y, z)
 *   - cflib cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
 *
 * It intentionally omits optional LPS, Lighthouse, and peer-localization
 * features so an OptiTrack-only build has no dependency on those modules.
 */

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "crtp_localization_service.h"
#include "estimator.h"
#include "log.h"
#include "param.h"
#include "stabilizer_types.h"


#define LOC_CHANNEL_EXT_POSITION 0
#define LOC_CHANNEL_GENERIC      1

#define LOC_TYPE_EXT_POSE        8


static bool isInit = false;

static float extPosStdDev = 0.01f;
static float extQuatStdDev = 4.5e-3f;

static positionMeasurement_t extPosition;
static poseMeasurement_t extPose;

static uint32_t tickOfLastPacket = 0;


static void handleExternalPosition(const CRTPPacket *packet)
{
    if (packet == NULL ||
        packet->size < sizeof(struct CrtpExtPosition)) {
        return;
    }

    const struct CrtpExtPosition *data =
        (const struct CrtpExtPosition *)packet->data;

    extPosition.x = data->x;
    extPosition.y = data->y;
    extPosition.z = data->z;
    extPosition.stdDev = extPosStdDev;

    estimatorEnqueuePosition(&extPosition);
    tickOfLastPacket = (uint32_t)xTaskGetTickCount();
}


static void handleExternalPose(const CRTPPacket *packet)
{
    /*
     * Generic localization packets begin with a one-byte type field.
     * The CrtpExtPose structure starts at packet->data[1].
     */
    if (packet == NULL ||
        packet->size < (1 + sizeof(struct CrtpExtPose))) {
        return;
    }

    const struct CrtpExtPose *data =
        (const struct CrtpExtPose *)&packet->data[1];

    extPose.x = data->x;
    extPose.y = data->y;
    extPose.z = data->z;

    extPose.quat.x = data->qx;
    extPose.quat.y = data->qy;
    extPose.quat.z = data->qz;
    extPose.quat.w = data->qw;

    extPose.stdDevPos = extPosStdDev;
    extPose.stdDevQuat = extQuatStdDev;

    estimatorEnqueuePose(&extPose);
    tickOfLastPacket = (uint32_t)xTaskGetTickCount();
}


static void handleGenericLocalization(const CRTPPacket *packet)
{
    if (packet == NULL || packet->size < 1) {
        return;
    }

    const uint8_t type = packet->data[0];

    if (type == LOC_TYPE_EXT_POSE) {
        handleExternalPose(packet);
    }
}


static void localizationCrtpCallback(CRTPPacket *packet)
{
    if (packet == NULL) {
        return;
    }

    switch (packet->channel) {
        case LOC_CHANNEL_EXT_POSITION:
            handleExternalPosition(packet);
            break;

        case LOC_CHANNEL_GENERIC:
            handleGenericLocalization(packet);
            break;

        default:
            break;
    }
}


void locSrvInit(void)
{
    if (isInit) {
        return;
    }

    crtpRegisterPortCB(
        CRTP_PORT_LOCALIZATION,
        localizationCrtpCallback
    );

    isInit = true;
}


void locSrvSendRangeFloat(uint8_t id, float range)
{
    /*
     * Required by the public header but intentionally unused in this
     * OptiTrack-only build.
     */
    (void)id;
    (void)range;
}


LOG_GROUP_START(ext_pos)
LOG_ADD(LOG_FLOAT, X, &extPosition.x)
LOG_ADD(LOG_FLOAT, Y, &extPosition.y)
LOG_ADD(LOG_FLOAT, Z, &extPosition.z)
LOG_GROUP_STOP(ext_pos)


LOG_GROUP_START(locSrvZ)
LOG_ADD(LOG_UINT32, tick, &tickOfLastPacket)
LOG_GROUP_STOP(locSrvZ)


PARAM_GROUP_START(locSrv)
PARAM_ADD(PARAM_FLOAT, extPosStdDev, &extPosStdDev)
PARAM_ADD(PARAM_FLOAT, extQuatStdDev, &extQuatStdDev)
PARAM_GROUP_STOP(locSrv)