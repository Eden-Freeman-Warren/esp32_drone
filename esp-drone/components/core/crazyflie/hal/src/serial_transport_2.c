#include "serial_transport_2.h"

#include <stdbool.h>
#include <string.h>

#include "esp_err.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static QueueHandle_t rxQueue = NULL;

static uint8_t xiaoMac[6] = {
    0x34, 0x85, 0x18, 0xAB, 0xED, 0xC0
};

/*
 * Runs in ESP-NOW/Wi-Fi task context.
 * Keep it short: copy the received packet into a one-item queue.
 */
static void onDataRecv(const esp_now_recv_info_t *info,
                       const uint8_t *data,
                       int len)
{
    (void)info;

    if (rxQueue == NULL || data == NULL || len <= 0 ||
        len > SERIAL_PACKET_SIZE) {
        return;
    }

    SerialPacket pkt = {0};
    pkt.size = (uint8_t)len;
    memcpy(pkt.data, data, len);

    // Keep only the newest command. Old setpoints are useless.
    xQueueOverwrite(rxQueue, &pkt);
}

void serialInit(void)
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    uint8_t staMac[6];
    esp_wifi_get_mac(WIFI_IF_STA, staMac);                 
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_now_init();

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, xiaoMac, 6);
    peer.channel = 1;
    peer.encrypt = false;

    if (!esp_now_is_peer_exist(xiaoMac)) {
        esp_now_add_peer(&peer);
    }

    rxQueue = xQueueCreate(1, sizeof(SerialPacket));
    configASSERT(rxQueue != NULL);

    esp_now_register_recv_cb(onDataRecv);
}

bool serialGetDataBlocking(SerialPacket *pkt)
{
    if (pkt == NULL || rxQueue == NULL) {
        return false;
    }

    return xQueueReceive(rxQueue, pkt, portMAX_DELAY) == pdTRUE;
}

bool serialSendData(uint32_t size, uint8_t *data)
{
    if (data == NULL || size == 0 || size > SERIAL_PACKET_SIZE) {
        return false;
    }

    return esp_now_send(xiaoMac, data, size) == ESP_OK;
}