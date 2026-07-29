#include "serial_transport_2.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "esp_netif.h"



static SerialPacket latestPacket;
static volatile bool hasPacket = false;

// Receive callback
static void onDataRecv(const esp_now_recv_info_t *info,
                       const uint8_t *data,
                       int len)
                       
{
    // printf("ESP-NOW packet received! len=%d\n", len);
    

    if (len > SERIAL_PACKET_SIZE) {
        len = SERIAL_PACKET_SIZE;
    }

    memcpy(latestPacket.data, data, len);
    latestPacket.size = len;
    hasPacket = true;
}
void serialInit(void)
{
    // REQUIRED for ESP-NOW
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    // ESP-NOW init
    esp_now_init();
    uint8_t xiao_mac[6] = {
    0x34, 0x85, 0x18, 0xAB, 0xED, 0xC0
   };

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, xiao_mac, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if (!esp_now_is_peer_exist(xiao_mac)) {
       esp_now_add_peer(&peer);
    }

    // Register callback (CRITICAL)
    esp_now_register_recv_cb(onDataRecv);

    

    hasPacket = false;
}

bool serialGetDataBlocking(SerialPacket *pkt)
{
    while (!hasPacket) {
        vTaskDelay(1);
    }

    hasPacket = false;
    memcpy(pkt, &latestPacket, sizeof(SerialPacket));

    return true;
}

bool serialSendData(uint32_t size, uint8_t *data)
{
    // Broadcast (no peer setup for now)
    uint8_t broadcast_mac[6] = {0xff,0xff,0xff,0xff,0xff,0xff};

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if (!esp_now_is_peer_exist(broadcast_mac)) {
        esp_now_add_peer(&peer);
    }

    return esp_now_send(broadcast_mac, data, size) == ESP_OK;
}
