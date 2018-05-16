//#include <WiFi.h>
//#include <Wire.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs_flash.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_system.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "esp_wifi.h"
#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))
#define maxCh 13 //max Channel -> US = 11, EU = 13, Japan = 14
int curChannel = 1;
static const char* TAG = "PSensor";

#define MAC_SNIFF_WIFI 0
#define MAC_SNIFF_BLE  1
#define MIN_RSSI -90
#define LED_GPIO_PIN                    GPIO_NUM_2

const wifi_promiscuous_filter_t filt={ //Idk what this does
    .filter_mask=WIFI_PROMIS_FILTER_MASK_MGMT|WIFI_PROMIS_FILTER_MASK_DATA
};
uint64_t Macs[500]={0};
uint8_t imac=0;
uint8_t level = 0;

typedef struct {
    unsigned frame_ctrl:16;
    unsigned duration_id:16;
    uint8_t addr1[6]; /* receiver address */
    uint8_t addr2[6]; /* sender address */
    uint8_t addr3[6]; /* filtering address */
    unsigned sequence_ctrl:16;
    uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
    wifi_ieee80211_mac_hdr_t hdr;
    uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

uint64_t mac2int(const uint8_t *macadd){
	uint64_t addr =0;
	addr =  ( (uint32_t)macadd[0] | ( (uint32_t)macadd[1] << 8 ) | (uint32_t)macadd[2] << 16 ) | ( (uint32_t)macadd[3] << 24 ) | ( (uint64_t)macadd[4] << 32 ) | ( (uint64_t)macadd[5] << 48 );
        //ESP_LOGI(TAG, "Mac2int : %llu ", addr);
	return addr;
}

void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type) {
    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

    //printf(ppkt->rx_ctrl.rssi); // rssi is negative value
    if(ppkt->rx_ctrl.rssi >= MIN_RSSI){
        gpio_set_level(LED_GPIO_PIN, level ^= 1);
    	//uint8_t *p = (uint8_t *) hdr->addr2;
    	uint8_t existe=0;
        //ESP_LOGI(TAG, "Mac len  %d ", ARRAY_SIZE(hdr->addr2));
    	if(ARRAY_SIZE(hdr->addr2) == 6) {
		for(int im=0;im<imac;im++){
			if(Macs[im]==mac2int(hdr->addr2)){
				//ESP_LOGI(TAG, "Ya Existe la MAC ..");
				existe=1;
				break;
			}
		}
		if(existe==0){	
			Macs[imac]=mac2int(hdr->addr2);	
			imac++;
    			ESP_LOGI(TAG, "Agrego index:%d - Mac:%02x:%02x:%02x:%02x:%02x:%02x - RSSI:%d ",imac ,hdr->addr2[0],hdr->addr2[1],hdr->addr2[2],hdr->addr2[3],hdr->addr2[4],hdr->addr2[5], ppkt->rx_ctrl.rssi);
		}
    		//ESP_LOGI(TAG, "WiFi RSSI %d ", ppkt->rx_ctrl.rssi);
    	}
    }
}

void app_main()
{
 /* setup wifi */
  //wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  //esp_wifi_init(&cfg);
  //esp_wifi_set_storage(WIFI_STORAGE_RAM);
  //esp_wifi_set_mode(WIFI_MODE_NULL);
  //esp_wifi_start();
  //esp_wifi_set_promiscuous(true);
  //esp_wifi_set_promiscuous_filter(&filt);
  //esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
  gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  cfg.nvs_enable = 0; // we don't need any wifi settings from NVRAM
  wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT}; // we need only MGMT frames
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));                                           // configure Wifi with cfg
  //ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country));           // set locales for RF and channels
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));        // we don't need NVRAM
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));      // set MAC frame filter
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));                        // now switch on monitor mode
  ESP_ERROR_CHECK(esp_wifi_set_channel(curChannel, WIFI_SECOND_CHAN_NONE));

    //Serial.println("Changed channel:" + String(curChannel));
  while(true) {
      while(curChannel <= maxCh){
        ESP_LOGI(TAG, "------- Scaneando Canal: %d ---------",curChannel);
      	esp_wifi_set_channel(curChannel, WIFI_SECOND_CHAN_NONE);
      	vTaskDelay(5000/portTICK_RATE_MS);
      	curChannel++;
      }
      curChannel = 1;
      //ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
      ESP_LOGI(TAG, "-----------------------");
      ESP_LOGI(TAG, "Cantidad de Moviles: %d ", imac);
      ESP_LOGI(TAG, "-----------------------");
      imac=0;
      //vTaskDelay(3000 / portTICK_RATE_MS);
      //ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));                        // now switch on monitor mode
  }
}


