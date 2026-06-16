#include <stdio.h>
#include <string.h>
#include <math.h>
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CSI_RADAR";

#define TX_DELAY_MS         100
#define MOTION_THRESHOLD    (CONFIG_MOTION_THRESHOLD / 10.0)
#define DEBOUNCE_TIME_MS    CONFIG_DEBOUNCE_MS

// Chip identity string embedded in every JSON frame and log
#if CONFIG_IDF_TARGET_ESP32S3
  #define CHIP_ID "s3"
#elif CONFIG_IDF_TARGET_ESP32C5
  #define CHIP_ID "c5"
#elif CONFIG_IDF_TARGET_ESP32
  #define CHIP_ID "esp32"
#else
  #define CHIP_ID "unk"
#endif

// ─── RX-only code ─────────────────────────────────────────────────────────────
#ifdef CONFIG_CSI_ROLE_RX

static uint32_t last_trigger_ms = 0;

static void send_notification(double diff) {
    char body[192];
    snprintf(body, sizeof(body),
             "{\"trigger\":\"radar\",\"id\":\"" CONFIG_DEVICE_ID
             "\",\"chip\":\"" CHIP_ID "\",\"diff\":%.2f}", diff);

    esp_http_client_config_t cfg = {
        .url    = CONFIG_NOTIFY_URL,
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_post_field(client, body, strlen(body));
    esp_http_client_set_header(client, "Content-Type", "application/json");

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "[%s] alert sent (diff=%.2f)", CONFIG_DEVICE_ID, diff);
    } else {
        ESP_LOGE(TAG, "[%s] alert failed: %s", CONFIG_DEVICE_ID, esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
    if (!info || !info->buf || info->len == 0) return;

    static uint32_t csi_count = 0;

    // Skip invalid first word (hardware limitation on some packets)
    int start = info->first_word_invalid ? 4 : 0;
    int valid_len = info->len - start;
    if (valid_len <= 1) return;

    const wifi_pkt_rx_ctrl_t *rx = &info->rx_ctrl;

#ifdef CONFIG_CSI_SERIAL_OUTPUT
    // One JSON line per frame → RPi reads via /dev/ttyUSB0
    // Format: {"t":ms,"id":"...","chip":"...","rssi":-65,"mac":"aa:bb:cc:dd:ee:ff",
    //          "ch":1,"fwi":0,"len":128,"csi":[i,q,i,q,...]}
    printf("{\"t\":%lld,\"id\":\"" CONFIG_DEVICE_ID "\",\"chip\":\"" CHIP_ID
           "\",\"rssi\":%d,\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\""
           ",\"ch\":%d,\"fwi\":%d,\"len\":%d,\"csi\":[",
           esp_timer_get_time() / 1000LL,
           rx->rssi,
           info->mac[0], info->mac[1], info->mac[2],
           info->mac[3], info->mac[4], info->mac[5],
           rx->channel,
           info->first_word_invalid ? 1 : 0,
           valid_len);

    for (int i = start; i < info->len; i++) {
        if (i > start) putchar(',');
        printf("%d", info->buf[i]);  // buf is int8_t* — signed I/Q values
    }
    printf("]}\n");
    fflush(stdout);
#endif

    // Motion detection: track magnitude variance across I/Q pairs
    double sum = 0.0;
    int pairs = valid_len / 2;
    for (int i = start; i < info->len - 1; i += 2) {
        double re = info->buf[i];
        double im = info->buf[i + 1];
        sum += sqrt(re * re + im * im);
    }
    double mean = sum / pairs;

    static double last_mean = 0.0;
    double diff = fabs(mean - last_mean);
    last_mean = mean;

    if (diff > MOTION_THRESHOLD) {
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now_ms - last_trigger_ms > DEBOUNCE_TIME_MS) {
            ESP_LOGW(TAG, "[%s] MOTION diff=%.2f (threshold=%.1f)",
                     CONFIG_DEVICE_ID, diff, MOTION_THRESHOLD);
            send_notification(diff);
            last_trigger_ms = now_ms;
        }
    }

    if (csi_count % 10 == 0) {
        ESP_LOGI(TAG, "[%s/%s] #%lu rssi=%d ch=%d diff=%.2f",
                 CONFIG_DEVICE_ID, CHIP_ID,
                 csi_count, rx->rssi, rx->channel, diff);
    }
    csi_count++;
}

#endif // CONFIG_CSI_ROLE_RX

// ─── WiFi APSTA init (shared TX + RX) ─────────────────────────────────────────
static void wifi_init_apsta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid           = "CSI_RADAR",
            .ssid_len       = strlen("CSI_RADAR"),
            .channel        = CONFIG_WIFI_CHANNEL,
            .password       = "",
            .max_connection = 4,
            .authmode       = WIFI_AUTH_OPEN,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));

#ifdef CONFIG_CSI_ROLE_RX
    wifi_config_t sta_cfg = {
        .sta = {
            .ssid     = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());

#ifdef CONFIG_CSI_ROLE_RX
    ESP_ERROR_CHECK(esp_wifi_connect());
#endif

    ESP_LOGI(TAG, "[%s/%s] WiFi started — ch %d",
             CONFIG_DEVICE_ID, CHIP_ID, CONFIG_WIFI_CHANNEL);
}

// ─── TX task ──────────────────────────────────────────────────────────────────
#ifdef CONFIG_CSI_ROLE_TX
static void esp_now_tx_task(void *arg) {
    uint8_t bcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, bcast, 6);
    peer.channel = CONFIG_WIFI_CHANNEL;
    peer.ifidx   = WIFI_IF_AP;
    peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    uint8_t pkt[32];
    memset(pkt, 0, sizeof(pkt));
    memcpy(pkt, "CSI-PULSE", 9);   // 9-byte marker
    uint32_t seq = 0;

    while (1) {
        memcpy(pkt + 28, &seq, 4);
        esp_err_t rc = esp_now_send(bcast, pkt, sizeof(pkt));
        if (rc != ESP_OK) {
            ESP_LOGW(TAG, "[%s] send fail: %s", CONFIG_DEVICE_ID, esp_err_to_name(rc));
        }
        seq++;
        vTaskDelay(pdMS_TO_TICKS(TX_DELAY_MS));
    }
}
#endif // CONFIG_CSI_ROLE_TX

// ─── Entry point ──────────────────────────────────────────────────────────────
void app_main(void) {
    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_apsta();
    ESP_ERROR_CHECK(esp_now_init());

#ifdef CONFIG_CSI_ROLE_TX
    xTaskCreate(esp_now_tx_task, "espnow_tx", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "[%s/%s] TX — %d ms interval, ch %d",
             CONFIG_DEVICE_ID, CHIP_ID, TX_DELAY_MS, CONFIG_WIFI_CHANNEL);
#endif

#ifdef CONFIG_CSI_ROLE_RX
    // CSI config differs between HE (WiFi 6 / C5) and legacy (ESP32, S3) chips
#ifdef CONFIG_SOC_WIFI_HE_SUPPORT
    // ESP32-C5: bitfield-based wifi_csi_acquire_config_t
    wifi_csi_config_t csi_cfg = {
        .enable             = 1,
        .acquire_csi_legacy = 1,   // L-LTF  (802.11g)
        .acquire_csi_ht20   = 1,   // HT-LTF (802.11n 20 MHz)
        .acquire_csi_ht40   = 1,   // HT-LTF (802.11n 40 MHz)
        .acquire_csi_su     = 1,   // HE-LTF (802.11ax SU)
        .dump_ack_en        = 1,
    };
#else
    // ESP32 / ESP32-S3: bool-based config
    wifi_csi_config_t csi_cfg = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .dump_ack_en       = true,
    };
#endif
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    ESP_LOGI(TAG, "[%s/%s] RX — CSI enabled, ch %d",
             CONFIG_DEVICE_ID, CHIP_ID, CONFIG_WIFI_CHANNEL);
#endif
}
