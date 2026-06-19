#include <stdio.h>
#include <string.h>
#include <math.h>
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_CSI_ROLE_RX
#include "mqtt_client.h"
#include "ping/ping_sock.h"
#include "lwip/ip4_addr.h"
#endif

#ifdef CONFIG_ENABLE_CAMERA
#include "esp_camera.h"
#include "esp_http_server.h"
#endif

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

// Filled in from IP_EVENT_STA_GOT_IP — used to build the camera snapshot URL
// advertised in MQTT alerts.
static char s_ip[16] = "0.0.0.0";

// ─── MQTT alert transport ─────────────────────────────────────────────────────
static esp_mqtt_client_handle_t s_mqtt = NULL;

static void mqtt_event_handler(void *args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    (void)args; (void)base; (void)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "[%s] MQTT connected → %s",
                 CONFIG_DEVICE_ID, CONFIG_MQTT_BROKER_URI);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "[%s] MQTT disconnected", CONFIG_DEVICE_ID);
        break;
    default:
        break;
    }
}

static void mqtt_start(void) {
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URI,
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    if (!s_mqtt) {
        ESP_LOGE(TAG, "[%s] MQTT init failed", CONFIG_DEVICE_ID);
        return;
    }
    esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt);
}

static void send_notification(double diff) {
    if (!s_mqtt) return;

    char body[256];
#ifdef CONFIG_ENABLE_CAMERA
    snprintf(body, sizeof(body),
             "{\"trigger\":\"radar\",\"id\":\"" CONFIG_DEVICE_ID
             "\",\"chip\":\"" CHIP_ID "\",\"diff\":%.2f"
             ",\"snapshot\":\"http://%s/snapshot\""
             ",\"stream\":\"http://%s/stream\"}",
             diff, s_ip, s_ip);
#else
    snprintf(body, sizeof(body),
             "{\"trigger\":\"radar\",\"id\":\"" CONFIG_DEVICE_ID
             "\",\"chip\":\"" CHIP_ID "\",\"diff\":%.2f}", diff);
#endif

    // QoS 1 so a single motion event isn't silently dropped on a flaky link.
    int mid = esp_mqtt_client_publish(s_mqtt, CONFIG_MQTT_MOTION_TOPIC,
                                      body, 0, 1, 0);
    if (mid >= 0) {
        ESP_LOGI(TAG, "[%s] MQTT publish %s (diff=%.2f)",
                 CONFIG_DEVICE_ID, CONFIG_MQTT_MOTION_TOPIC, diff);
    } else {
        ESP_LOGE(TAG, "[%s] MQTT publish failed (not connected?)", CONFIG_DEVICE_ID);
    }
}

// ─── Onboard camera (XIAO ESP32-S3 Sense, OV2640) ─────────────────────────────
#ifdef CONFIG_ENABLE_CAMERA
// Seeed Studio XIAO ESP32-S3 Sense DVP pin map
#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   10
#define CAM_PIN_SIOD   40
#define CAM_PIN_SIOC   39
#define CAM_PIN_D7     48
#define CAM_PIN_D6     11
#define CAM_PIN_D5     12
#define CAM_PIN_D4     14
#define CAM_PIN_D3     16
#define CAM_PIN_D2     18
#define CAM_PIN_D1     17
#define CAM_PIN_D0     15
#define CAM_PIN_VSYNC  38
#define CAM_PIN_HREF   47
#define CAM_PIN_PCLK   13

static esp_err_t camera_init(void) {
    camera_config_t config = {
        .pin_pwdn     = CAM_PIN_PWDN,
        .pin_reset    = CAM_PIN_RESET,
        .pin_xclk     = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7, .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5, .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3, .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1, .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href  = CAM_PIN_HREF,
        .pin_pclk  = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_VGA,   // 640x480 — keeps the JPEG small
        .jpeg_quality = 12,              // 0-63, lower = better quality
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
    };
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[%s] camera init failed: 0x%x (PSRAM enabled?)",
                 CONFIG_DEVICE_ID, err);
    }
    return err;
}

static esp_err_t snapshot_handler(httpd_req_t *req) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=snapshot.jpg");
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    char part_buf[64];
    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { res = ESP_FAIL; break; }
        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
        if ((res = httpd_resp_send_chunk(req, STREAM_BOUNDARY,
                                         strlen(STREAM_BOUNDARY))) != ESP_OK ||
            (res = httpd_resp_send_chunk(req, part_buf, hlen)) != ESP_OK ||
            (res = httpd_resp_send_chunk(req, (const char *)fb->buf,
                                         fb->len)) != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }
        esp_camera_fb_return(fb);
    }
    return res;
}

static void start_camera_server(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.ctrl_port   = 32768;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "[%s] camera HTTP server failed to start", CONFIG_DEVICE_ID);
        return;
    }
    httpd_uri_t snap = { .uri = "/snapshot", .method = HTTP_GET,
                         .handler = snapshot_handler };
    httpd_uri_t strm = { .uri = "/stream", .method = HTTP_GET,
                         .handler = stream_handler };
    httpd_register_uri_handler(server, &snap);
    httpd_register_uri_handler(server, &strm);
    ESP_LOGI(TAG, "[%s] camera server up → http://%s/snapshot  http://%s/stream",
             CONFIG_DEVICE_ID, s_ip, s_ip);
}
#endif // CONFIG_ENABLE_CAMERA

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

// ─── Router ping (forces a steady stream of real CSI-triggering frames) ───────
// ESP-NOW pulses from separate TX boards turned out unreliable as a CSI
// trigger (their source MAC never showed up in captured frames). Pinging the
// gateway we're already associated with does — confirmed via serial monitor —
// so we use that as the primary CSI sample-rate driver instead.
#define ROUTER_PING_INTERVAL_MS 50

static void on_ping_success(esp_ping_handle_t hdl, void *args) {
    (void)hdl; (void)args;  // just generating traffic; CSI capture happens via wifi_csi_rx_cb
}

static void start_router_ping(esp_ip4_addr_t gw_addr) {
    esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();
    config.target_addr.u_addr.ip4.addr = gw_addr.addr;
    config.target_addr.type = ESP_IPADDR_TYPE_V4;
    config.count = ESP_PING_COUNT_INFINITE;
    config.interval_ms = ROUTER_PING_INTERVAL_MS;
    config.timeout_ms = 1000;
    config.data_size = 32;

    esp_ping_callbacks_t cbs = {
        .on_ping_success = on_ping_success,
        .on_ping_timeout = NULL,
        .on_ping_end = NULL,
        .cb_args = NULL,
    };
    esp_ping_handle_t ping;
    if (esp_ping_new_session(&config, &cbs, &ping) == ESP_OK) {
        esp_ping_start(ping);
        ESP_LOGI(TAG, "[%s/%s] router ping started — %d ms interval, target " IPSTR,
                 CONFIG_DEVICE_ID, CHIP_ID, ROUTER_PING_INTERVAL_MS, IP2STR(&gw_addr));
    } else {
        ESP_LOGE(TAG, "[%s/%s] failed to start router ping", CONFIG_DEVICE_ID, CHIP_ID);
    }
}

static void on_got_ip(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)arg; (void)event_base; (void)event_id;
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&event->ip_info.ip));
    start_router_ping(event->ip_info.gw);
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
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL, NULL));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());

    // Modem sleep is on by default once STA connects, which duty-cycles the
    // radio and causes CSI capture to miss most incoming frames between
    // beacons. Disable it so CSI sampling rate matches the TX pulse rate.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

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

#ifdef CONFIG_ENABLE_CAMERA
    if (camera_init() == ESP_OK) {
        start_camera_server();
    }
#endif

    mqtt_start();
#endif
}
