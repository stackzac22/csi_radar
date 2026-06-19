idf_component_register(SRCS "csi_radar.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES nvs_flash esp_wifi esp_event esp_netif mqtt esp_http_server esp_timer)
