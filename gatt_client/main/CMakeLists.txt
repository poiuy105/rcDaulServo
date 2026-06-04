idf_component_register(SRCS "gattc_demo.c"
                    PRIV_REQUIRES bt nvs_flash esp_adc driver led_strip
                    INCLUDE_DIRS "." "../../protocol")
# Force rebuild: update timestamp

