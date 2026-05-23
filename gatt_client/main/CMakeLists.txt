idf_component_register(SRCS "gattc_demo.c"
                    PRIV_REQUIRES bt nvs_flash esp_adc driver
                    INCLUDE_DIRS "." "../../protocol")
