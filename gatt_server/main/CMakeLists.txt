idf_component_register(SRCS "gatts_demo.c"
                    PRIV_REQUIRES bt nvs_flash driver
                    INCLUDE_DIRS "." "../../protocol")
