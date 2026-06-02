idf_component_register(SRCS "gatts_demo.c"
                    PRIV_REQUIRES bt nvs_flash driver radar
                    INCLUDE_DIRS "." "../../protocol")
