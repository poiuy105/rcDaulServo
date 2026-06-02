idf_component_register(SRCS "gatts_demo.c"
                    PRIV_REQUIRES bt nvs_flash driver ld6004
                    INCLUDE_DIRS "." "../../protocol")
