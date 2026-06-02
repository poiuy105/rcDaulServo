idf_component_register(
    SRCS "radar_ld6004.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_event nvs_flash
)
