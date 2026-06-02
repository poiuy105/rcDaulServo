# LD2460 radar driver sources
set(LD2460_SRCS src/ld2460/radar_ld2460.c)

# LD2450 radar driver sources
set(LD2450_SRCS src/ld2450/radar_ld2450.c)

# LD6004 radar driver sources
set(LD6004_SRCS src/ld6004/radar_ld6004.c)

# LD6002B radar driver sources
set(LD6002B_SRCS src/ld6002b/radar_ld6002b.c)

# LD2461 radar driver sources
set(LD2461_SRCS src/ld2461/radar_ld2461.c)

# LD2452 radar driver sources
set(LD2452_SRCS src/ld2452/radar_ld2452.c)

# R60ABD1 radar driver sources
set(R60ABD1_SRCS src/r60abd1/radar_r60abd1.c)

# Select sources based on configured radar type
if(CONFIG_RADAR_LD2460)
    set(RADAR_SRCS ${LD2460_SRCS})
    set(RADAR_INCLUDES include src/ld2460)
elseif(CONFIG_RADAR_LD2450)
    set(RADAR_SRCS ${LD2450_SRCS})
    set(RADAR_INCLUDES include src/ld2450)
elseif(CONFIG_RADAR_LD6004)
    set(RADAR_SRCS ${LD6004_SRCS})
    set(RADAR_INCLUDES include src/ld6004)
elseif(CONFIG_RADAR_LD6002B)
    set(RADAR_SRCS ${LD6002B_SRCS})
    set(RADAR_INCLUDES include src/ld6002b)
elseif(CONFIG_RADAR_LD2461)
    set(RADAR_SRCS ${LD2461_SRCS})
    set(RADAR_INCLUDES include src/ld2461)
elseif(CONFIG_RADAR_LD2452)
    set(RADAR_SRCS ${LD2452_SRCS})
    set(RADAR_INCLUDES include src/ld2452)
elseif(CONFIG_RADAR_R60ABD1)
    set(RADAR_SRCS ${R60ABD1_SRCS})
    set(RADAR_INCLUDES include src/r60abd1)
endif()

idf_component_register(SRCS ${RADAR_SRCS}
                       INCLUDE_DIRS ${RADAR_INCLUDES}
                       REQUIRES esp_event esp_driver_uart)
