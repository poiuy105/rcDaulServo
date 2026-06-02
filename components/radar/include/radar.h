/*
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unified Radar Component Header
 *
 * Compile-time radar type selection via Kconfig.
 * Switching radar type only requires changing menuconfig;
 * application code using radar_config_t / radar_handle_t / radar_data_t
 * needs zero modification.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_event.h"

/* ============ Compile-time radar type selection ============ */

#if defined(CONFIG_RADAR_R60ABD1)
    #include "radar_r60abd1.h"

    typedef r60abd1_config_t      radar_config_t;
    typedef r60abd1_handle_t      radar_handle_t;
    typedef r60abd1_event_id_t    radar_event_id_t;
    typedef r60abd1_data_t        radar_data_t;

    #define RADAR_CONFIG_DEFAULT()    R60ABD1_DEFAULT_CONFIG()
    #define RADAR_INIT(cfg)           r60abd1_init(cfg)
    #define RADAR_DEINIT(hdl)         r60abd1_deinit(hdl)
    #define RADAR_ADD_HANDLER(h, fn, arg)  r60abd1_add_handler(h, fn, arg)
    #define RADAR_RM_HANDLER(h, fn)       r60abd1_remove_handler(h, fn)
    #define RADAR_EVENT_BASE         R60ABD1_EVENT
    #define RADAR_EVENT_TARGET       R60ABD1_EVENT_DATA_UPDATE

#elif defined(CONFIG_RADAR_LD2461)
    #include "radar_ld2461.h"

    typedef ld2461_config_t       radar_config_t;
    typedef ld2461_handle_t       radar_handle_t;
    typedef ld2461_event_type_t   radar_event_id_t;
    typedef ld2461_data_t         radar_data_t;

    #define RADAR_CONFIG_DEFAULT()    LD2461_DEFAULT_CONFIG()
    #define RADAR_INIT(cfg)           ld2461_init(cfg)
    #define RADAR_DEINIT(hdl)         ld2461_deinit(hdl)
    #define RADAR_ADD_HANDLER(h, fn, arg)  ld2461_add_handler(h, fn, arg)
    #define RADAR_RM_HANDLER(h, fn)       ld2461_remove_handler(h, fn)
    #define RADAR_EVENT_BASE         LD2461_EVENT
    #define RADAR_EVENT_TARGET       LD2461_EVENT_COORDINATES

#elif defined(CONFIG_RADAR_LD2452)
    #include "radar_ld2452.h"

    typedef ld2452_config_t       radar_config_t;
    typedef ld2452_handle_t       radar_handle_t;
    typedef ld2452_event_id_t     radar_event_id_t;
    typedef ld2452_data_t         radar_data_t;

    #define RADAR_CONFIG_DEFAULT()    LD2452_DEFAULT_CONFIG()
    #define RADAR_INIT(cfg)           ld2452_init(cfg)
    #define RADAR_DEINIT(hdl)         ld2452_deinit(hdl)
    #define RADAR_ADD_HANDLER(h, fn, arg)  ld2452_add_handler(h, fn, arg)
    #define RADAR_RM_HANDLER(h, fn)       ld2452_remove_handler(h, fn)
    #define RADAR_EVENT_BASE         LD2452_EVENT
    #define RADAR_EVENT_TARGET       LD2452_EVENT_TARGET_UPDATE

#elif defined(CONFIG_RADAR_LD2450)
    #include "radar_ld2450.h"

    typedef ld2450_config_t     radar_config_t;
    typedef ld2450_handle_t     radar_handle_t;
    typedef ld2450_event_id_t   radar_event_id_t;
    typedef ld2450_data_t       radar_data_t;

    #define RADAR_CONFIG_DEFAULT()    LD2450_CONFIG_DEFAULT()
    #define RADAR_INIT(cfg)           ld2450_init(cfg)
    #define RADAR_DEINIT(hdl)         ld2450_deinit(hdl)
    #define RADAR_ADD_HANDLER(h, fn, arg)  ld2450_add_handler(h, fn, arg)
    #define RADAR_RM_HANDLER(h, fn)       ld2450_remove_handler(h, fn)
    #define RADAR_EVENT_BASE         ESP_LD2450_EVENT
    #define RADAR_EVENT_TARGET       LD2450_EVENT_TARGET_UPDATE

#elif defined(CONFIG_RADAR_LD6002B)
    #include "radar_ld6002b.h"

    typedef ld6002b_config_t     radar_config_t;
    typedef ld6002b_handle_t     radar_handle_t;
    typedef ld6002b_event_id_t   radar_event_id_t;
    typedef ld6002b_data_t       radar_data_t;

    #define RADAR_CONFIG_DEFAULT()    LD6002B_CONFIG_DEFAULT()
    #define RADAR_INIT(cfg)           ld6002b_init(cfg)
    #define RADAR_DEINIT(hdl)         ld6002b_deinit(hdl)
    #define RADAR_ADD_HANDLER(h, fn, arg)  ld6002b_add_handler(h, fn, arg)
    #define RADAR_RM_HANDLER(h, fn)       ld6002b_remove_handler(h, fn)
    #define RADAR_EVENT_BASE         ESP_LD6002B_EVENT
    #define RADAR_EVENT_TARGET       LD6002B_EVENT_TARGET_UPDATE

#elif defined(CONFIG_RADAR_LD6004)
    #include "radar_ld6004.h"

    typedef ld6004_config_t       radar_config_t;
    typedef ld6004_handle_t       radar_handle_t;
    typedef ld6004_event_id_t     radar_event_id_t;
    typedef ld6004_data_t         radar_data_t;

    #define RADAR_CONFIG_DEFAULT()    LD6004_CONFIG_DEFAULT()
    #define RADAR_INIT(cfg)           ld6004_init(cfg)
    #define RADAR_DEINIT(hdl)         ld6004_deinit(hdl)
    #define RADAR_ADD_HANDLER(h, fn, arg)  ld6004_add_handler(h, fn, arg)
    #define RADAR_RM_HANDLER(h, fn)       ld6004_remove_handler(h, fn)
    #define RADAR_EVENT_BASE         ESP_LD6004_EVENT
    #define RADAR_EVENT_TARGET       LD6004_EVENT_TARGET_UPDATE

#elif defined(CONFIG_RADAR_LD2460)
    #include "radar_ld2460.h"

    typedef ld2460_config_t       radar_config_t;
    typedef ld2460_handle_t       radar_handle_t;
    typedef ld2460_event_id_t     radar_event_id_t;
    typedef ld2460_data_t         radar_data_t;

    #define RADAR_CONFIG_DEFAULT()    LD2460_CONFIG_DEFAULT()
    #define RADAR_INIT(cfg)           ld2460_init(cfg)
    #define RADAR_DEINIT(hdl)         ld2460_deinit(hdl)
    #define RADAR_ADD_HANDLER(h, fn, arg)  ld2460_add_handler(h, fn, arg)
    #define RADAR_RM_HANDLER(h, fn)       ld2460_remove_handler(h, fn)
    #define RADAR_EVENT_BASE         ESP_LD2460_EVENT
    #define RADAR_EVENT_TARGET       LD2460_EVENT_TARGET_UPDATE

#else
    #error "No radar type selected. Please select a radar type in menuconfig (Radar Configuration)."
#endif

#ifdef __cplusplus
}
#endif
