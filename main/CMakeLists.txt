set(priv_req driver esp_common bt nvs_flash esp_event)

idf_component_register(SRCS "main.cpp" 
INCLUDE_DIRS "." 
REQUIRES ulp
REQUIRES nvs_flash
WHOLE_ARCHIVE
PRIV_REQUIRES ${priv_req})

ulp_add_project("lp_core_main" "${CMAKE_SOURCE_DIR}/main/lp_core/")
