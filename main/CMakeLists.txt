idf_component_register(
    SRCS 
        "main.c"

        "./wifi_custom/wifi_custom.c"
    INCLUDE_DIRS 
        "."

        "./wifi_custom"
    REQUIRES
        "driver"
        "nvs_flash"
        "mqtt"
        "esp_wifi"
)
