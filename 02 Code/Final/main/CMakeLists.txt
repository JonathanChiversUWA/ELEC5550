idf_component_register(SRCS "final.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES usb esp_driver_gpio esp_driver_uart
                       )
