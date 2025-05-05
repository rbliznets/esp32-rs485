idf_component_register(SRCS "CRS485Task.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES task esp_driver_gpio esp_driver_uart)
 