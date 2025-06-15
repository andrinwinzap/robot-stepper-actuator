idf_component_register(SRCS "src/as5600.c"
                        PRIV_REQUIRES esp_timer
                       INCLUDE_DIRS "include"
                       REQUIRES driver)
