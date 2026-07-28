idf_component_register(SRCS "main.c" "mpu9250.c" "voting.c" "fdir.c" "tx.c" "fusion.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_driver_spi esp_driver_twai)
set_source_files_properties(tx.c PROPERTIES COMPILE_FLAGS "-Wno-error=cpp")