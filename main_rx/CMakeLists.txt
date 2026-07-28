 cmake_minimum_required(VERSION 4.3)
idf_component_register(SRCS "main_rx.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_driver_twai)
set_source_files_properties(main_rx.c PROPERTIES COMPILE_FLAGS "-Wno-error=cpp")