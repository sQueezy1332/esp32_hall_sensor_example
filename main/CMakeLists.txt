file(GLOB_RECURSE srcs "main.c" "hall/*.c" "hall/*.h")

idf_component_register(
    SRCS "${srcs}"
	REQUIRES esp_adc esp_driver_gpio
    INCLUDE_DIRS "./hall"
)
