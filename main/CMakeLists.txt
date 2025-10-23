file(GLOB_RECURSE srcs "main.cpp" "hall/*.c" "hall/*.h")

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "./hall"
)

set(ENV{ARDUINO_SKIP_TICK_CHECK} true)
