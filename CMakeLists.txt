FILE(GLOB_RECURSE sources ./*.cpp)

idf_component_register(
        SRCS ${sources}
        INCLUDE_DIRS .
        REQUIRES driver NMEA2000 esp_event freertos soc
)