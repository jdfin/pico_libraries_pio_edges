
add_library(pio_edges INTERFACE) 

target_sources(pio_edges INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/pio_edges.cpp
)

pico_generate_pio_header(pio_edges
    ${CMAKE_CURRENT_LIST_DIR}/src/edges.pio
)

target_include_directories(pio_edges INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(pio_edges INTERFACE
    pico_stdlib
    pico_status_led
    hardware_gpio
    hardware_pio
    hardware_pwm
    misc
)

