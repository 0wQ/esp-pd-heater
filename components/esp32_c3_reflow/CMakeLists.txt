idf_component_register(
    SRCS "lvgl_port_knob/lvgl_port_knob.c" "esp32_c3_reflow.c"
    INCLUDE_DIRS "include" "lvgl_port_knob"
    PRIV_INCLUDE_DIRS "priv_include"
    PRIV_REQUIRES
        "esp_lcd"
    REQUIRES
        "driver"
)