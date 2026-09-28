idf_component_register(
    SRCS "src/hap_apple_chars.c" "src/hap_apple_servs.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_hap_core
)
component_compile_options(-Wno-unused-function -std=gnu17)
