set(srcs src/esp_mfi_aes.c src/esp_mfi_base64.c src/esp_mfi_rand.c src/esp_mfi_sha.c src/hap_platform_httpd.c src/hap_platform_keystore.c src/hap_platform_memory.c src/hap_platform_os.c)

if(NOT CONFIG_IDF_TARGET_ESP8266)
    list(APPEND srcs src/esp_mfi_i2c.c)
endif()

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.0")
    list(APPEND priv_req driver)
endif()

# IDF 6 / mbedtls 4 moved AES/SHA headers under mbedtls/private/ — shim for HAP.
# IDF 5.4 still ships the public headers, and the shim would hide them.
set(hap_priv_inc "")
set(mbedtls_priv_inc "${IDF_PATH}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/include")
if(EXISTS "${mbedtls_priv_inc}")
    list(APPEND hap_priv_inc "mbedtls_compat" "${mbedtls_priv_inc}")
endif()

idf_component_register(SRCS ${srcs}
                        INCLUDE_DIRS "include"
                        PRIV_INCLUDE_DIRS ${hap_priv_inc}
                        REQUIRES esp_http_server
                        PRIV_REQUIRES ${priv_req} mbedtls nvs_flash mdns esp_hap_core)
component_compile_options(-Wno-unused-function -std=gnu17)
