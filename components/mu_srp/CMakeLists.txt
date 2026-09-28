idf_component_register(
    SRCS "mu_srp.c"
    INCLUDE_DIRS "."
    REQUIRES mbedtls
    PRIV_REQUIRES hkdf-sha
)
