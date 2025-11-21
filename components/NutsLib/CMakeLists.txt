idf_component_register(SRCS "NutsLib.c" 
                    SRCS "user.c" 
                    REQUIRES driver 
                    PRIV_REQUIRES mbedtls 
                    INCLUDE_DIRS "include")
