idf_component_register(SRCS "app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES HCF_ADC
                    REQUIRES HCF_IOTEC
                    REQUIRES HCF_LCD
                    REQUIRES HCF_MP
                    REQUIRES HCF_SOFT
                    )
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common
    )

