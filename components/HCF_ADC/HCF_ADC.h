#ifndef __HCF_ADC_H
    #define __HCF_ADC_H
   
    #include "esp_err.h"

    // Inicializa o componente ADC
    esp_err_t iniciar_adc_CH0CH3(void);

    esp_err_t iniciar_adc_CHX(int canal_usado);

    // Lê o valor de uma entrada analógica
    esp_err_t adc_CH0(uint32_t *value);

    esp_err_t adc_CH3(uint32_t *value);

    esp_err_t adc_CHX(uint32_t *value, int canal_usado);

    // Finaliza o componente ADC
    void adc_limpar(void);

#endif