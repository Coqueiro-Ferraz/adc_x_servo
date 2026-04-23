#ifndef __HCF_LCD_H
    #define __HCF_LCD__H
    //#include "ioplaca.h"

    void iniciar_lcd(void);
    //void lcd595_pulse(void);
    void escreve_lcd(int linha, int coluna, const char *str);
    void lcd_byte(uint8_t byte, uint8_t rs);
    void limpar_lcd(void);
    //void Enviar_lcd595(uint8_t dado);

#endif