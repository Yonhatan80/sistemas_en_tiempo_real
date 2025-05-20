// funcion.h
#ifndef RGB_H
#define RGB_H

#include <stdint.h> // si necesitas tipos como uint32_t
#include "esp_err.h" // Manejo de errores

void saludar(void);
int sumar(int a, int b);
//añadido
// void configurar_led_pwm(ledc_channel_t canal, uint32_t duty, int gpio_num);

#endif