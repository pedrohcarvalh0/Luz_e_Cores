#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "pico/bootrom.h"
#include "ssd1306.h"
#include "font.h"
#include "lib/bh1750_light_sensor.h"
#include "ws2812.pio.h"

// ===== DEFINIÇÕES DE PINOS E ENDEREÇOS =====
// Extensor I2C conectado na I2C0 da BitDogLab (J1)
// Sensor GY-302 (BH1750) conectado na J2 do extensor - endereço 0x23
// Sensor GY-33 (TCS34725) conectado na J3 do extensor - endereço 0x29
// Usar código de verificar endereços: https://github.com/wiltonlacerda/EmbarcaTechResU3Ex04
#define I2C_SENSORS_PORT i2c0
#define I2C_SENSORS_SDA 0
#define I2C_SENSORS_SCL 1

// Endereços dos sensores no extensor I2C
#define GY33_I2C_ADDR 0x29      // Sensor de cor na J3
#define BH1750_I2C_ADDR 0x23    // Sensor de luz na J2

// Display OLED SSD1306
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define OLED_ADDR 0x3C

// Matriz LED WS2812 e Buzzer
#define WS2812_PIN 7
#define NUM_PIXELS 25
#define BUZZER_PIN 10

// Botões
#define BTN_A_PIN 5
#define BTN_B_PIN 6

// ===== REGISTROS DO SENSOR GY-33 =====
#define ENABLE_REG 0x80
#define ATIME_REG 0x81
#define CONTROL_REG 0x8F
#define ID_REG 0x92
#define STATUS_REG 0x93
#define CDATA_REG 0x94
#define RDATA_REG 0x96
#define GDATA_REG 0x98
#define BDATA_REG 0x9A

// ===== CONFIGURAÇÕES DO SISTEMA =====
#define LUX_THRESHOLD_HIGH 800  // Limite alto de luminosidade para alerta do buzzer
#define UPDATE_INTERVAL_MS 200  // Intervalo de atualização em ms

// ===== VARIÁVEIS GLOBAIS =====
PIO pio = pio0;
int sm = 0;
ssd1306_t ssd;

// Estrutura para dados dos sensores
typedef struct {
    uint16_t red, green, blue, clear;
    uint16_t lux;
} sensor_data_t;

sensor_data_t current_data = {0};
bool matrix_mode = true; // true = cor, false = luminosidade

// ===== FUNÇÕES DO SENSOR GY-33 =====
void gy33_write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(I2C_SENSORS_PORT, GY33_I2C_ADDR, buffer, 2, false);
}

uint16_t gy33_read_register(uint8_t reg) {
    uint8_t buffer[2];
    i2c_write_blocking(I2C_SENSORS_PORT, GY33_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_SENSORS_PORT, GY33_I2C_ADDR, buffer, 2, false);
    return (buffer[1] << 8) | buffer[0];
}

void gy33_init() {
    gy33_write_register(ENABLE_REG, 0x03);  // Power ON + ADC Enable
    gy33_write_register(ATIME_REG, 0xF5);   // Integration time: 103ms
    gy33_write_register(CONTROL_REG, 0x00); // Gain: 1x
}

void gy33_read_color(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
    *c = gy33_read_register(CDATA_REG);
    *r = gy33_read_register(RDATA_REG);
    *g = gy33_read_register(GDATA_REG);
    *b = gy33_read_register(BDATA_REG);
}

// ===== FUNÇÕES DO BUZZER =====
void buzzer_init() {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

void play_sound(int frequency, int duration_ms) {
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    if (frequency <= 0) {
        pwm_set_gpio_level(BUZZER_PIN, 0);
        sleep_ms(duration_ms);
        return;
    }
    float divider = 20.0f;
    pwm_set_clkdiv(slice_num, divider);
    uint16_t wrap = (125000000 / (frequency * divider)) - 1;
    pwm_set_wrap(slice_num, wrap);
    pwm_set_gpio_level(BUZZER_PIN, wrap / 2);
    sleep_ms(duration_ms);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

void play_high_light_alert() {
    play_sound(1000, 200);
    sleep_ms(100);
    play_sound(800, 200);
}

// ===== FUNÇÕES DA MATRIZ LED WS2812 =====
uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

void clear_matrix() {
    for (int i = 0; i < NUM_PIXELS; i++) {
        put_pixel(0);
    }
}

void display_color_mode(uint16_t r, uint16_t g, uint16_t b) {
    // Normaliza os valores RGB para 0-255
    uint8_t red = (r * 255) / 4095;
    uint8_t green = (g * 255) / 4095;
    uint8_t blue = (b * 255) / 4095;
    
    // Limita os valores máximos para não sobrecarregar os LEDs
    red = red > 80 ? 80 : red;
    green = green > 80 ? 80 : green;
    blue = blue > 80 ? 80 : blue;
    
    uint32_t color = urgb_u32(red, green, blue);
    
    // Acende todos os LEDs com a cor detectada
    for (int i = 0; i < NUM_PIXELS; i++) {
        put_pixel(color);
    }
}

void display_brightness_mode(uint16_t lux) {
    // Calcula quantos LEDs acender baseado na luminosidade (0-1000 lux)
    int leds_to_light = (lux > 1000) ? NUM_PIXELS : (lux * NUM_PIXELS) / 1000;
    
    // Calcula intensidade branca baseada na luminosidade
    uint8_t white_intensity = (lux > 1000) ? 100 : (lux * 100) / 1000;
    uint32_t white_color = urgb_u32(white_intensity, white_intensity, white_intensity);
    
    for (int i = 0; i < NUM_PIXELS; i++) {
        if (i < leds_to_light) {
            put_pixel(white_color);
        } else {
            put_pixel(0);
        }
    }
}

void update_oled_display(sensor_data_t *data) {
    char str_red[16], str_green[16], str_blue[16], str_lux[16];
    char mode_line1[16], mode_line2[16];
    
    // Converte valores para strings
    sprintf(str_red, "R: %d", data->red);
    sprintf(str_green, "G: %d", data->green);
    sprintf(str_blue, "B: %d", data->blue);
    sprintf(str_lux, "LUX: %d", data->lux);
    
    // Divide o modo em duas linhas para melhor visualização
    if (matrix_mode) {
        sprintf(mode_line1, "Modo: Cores");
        sprintf(mode_line2, "(GY-33)");
    } else {
        sprintf(mode_line1, "Modo: Luz");
        sprintf(mode_line2, "(GY-302)");
    }
    
    // Atualiza display
    ssd1306_fill(&ssd, false);
    
    // Título
    ssd1306_draw_string(&ssd, "SENSOR MONITOR", 8, 2);
    ssd1306_line(&ssd, 0, 12, 128, 12, true);
    
    // Valores RGB
    ssd1306_draw_string(&ssd, str_red, 4, 16);
    ssd1306_draw_string(&ssd, str_green, 4, 24);
    ssd1306_draw_string(&ssd, str_blue, 4, 32);
    
    // Luminosidade
    ssd1306_draw_string(&ssd, str_lux, 4, 42);
    
    // Modo atual em duas linhas
    ssd1306_draw_string(&ssd, mode_line1, 4, 52);
    ssd1306_draw_string(&ssd, mode_line2, 4, 60);
    
    ssd1306_send_data(&ssd);
}

void handle_button_a() {
    static uint32_t last_press = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Debounce de 300ms
    if (current_time - last_press > 300) {
        if (!gpio_get(BTN_A_PIN)) {
            matrix_mode = !matrix_mode; // Alterna modo
            play_sound(200, 150); // Som grave de 200Hz por 150ms
            last_press = current_time;
        }
    }
}

// ===== FUNÇÃO DE INTERRUPÇÃO PARA BOOTSEL =====
void gpio_irq_handler(uint gpio, uint32_t events) {
    if (gpio == BTN_B_PIN) {
        reset_usb_boot(0, 0);
    }
}

// ===== FUNÇÃO PRINCIPAL =====
int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    // ===== INICIALIZAÇÃO DOS PERIFÉRICOS =====
    
    // Botão A para alternar modos
    gpio_init(BTN_A_PIN);
    gpio_set_dir(BTN_A_PIN, GPIO_IN);
    gpio_pull_up(BTN_A_PIN);
    
    // Botão BOOTSEL
    gpio_init(BTN_B_PIN);
    gpio_set_dir(BTN_B_PIN, GPIO_IN);
    gpio_pull_up(BTN_B_PIN);
    gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    
    // Inicializa I2C0 para comunicação com extensor e sensores
    i2c_init(I2C_SENSORS_PORT, 400 * 1000);
    gpio_set_function(I2C_SENSORS_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SENSORS_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SENSORS_SDA);
    gpio_pull_up(I2C_SENSORS_SCL);
    
    // I2C para display OLED
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);
    
    // Inicializa display OLED
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, OLED_ADDR, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
    
    
    // Inicializa sensores via extensor I2C
    gy33_init();
    bh1750_power_on(I2C_SENSORS_PORT);
    
    // Inicializa buzzer
    buzzer_init();
    
    // Inicializa matriz LED WS2812
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, false);
    
    // Som de inicialização
    play_sound(800, 200);
    play_sound(1000, 200);
    
    // ===== LOOP PRINCIPAL =====
    uint32_t last_alert_time = 0;
    
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        handle_button_a();
        
        // ===== LEITURA DOS SENSORES =====
        gy33_read_color(&current_data.red, &current_data.green, &current_data.blue, &current_data.clear);
        current_data.lux = bh1750_read_measurement(I2C_SENSORS_PORT);
        
        // ===== ATUALIZAÇÃO DO DISPLAY OLED =====
        update_oled_display(&current_data);
        
        if (matrix_mode) {
            // Modo cor: exibe cor detectada
            display_color_mode(current_data.red, current_data.green, current_data.blue);
        } else {
            // Modo luminosidade: LEDs brancos proporcionais à luz
            display_brightness_mode(current_data.lux);
        }
        
        if (current_data.lux > LUX_THRESHOLD_HIGH) {
            if (current_time - last_alert_time > 2000) { // Alerta a cada 2 segundos
                play_high_light_alert();
                last_alert_time = current_time;
            }
        }
        
        sleep_ms(UPDATE_INTERVAL_MS);
    }
    
    return 0;
}
