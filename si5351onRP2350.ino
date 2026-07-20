#include <Arduino.h>
#include <Wire.h>
#include <si5351.h>
#include "hardware/clocks.h" // SDK library for overclocking

#define OLED_ADDR     0x3C // OLED screen I2C target address (0x3C or 0x3D)

// Hardware Pin Definitions for RP2350A
#define PIN_I2C_SDA   16  // GP16 (Physical Pin 21)
#define PIN_I2C_SCL   17  // GP17 (Physical Pin 22)

#define PIN_BTN_PAGE  18  // GP18 - Cycle active channels (0 to 2)
#define PIN_BTN_STEP  19  // GP19 - Change tuning step size
#define PIN_BTN_SAVE  20  // GP20 - Save active frequency to channel memory
#define PIN_ENC_A     21  // GP21 - Rotary encoder phase A
#define PIN_ENC_B     22  // GP22 - Rotary encoder phase B

#define UI_FRAME_INTERVAL 16 // ~62.5 FPS

Si5351 si5351; // Global instance of the Si5351 clock generator library

unsigned long last_ui_time = 0; // Tracks timestamp of last transmitted frame

const uint64_t step_sizes[10] = {
    1ULL, 10ULL, 100ULL, 1000ULL, 10000ULL, 
    100000ULL, 1000000ULL, 10000000ULL, 100000000ULL, 1000000000ULL
};

const char* step_labels[10] = {
    "0.1 Hz", "1 Hz", "10 Hz", "100 Hz", "1 kHz", 
    "10 kHz", "100 kHz", "1 MHz", "10 MHz", "100 MHz"
};

typedef struct {
    uint64_t freq_hz10; 
    uint8_t  step_index;
    si5351_clock clk_port;
} ChannelState;

ChannelState channels[3] = {
    {100000000ULL, 5, SI5351_CLK0},  
    {4550000ULL,   4, SI5351_CLK1},  
    {80000ULL,     3, SI5351_CLK2}   
};

uint8_t active_ch = 0;              
uint64_t tuning_freq = 100000000ULL; 
bool ui_needs_update = true;        

int old_state = 0;

struct Button {
    uint8_t pin;
    bool last_state;
    uint32_t last_debounce_time;
    bool pressed;
};

Button btn_step = {PIN_BTN_STEP, HIGH, 0, true};
Button btn_save = {PIN_BTN_SAVE, HIGH, 0, true};
Button btn_page = {PIN_BTN_PAGE, HIGH, 0, true};

uint8_t frame_buffer[1024]; 

const uint8_t font5x7[][5] PROGMEM = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},{0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},{0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},{0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},{0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},{0x7F,10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},{0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x0A,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},{0x10,0x08,0x08,0x10,0x08},{0x7F,0x7F,0x7F,0x7F,0x7F}
};

void oled_command(uint8_t cmd) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x00); 
    Wire.write(cmd);
    Wire.endTransmission();
}

void oled_init() {
    oled_command(0xAE); 
    oled_command(0x20); 
    oled_command(0x02); 
    oled_command(0xA1); 
    oled_command(0xC8); 
    oled_command(0x8D); 
    oled_command(0x14); 
    oled_command(0xAF); 
}

void oled_clear() {
    memset(frame_buffer, 0, sizeof(frame_buffer)); 
}

void oled_draw_char(uint8_t x, uint8_t y_page, char c) {
    if (c < 32 || c > 127) return; 
    uint16_t idx = (y_page * 128) + x;
    for (uint8_t i = 0; i < 5; i++) {
        if (idx + i < 1024) {
            frame_buffer[idx + i] = pgm_read_byte(&(font5x7[c - 32][i])); 
        }
    }
}

void oled_draw_string(uint8_t x, uint8_t y_page, const char* str) {
    while (*str) {
        oled_draw_char(x, y_page, *str++); 
        x += 6; 
    }
}

void oled_show() {
    for (uint8_t page = 0; page < 8; page++) {
        oled_command(0xB0 + page); 
        oled_command(0x00);        
        oled_command(0x10);        
        
        for (uint8_t chunk = 0; chunk < 8; chunk++) {
            Wire.beginTransmission(OLED_ADDR);
            Wire.write(0x40); 
            for (uint8_t i = 0; i < 16; i++) {
                Wire.write(frame_buffer[page * 128 + chunk * 16 + i]); 
            }
            Wire.endTransmission();
        }
    }
}

void format_top_bar_freq(uint64_t freq_hz10, char* buffer) {
    uint64_t hz = freq_hz10 / 10ULL;
    if (hz >= 1000000ULL) {
        uint32_t mhz_w = hz / 1000000ULL;
        uint32_t mhz_f = (hz % 1000000ULL) / 100000ULL; 
        sprintf(buffer, "%u.%uM", (unsigned int)mhz_w, (unsigned int)mhz_f);
    } else if (hz >= 1000ULL) {
        uint32_t khz_w = hz / 1000ULL;
        uint32_t khz_f = (hz % 1000ULL) / 100ULL;
        sprintf(buffer, "%u.%uk", (unsigned int)khz_w, (unsigned int)khz_f);
    } else {
        uint32_t hz_w = hz;
        uint32_t hz_f = freq_hz10 % 10ULL;
        sprintf(buffer, "%u.%uH", (unsigned int)hz_w, (unsigned int)hz_f);
    }
}

void update_si5351(uint8_t ch, uint64_t freq_to_set) {
    uint64_t freq_cHz = freq_to_set * 10ULL; 
    si5351.set_freq(freq_cHz, channels[ch].clk_port); 
}

bool read_button(Button *btn) {
    bool current_state = digitalRead(btn->pin);
    if (current_state != btn->last_state) {
        btn->last_debounce_time = millis(); 
    }
    if ((millis() - btn->last_debounce_time) > 30) { 
        if (current_state != btn->pressed) {
            btn->pressed = current_state;
            if (btn->pressed == LOW) { 
                btn->last_state = current_state;
                return true; 
            }
        }
    }
    btn->last_state = current_state;
    return false;
}

void process_inputs() {
    int sig1 = digitalRead(PIN_ENC_A);
    int sig2 = digitalRead(PIN_ENC_B);
    int current_state = (sig1 << 1) | sig2;

    if (current_state != old_state) {
        int state_combination = (old_state << 2) | current_state;
        
        if (state_combination == 0b1101) { 
            uint64_t total_change = step_sizes[channels[active_ch].step_index];
            if (tuning_freq + total_change <= 1600000000ULL) { // ceiling set strictly to 160 mhz
                tuning_freq += total_change;
            } else {
                tuning_freq = 1600000000ULL; 
            }
            update_si5351(active_ch, tuning_freq); 
            ui_needs_update = true;
        }
        else if (state_combination == 0b1110) { 
            uint64_t total_change = step_sizes[channels[active_ch].step_index];
            if (tuning_freq >= total_change && (tuning_freq - total_change) >= 80000ULL) { // floor set strictly to 8 khz
                tuning_freq -= total_change;
            } else {
                tuning_freq = 80000ULL; 
            }
            update_si5351(active_ch, tuning_freq); 
            ui_needs_update = true;
        }
        old_state = current_state;
    }

    if (read_button(&btn_step)) {
        channels[active_ch].step_index = (channels[active_ch].step_index + 1) % 10; 
        ui_needs_update = true;
    }
    
    if (read_button(&btn_page)) {
        update_si5351(active_ch, channels[active_ch].freq_hz10); 
        active_ch = (active_ch + 1) % 3;
        tuning_freq = channels[active_ch].freq_hz10; 
        ui_needs_update = true;
    }
    
    if (read_button(&btn_save)) {
        channels[active_ch].freq_hz10 = tuning_freq; 
        
        oled_clear();
        
        char f1[12], f2[12], f3[12], line_buf[32];
        format_top_bar_freq((active_ch == 0) ? tuning_freq : channels[0].freq_hz10, f1);
        format_top_bar_freq((active_ch == 1) ? tuning_freq : channels[1].freq_hz10, f2);
        format_top_bar_freq((active_ch == 2) ? tuning_freq : channels[2].freq_hz10, f3);
        
        sprintf(line_buf, "CH0:%s CH1:%s", f1, f2);
        oled_draw_string(0, 0, line_buf);
        sprintf(line_buf, "CH2:%s", f3);
        oled_draw_string(0, 1, line_buf);
        
        memset(&frame_buffer[128 * 1], 0xFF, 128); 
        oled_draw_string(32, 4, "RAM SAVED");
        oled_show();
        
        delay(350); 
        ui_needs_update = true;
    }
}

void render_screen() {
    oled_clear();

    char f1[12], f2[12], f3[12]; 
    char line_buf[32];
    
    format_top_bar_freq((active_ch == 0) ? tuning_freq : channels[0].freq_hz10, f1);
    format_top_bar_freq((active_ch == 1) ? tuning_freq : channels[1].freq_hz10, f2);
    format_top_bar_freq((active_ch == 2) ? tuning_freq : channels[2].freq_hz10, f3);
    
    sprintf(line_buf, "CH0:%s CH1:%s", f1, f2);
    oled_draw_string(0, 0, line_buf);
    
    sprintf(line_buf, "CH2:%s", f3);
    oled_draw_string(0, 1, line_buf);
    
    for (uint8_t col = 0; col < 128; col++) {
        frame_buffer[128 * 1 + col] |= 0x80; 
    }

    char ui_buf[32];
    sprintf(ui_buf, "TUNING CH: %u", (unsigned int)active_ch);
    oled_draw_string(0, 2, ui_buf);

    sprintf(ui_buf, "STEP: %s", step_labels[channels[active_ch].step_index]);
    oled_draw_string(0, 4, ui_buf);

    if (tuning_freq >= 10000000ULL) { 
        uint64_t hz = tuning_freq / 10ULL;
        uint32_t mhz_w = hz / 1000000ULL;
        uint32_t mhz_f = hz % 1000000ULL;
        sprintf(ui_buf, "WORK: %u.%06u MHz", (unsigned int)mhz_w, (unsigned int)mhz_f);
    } else if (tuning_freq >= 10000ULL) { 
        uint32_t khz_w = tuning_freq / 10000ULL;
        uint32_t khz_f = tuning_freq % 10000ULL;
        sprintf(ui_buf, "WORK: %u.%04u kHz", (unsigned int)khz_w, (unsigned int)khz_f);
    } else { 
        uint32_t hz_w = tuning_freq / 10ULL;
        uint32_t hz_f = tuning_freq % 10ULL;
        sprintf(ui_buf, "WORK: %u.%u Hz", (unsigned int)hz_w, (unsigned int)hz_f);
    }
    oled_draw_string(0, 6, ui_buf);

    oled_show(); 
}

void setup() {
    // Set system clock to 240 MHz
    set_sys_clock_khz(240000, true);

    // Configure hardware I2C pins for RP2350A (GP16 = SDA, GP17 = SCL)
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin(); 
    Wire.setClock(400000); 
    
    pinMode(PIN_BTN_STEP, INPUT_PULLUP);
    pinMode(PIN_BTN_PAGE, INPUT_PULLUP);
    pinMode(PIN_BTN_SAVE, INPUT_PULLUP);
    
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    
    int sig1 = digitalRead(PIN_ENC_A);
    int sig2 = digitalRead(PIN_ENC_B);
    old_state = (sig1 << 1) | sig2;

    oled_init();

    if(!si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0)) {
        oled_clear();
        oled_draw_string(10, 3, "Si5351 NOT FOUND!");
        oled_show();
        for(;;); 
    }

    tuning_freq = channels[active_ch].freq_hz10;
    
    update_si5351(0, channels[0].freq_hz10);
    update_si5351(1, channels[1].freq_hz10);
    update_si5351(2, channels[2].freq_hz10);
    
    si5351.output_enable(SI5351_CLK0, 1);
    si5351.output_enable(SI5351_CLK1, 1);
    si5351.output_enable(SI5351_CLK2, 1);

    render_screen();
    last_ui_time = millis(); 
}

void loop() {
    process_inputs(); 

    if (ui_needs_update && (millis() - last_ui_time >= UI_FRAME_INTERVAL)) {
        render_screen();
        last_ui_time = millis(); 
        ui_needs_update = false; 
    }

    delay(1); 
}
