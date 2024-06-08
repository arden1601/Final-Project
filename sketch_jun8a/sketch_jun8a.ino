#include <TFT_eSPI.h>
#include <Arduino.h>
#include <lvgl.h>
#include "./ui.h"

// Screen configuration
#define CrowPanel_24
#define LDR_PIN 34
//#define CrowPanel_28
//#define CrowPanel_35

#if defined(CrowPanel_35)
static const uint16_t screenWidth = 480;
static const uint16_t screenHeight = 320;
#elif defined(CrowPanel_24)
static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 240;
#elif defined(CrowPanel_28)
static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 240;
#endif

bool state = false;  // Initial screen state

TFT_eSPI lcd = TFT_eSPI(); /* TFT entity */
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[screenWidth * screenHeight / 13];

// Display flush function
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    lcd.startWrite();
    lcd.setAddrWindow(area->x1, area->y1, w, h);
    lcd.pushColors((uint16_t *)&color_p->full, w * h, true);
    lcd.endWrite();

    lv_disp_flush_ready(disp);
}

// Initialization function for UI and display
void lcd_player_setup() {
    lcd.fillScreen(TFT_BLACK);
    ui_init();
}

// Task for handling LVGL tasks
void lvglTask(void *pvParameters) {
    while (1) {
        lv_timer_handler();  // Handle LVGL tasks
        vTaskDelay(pdMS_TO_TICKS(5));  // Delay for 5 milliseconds
    }
}

// Task for reading sensor values and switching screens
void sensorTask(void *pvParameters) {
    while (1) {
        int sensVal = analogRead(LDR_PIN); // Read sensor value
        Serial.println(sensVal); // Print sensor value for debugging

        if (sensVal < 1000) { // Threshold check
            if (!state) {
                Serial.println("Switching to Screen2");
                lv_tick_inc(1000); // Increment LVGL tick counter
                lv_scr_load(ui_Screen2);
                state = !state;
            }
        } else {
            if (state) {
                Serial.println("Switching to Screen1");
                lv_tick_inc(1000); // Increment LVGL tick counter
                lv_scr_load(ui_Screen1);
                state = !state;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 milliseconds
    }
}

// Setup function
void setup() {
    Serial.begin(9600); /*serial init */
    pinMode(LDR_PIN, INPUT);
    pinMode(25, OUTPUT);
    digitalWrite(25, LOW);

    lcd.begin();
    lcd.setRotation(1);
    lcd.fillScreen(TFT_BLACK);
    delay(100);

    pinMode(27, OUTPUT);
    digitalWrite(27, HIGH);

    lv_init();

    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, screenWidth * screenHeight / 13);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // Initialize the UI
    lcd_player_setup();

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(lvglTask, "LVGL Task", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 4096, NULL, 1, NULL, 1);

    Serial.println("Setup done");
}

// Empty loop function as tasks are handled by FreeRTOS
void loop() {
    // Do nothing here, as tasks are running in FreeRTOS
}
