#include <Arduino.h>

#include "HTTPClient.h"
#include "SD_MMC.h"
// #include "WiFi.h"
#include "Wire.h"
#include "display.h"

bool touch_pin_get_int = false;
void deep_sleep(void);

void print_chip_info(void) {
  Serial.print("Chip: ");
  Serial.println(ESP.getChipModel());
  Serial.print("ChipRevision: ");
  Serial.println(ESP.getChipRevision());
  Serial.print("Psram size: ");
  Serial.print(ESP.getPsramSize() / 1024);
  Serial.println("KB");
  Serial.print("Flash size: ");
  Serial.print(ESP.getFlashChipSize() / 1024);
  Serial.println("KB");
  Serial.print("CPU frequency: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println("MHz");
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
  lv_disp_flush_ready(drv);
}

static void lv_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  touch_point_t p = {0};
  if (touch_pin_get_int) {
#if defined(TOUCH_MODULE_FT3267)
    uint8_t touch_points_num;
    ft3267_read_pos(&touch_points_num, &p.x, &p.y);
    data->point.x = p.x;
    data->point.y = p.y;
#elif defined(TOUCH_MODULE_CST820)
    touch.read();
    TP_Point t = touch.getPoint(0);
    data->point.x = t.x;
    data->point.y = t.y;
#endif
    data->state = LV_INDEV_STATE_PR;
    touch_pin_get_int = false;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
  lv_msg_send(MSG_TOUCH_UPDATE, &p);
}

void deep_sleep(void) {
  // WiFi.disconnect();
  detachInterrupt(TP_INT_PIN);
  xl.pinMode8(0, 0xff, INPUT);
  xl.pinMode8(1, 0xff, INPUT);
  xl.read_all_reg();
  // If the SD card is initialized, it needs to be unmounted.
  if (SD_MMC.cardSize()) SD_MMC.end();

  digitalWrite(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL);

  Serial.println("Enter deep sleep");
  delay(100);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)TP_INT_PIN, 0);
  esp_deep_sleep_start();
}

void SD_init(void) {
  xl.digitalWrite(SD_CS_PIN, 1);  // To use SDIO one-line mode, you need to pull the CS pin high
  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");

  if (cardType == CARD_MMC)
    Serial.println("MMC");
  else if (cardType == CARD_SD)
    Serial.println("SDSC");
  else if (cardType == CARD_SDHC)
    Serial.println("SDHC");
  else
    Serial.println("UNKNOWN");

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void setup() {
  static lv_disp_draw_buf_t disp_buf;  // contains internal graphic buffer(s) called draw buffer(s)
  static lv_disp_drv_t disp_drv;       // contains callback functions
  static lv_indev_drv_t indev_drv;

  pinMode(BAT_VOLT_PIN, ANALOG);

  Wire.begin(IIC_SDA_PIN, IIC_SCL_PIN, (uint32_t)400000);
  Serial.begin(115200);
  xl.begin();
  uint8_t pin = (1 << PWR_EN_PIN) | (1 << LCD_CS_PIN) | (1 << TP_RES_PIN) | (1 << LCD_SDA_PIN) | (1 << LCD_CLK_PIN) |
                (1 << LCD_RST_PIN) | (1 << SD_CS_PIN);

  xl.pinMode8(0, pin, OUTPUT);
  xl.digitalWrite(PWR_EN_PIN, 1);
  print_chip_info();
  pinMode(EXAMPLE_PIN_NUM_BK_LIGHT, OUTPUT);
  digitalWrite(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
  SD_init();

  xl.digitalWrite(TP_RES_PIN, 0);
  delay(200);
  xl.digitalWrite(TP_RES_PIN, 1);
#if defined(TOUCH_MODULE_FT3267)
  ft3267_init(Wire);
#elif defined(TOUCH_MODULE_CST820)
  touch.init();
#endif
  tft_init();
  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_rgb_panel_config_t panel_config = {
      .clk_src = LCD_CLK_SRC_PLL160M,
      .timings =
          {
              .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
              .h_res = EXAMPLE_LCD_H_RES,
              .v_res = EXAMPLE_LCD_V_RES,
              // The following parameters should refer to LCD spec
              .hsync_pulse_width = 1,
              .hsync_back_porch = 30,
              .hsync_front_porch = 50,
              .vsync_pulse_width = 1,
              .vsync_back_porch = 30,
              .vsync_front_porch = 20,
              .flags =
                  {
                      .pclk_active_neg = 1,
                  },
          },
      .data_width = 16,  // RGB565 in parallel mode, thus 16bit in width
      .psram_trans_align = 64,
      .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
      .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
      .de_gpio_num = EXAMPLE_PIN_NUM_DE,
      .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
      .data_gpio_nums =
          {
              // EXAMPLE_PIN_NUM_DATA0,
              EXAMPLE_PIN_NUM_DATA13,
              EXAMPLE_PIN_NUM_DATA14,
              EXAMPLE_PIN_NUM_DATA15,
              EXAMPLE_PIN_NUM_DATA16,
              EXAMPLE_PIN_NUM_DATA17,

              EXAMPLE_PIN_NUM_DATA6,
              EXAMPLE_PIN_NUM_DATA7,
              EXAMPLE_PIN_NUM_DATA8,
              EXAMPLE_PIN_NUM_DATA9,
              EXAMPLE_PIN_NUM_DATA10,
              EXAMPLE_PIN_NUM_DATA11,
              // EXAMPLE_PIN_NUM_DATA12,

              EXAMPLE_PIN_NUM_DATA1,
              EXAMPLE_PIN_NUM_DATA2,
              EXAMPLE_PIN_NUM_DATA3,
              EXAMPLE_PIN_NUM_DATA4,
              EXAMPLE_PIN_NUM_DATA5,
          },
      .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
      .on_frame_trans_done = NULL,
      .user_ctx = NULL,
      .flags =
          {
              .fb_in_psram = 1,  // allocate frame buffer in PSRAM
          },
  };
  ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

  esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 480, 480, logo_img);
  delay(1000);

  lv_init();
  // alloc draw buffers used by LVGL from PSRAM
  lv_color_t *buf1 =
      (lv_color_t *)heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf1);
  lv_color_t *buf2 =
      (lv_color_t *)heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf2);
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);

  Serial.println("Register display driver to LVGL");
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = EXAMPLE_LCD_H_RES;
  disp_drv.ver_res = EXAMPLE_LCD_V_RES;
  disp_drv.flush_cb = example_lvgl_flush_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;
  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lv_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  pinMode(TP_INT_PIN, INPUT_PULLUP);
  attachInterrupt(TP_INT_PIN, [] { touch_pin_get_int = true; }, FALLING);

  Serial.println("Display LVGL Scatter Chart");

  ui_begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t Millis;
  delay(2);
  lv_timer_handler();
  if (millis() - Millis > 50) {
    float v = (analogRead(BAT_VOLT_PIN) * 2 * 3.3) / 4096;
    lv_msg_send(MSG_BAT_VOLT_UPDATE, &v);
    Millis = millis();
  }
}



