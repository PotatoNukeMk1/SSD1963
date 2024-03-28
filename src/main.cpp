#include <Arduino.h>
#include <esp_err.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_ssd1963.h>
#include <lvgl.h>
#include <demos/lv_demos.h>

#define LCD_DATA0 18
#define LCD_DATA1 17
#define LCD_DATA2 16
#define LCD_DATA3 15
#define LCD_DATA4 7
#define LCD_DATA5 6
#define LCD_DATA6 5
#define LCD_DATA7 4
#define LCD_PCLK  11
#define LCD_CS    14
#define LCD_DC    13
#define LCD_RST   10
#define LCD_BL    47
#define LCD_H_RES 800
#define LCD_V_RES 480
#define DISP_BUF_SIZE (LCD_H_RES * (LCD_V_RES / 10) * (LV_COLOR_DEPTH / 8))

const char *TAG = "SSD1963 Demo";

bool disp_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
  lv_display_flush_ready(static_cast<lv_display_t *>(user_ctx));
  return false;
}

void disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
  // lv_draw_sw_rgb565_swap(px_map, (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1));
  esp_lcd_panel_handle_t panel_handle = static_cast<esp_lcd_panel_handle_t>(lv_display_get_user_data(disp));
  esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
}

void set_angle(void * obj, int32_t v)
{
  lv_arc_set_value((lv_obj_t *)obj, v);
}

void setup() {
  Serial.begin(115200);

  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);

  lv_init();
  lv_display_t *disp;
  disp = lv_display_create(LCD_H_RES, LCD_V_RES);

  ESP_LOGI(TAG, "Initialize Intel 8080 bus");
  esp_lcd_i80_bus_handle_t i80_bus = NULL;
  esp_lcd_i80_bus_config_t bus_config = {
    .dc_gpio_num = LCD_DC,
    .wr_gpio_num = LCD_PCLK,
    .clk_src = LCD_CLK_SRC_PLL160M,
    .data_gpio_nums = {
      LCD_DATA0,
      LCD_DATA1,
      LCD_DATA2,
      LCD_DATA3,
      LCD_DATA4,
      LCD_DATA5,
      LCD_DATA6,
      LCD_DATA7,
    },
    .bus_width = 8,
    .max_transfer_bytes = DISP_BUF_SIZE,
    .psram_trans_align = 64,
    .sram_trans_align = 4
  };
  ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

  ESP_LOGI(TAG, "Install panel IO");
  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_i80_config_t io_config = {
    .cs_gpio_num = LCD_CS,
    .pclk_hz = 10000000,
    .trans_queue_depth = 10,
    .on_color_trans_done = disp_flush_ready,
    .user_ctx = disp,
    .lcd_cmd_bits = 16,
    .lcd_param_bits = 8,
    .dc_levels = {
      .dc_idle_level = 0,
      .dc_cmd_level = 0,
      .dc_dummy_level = 0,
      .dc_data_level = 1
    },
    .flags = {
      .swap_color_bytes = 0,
    }
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1963 panel driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
#else
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
#endif
        .bits_per_pixel = 24,
    };
    esp_lcd_panel_handle_t panel_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1963(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    ESP_ERROR_CHECK(esp_lcd_panel_disp_off(panel_handle, false));
#else
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
#endif

  // void *buf_1 = heap_caps_malloc(DISP_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  // void *buf_2 = heap_caps_malloc(DISP_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  void *buf_1 = ps_malloc(DISP_BUF_SIZE);
  void *buf_2 = ps_malloc(DISP_BUF_SIZE);

  lv_display_set_flush_cb(disp, disp_flush);
  lv_display_set_user_data(disp, panel_handle);
  lv_display_set_buffers(disp, buf_1, buf_2, DISP_BUF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);

  // lv_obj_t * arc = lv_arc_create(lv_screen_active());
  // lv_arc_set_rotation(arc, 270);
  // lv_arc_set_bg_angles(arc, 0, 360);
  // lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
  // lv_obj_remove_flag(arc, LV_OBJ_FLAG_CLICKABLE);
  // lv_obj_center(arc);

  // lv_anim_t a;
  // lv_anim_init(&a);
  // lv_anim_set_var(&a, arc);
  // lv_anim_set_exec_cb(&a, set_angle);
  // lv_anim_set_duration(&a, 1000);
  // lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
  // lv_anim_set_repeat_delay(&a, 500);
  // lv_anim_set_values(&a, 0, 100);
  // lv_anim_start(&a);

  lv_demo_music();

  ESP_LOGI(TAG, "Ready");
}

void loop() {
  lv_task_handler();
  lv_tick_inc(5);
  delay(5);
}