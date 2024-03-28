/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

#include "esp_lcd_ssd1963.h"

static const char *TAG = "ssd1963";

static esp_err_t panel_ssd1963_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1963_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1963_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ssd1963_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_ssd1963_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_ssd1963_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_ssd1963_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_ssd1963_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_ssd1963_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_val; // save current value of LCD_CMD_COLMOD register
    const ssd1963_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
} ssd1963_panel_t;

esp_err_t esp_lcd_new_panel_ssd1963(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    ssd1963_panel_t *ssd1963 = NULL;
    gpio_config_t io_conf = { 0 };

    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ssd1963 = (ssd1963_panel_t *)calloc(1, sizeof(ssd1963_panel_t));
    ESP_GOTO_ON_FALSE(ssd1963, ESP_ERR_NO_MEM, err, TAG, "no mem for ssd1963 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num;
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    switch (panel_dev_config->color_space) {
    case ESP_LCD_COLOR_SPACE_RGB:
        ssd1963->madctl_val = 0;
        break;
    case ESP_LCD_COLOR_SPACE_BGR:
        ssd1963->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }
#else
    switch (panel_dev_config->rgb_endian) {
    case LCD_RGB_ENDIAN_RGB:
        ssd1963->madctl_val = 0;
        break;
    case LCD_RGB_ENDIAN_BGR:
        ssd1963->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported rgb endian");
        break;
    }
#endif

    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        ssd1963->colmod_val = 0x03;
        ssd1963->fb_bits_per_pixel = 16;
        break;
    case 24: // RGB888
        ssd1963->colmod_val = 0x00;
        ssd1963->fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    ssd1963->io = io;
    ssd1963->reset_gpio_num = panel_dev_config->reset_gpio_num;
    ssd1963->reset_level = panel_dev_config->flags.reset_active_high;
    if (panel_dev_config->vendor_config) {
        ssd1963->init_cmds = ((ssd1963_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds;
        ssd1963->init_cmds_size = ((ssd1963_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds_size;
    }
    ssd1963->base.del = panel_ssd1963_del;
    ssd1963->base.reset = panel_ssd1963_reset;
    ssd1963->base.init = panel_ssd1963_init;
    ssd1963->base.draw_bitmap = panel_ssd1963_draw_bitmap;
    ssd1963->base.invert_color = panel_ssd1963_invert_color;
    ssd1963->base.set_gap = panel_ssd1963_set_gap;
    ssd1963->base.mirror = panel_ssd1963_mirror;
    ssd1963->base.swap_xy = panel_ssd1963_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    ssd1963->base.disp_off = panel_ssd1963_disp_on_off;
#else
    ssd1963->base.disp_on_off = panel_ssd1963_disp_on_off;
#endif
    *ret_panel = &(ssd1963->base);
    ESP_LOGD(TAG, "new ssd1963 panel @%p", ssd1963);

    ESP_LOGI(TAG, "LCD panel create success");

    return ESP_OK;

err:
    if (ssd1963) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(ssd1963);
    }
    return ret;
}

static esp_err_t panel_ssd1963_del(esp_lcd_panel_t *panel)
{
    ssd1963_panel_t *ssd1963 = __containerof(panel, ssd1963_panel_t, base);

    if (ssd1963->reset_gpio_num >= 0) {
        gpio_reset_pin(ssd1963->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del ssd1963 panel @%p", ssd1963);
    free(ssd1963);
    return ESP_OK;
}

static esp_err_t panel_ssd1963_reset(esp_lcd_panel_t *panel)
{
    ssd1963_panel_t *ssd1963 = __containerof(panel, ssd1963_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1963->io;

    // perform hardware reset
    if (ssd1963->reset_gpio_num >= 0) {
	    /* perform hardware reset */
        gpio_set_level(ssd1963->reset_gpio_num, !ssd1963->reset_level);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(ssd1963->reset_gpio_num, ssd1963->reset_level);
        vTaskDelay(pdMS_TO_TICKS(200));
	    gpio_set_level(ssd1963->reset_gpio_num, !ssd1963->reset_level);
	    vTaskDelay(pdMS_TO_TICKS(200));
    } else { // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0x01, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(120)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

static const ssd1963_lcd_init_cmd_t vendor_specific_init_default[] = {
//  {cmd, { data }, data_size, delay_ms}
    {0xE2, (uint8_t []){0x23, 0x02, 0x54}, 3, 0},
    {0xE0, (uint8_t []){0x01}, 1, 1},
    {0xE0, (uint8_t []){0x03}, 1, 1},
    {0x01, (uint8_t []){0x00}, 1, 1},
    {0xE6, (uint8_t []){0x03, 0x33, 0x33}, 3, 0},
    {0xB0, (uint8_t []){0x20, 0x00, ((800 - 1) >> 8), (800 - 1) & 0xff, ((480 - 1) >> 8), (480 - 1) & 0xff, 0x00}, 7, 0},
    {0xB4, (uint8_t []){0x04, 0x1f, 0x00, 0xd2, 0x00, 0x00, 0x00, 0x00}, 8, 0},
    {0xB6, (uint8_t []){0x02, 0x0c, 0x00, 0x22, 0x00, 0x00, 0x00}, 7, 0},
    {0xBA, (uint8_t []){0x01}, 1, 1},
    {0xB8, (uint8_t []){0x0f, 0x01}, 2, 0},
    {0x36, (uint8_t []){0x00}, 1, 0},
    {0x3A, (uint8_t []){0x50}, 1, 0},
    {0xF0, (uint8_t []){0x00}, 1, 0},
    {0xBC, (uint8_t []){0x40, 0x80, 0x40, 0x01}, 4, 1},
    {0xBE, (uint8_t []){0x06, 0x80, 0x01, 0xf0, 0x00, 0x00}, 6, 0},
    {0xD0, (uint8_t []){0x0d}, 1, 0},
    {0x2A, (uint8_t []){0 >> 8, 0 & 0xff, (800 - 1) >> 8, (800 - 1) & 0xff}, 4, 0},
    {0x2B, (uint8_t []){0 >> 8, 0 & 0xff, (480 - 1) >> 8, (480 - 1) & 0xff}, 4, 0}
};

static esp_err_t panel_ssd1963_init(esp_lcd_panel_t *panel)
{
    ssd1963_panel_t *ssd1963 = __containerof(panel, ssd1963_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1963->io;

    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), TAG, "send command failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        ssd1963->madctl_val,
    }, 1), TAG, "send command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, (uint8_t[]) {
        ssd1963->colmod_val,
    }, 1), TAG, "send command failed");

    const ssd1963_lcd_init_cmd_t *init_cmds = NULL;
    uint16_t init_cmds_size = 0;
    if (ssd1963->init_cmds) {
        init_cmds = ssd1963->init_cmds;
        init_cmds_size = ssd1963->init_cmds_size;
    } else {
        init_cmds = vendor_specific_init_default;
        init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(ssd1963_lcd_init_cmd_t);
    }

    bool is_cmd_overwritten = false;
    for (int i = 0; i < init_cmds_size; i++) {
        // Check if the command has been used or conflicts with the internal
        switch (init_cmds[i].cmd) {
        case LCD_CMD_MADCTL:
            is_cmd_overwritten = true;
            ssd1963->madctl_val = ((uint8_t *)init_cmds[i].data)[0];
            break;
        case 0xF0:
            is_cmd_overwritten = true;
            ssd1963->colmod_val = ((uint8_t *)init_cmds[i].data)[0];
            break;
        default:
            is_cmd_overwritten = false;
            break;
        }

        if (is_cmd_overwritten) {
            ESP_LOGW(TAG, "The %02Xh command has been used and will be overwritten by external initialization sequence", init_cmds[i].cmd);
        }

        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));
    }
    ESP_LOGD(TAG, "send init commands success");

    return ESP_OK;
}

static esp_err_t panel_ssd1963_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    ssd1963_panel_t *ssd1963 = __containerof(panel, ssd1963_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = ssd1963->io;

    x_start += ssd1963->x_gap;
    x_end += ssd1963->x_gap;
    y_start += ssd1963->y_gap;
    y_end += ssd1963->y_gap;

    // define an area of frame memory where MCU can access
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
        x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF,
        (x_end - 1) & 0xFF,
    }, 4), TAG, "send command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF,
        y_start & 0xFF,
        ((y_end - 1) >> 8) & 0xFF,
        (y_end - 1) & 0xFF,
    }, 4), TAG, "send command failed");
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * ssd1963->fb_bits_per_pixel / 8;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len), TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t panel_ssd1963_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    ssd1963_panel_t *ssd1963 = __containerof(panel, ssd1963_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1963->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_ssd1963_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ssd1963_panel_t *ssd1963 = __containerof(panel, ssd1963_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1963->io;
    if (mirror_x) {
        ssd1963->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        ssd1963->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        ssd1963->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        ssd1963->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        ssd1963->madctl_val
    }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_ssd1963_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ssd1963_panel_t *ssd1963 = __containerof(panel, ssd1963_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1963->io;
    if (swap_axes) {
        ssd1963->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        ssd1963->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        ssd1963->madctl_val
    }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_ssd1963_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    ssd1963_panel_t *ssd1963 = __containerof(panel, ssd1963_panel_t, base);
    ssd1963->x_gap = x_gap;
    ssd1963->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_ssd1963_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    ssd1963_panel_t *ssd1963 = __containerof(panel, ssd1963_panel_t, base);
    esp_lcd_panel_io_handle_t io = ssd1963->io;
    int command = 0;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    on_off = !on_off;
#endif

    if (on_off) {
        command = LCD_CMD_DISPON;
    } else {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}
