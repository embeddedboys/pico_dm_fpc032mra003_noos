// Copyright (c) 2024 embeddedboys developers

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:

// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "pico/time.h"
#define pr_fmt(fmt) "r61581: " fmt

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>

#include "pico/stdio.h"
#include "pico/stdio_uart.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "r61581.h"

/*
 * r61581 Command Table
 */

#define DRV_NAME "r61581"

#define pr_debug printf

struct r61581_priv;

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

struct r61581_operations {
    int (*init_display)(struct r61581_priv *priv);
    int (*reset)(struct r61581_priv *priv);
    int (*clear)(struct r61581_priv *priv, u16 clear);
    int (*blank)(struct r61581_priv *priv, bool on);
    int (*sleep)(struct r61581_priv *priv, bool on);
    int (*set_var)(struct r61581_priv *priv);
    int (*set_addr_win)(struct r61581_priv *priv, int xs, int ys, int xe, int ye);
    int (*set_cursor)(struct r61581_priv *priv, int x, int y);
};

struct r61581_display {
    u32                     xres;
    u32                     yres;
    u32                     bpp;
    u32                     rotate;
};

struct r61581_priv {
    u8                      *buf;

    struct {
        int reset;
        int cs;   /* chip select */
        int rs;   /* register/data select */
        int wr;   /* write signal */
        int rd;   /* read signal */
        int bl;   /* backlight */
        int db[LCD_PIN_DB_COUNT];
    } gpio;

    /* device specific */
    const struct r61581_operations  *tftops;
    struct r61581_display           *display;
} g_priv;

#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof(arr[0]))
#define dm_gpio_set_value(p,v) gpio_put(p, v)
#define mdelay(v) sleep_ms(v)

extern int i80_pio_init(uint8_t db_base, uint8_t db_count, uint8_t pin_wr);
extern int i80_write_buf_rs(void *buf, size_t len, bool rs);

static void fbtft_write_gpio8_wr(struct r61581_priv *priv, void *buf, size_t len)
{
    u8 data;
    int i;
#ifndef DO_NOT_OPTIMIZE_FBTFT_WRITE_GPIO
    static u8 prev_data;
#endif

    /* Start writing by pulling down /WR */
    dm_gpio_set_value(priv->gpio.wr, 1);

    while (len) {
        data = *(u8 *)buf;

        /* Start writing by pulling down /WR */
        dm_gpio_set_value(priv->gpio.wr, 0);

        // printf("data : 0x%x\n", data);

        /* Set data */
#ifndef DO_NOT_OPTIMIZE_FBTFT_WRITE_GPIO
        if (data == prev_data) {
            dm_gpio_set_value(priv->gpio.wr, 1); /* used as delay */
        } else {
            for (i = 0; i < 8; i++) {
                if ((data & 1) != (prev_data & 1))
                    dm_gpio_set_value(priv->gpio.db[i],
                                      data & 1);
                data >>= 1;
                prev_data >>= 1;
            }
        }
#else
        for (i = 0; i < 8; i++) {
            dm_gpio_set_value(&priv->gpio.db[i], data & 1);
            data >>= 1;
        }
#endif

        /* Pullup /WR */
        dm_gpio_set_value(priv->gpio.wr, 1);

#ifndef DO_NOT_OPTIMIZE_FBTFT_WRITE_GPIO
        prev_data = *(u8 *)buf;
#endif
        buf ++;
        len --;
    }
}

static void fbtft_write_gpio8_wr_rs(struct r61581_priv *priv, void *buf, size_t len, bool rs)
{
    dm_gpio_set_value(priv->gpio.rs, rs);
    fbtft_write_gpio8_wr(priv, buf, len);
}

/* rs=0 means writing register, rs=1 means writing data */
#if DISP_OVER_PIO
    #define write_buf_rs(p, b, l, r) i80_write_buf_rs(b, l, r)
#else
    #define write_buf_rs(p, b, l, r) fbtft_write_gpio16_wr_rs(p, b, l, r)
#endif

static int r61581_write_reg(struct r61581_priv *priv, int len, ...)
{
    u8 *buf = (u8 *)priv->buf;
    va_list args;
    int i;

    va_start(args, len);
    *buf = (u8)va_arg(args, unsigned int);
    write_buf_rs(priv, buf, sizeof(u8), 0);
    len--;

    /* if there no privams */
    if (len == 0)
        return 0;

    for (i = 0; i < len; i++) {
        *buf = (u8)va_arg(args, unsigned int);
        buf++;
    }

    len++;
    write_buf_rs(priv, priv->buf, len, 1);
    va_end(args);

    return 0;
}
#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
#define write_reg(priv, ...) \
    r61581_write_reg(priv, NUMARGS(__VA_ARGS__), __VA_ARGS__)

static int r61581_reset(struct r61581_priv *priv)
{
    dm_gpio_set_value(priv->gpio.reset, 1);
    mdelay(10);
    dm_gpio_set_value(priv->gpio.reset, 0);
    mdelay(10);
    dm_gpio_set_value(priv->gpio.reset, 1);
    mdelay(10);
    return 0;
}


static int r61581_set_var(struct r61581_priv *priv)
{
    pr_debug("%s\n", __func__);
    return 0;
}

static int r61581_init_display(struct r61581_priv *priv)
{
    pr_debug("%s, writing initial sequence...\n", __func__);
    r61581_reset(priv);
    dm_gpio_set_value(priv->gpio.rd, 1);
    mdelay(150);

    write_reg(priv, 0xB0, 0x00);
    write_reg(priv, 0xB3, 0x02, 0x00, 0x00, 0x00);

    /* Backlight control */

    write_reg(priv, 0xC0, 0x13, 0x3B, 0x00, 0x02, 0x00, 0x01, 0x00, 0x43);
    write_reg(priv, 0xC1, 0x08, 0x16, 0x08, 0x08);
    write_reg(priv, 0xC4, 0x11, 0x07, 0x03, 0x03);
    write_reg(priv, 0xC6, 0x00);
    write_reg(priv, 0xC8, 0x03, 0x03, 0x13, 0x5C, 0x03, 0x07, 0x14, 0x08, 0x00, 0x21, 0x08, 0x14, 0x07, 0x53, 0x0C, 0x13, 0x03, 0x03, 0x21, 0x00);
    write_reg(priv, 0x0C, 0x55);
    write_reg(priv, 0x36, (1 << 6) | (1 << 5));
    write_reg(priv, 0x38);
    write_reg(priv, 0x3A, 0x55);
    write_reg(priv, 0xD0, 0x07, 0x07, 0x1D, 0x03);
    write_reg(priv, 0xD1, 0x03, 0x30, 0x10);
    write_reg(priv, 0xD2, 0x03, 0x14, 0x04);

    write_reg(priv, 0x11);
    mdelay(10);
    write_reg(priv, 0x29);

    return 0;
}

static int r61581_set_addr_win(struct r61581_priv *priv, int xs, int ys, int xe,
                                int ye)
{
    /* set column adddress */
    write_reg(priv, 0x2A, xs >> 8, xs, xe >> 8, xe);

    /* set row address */
    write_reg(priv, 0x2B, ys >> 8, ys, ye >> 8, ye);

    /* write start */
    write_reg(priv, 0x2C);
    return 0;
}

static int r61581_clear(struct r61581_priv *priv, u16 clear)
{
    u32 width = priv->display->xres;
    u32 height = priv->display->yres;
    int x, y;

    pr_debug("clearing screen (%d x %d) with color 0x%x\n", width, height, clear);

    priv->tftops->set_addr_win(priv, 0, 0,
                         priv->display->xres - 1,
                         priv->display->yres - 1);

    for (x = 0; x < width; x++) {
        for (y = 0; y < height; y++) {
            write_buf_rs(priv, &clear, sizeof(u16), 1);
        }
    }

    return 0;
}

static int r61581_blank(struct r61581_priv *priv, bool on)
{
    pr_debug("%s\n", __func__);
    return 0;
}

static int r61581_sleep(struct r61581_priv *priv, bool on)
{
    pr_debug("%s\n", __func__);
    return 0;
}

static const struct r61581_operations default_r61581_ops = {
    .init_display    = r61581_init_display,
    .reset           = r61581_reset,
    .clear           = r61581_clear,
    .blank           = r61581_blank,
    .sleep           = r61581_sleep,
    .set_var         = r61581_set_var,
    .set_addr_win    = r61581_set_addr_win,
};

static int r61581_gpio_init(struct r61581_priv *priv)
{
    printf("initializing gpios...\n");

#if DISP_OVER_PIO
    gpio_init(priv->gpio.reset);
    // gpio_init(priv->gpio.bl);
    // gpio_init(priv->gpio.cs);
    gpio_init(priv->gpio.rs);
    // gpio_init(priv->gpio.rd);

    gpio_set_dir(priv->gpio.reset, GPIO_OUT);
    // gpio_set_dir(priv->gpio.bl, GPIO_OUT);
    // gpio_set_dir(priv->gpio.cs, GPIO_OUT);
    gpio_set_dir(priv->gpio.rs, GPIO_OUT);
    // gpio_set_dir(priv->gpio.rd, GPIO_OUT);
#else
    int *pp = (int *)&priv->gpio;

    int len = sizeof(priv->gpio)/sizeof(priv->gpio.reset);

    while(len--) {
        gpio_init(*pp);
        gpio_set_dir(*pp, GPIO_OUT);
        pp++;
    }
#endif
    return 0;
}

static int r61581_hw_init(struct r61581_priv *priv)
{
    printf("initializing hardware...\n");

#if DISP_OVER_PIO
    i80_pio_init(priv->gpio.db[0], ARRAY_SIZE(priv->gpio.db), priv->gpio.wr);
#endif
    r61581_gpio_init(priv);

    priv->tftops->init_display(priv);
    /* clear screen to black */
    // priv->tftops->clear(priv, 0x0);

    return 0;
}

static struct r61581_display default_r61581_display = {
    .xres   = LCD_HOR_RES,
    .yres   = LCD_VER_RES,
    .bpp    = 16,
    .rotate = 0,
};

/* ########### standalone ######## */
static inline void r61581_write_cmd(uint8_t cmd)
{
    write_buf_rs(&g_priv, &cmd, sizeof(cmd), 0);
}
#define write_cmd r61581_write_cmd
static inline void r61581_write_data(uint8_t data)
{
    write_buf_rs(&g_priv, &data, sizeof(data), 1);
}
#define write_data r61581_write_data

#include "lvgl/lvgl.h"
void r61581_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
#if 1
    write_cmd(0x2A);
    write_data(area->x1 >> 8);
    write_data(area->x1);
    write_data(area->x2 >> 8);
    write_data(area->x2);

    /* set row address */
    write_cmd(0x2B);
    write_data(area->y1 >> 8);
    write_data(area->y1);
    write_data(area->y2 >> 8);
    write_data(area->y2);

    /* write start */
    write_cmd(0x2C);

    /* swap rgb565 buffer order */
    lv_draw_sw_rgb565_swap(px_map, lv_area_get_size(area));

    write_buf_rs(&g_priv, (void *)px_map, lv_area_get_size(area) * 2, 1);
#else
    struct ili9488_priv *priv = &g_priv;
    priv->tftops->set_addr_win(priv, area->x1, area->y1, area->x2, area->y2);
    write_buf_rs(priv, (void *)px_map, lv_area_get_size(area) * 2, 1);
#endif
    lv_disp_flush_ready(disp);
}
/* ########### standlone ######## */

#define BUF_SIZE 64
static int r61581_probe(struct r61581_priv *priv)
{
    pr_debug("r61581 probing ...\n");

    priv->buf = (u8 *)malloc(BUF_SIZE);

    priv->display = &default_r61581_display;
    priv->tftops = &default_r61581_ops;

    priv->gpio.bl    = LCD_PIN_BL;
    priv->gpio.reset = LCD_PIN_RST;
    // priv->gpio.rd    = 21;
    priv->gpio.rs    = LCD_PIN_RS;
    priv->gpio.wr    = LCD_PIN_WR;
    priv->gpio.cs    = LCD_PIN_CS;

    /* pin0 - pin15 for I8080 data bus */
    for (int i = LCD_PIN_DB_BASE; i < ARRAY_SIZE(priv->gpio.db); i++)
        priv->gpio.db[i] = i;

    r61581_hw_init(priv);

    return 0;
}

int r61581_driver_init(void)
{
    r61581_probe(&g_priv);
    return 0;
}