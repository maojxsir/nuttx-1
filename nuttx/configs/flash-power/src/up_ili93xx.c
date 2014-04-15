/************************************************************************************
 * configs/shenzhou/src/up_ili93xx.c
 * arch/arm/src/board/up_ili93xx.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32.h"
#include "stm32f407-internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration **********************************************************************/
/* CONFIG_STM32_ILI1505_DISABLE may be defined to disable the LCD_ILI1505
 * CONFIG_STM32_ILI9300_DISABLE may be defined to disable the LCD_ILI9300
 * CONFIG_STM32_ILI9320_DISABLE may be defined to disable the LCD_ILI9320
 * CONFIG_STM32_ILI9321_DISABLE may be defined to disable the LCD_ILI9321
 * CONFIG_STM32_ILI9325_DISABLE may be defined to disabled the LCD_ILI9325
 * CONFIG_STM32_ILI9328_DISABLE may be defined to disabled the LCD_ILI9328
 * CONFIG_STM32_ILI9331_DISABLE may be defined to disabled the LCD_ILI9331
 * CONFIG_STM32_ILI9919_DISABLE may be defined to disabled the LCD_ILI9919
 */

#undef HAVE_LCD
#if !defined(CONFIG_STM32_ILI1505_DISABLE)
#  define HAVE_LCD 1
#elif !defined(CONFIG_STM32_ILI9300_DISABLE)
#  define HAVE_LCD 1
#elif !defined(CONFIG_STM32_ILI9320_DISABLE)
#  define HAVE_LCD 1
#elif !defined(CONFIG_STM32_ILI9321_DISABLE)
#  define HAVE_LCD 1
#elif !defined(CONFIG_STM32_ILI9325_DISABLE)
#  define HAVE_LCD 1
#elif !defined(CONFIG_STM32_ILI9328_DISABLE)
#  define HAVE_LCD 1
#elif !defined(CONFIG_STM32_ILI9331_DISABLE)
#  define HAVE_LCD 1
#elif !defined(CONFIG_STM32_ILI9919_DISABLE)
#  define HAVE_LCD 1
#endif

#ifdef HAVE_LCD

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 1
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER) || CONFIG_LCD_MAXPOWER < 1
#  define CONFIG_LCD_MAXPOWER 1
#endif

#if CONFIG_LCD_MAXPOWER > 255
#  error "CONFIG_LCD_MAXPOWER must be less than 256 to fit in uint8_t"
#endif

/* Check orientation */

#if defined(CONFIG_LCD_PORTRAIT)
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE) || defined(CONFIG_LCD_RPORTRAIT)
#    error "Cannot define both portrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_RPORTRAIT)
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#    error "Cannot define both rportrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_LANDSCAPE)
#  ifdef CONFIG_LCD_RLANDSCAPE
#    error "Cannot define both landscape and any other orientations"
#  endif
#elif !defined(CONFIG_LCD_RLANDSCAPE)
#  define CONFIG_LCD_LANDSCAPE 1
#endif


/* Define CONFIG_DEBUG_LCD to enable detailed LCD debug output. Verbose debug must
 * also be enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#  undef CONFIG_DEBUG_LCD
#  undef CONFIG_LCD_REGDEBUG
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LCD
#endif

/* Display/Color Properties ***********************************************************/
/* Display Resolution */

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE) 
#  define STM32_XRES          320
#  define STM32_YRES          240
#else
#  define STM32_XRES          240
#  define STM32_YRES          320
#endif

/* Color depth and format */

#define STM32_BPP             16
#define STM32_COLORFMT        FB_FMT_RGB16_565

// LCD CS is GPIO
#define STM32_LCDBASE         ((uint16_t*)(0x60000000))
#define LCD                   ((struct lcd_regs_s *)STM32_LCDBASE)

#define DRV_CODE_READ         (0x00)
#define START_OSC             (0x00)
#define DRV_OUTPUT_CTL        (0x01)
#define LCD_DRIVING_CTL       (0x02)
#define ENTRY_MODE            (0x03)
#define RESIZE_CTL            (0x04)
#define DISP_CTL_1            (0x07)
#define DISP_CTL_2            (0x08)
#define DISP_CTL_3            (0x09)
#define DISP_CTL_4            (0x0A)
#define RGB_IF_CTL_1          (0x0C)
#define FRAME_MAKER_POS       (0x0D)
#define RGB_IF_CTL_2          (0x0F)
#define POWER_CTL_1           (0x10)
#define POWER_CTL_2           (0x11)
#define POWER_CTL_3           (0x12)
#define POWER_CTL_4           (0x13)
#define GRAM_ADDR_SET_H       (0x20)
#define GRAM_ADDR_SET_V       (0x21)
#define GRAM_WRITE_DATA       (0x22)
#define GRAM_READ_DATA        (0x22)
#define LCD_REG_40            (40)
#define POWER_CTL_7           (0x29)
#define FRAME_RATE_COLOR_CTL  (0x2B)
#define GAMMA_CTL_1           (0x30)
#define GAMMA_CTL_2           (0x31)
#define GAMMA_CTL_3           (0x32)
#define GAMMA_CTL_4           (0x35)
#define GAMMA_CTL_5           (0x36)
#define GAMMA_CTL_6           (0x37)
#define GAMMA_CTL_7           (0x38)
#define GAMMA_CTL_8           (0x39)
#define GAMMA_CTL_9           (0x3C)
#define GAMMA_CTL_10          (0x3D)
#define H_ADDR_START_POS      (0x50)
#define H_ADDR_END_POS        (0x51)
#define V_ADDR_START_POS      (0x52)
#define V_ADDR_END_POS        (0x53)
#define DRV_OUTPUT_CTL_2      (0x60)
#define BASE_IMG_DISP_CTL     (0x61)
#define V_SCROLL_CTL          (0x6A)
#define PORTIAL_IMG_1_POS     (0x80)
#define PORTIAL_IMG_1_START   (0x81)
#define PORTIAL_IMG_1_END     (0x82)
#define PORTIAL_IMG_2_POS     (0x83)
#define PORTIAL_IMG_2_START   (0x84)
#define PORTIAL_IMG_2_END     (0x85)
#define PANEL_IF_CTL_1        (0x90)
#define PANEL_IF_CTL_2        (0x92)
#define PANEL_IF_CTL_3        (0x93)
#define PANEL_IF_CTL_4        (0x95)
#define PANEL_IF_CTL_5        (0x97)
#define PANEL_IF_CTL_6        (0x98)

#define LCD_REG_11            (11)
#define LCD_REG_23            (23)
#define LCD_REG_14            (14)
#define LCD_REG_30            (30)
#define LCD_REG_37            (37)
#define LCD_REG_38            (38)
#define LCD_REG_39            (39)
#define LCD_REG_58            (58)
#define LCD_REG_59            (59)
#define LCD_REG_60            (60)
#define LCD_REG_61            (61)
#define LCD_REG_42            (42)
#define LCD_REG_227           (227)
#define LCD_REG_231           (231)
#define LCD_REG_239           (239)
#define LCD_REG_78            (0x4E)
#define LCD_REG_79            (0x4F)


/* LCD IDs */

#define ILI1505_ID            0x1505
#define ILI9300_ID            0x9300
#define ILI9320_ID            0x9320
#define ILI9321_ID            0x9321
#define ILI9325_ID            0x9325
#define ILI9328_ID            0x9328
#define ILI9331_ID            0x9331
#define ILI9919_ID            0x9919

/* Debug ******************************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg              dbg
#  define lcdvdbg             vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

#undef lcddbg
#define lcddbg printf
#define lcdvdbg printf

/************************************************************************************
 * Private Type Definition
 ************************************************************************************/

/* LCD type */

enum lcd_type_e
{
  LCD_TYPE_UNKNOWN = 0,
  LCD_TYPE_ILI1505,
  LCD_TYPE_ILI9300,
  LCD_TYPE_ILI9320,
  LCD_TYPE_ILI9321,
  LCD_TYPE_ILI9325,
  LCD_TYPE_ILI9328,
  LCD_TYPE_ILI9331,
  LCD_TYPE_ILI9919
};

/* This structure describes the LCD registers */

struct lcd_regs_s
{
  volatile uint16_t address;
  volatile uint16_t value;
};

/* This structure describes the state of this driver */

struct stm32_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  uint8_t  type;        /* LCD type. See enum lcd_type_e */
  uint8_t  power;       /* Current power setting */
  bool     output;      /* True: Configured for output */
};

/************************************************************************************
 * Private Function Protototypes
 ************************************************************************************/

static void stm32_writereg(FAR struct stm32_dev_s *priv, uint8_t regaddr,
                           uint16_t regval);
static uint16_t stm32_readreg(FAR struct stm32_dev_s *priv, uint8_t regaddr);
static void stm32_gramselect(FAR struct stm32_dev_s *priv);
static void stm32_writegram(FAR struct stm32_dev_s *priv, uint16_t rgbval);
static inline uint16_t stm32_readgram(FAR struct stm32_dev_s *priv);
static void stm32_readnosetup(FAR struct stm32_dev_s *priv, FAR uint16_t *accum);
static uint16_t stm32_readnoshift(FAR struct stm32_dev_s *priv, FAR uint16_t *accum);
static void stm32_setcursor(FAR struct stm32_dev_s *priv, uint16_t col, uint16_t row);

/* LCD Data Transfer Methods */

static int stm32_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
             size_t npixels);
static int stm32_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
             size_t npixels);

/* LCD Configuration */

static int stm32_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int stm32_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
             FAR struct lcd_planeinfo_s *pinfo);

/* LCD RGB Mapping */

#ifdef CONFIG_FB_CMAP
#  error "RGB color mapping not supported by this driver"
#endif

/* Cursor Controls */

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

/* LCD Specific Controls */

static int stm32_getpower(struct lcd_dev_s *dev);
static int stm32_setpower(struct lcd_dev_s *dev, int power);
static int stm32_getcontrast(struct lcd_dev_s *dev);
static int stm32_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

#if !defined(CONFIG_STM32_ILI9300_DISABLE) || !defined(CONFIG_STM32_ILI9320_DISABLE) || !defined(CONFIG_STM32_ILI9321_DISABLE)
static void stm32_lcd9300init(FAR struct stm32_dev_s *priv, enum lcd_type_e lcdtype);
#endif
#if !defined(CONFIG_STM32_ILI9325_DISABLE) || !defined(CONFIG_STM32_ILI9328_DISABLE)
static void stm32_lcd9325init(FAR struct stm32_dev_s *priv, enum lcd_type_e lcdtype);
#endif
#ifndef CONFIG_STM32_ILI9919_DISABLE
static inline void stm32_lcd9919init(FAR struct stm32_dev_s *priv);
#endif
#ifndef CONFIG_STM32_ILI1505_DISABLE
static inline void stm32_lcd1505init(FAR struct stm32_dev_s *priv);
#endif
static inline int stm32_lcdinitialize(FAR struct stm32_dev_s *priv);

/************************************************************************************
 * Private Data
 ************************************************************************************/
/* LCD GPIO configurations */

static const uint32_t g_lcdctrl[] =
{
  GPIO_LCD_RS, GPIO_LCD_CS, GPIO_LCD_RST,GPIO_FSMC_NWE,GPIO_FSMC_NOE,
};
#define NLCD_CONFIG (sizeof(g_lcdctrl)/sizeof(uint32_t))

/* This is working memory allocated by the LCD driver for each LCD device
 * and for each color plane.  This memory will hold one raster line of data.
 * The size of the allocated run buffer must therefore be at least
 * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
 * bitwidth of the underlying pixel type.
 *
 * If there are multiple planes, they may share the same working buffer
 * because different planes will not be operate on concurrently.  However,
 * if there are multiple LCD devices, they must each have unique run buffers.
 */

static uint16_t g_runbuffer[STM32_XRES];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = STM32_COLORFMT,    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = STM32_XRES,        /* Horizontal resolution in pixel columns */
  .yres    = STM32_YRES,        /* Vertical resolution in pixel rows */
  .nplanes = 1,                    /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo = 
{
  .putrun = stm32_putrun,       /* Put a run into LCD memory */
  .getrun = stm32_getrun,       /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer, /* Run scratch buffer */
  .bpp    = STM32_BPP,          /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct stm32_dev_s g_lcddev = 
{
  .dev =
  {
    /* LCD Configuration */
 
    .getvideoinfo = stm32_getvideoinfo,
    .getplaneinfo = stm32_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = stm32_getpower,
    .setpower     = stm32_setpower,
    .getcontrast  = stm32_getcontrast,
    .setcontrast  = stm32_setcontrast,
  },
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name:  stm32_writereg
 *
 * Description:
 *   Write to an LCD register
 *
 ************************************************************************************/

static void stm32_writereg(FAR struct stm32_dev_s *priv, uint8_t regaddr, uint16_t regval)
{

  /* Write the 8-bit register index */

  stm32_gpiowrite(GPIO_LCD_CS,0);
  stm32_gpiowrite(GPIO_LCD_RS,0);
  putreg16(regaddr,STM32_LCDBASE);

  /* Then write the 16-bit register value */
  stm32_gpiowrite(GPIO_LCD_RS,1);
  putreg16(regval,STM32_LCDBASE);
  

  //disable LCD Read/Write operation
  stm32_gpiowrite(GPIO_LCD_CS,1);
}

/************************************************************************************
 * Name:  stm32_readreg
 *
 * Description:
 *   Read from an LCD register
 *
 ************************************************************************************/

static uint16_t stm32_readreg(FAR struct stm32_dev_s *priv, uint8_t regaddr)
{
  uint16_t regval;

  stm32_gpiowrite(GPIO_LCD_CS,0);

  stm32_gpiowrite(GPIO_LCD_RS,0);
  putreg16(regaddr, STM32_LCDBASE);
  stm32_gpiowrite(GPIO_LCD_RS,1);
  regval = getreg16(STM32_LCDBASE);

  stm32_gpiowrite(GPIO_LCD_CS,1);
    
  return regval;
}

/************************************************************************************
 * Name:  stm32_gramselect
 *
 * Description:
 *   Setup to read or write multiple pixels to the GRAM memory
 *
 ************************************************************************************/

static void stm32_gramselect(FAR struct stm32_dev_s *priv)
{

  stm32_gpiowrite(GPIO_LCD_CS,0);
  /* Write the command */

  stm32_gpiowrite(GPIO_LCD_RS,0);
  putreg16(GRAM_WRITE_DATA, STM32_LCDBASE);
  
  stm32_gpiowrite(GPIO_LCD_RS,1);
  stm32_gpiowrite(GPIO_LCD_CS,1);
}

/************************************************************************************
 * Name:  stm32_writegram
 *
 * Description:
 *   Write one pixel to the GRAM memory
 *
 ************************************************************************************/

static inline void stm32_writegram(FAR struct stm32_dev_s *priv, uint16_t rgbval)
{

  /* Write the value (GRAM register already selected) */

  stm32_gpiowrite(GPIO_LCD_CS,0);
  putreg16(rgbval,STM32_LCDBASE);
  stm32_gpiowrite(GPIO_LCD_CS,1);
}

/************************************************************************************
 * Name:  stm32_readgram
 *
 * Description:
 *   Read one 16-bit pixel to the GRAM memory
 *
 ************************************************************************************/

static inline uint16_t stm32_readgram(FAR struct stm32_dev_s *priv)
{
  uint16_t regval;

  /* Read the 16-bit value */
  stm32_gpiowrite(GPIO_LCD_CS,0);
  stm32_gpiowrite(GPIO_LCD_RS,1);
  
  regval = getreg16(STM32_LCDBASE);
  
  stm32_gpiowrite(GPIO_LCD_CS,1);
    
  return regval;
}

/************************************************************************************
 * Name:  stm32_readnosetup
 *
 * Description:
 *   Prime the operation by reading one pixel from the GRAM memory if necessary for
 *   this LCD type.  When reading 16-bit gram data, there may be some shifts in the
 *   returned data:
 *
 *   - ILI932x: Discard first dummy read; no shift in the return data
 *
 ************************************************************************************/
 
static void stm32_readnosetup(FAR struct stm32_dev_s *priv, FAR uint16_t *accum)
{
  /* Read-ahead one pixel */

  *accum  = stm32_readgram(priv);
}

/************************************************************************************
 * Name:  stm32_readnoshift
 *
 * Description:
 *   Read one correctly aligned pixel from the GRAM memory.  Possibly shifting the
 *   data and possibly swapping red and green components.
 *
 *   - ILI932x: Unknown -- assuming colors are in the color order 
 *
 ************************************************************************************/

static uint16_t stm32_readnoshift(FAR struct stm32_dev_s *priv, FAR uint16_t *accum)
{
  /* Read the value (GRAM register already selected) */

  return stm32_readgram(priv);
}

/************************************************************************************
 * Name:  stm32_setcursor
 *
 * Description:
 *   Set the cursor position.  In landscape mode, the "column" is actually the physical
 *   Y position and the "row" is the physical X position.
 *
 ************************************************************************************/

static void stm32_setcursor(FAR struct stm32_dev_s *priv, uint16_t col, uint16_t row)
{
  uint32_t address = col*STM32_XRES + row;

  if (priv->type == LCD_TYPE_ILI9919)
    {
      stm32_writereg(priv, LCD_REG_78, col); /* GRAM horizontal address */
      stm32_writereg(priv, LCD_REG_79, row); /* GRAM vertical address */
    }
  else
    {
      stm32_writereg(priv, GRAM_ADDR_SET_H, (uint16_t)(address&0xFF)); /* GRAM vertical address */
      stm32_writereg(priv, GRAM_ADDR_SET_V, (uint16_t)(address>>8)); /* GRAM horizontal address */
    }
}

/************************************************************************************
 * Name:  stm32_dumprun
 *
 * Description:
 *   Dump the contexts of the run buffer:
 *
 *  run     - The buffer in containing the run read to be dumped
 *  npixels - The number of pixels to dump
 *
 ************************************************************************************/

#if 0 /* Sometimes useful */
static void stm32_dumprun(FAR const char *msg, FAR uint16_t *run, size_t npixels)
{
  int i, j;

  syslog("\n%s:\n", msg);
  for (i = 0; i < npixels; i += 16)
    {
      up_putc(' ');
      syslog(" ");
      for (j = 0; j < 16; j++)
        {
          syslog(" %04x", *run++);
        }
      up_putc('\n');
    }
}
#endif

/************************************************************************************
 * Name:  stm32_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ************************************************************************************/

static int stm32_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                       size_t npixels)
{
  FAR struct stm32_dev_s *priv = &g_lcddev;
  FAR const uint16_t *src = (FAR const uint16_t*)buffer;
  int i;
 
  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Write the run to GRAM */

#ifdef CONFIG_LCD_LANDSCAPE
  /* Convert coordinates */

  /* Write the GRAM data, manually incrementing X */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      stm32_setcursor(priv, col, row);
      stm32_gramselect(priv);
      stm32_writegram(priv, *src++);

      /* Increment to next column */

      col++;
    }
#elif defined(CONFIG_LCD_RLANDSCAPE)
  /* Convert coordinates */

  col = (STM32_XRES-1) - col;
  row = (STM32_YRES-1) - row;

  /* Set the cursor position */

  stm32_setcursor(priv, col, row);

  /* Then write the GRAM data, auto-decrementing X */

  stm32_gramselect(priv);
  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position (auto-decrements to the next column) */

      stm32_writegram(priv, *src++);
    }
#elif defined(CONFIG_LCD_PORTRAIT)
  /* Convert coordinates */

  col = (STM32_XRES-1) - col;

  /* Then write the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      stm32_setcursor(priv, row, col);
      stm32_gramselect(priv);
      stm32_writegram(priv, *src++);

      /* Increment to next column */

      col--;
    }
#else /* CONFIG_LCD_RPORTRAIT */
  /* Convert coordinates */

  row = (STM32_YRES-1) - row;
  
  /* Then write the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      stm32_setcursor(priv, row, col);
      stm32_gramselect(priv);
      stm32_writegram(priv, *src++);

      /* Decrement to next column */

      col++;
    }
#endif
  return OK;
}

/************************************************************************************
 * Name:  stm32_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 ************************************************************************************/

static int stm32_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                       size_t npixels)
{
  FAR struct stm32_dev_s *priv = &g_lcddev;
  FAR uint16_t *dest = (FAR uint16_t*)buffer;
  void (*readsetup)(FAR struct stm32_dev_s *priv, FAR uint16_t *accum);
  uint16_t (*readgram)(FAR struct stm32_dev_s *priv, FAR uint16_t *accum);
  uint16_t accum;
  int i;
 
  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Configure according to the LCD type.  Kind of silly with only one LCD type */

  switch (priv->type)
   {
     case LCD_TYPE_ILI1505:
     case LCD_TYPE_ILI9300:
     case LCD_TYPE_ILI9320:
     case LCD_TYPE_ILI9321:
     case LCD_TYPE_ILI9325:
     case LCD_TYPE_ILI9328:
     case LCD_TYPE_ILI9331:
     case LCD_TYPE_ILI9919:
       readsetup = stm32_readnosetup;
       readgram  = stm32_readnoshift;
       break;

     case LCD_TYPE_UNKNOWN:
     default:  /* Shouldn't happen */
       return -ENOSYS;
   }
 
  /* Read the run from GRAM */

#ifdef CONFIG_LCD_LANDSCAPE
  /* Convert coordinates */

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel from this position */

      stm32_setcursor(priv, row, col);
      stm32_gramselect(priv);
      readsetup(priv, &accum);
      *dest++ = readgram(priv, &accum);

      /* Increment to next column */

      col++;
    }
#elif defined(CONFIG_LCD_RLANDSCAPE)
  /* Convert coordinates */

  col = (STM32_XRES-1) - col;
  row = (STM32_YRES-1) - row;

  /* Set the cursor position */

  stm32_setcursor(priv, col, row);

  /* Then read the GRAM data, auto-decrementing Y */

  stm32_gramselect(priv);

  /* Prime the pump for unaligned read data */

  readsetup(priv, &accum);

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel from this position (autoincrements to the next row) */

      *dest++ = readgram(priv, &accum);
    }
#elif defined(CONFIG_LCD_PORTRAIT)
  /* Convert coordinates */

  col = (STM32_XRES-1) - col;

  /* Then read the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel from this position */

      stm32_setcursor(priv, row, col);
      stm32_gramselect(priv);
      readsetup(priv, &accum);
      *dest++ = readgram(priv, &accum);

      /* Increment to next column */

      col--;
    }
#else /* CONFIG_LCD_RPORTRAIT */
  /* Convert coordinates */

  row = (STM32_YRES-1) - row;
  
  /* Then write the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      stm32_setcursor(priv, row, col);
      stm32_gramselect(priv);
      readsetup(priv, &accum);
      *dest++ = readgram(priv, &accum);

      /* Decrement to next column */

      col++;
    }
#endif

  return OK;
}

/************************************************************************************
 * Name:  stm32_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ************************************************************************************/

static int stm32_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/************************************************************************************
 * Name:  stm32_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ************************************************************************************/

static int stm32_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/************************************************************************************
 * Name:  stm32_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 ************************************************************************************/

static int stm32_getpower(struct lcd_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  lcdvdbg("power: %d\n", 0);
  return priv->power;
}

/************************************************************************************
 * Name:  stm32_poweroff
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 ************************************************************************************/

static int stm32_poweroff(FAR struct stm32_dev_s *priv)
{
  /* Turn the display off */
  uint16_t regval = stm32_readreg(priv,DISP_CTL_1);
  regval &= ~(0x3);
  stm32_writereg(priv, DISP_CTL_1, 0); 

  /* Remember the power off state */

  priv->power = 0;
  return OK;
}

/************************************************************************************
 * Name:  stm32_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 ************************************************************************************/

static int stm32_setpower(struct lcd_dev_s *dev, int power)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  lcdvdbg("power: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      /* Then turn the display on */

#ifndef CONFIG_STM32_ILI9300_DISABLE
      if (priv->type == LCD_TYPE_ILI9300)
        {
          stm32_writereg(priv, DISP_CTL_1, 0x0173);
        }
      else
#endif
#ifndef CONFIG_STM32_ILI9320_DISABLE
      if (priv->type == LCD_TYPE_ILI9320)
        {
          stm32_writereg(priv, DISP_CTL_1, 0x0173);
        }
      else
#endif
#ifndef CONFIG_STM32_ILI9321_DISABLE
      if (priv->type == LCD_TYPE_ILI9321)
        {
          stm32_writereg(priv, DISP_CTL_1, 0x0173);
        }
      else
#endif
#ifndef CONFIG_STM32_ILI9325_DISABLE
      if (priv->type == LCD_TYPE_ILI9325)
        {
          stm32_writereg(priv, DISP_CTL_1,   0x0133);
        }
      else
#endif
#ifndef CONFIG_STM32_ILI9328_DISABLE
      if (priv->type == LCD_TYPE_ILI9328)
        {
          stm32_writereg(priv, DISP_CTL_1,   0x0133);
        }
      else
#endif
#ifndef CONFIG_STM32_ILI9331_DISABLE
      if (priv->type == LCD_TYPE_ILI9331)
        {
          stm32_writereg(priv, DISP_CTL_1,   0x0021);
          up_mdelay(50);
          stm32_writereg(priv, DISP_CTL_1,   0x0061);
          up_mdelay(50);
          stm32_writereg(priv, DISP_CTL_1,   0x0133);  /* 262K color and display ON */
        }
      else
#endif
#ifndef CONFIG_STM32_ILI9919_DISABLE
      if (priv->type == LCD_TYPE_ILI9919)
        {
          stm32_writereg(priv, DISP_CTL_1,  0x0033);
        }
      else
#endif
#ifndef CONFIG_STM32_ILI1505_DISABLE
      if (priv->type == LCD_TYPE_ILI1505)
        {
          stm32_writereg(priv, DISP_CTL_1,   0x0021);
          up_mdelay(20);
          stm32_writereg(priv, DISP_CTL_1,   0x0061);
          up_mdelay(20);
          stm32_writereg(priv, DISP_CTL_1,   0x0173);
        }
      else
#endif
        {
          gdbg("Unsupported LCD: %d\n", priv->type);
        }

      up_mdelay(50);
      priv->power = power;
    }
  else
    {
      /* Turn the display off */

      stm32_poweroff(priv);
    }

  return OK;
}

/************************************************************************************
 * Name:  stm32_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ************************************************************************************/

static int stm32_getcontrast(struct lcd_dev_s *dev)
{
  lcdvdbg("Not implemented\n");
  return -ENOSYS;
}

/************************************************************************************
 * Name:  stm32_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ************************************************************************************/

static int stm32_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  lcdvdbg("contrast: %d\n", contrast);
  return -ENOSYS;
}

/************************************************************************************
 * Name:  stm32_lcd9300init
 *
 * Description:
 *   Initialize the ILI9300/9220/9321 LCD.
 *
 ************************************************************************************/

#if !defined(CONFIG_STM32_ILI9300_DISABLE) || !defined(CONFIG_STM32_ILI9320_DISABLE) || !defined(CONFIG_STM32_ILI9321_DISABLE)
static void stm32_lcd9300init(FAR struct stm32_dev_s *priv, enum lcd_type_e lcdtype)
{
  stm32_writereg(priv, START_OSC,   0x0001); /* Start internal OSC */
  stm32_writereg(priv, DRV_OUTPUT_CTL, 0x0100); /* Driver Output Control */
  stm32_writereg(priv, LCD_DRIVING_CTL, 0x0700); /* LCD Driver Waveform Control */
  stm32_writereg(priv, ENTRY_MODE,   0x1018); /* Set GRAM write direction and BGR=1 (0x1030)*/

  stm32_writereg(priv, RESIZE_CTL,   0x0000); /* Scalling Control */
  stm32_writereg(priv, DISP_CTL_2,   0x0202); /* Set the back porch and front porch (0x0207) */
  stm32_writereg(priv, DISP_CTL_3,   0x0000); /* Set non-display area refresh cycle ISC[3:0] */
  stm32_writereg(priv, DISP_CTL_4,  0x0000); /* Frame Cycle Control */
  stm32_writereg(priv, RGB_IF_CTL_1,  (1<<0)); /* RGB interface setting (0x0000) */
  stm32_writereg(priv, FRAME_MAKER_POS,  0x0000); /* Frame Maker Position */
  stm32_writereg(priv, RGB_IF_CTL_2,  0x0000); /* RGB interface polarity */

  up_mdelay(50);
  stm32_writereg(priv, DISP_CTL_1,   0x0101); /* Display Control */
  up_mdelay(50);

  /* Power On sequence */

  stm32_writereg(priv, POWER_CTL_1,  (1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));    /* Power Control 1 (0x16b0) */
  stm32_writereg(priv, POWER_CTL_2,  0x0007); /* Power Control 2 (0x0001) */
  stm32_writereg(priv, POWER_CTL_3,  (1<<8)|(1<<4)|(0<<0)); /* Power Control 3 (0x0138) */
  stm32_writereg(priv, POWER_CTL_4,  0x0b00); /* VDV[4:0] for VCOM amplitude */
  stm32_writereg(priv, POWER_CTL_7,  0x0000); /* VCM[4:0] for VCOMH */

  stm32_writereg(priv, FRAME_RATE_COLOR_CTL,  (1<<14)|(1<<4));

  stm32_writereg(priv, H_ADDR_START_POS,  0);      /* Set X Start */
  stm32_writereg(priv, H_ADDR_END_POS,  239);    /* Set X End */
  stm32_writereg(priv, V_ADDR_START_POS,  0);      /* Set Y Start */
  stm32_writereg(priv, V_ADDR_END_POS,  319);    /* Set Y End */

  stm32_writereg(priv, DRV_OUTPUT_CTL_2,  0x2700); /* Driver Output Control */
  stm32_writereg(priv, BASE_IMG_DISP_CTL,  0x0001); /* Driver Output Control */
  stm32_writereg(priv, V_SCROLL_CTL, 0x0000); /* Vertical Srcoll Control */

  stm32_writereg(priv, PORTIAL_IMG_1_POS, 0x0000); /* Display Position? Partial Display 1 */
  stm32_writereg(priv, PORTIAL_IMG_1_START, 0x0000); /* RAM Address Start? Partial Display 1 */
  stm32_writereg(priv, PORTIAL_IMG_1_END, 0x0000); /* RAM Address End-Partial Display 1 */
  stm32_writereg(priv, PORTIAL_IMG_2_POS, 0x0000); /* Display Position? Partial Display 2 */
  stm32_writereg(priv, PORTIAL_IMG_2_START, 0x0000); /* RAM Address Start? Partial Display 2 */
  stm32_writereg(priv, PORTIAL_IMG_2_END, 0x0000); /* RAM Address End? Partial Display 2 */

  stm32_writereg(priv, PANEL_IF_CTL_1, (0<<7)|(16<<0)); /* Frame Cycle Control (0x0013) */
  stm32_writereg(priv, PANEL_IF_CTL_2, 0x0000); /* Panel Interface Control 2 */
  stm32_writereg(priv, PANEL_IF_CTL_3, 0x0001); /* Panel Interface Control 3 */
  stm32_writereg(priv, PANEL_IF_CTL_4, 0x0110); /* Frame Cycle Control */
  stm32_writereg(priv, PANEL_IF_CTL_5, (0<<8));
  stm32_writereg(priv, PANEL_IF_CTL_6, 0x0000); /* Frame Cycle Control */
  up_mdelay(50);
  stm32_writereg(priv, DISP_CTL_1,   0x0000); /* Display off */
}
#endif

/************************************************************************************
 * Name:  stm32_lcd9331init
 *
 * Description:
 *   Initialize the ILI9331 LCD.
 *
 ************************************************************************************/

#ifndef CONFIG_STM32_ILI9331_DISABLE
static void stm32_lcd9331init(FAR struct stm32_dev_s *priv)
{
  stm32_writereg(priv, LCD_REG_231, 0x1014);
  stm32_writereg(priv, DRV_OUTPUT_CTL, 0x0100); /* Set SS and SM bit */
  stm32_writereg(priv, LCD_DRIVING_CTL, 0x0200); /* Set 1 line inversion */
  stm32_writereg(priv, ENTRY_MODE,   0x1030); /* Set GRAM write direction and BGR=1 */
  stm32_writereg(priv, DISP_CTL_2,   0x0202); /* Set the back porch and front porch */
  stm32_writereg(priv, DISP_CTL_3,   0x0000); /* Set non-display area refresh cycle ISC[3:0] */
  stm32_writereg(priv, DISP_CTL_4,  0x0000); /* FMARK function */
  stm32_writereg(priv, RGB_IF_CTL_1,  0x0000); /* RGB interface setting */
  stm32_writereg(priv, FRAME_MAKER_POS,  0x0000); /* Frame marker Position */
  stm32_writereg(priv, RGB_IF_CTL_2,  0x0000); /* RGB interface polarity */

  /* Power On sequence */

  stm32_writereg(priv, POWER_CTL_1,  0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  stm32_writereg(priv, POWER_CTL_2,  0x0007); /* DC1[2:0], DC0[2:0], VC[2:0] */
  stm32_writereg(priv, POWER_CTL_3,  0x0000); /* VREG1OUT voltage */
  stm32_writereg(priv, POWER_CTL_4,  0x0000); /* VDV[4:0] for VCOM amplitude */
  up_mdelay(200);                            /* Dis-charge capacitor power voltage */
  stm32_writereg(priv, POWER_CTL_1,  0x1690); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  stm32_writereg(priv, POWER_CTL_2,  0x0227); /* DC1[2:0], DC0[2:0], VC[2:0] */
  up_mdelay(50);
  stm32_writereg(priv, POWER_CTL_3,  0x000c); /* Internal reference voltage= Vci; */
  up_mdelay(POWER_CTL_4);
  stm32_writereg(priv, POWER_CTL_4,  0x0800); /* Set VDV[4:0] for VCOM amplitude */
  stm32_writereg(priv, POWER_CTL_7,  0x0011); /* Set VCM[5:0] for VCOMH */
  stm32_writereg(priv, FRAME_RATE_COLOR_CTL,  0x000b); /* Set Frame Rate */
  up_mdelay(50);
  stm32_writereg(priv, GRAM_ADDR_SET_H,  0x0000); /* GRAM horizontal Address */
  stm32_writereg(priv, GRAM_ADDR_SET_V,  0x0000); /* GRAM Vertical Address */

  /* Adjust the Gamma Curve */

  stm32_writereg(priv, GAMMA_CTL_1,  0x0000);
  stm32_writereg(priv, GAMMA_CTL_2,  0x0106);
  stm32_writereg(priv, GAMMA_CTL_3,  0x0000);
  stm32_writereg(priv, GAMMA_CTL_4,  0x0204);
  stm32_writereg(priv, GAMMA_CTL_5,  0x160a);
  stm32_writereg(priv, GAMMA_CTL_6,  0x0707);
  stm32_writereg(priv, GAMMA_CTL_7,  0x0106);
  stm32_writereg(priv, GAMMA_CTL_8,  0x0707);
  stm32_writereg(priv, GAMMA_CTL_9,  0x0402);
  stm32_writereg(priv, GAMMA_CTL_10,  0x0c0f);

  /* Set GRAM area */

  stm32_writereg(priv, H_ADDR_START_POS,  0x0000); /* Horizontal GRAM Start Address */
  stm32_writereg(priv, H_ADDR_END_POS,  0x00ef); /* Horizontal GRAM End Address */
  stm32_writereg(priv, V_ADDR_START_POS,  0x0000); /* Vertical GRAM Start Address */
  stm32_writereg(priv, V_ADDR_END_POS,  0x013f); /* Vertical GRAM Start Address */
  stm32_writereg(priv, DRV_OUTPUT_CTL_2,  0x2700); /* Gate Scan Line */
  stm32_writereg(priv, BASE_IMG_DISP_CTL,  0x0001); /* NDL,VLE, REV */
  stm32_writereg(priv, V_SCROLL_CTL, 0x0000); /* set scrolling line */

  /* Partial Display Control */

  stm32_writereg(priv, PORTIAL_IMG_1_POS, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_1_START, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_1_END, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_2_POS, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_2_START, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_2_END, 0x0000);

  /* Panel Control */

  stm32_writereg(priv, PANEL_IF_CTL_1, 0x0010);
  stm32_writereg(priv, PANEL_IF_CTL_2, 0x0600);
  stm32_writereg(priv, DISP_CTL_1,   0x0000); /* Display off */
}
#endif

/************************************************************************************
 * Name:  stm32_lcd9325init
 *
 * Description:
 *   Initialize the ILI9325/9228 LCD.
 *
 ************************************************************************************/

#if !defined(CONFIG_STM32_ILI9325_DISABLE) || !defined(CONFIG_STM32_ILI9328_DISABLE)
static void stm32_lcd9325init(FAR struct stm32_dev_s *priv, enum lcd_type_e lcdtype)
{
  stm32_writereg(priv, LCD_REG_227, 0x3008);
  stm32_writereg(priv, LCD_REG_231, 0x0012);
  stm32_writereg(priv, LCD_REG_239, 0x1231); /* Set the internal vcore voltage */
/*stm32_writereg(priv, LCD_REG_231, 0x0010); */
  stm32_writereg(priv, START_OSC,   0x0001); /* Start internal osc */
  stm32_writereg(priv, DRV_OUTPUT_CTL, 0x0100); /* Set SS and SM bit */
  stm32_writereg(priv, LCD_DRIVING_CTL, 0x0700); /* Power on sequence */
  stm32_writereg(priv, ENTRY_MODE,   (1<<12)|(1<<5)|(1<<4) ); /* 65K */
  stm32_writereg(priv, RESIZE_CTL,   0x0000); /* Resize register */
  stm32_writereg(priv, DISP_CTL_2,   0x0207); /* Set the back porch and front porch */
  stm32_writereg(priv, DISP_CTL_3,   0x0000); /* Set non-display area refresh cycle ISC[3:0] */
  stm32_writereg(priv, DISP_CTL_4,  0x0000); /* FMARK function */
  stm32_writereg(priv, RGB_IF_CTL_1,  0x0001); /* RGB interface setting */
  stm32_writereg(priv, FRAME_MAKER_POS,  0x0000); /* Frame marker Position (0x0f3c) */
  stm32_writereg(priv, RGB_IF_CTL_2,  0x0000); /* RGB interface polarity */

  /* Power On sequence */

  stm32_writereg(priv, POWER_CTL_1,  0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  stm32_writereg(priv, POWER_CTL_2,  0x0007); /* DC1[2:0], DC0[2:0], VC[2:0] */
  stm32_writereg(priv, POWER_CTL_3,  0x0000); /* VREG1OUT voltage */
  stm32_writereg(priv, POWER_CTL_4,  0x0000); /* VDV[4:0] for VCOM amplitude */
  up_mdelay(100);
  stm32_writereg(priv, POWER_CTL_1,  0x1590); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  stm32_writereg(priv, POWER_CTL_2,  0x0227); /* DC1[2:0], DC0[2:0], VC[2:0] */
  up_mdelay(100);
  stm32_writereg(priv, POWER_CTL_3,  0x009c); /* VREG1OUT voltage */
  up_mdelay(POWER_CTL_4);
  stm32_writereg(priv, POWER_CTL_4,  0x1900); /* VDV[4:0] for VCOM amplitude */
  stm32_writereg(priv, POWER_CTL_7,  0x0023); /* VCM[4:0] for VCOMH */
  stm32_writereg(priv, FRAME_RATE_COLOR_CTL,  0x000e);
  up_mdelay(100);
  stm32_writereg(priv, GRAM_ADDR_SET_H,  0x0000); /* GRAM horizontal Address */
  stm32_writereg(priv, GRAM_ADDR_SET_V,  0x0000); /* GRAM Vertical Address */
  up_mdelay(100);

  /* Adjust the Gamma Curve */

  stm32_writereg(priv, GAMMA_CTL_1,  0x0007);
  stm32_writereg(priv, GAMMA_CTL_2,  0x0707);
  stm32_writereg(priv, GAMMA_CTL_3,  0x0006);
  stm32_writereg(priv, GAMMA_CTL_4,  0x0704);
  stm32_writereg(priv, GAMMA_CTL_5,  0x1f04);
  stm32_writereg(priv, GAMMA_CTL_6,  0x0004);
  stm32_writereg(priv, GAMMA_CTL_7,  0x0000);
  stm32_writereg(priv, GAMMA_CTL_8,  0x0706);
  stm32_writereg(priv, GAMMA_CTL_9,  0x0701);
  stm32_writereg(priv, GAMMA_CTL_10,  0x000f);
  up_mdelay(100);

  /* Set GRAM area */

  stm32_writereg(priv, H_ADDR_START_POS,  0x0000); /* Horizontal GRAM Start Address */
  stm32_writereg(priv, H_ADDR_END_POS,  0x00ef); /* Horizontal GRAM End Address */
  stm32_writereg(priv, V_ADDR_START_POS,  0x0000); /* Vertical GRAM Start Address */
  stm32_writereg(priv, V_ADDR_END_POS,  0x013f); /* Vertical GRAM End Address */
  stm32_writereg(priv, DRV_OUTPUT_CTL_2,  0xa700); /* Gate Scan Line */
  stm32_writereg(priv, BASE_IMG_DISP_CTL,  0x0001); /* NDL, VLE, REV */
  stm32_writereg(priv, V_SCROLL_CTL, 0x0000); /* Set scrolling line */

  /* Partial Display Control */

  stm32_writereg(priv, PORTIAL_IMG_1_POS, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_1_START, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_1_END, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_2_POS, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_2_START, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_2_END, 0x0000);

  /* Panel Control */

  stm32_writereg(priv, PANEL_IF_CTL_1, 0x0010);
  stm32_writereg(priv, PANEL_IF_CTL_2, 0x0600);

  if (lcdtype == LCD_TYPE_ILI9328)
    {
      stm32_writereg(priv, PANEL_IF_CTL_3, 0x0003);
      stm32_writereg(priv, PANEL_IF_CTL_4, 0x0110);
      stm32_writereg(priv, PANEL_IF_CTL_5, 0x0000);
      stm32_writereg(priv, PANEL_IF_CTL_6, 0x0000);
    }

  stm32_writereg(priv, DISP_CTL_1,   0x0000); /* Display off */
  stm32_writereg(priv, GRAM_ADDR_SET_H,  0x0000); /* GRAM horizontal Address */
  stm32_writereg(priv, GRAM_ADDR_SET_V,  0x0000); /* GRAM Vertical Address */
}
#endif

/************************************************************************************
 * Name:  stm32_lcd9919init
 *
 * Description:
 *   Initialize the ILI9919 LCD.
 *
 ************************************************************************************/

#ifndef CONFIG_STM32_ILI9919_DISABLE
static inline void stm32_lcd9919init(FAR struct stm32_dev_s *priv)
{
  /* Power on reset, display off */

  stm32_writereg(priv, LCD_REG_40, 0x0006);
  stm32_writereg(priv, START_OSC,  0x0001); /* Start internal OSC */
  stm32_writereg(priv, POWER_CTL_1, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  stm32_writereg(priv, DRV_OUTPUT_CTL,0x72ef);
  stm32_writereg(priv, LCD_DRIVING_CTL,0x0600); /* Set 1 line inversion */
  stm32_writereg(priv, ENTRY_MODE,  0x6a38);
  stm32_writereg(priv, POWER_CTL_2, 0x6874); /* DC1[2:0], DC0[2:0], VC[2:0] (0x0070) */

  /* RAM write data mask */

  stm32_writereg(priv, RGB_IF_CTL_2, 0x0000); /* RGB interface polarity */

  stm32_writereg(priv, LCD_REG_11, 0x5308);
  stm32_writereg(priv, RGB_IF_CTL_1, 0x0003); /* RGB interface setting */
  stm32_writereg(priv, FRAME_MAKER_POS, 0x000a); /* Frame marker Position */
  stm32_writereg(priv, LCD_REG_14, 0x2e00); /* 0x0030 */
  stm32_writereg(priv, LCD_REG_30, 0x00be);
  stm32_writereg(priv, LCD_REG_37, 0x8000);
  stm32_writereg(priv, LCD_REG_38, 0x7800);
  stm32_writereg(priv, LCD_REG_39, 0x0078);
  stm32_writereg(priv, LCD_REG_78, 0x0000);
  stm32_writereg(priv, LCD_REG_79, 0x0000);
  stm32_writereg(priv, POWER_CTL_3, 0x08d9); /* VREG1OUT voltage POWER_CTL_4

  /* Adjust the Gamma Curve */

  stm32_writereg(priv, GAMMA_CTL_1, 0x0000); /* 0x0007 */
  stm32_writereg(priv, GAMMA_CTL_2, 0x0104); /* 0x0203 */
  stm32_writereg(priv, GAMMA_CTL_3, 0x0100); /* 0x0001 */
  stm32_writereg(priv, GAMMA_CTL_4, 0x0305); /* 0x0007 */
  stm32_writereg(priv, GAMMA_CTL_5, 0x0505); /* 0x0007 */
  stm32_writereg(priv, GAMMA_CTL_6, 0x0305); /* 0x0407 */
  stm32_writereg(priv, GAMMA_CTL_7, 0x0707); /* 0x0407 */
  stm32_writereg(priv, GAMMA_CTL_8, 0x0300); /* 0x0607 */
  stm32_writereg(priv, GAMMA_CTL_9, 0x1200); /* 0x0106 */
  stm32_writereg(priv, GAMMA_CTL_10, 0x0800);

  stm32_writereg(priv, DISP_CTL_1,  0x0000); /* Display off */
}
#endif

/************************************************************************************
 * Name:  stm32_lcd1505init
 *
 * Description:
 *   Initialize the ILI1505 LCD.
 *
 ************************************************************************************/

#ifndef CONFIG_STM32_ILI1505_DISABLE
static inline void stm32_lcd1505init(FAR struct stm32_dev_s *priv)
{
  stm32_writereg(priv, DISP_CTL_1,   0x0000);
  up_mdelay(5);
  stm32_writereg(priv, POWER_CTL_3,  0x011c);
  stm32_writereg(priv, POWER_CTL_4, 0x0001); /* NVM */
  stm32_writereg(priv, DISP_CTL_2,   0x000f);
  stm32_writereg(priv, DISP_CTL_4,  0x0008);
  stm32_writereg(priv, FRAME_MAKER_POS,  0x0008);

  /* Adjust the Gamma Curve */

  stm32_writereg(priv, GAMMA_CTL_1,  0x0707);
  stm32_writereg(priv, GAMMA_CTL_2,  0x0007); /* 0x0707 */
  stm32_writereg(priv, GAMMA_CTL_3,  0x0603);
  stm32_writereg(priv, GAMMA_CTL_4,  0x0700);
  stm32_writereg(priv, GAMMA_CTL_5,  0x0202);
  stm32_writereg(priv, GAMMA_CTL_6,  0x0002); /* 0x0606 */
  stm32_writereg(priv, GAMMA_CTL_7,  0x1f0f);
  stm32_writereg(priv, GAMMA_CTL_8,  0x0707); /* 0x0f0f, 0x0105 */
  stm32_writereg(priv, GAMMA_CTL_9,  0x0000);
  stm32_writereg(priv, GAMMA_CTL_10,  0x0000);
  stm32_writereg(priv, LCD_REG_58,  0x0707);
  stm32_writereg(priv, LCD_REG_59,  0x0000); /* 0x0303 */
  stm32_writereg(priv, LCD_REG_60,  0x0007); /* 0x0707 */
  stm32_writereg(priv, LCD_REG_61,  0x0000); /* 0x1313, 0x1f08 */
  up_mdelay(5);

  stm32_writereg(priv, DISP_CTL_1,   0x0001);
  stm32_writereg(priv, LCD_REG_23,  0x0001); /* Power supply startup enable */
  up_mdelay(5);

  /* Power Control */

  stm32_writereg(priv, POWER_CTL_1,  0x17a0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  stm32_writereg(priv, POWER_CTL_2,  0x0217); /* Reference voltage VC[2:0] Vciout = 1.00*Vcivl */
  stm32_writereg(priv, POWER_CTL_3,  0x011e); /* Vreg1out = Vcilvl*1.80 (0x011c) */
  stm32_writereg(priv, POWER_CTL_4,  0x0f00); /* VDV[4:0]-->VCOM Amplitude VcomL = VcomH - Vcom Ampl */
  stm32_writereg(priv, LCD_REG_42,  0x0000);
  stm32_writereg(priv, POWER_CTL_7,  0x000a); /* Vcomh = VCM1[4:0]*Vreg1out gate source voltage (0x001f) */
  stm32_writereg(priv, POWER_CTL_3,  0x013e); /* Power supply on (0x013c) POWER_CTL_4

  /* Coordinates Control */

  stm32_writereg(priv, H_ADDR_START_POS,  0x0000); /* Horizontal GRAM Start Address */
  stm32_writereg(priv, H_ADDR_END_POS,  0x00ef); /* Horizontal GRAM End Address */
  stm32_writereg(priv, V_ADDR_START_POS,  0x0000); /* Vertical GRAM Start Address */
  stm32_writereg(priv, V_ADDR_END_POS,  0x013f); /* Vertical GRAM End Address */

  /* Panel Image Control */

  stm32_writereg(priv, DRV_OUTPUT_CTL_2,  0x2700); /* Gate Scan Line */
  stm32_writereg(priv, BASE_IMG_DISP_CTL,  0x0001); /* NDL, VLE, REV */
  stm32_writereg(priv, V_SCROLL_CTL, 0x0000); /* Set scrolling line */

  /* Partial Image Control */

  stm32_writereg(priv, PORTIAL_IMG_1_POS, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_1_START, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_1_END, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_2_POS, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_2_START, 0x0000);
  stm32_writereg(priv, PORTIAL_IMG_2_END, 0x0000);

  /* Panel Interface Control */

  stm32_writereg(priv, PANEL_IF_CTL_1, 0x0013);
  stm32_writereg(priv, PANEL_IF_CTL_2, 0x0300);
  stm32_writereg(priv, PANEL_IF_CTL_3, 0x0005);
  stm32_writereg(priv, PANEL_IF_CTL_4, 0x0000);
  stm32_writereg(priv, PANEL_IF_CTL_5, 0x0000);
  stm32_writereg(priv, PANEL_IF_CTL_6, 0x0000);

  stm32_writereg(priv, DRV_OUTPUT_CTL, 0x0100); /* Set SS and SM bit */
  stm32_writereg(priv, LCD_DRIVING_CTL, 0x0700); /* Set 1 line inversion */
  stm32_writereg(priv, ENTRY_MODE,   0x1030); /* Set GRAM write direction and BGR=1 */
  stm32_writereg(priv, RESIZE_CTL,   0x0000); /* Resize register */
  stm32_writereg(priv, RGB_IF_CTL_1,  0x0000); /* RGB interface setting */
  stm32_writereg(priv, RGB_IF_CTL_2,  0x0000); /* RGB interface polarity */
  stm32_writereg(priv, GRAM_ADDR_SET_H,  0x0000); /* GRAM horizontal Address */
  stm32_writereg(priv, GRAM_ADDR_SET_V,  0x0000); /* GRAM Vertical Address */
  stm32_writereg(priv, DISP_CTL_1,   0x0000); /* Display off */
}
#endif

/************************************************************************************
 * Name:  stm32_lcdinitialize
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ************************************************************************************/

static inline int stm32_lcdinitialize(FAR struct stm32_dev_s *priv)
{
  uint16_t id;
  int ret = OK;

  /* Check LCD ID */

  stm32_writereg(priv, START_OSC, 0x0001); /* Start internal oscillator */
  up_mdelay(50);

  id = stm32_readreg(priv, START_OSC);     /* Read the ID register */
  lcddbg("LCD ID: %04x\n", id);


  /* Initialize the LCD hardware */

#ifndef CONFIG_STM32_ILI9300_DISABLE
  if (id == ILI9300_ID)
    {
      priv->type = LCD_TYPE_ILI9300;
      stm32_lcd9300init(priv, LCD_TYPE_ILI9325);
    }
  else
#endif
#ifndef CONFIG_STM32_ILI9320_DISABLE
  if (id == ILI9320_ID)
    {
      priv->type = LCD_TYPE_ILI9320;
      stm32_lcd9300init(priv, LCD_TYPE_ILI9320);
    }
  else
#endif
#ifndef CONFIG_STM32_ILI9321_DISABLE
  if (id == ILI9321_ID)
    {
      priv->type = LCD_TYPE_ILI9321;
      stm32_lcd9300init(priv, LCD_TYPE_ILI9321);
    }
  else
#endif
#ifndef CONFIG_STM32_ILI9331_DISABLE
  if (id == ILI9331_ID)
    {
      priv->type = LCD_TYPE_ILI9331;
      stm32_lcd9331init(priv);
    }
  else
#endif
#ifndef CONFIG_STM32_ILI9325_DISABLE
  if (id == ILI9325_ID)
    {
      priv->type = LCD_TYPE_ILI9325;
      stm32_lcd9325init(priv, LCD_TYPE_ILI9325);
    }
  else
#endif
#ifndef CONFIG_STM32_ILI9328_DISABLE
  if (id == ILI9328_ID)
    {
      priv->type = LCD_TYPE_ILI9328;
      stm32_lcd9325init(priv, LCD_TYPE_ILI9328);
    }
  else
#endif
#ifndef CONFIG_STM32_ILI1505_DISABLE
  if (id == 0x1505)
    {
      priv->type = LCD_TYPE_ILI1505;
      stm32_lcd1505init(priv);
    }
  else
#endif
#ifndef CONFIG_STM32_ILI9919_DISABLE
  if (id == 0x9919)
    {
      priv->type = LCD_TYPE_ILI9919;
      stm32_lcd9919init(priv);
    }
  else
#endif
    {
      lcddbg("Unsupported LCD type\n");
      ret = -ENODEV;
    }

  lcddbg("LCD type: %d\n", priv->type);
  return ret;
}
 /************************************************************************************
 * Public Functions
 ************************************************************************************/

 /************************************************************************************
  * Name: stm32_selectlcd
  *
  * Description:
  *   Initialize to the LCD
  *
  ************************************************************************************/
#define LCD_NADDRLINES   1   /* A16 */
#define LCD_NDATALINES   16  /* D0-15 */

 void stm32_selectlcd(void)
 {
   
  int i;

   /* Configure GPIO pins */

   stm32_extmemdata(LCD_NDATALINES);             /* Common data lines: D0-D15 */
   for (i = 0; i < NLCD_CONFIG; i++)
    {
      stm32_configgpio(g_lcdctrl[i]);
    }

   /* Enable AHB clocking to the FSMC */

   stm32_enablefsmc();

   /* Color LCD configuration (LCD configured as follow):
    * 
    *   - Data/Address MUX  = Disable   "FSMC_BCR_MUXEN" just not enable it.
    *   - Extended Mode     = Disable   "FSMC_BCR_EXTMOD"
    *   - Memory Type       = SRAM      "FSMC_BCR_SRAM"
    *   - Data Width        = 16bit     "FSMC_BCR_MWID16"
    *   - Write Operation   = Enable    "FSMC_BCR_WREN"
    *   - Asynchronous Wait = Disable
    */

   /* Bank1 NOR/SRAM control register configuration */

   putreg32(FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR1);

   /* Bank1 NOR/SRAM timing register configuration */

   putreg32(FSMC_BTR_ADDSET(5) | FSMC_BTR_ADDHLD(0) | FSMC_BTR_DATAST(9) | FSMC_BTR_BUSTRUN(0) |
            FSMC_BTR_CLKDIV(0) | FSMC_BTR_DATLAT(0) | FSMC_BTR_ACCMODA, STM32_FSMC_BTR1);

   putreg32(0xffffffff, STM32_FSMC_BWTR1);

   /* Enable the bank by setting the MBKEN bit */

   putreg32(FSMC_BCR_MBKEN | FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR1);
 }

/************************************************************************************
 * Name:  up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 ************************************************************************************/

int up_lcdinitialize(void)
{
  FAR struct stm32_dev_s *priv = &g_lcddev;
  int ret;
  int i;

  lcdvdbg("Initializing\n");

  /* Configure control pins */
  stm32_selectlcd();
  
  /* Configure and enable LCD */

  
  up_mdelay(50);
  stm32_gpiowrite(GPIO_LCD_RST,0);
  up_mdelay(50);
  stm32_gpiowrite(GPIO_LCD_RST,1);
  ret = stm32_lcdinitialize(priv);
  if (ret == OK)
    {
      /* Clear the display (setting it to the color 0=black) */

      stm32_lcdclear(0);

      /* Turn the display off */

      stm32_poweroff(priv);
    }

  return ret;
}

/************************************************************************************
 * Name:  up_lcdgetdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This allows support
 *   for multiple LCD devices.
 *
 ************************************************************************************/

FAR struct lcd_dev_s *up_lcdgetdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return &g_lcddev.dev;
}

/************************************************************************************
 * Name:  up_lcduninitialize
 *
 * Description:
 *   Unitialize the LCD support
 *
 ************************************************************************************/

void up_lcduninitialize(void)
{
  FAR struct stm32_dev_s *priv = &g_lcddev;

  /* Put the LCD in the lowest possible power state */

  stm32_poweroff(priv);

  /* Make sure that the LCD is not selected */

  stm32_gpiowrite(GPIO_LCD_CS,1);
}

/************************************************************************************
 * Name:  stm32_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the Shenzhou board.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   display is cleared by simply setting all GRAM memory to the specified color.
 *
 ************************************************************************************/

void stm32_lcdclear(uint16_t color)
{
  FAR struct stm32_dev_s *priv = &g_lcddev;
  uint32_t i = 0;

  stm32_setcursor(priv, 0, 0); 
  stm32_gramselect(priv);

  /* Write the selected color into the entire GRAM memory */

  stm32_gpiowrite(GPIO_LCD_CS,0);
  stm32_gpiowrite(GPIO_LCD_RS,1);
  for (i = 0; i < STM32_XRES * STM32_YRES; i++)
    {
      
      putreg16(color,STM32_LCDBASE);
    
    }

   stm32_gpiowrite(GPIO_LCD_CS,1);
}

#endif /* !HAVE_LCD */
