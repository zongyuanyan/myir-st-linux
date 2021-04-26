// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 Free Electrons
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/of_gpio.h>
#include <video/mipi_display.h>
#include <linux/spi/spidev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <drm/drm_device.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

/*SLEEP OUT command*/
#define ST7701S_SLLEP_OUT     		0x11
#define ST7701S_SLEEP_IN     	 	0x10 
#define ST7701S_NORMAL_DISPLAY_ON	0x13
#define ST7701S_ALL_PIX_ON  		0x23
#define ST7701S_DISPLAY_OFF 		0x28
#define ST7701S_DISPLAY_ON  		0x29
#define ST7701S_DISPLAY_DATA 		0x36
#define ST7701S_IDEL_MODE_OFF 		0x38
#define ST7701S_PIXEL_FORMAT 		0x3A
#define ST7701S_16_BIT_PIXEL   (5 << 4) 
#define ST7701S_18_BIT_PIXEL   (6 << 4)
#define ST7701S_24_BIT_PIXEL   GENMASK(6, 4)
#define ST7701S_SET_BRIGHT    		0x51
#define ST7701S_TEST_BRIGHE   		0xA0

 /* Command2 BKx selection command */
#define ST7701S_CMD2BKX_SEL          	0xFF

/* Command2, BK0 commands */
#define ST7701S_CMD2_BK0_PVGAMCTRL      0xB0  
#define ST7701S_CMD2_BK0_NVGAMCTRL      0xB1
#define ST7701S_CMD2_BK0_LNESET     	0xC0 
#define ST7701S_CMD2_BK0_PORCTRL_VBP    0xC1 
#define ST7701S_CMD2_BK0_INVSET     	0xC2 
#define ST7701S_CORLOR_CONTROL     		0xCD
#define ST7701S_RGBCTRL_CMD				0xC3
#define ST7701S_RGBCTRL_HV		BIT(7)
#define ST7701S_RGBCTRL_VSYNC_HIGH		BIT(3)
#define ST7701S_RGBCTRL_HSYNC_HIGH		BIT(2)
#define ST7701S_RGBCTRL_PCLK_HIGH		BIT(1)
//#define ST7701S_RGBCTRL_HBPRGB   0xC301
#define ST7701S_RGBCTRL_HBP(n)			((n) & 0xff)
//#define ST7701S_RGBCTRL_VBPRGB   0xC302
#define ST7701S_RGBCTRL_VBP(n)			((n) & 0xff)

/* Command2, BK1 commands */
#define ST7701S_CMD2_BK1_VRHS       	0xB0
#define ST7701S_CMD2_BK1_VCOM       	0xB1 
#define ST7701S_CMD2_BK1_VGHSS			0xB2 
#define ST7701S_CMD2_BK1_TESTCMD		0xB3
#define ST7701S_CMD2_BK1_VGLS			0xB5 
#define ST7701S_CMD2_BK1_PWCTRL1		0xB7 
#define ST7701S_CMD2_BK1_PWCTRL2		0xB8
#define ST7701S_CMD2_BK1_SPD1			0xC1
#define ST7701S_CMD2_BK1_SPD2			0xC2 
#define ST7701S_CMD2_BK1_MIPISET1		0xD0 

/**
 * Command2 with BK function selection.
 *
 * BIT[4, 0]: [CN2, BKXSEL]
 * 10 = CMD2BK0, Command2 BK0
 * 11 = CMD2BK1, Command2 BK1
 * 00 = Command2 disable
 */
#define ST7701S_CMD2BK1_SEL				0x11
#define ST7701S_CMD2BK0_SEL				0x10
#define ST7701S_CMD2BKX_SEL_NONE		0x00

/* Command2, BK0 bytes */
#define ST7701S_LINESET_LINE			0x69  
#define ST7701S_LINESET_LDE_EN			BIT(7) 
#define ST7701S_LINESET_LINEDELTA		GENMASK(1, 0) 
#define ST7701S_CMD2_BK0_LNESET_B1		ST7701S_LINESET_LINEDELTA 
#define ST7701S_CMD2_BK0_LNESET_B0		(ST7701S_LINESET_LDE_EN | ST7701S_LINESET_LINE)
#define ST7701S_INVSEL_DEFAULT			GENMASK(5, 4)
#define ST7701S_INVSEL_NLINV			GENMASK(2, 0)
#define ST7701S_INVSEL_RTNI				GENMASK(2, 1)
#define ST7701S_CMD2_BK0_INVSEL_B1		ST7701S_INVSEL_RTNI
#define ST7701S_CMD2_BK0_INVSEL_B0		(ST7701S_INVSEL_DEFAULT | ST7701S_INVSEL_NLINV)
#define ST7701S_CMD2_BK0_PORCTRL_B0(m)	((m).vtotal - (m).vsync_end)
#define ST7701S_CMD2_BK0_PORCTRL_B1(m)	((m).vsync_start - (m).vdisplay)
#define ST7701S_CMD2_BK0_RGBCTRL_DE      

/* Command2, BK1 bytes */
#define ST7701S_CMD2_BK1_VRHA_SET		0x45
#define ST7701S_CMD2_BK1_VCOM_SET		0x13  /*0.3375*/
#define ST7701S_CMD2_BK1_VGHSS_SET		GENMASK(2, 0) /*15V*/
#define ST7701S_CMD2_BK1_TESTCMD_VAL	BIT(7)
#define ST7701S_VGLS_DEFAULT		BIT(6)  /*默认值-9.51*/
#define ST7701S_VGLS_SEL			GENMASK(2, 0)
#define ST7701S_CMD2_BK1_VGLS_SET		(ST7701S_VGLS_DEFAULT | ST7701S_VGLS_SEL)
#define ST7701S_PWCTRL1_AP			BIT(7) /* Gamma OP bias, max Middle*/
#define ST7701S_PWCTRL1_APIS		BIT(2) /* Source OP input bias, min */
#define ST7701S_PWCTRL1_APOS		BIT(0) /* Source OP output bias, min */
#define ST7701S_CMD2_BK1_PWCTRL1_SET	(ST7701S_PWCTRL1_AP | ST7701S_PWCTRL1_APIS | \
					ST7701S_PWCTRL1_APOS)
#define ST7701S_PWCTRL2_AVDD		BIT(5) /* AVDD 6.6v */
#define ST7701S_PWCTRL2_AVCL		0x0    /* AVCL -4.4v */
#define ST7701S_CMD2_BK1_PWCTRL2_SET	(ST7701S_PWCTRL2_AVDD | ST7701S_PWCTRL2_AVCL)
#define ST7701S_SPD1_T2D			BIT(3)  /*1.6us*/
#define ST7701S_CMD2_BK1_SPD1_SET		(GENMASK(6, 4) | ST7701S_SPD1_T2D)
#define ST7701S_CMD2_BK1_SPD2_SET		ST7701S_CMD2_BK1_SPD1_SET
#define ST7701S_MIPISET1_EOT_EN		BIT(3)
#define ST7701S_CMD2_BK1_MIPISET1_SET	(BIT(7) | ST7701S_MIPISET1_EOT_E

/*
#define ST7701S_TEST(val, func)			\
	do {					\
		if ((val = (func)))		\
			return val;		\
	} while (0)
*/

struct ST7701S {
	struct drm_panel my_panel;
	struct spi_device *spi;
	struct gpio_desc *enable_gpio;
	struct backlight_device *backlight;
	struct regulator *power;
	int cs_gpio;
	int reset_gpio;
	int gpio_enable_value;
	unsigned int sleep_delay;

};

enum ST7701S_prefix {
	ST7701S_COMMAND = 0,
	ST7701S_DATA = 1,
};

static inline struct ST7701S *panel_to_ST7701S(struct drm_panel *panel)
{
	return container_of(panel, struct ST7701S, my_panel);
}

static int ST7701S_spi_write(struct ST7701S *ctx, enum ST7701S_prefix prefix,
			     u16 data)
{
	struct spi_transfer xfer = { };
	struct spi_message msg;
	
	u16 txbuf = ((prefix & 1) << 8) | data;

	spi_message_init(&msg);

	xfer.tx_buf = &txbuf;
	xfer.bits_per_word = 9;
	xfer.len = sizeof(txbuf);
	//xfer.cs_change =1;

	spi_message_add_tail(&xfer, &msg);
	return spi_sync(ctx->spi, &msg);
}

static int ST7701S_write_command(struct ST7701S *ctx, u16 cmd)
{
	int ret;
	ret = ST7701S_spi_write(ctx, ST7701S_COMMAND, cmd );
	return ret;
}

static int ST7701S_write_data(struct ST7701S *ctx, u16 cmd)
{
	int ret;
	ret = ST7701S_spi_write(ctx, ST7701S_DATA, cmd);
	 return ret;
}
static const struct drm_display_mode default_mode = {
	.clock = 26000,
	.hdisplay = 480,
	.hsync_start = 480 + 10,
	.hsync_end = 480 + 10 + 2,
	.htotal = 480 + 10 + 2 + 10,
	.vdisplay = 480,
	.vsync_start = 480 + 10,
	.vsync_end = 480 + 10 + 2,
	.vtotal = 480 + 10 + 2 + 16,
	.vrefresh = 60,
};

static int ST7701S_get_modes(struct drm_panel *panel)
{
	struct ST7701S *ctx = panel_to_ST7701S(panel);
	struct drm_connector *connector = ctx->my_panel.connector;
	struct drm_display_mode *mode;
	mode = drm_mode_duplicate(ctx->my_panel.drm, &default_mode);
	if (!mode) {
		dev_err(ctx->my_panel.drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);
	panel->connector->display_info.width_mm = 75;
	panel->connector->display_info.height_mm = 77;

	return 1;
}

static int ST7701S_prepare(struct drm_panel *panel )
{

	struct ST7701S *ctx = panel_to_ST7701S(panel);
	int ret;
	u16 *rdata;
	gpio_direction_output(ctx->reset_gpio,1);
	msleep(20);
	gpio_direction_output(ctx->reset_gpio,0);
	msleep(1);
	gpio_direction_output(ctx->reset_gpio,1);
	mdelay(10);
	ST7701S_write_command(ctx, ST7701S_SLLEP_OUT);
	msleep(150);

ST7701S_write_command(ctx,ST7701S_PIXEL_FORMAT);
ST7701S_write_data(ctx,0x60);/*18bit*/
//choose command2 BK0
ST7701S_write_command(ctx,ST7701S_CMD2BKX_SEL );
ST7701S_write_data(ctx,0x77);
ST7701S_write_data(ctx,0x01);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x10);
ST7701S_write_command(ctx,ST7701S_CMD2_BK0_LNESET);/*设置480行*/
ST7701S_write_data(ctx,0x3B);
ST7701S_write_data(ctx,0x00);
ST7701S_write_command(ctx,ST7701S_CMD2_BK0_PORCTRL_VBP);//v back front 
ST7701S_write_data(ctx,0x0E);//vbp=14
ST7701S_write_data(ctx,0x0A);//vfp=8
ST7701S_write_command(ctx,ST7701S_CMD2_BK0_INVSET);
ST7701S_write_data(ctx,0x31);
ST7701S_write_data(ctx,0x08);/*每行多少像素*/
/*ST7701S_write_data(ctx,0x88);
RGB HV mode,VSYNC HSYNC 低电平有效
DOCLK 时钟下降沿输入数据
DE=1使能引脚为高
*/
ST7701S_write_command(ctx,ST7701S_RGBCTRL_CMD);//RGB 控制
ST7701S_write_data(ctx,0x83);
/*HBP=10,HFP=10*/
ST7701S_write_data(ctx,0x0C);/*设置HBP*/
ST7701S_write_data(ctx,0x10);//VBP 10
ST7701S_write_command(ctx,ST7701S_CORLOR_CONTROL);
ST7701S_write_data(ctx,0x08);
//Positive Voltage Gamma
ST7701S_write_command(ctx,ST7701S_CMD2_BK0_PVGAMCTRL);
ST7701S_write_data(ctx,0x40);
ST7701S_write_data(ctx,0x01);
ST7701S_write_data(ctx,0x46);
ST7701S_write_data(ctx,0x0D);
ST7701S_write_data(ctx,0x13);
ST7701S_write_data(ctx,0x09);
ST7701S_write_data(ctx,0x05);
ST7701S_write_data(ctx,0x09);
ST7701S_write_data(ctx,0x09);
ST7701S_write_data(ctx,0x1B);
ST7701S_write_data(ctx,0x07);
ST7701S_write_data(ctx,0x15);
ST7701S_write_data(ctx,0x12);
ST7701S_write_data(ctx,0x4C);
ST7701S_write_data(ctx,0x10);
ST7701S_write_data(ctx,0xC8);
//Negative Voltage Gamma Control
ST7701S_write_command(ctx,ST7701S_CMD2_BK0_NVGAMCTRL);
ST7701S_write_data(ctx,0x40);
ST7701S_write_data(ctx,0x02);
ST7701S_write_data(ctx,0x86);
ST7701S_write_data(ctx,0x0D);
ST7701S_write_data(ctx,0x13);
ST7701S_write_data(ctx,0x09);
ST7701S_write_data(ctx,0x05);
ST7701S_write_data(ctx,0x09);
ST7701S_write_data(ctx,0x09);
ST7701S_write_data(ctx,0x1F);
ST7701S_write_data(ctx,0x07);
ST7701S_write_data(ctx,0x15);
ST7701S_write_data(ctx,0x12);
ST7701S_write_data(ctx,0x15);
ST7701S_write_data(ctx,0x19);
ST7701S_write_data(ctx,0x08);
//command bank1
ST7701S_write_command(ctx,ST7701S_CMD2BKX_SEL );
ST7701S_write_data(ctx,0x77);
ST7701S_write_data(ctx,0x01);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x11);

ST7701S_write_command(ctx,ST7701S_CMD2_BK1_VRHS);
ST7701S_write_data(ctx,0x4D);/*3.53 + 80*0.0125*/
//-------------------------------------------Vcom Setting---------------------------------------------------//
ST7701S_write_command(ctx,ST7701S_CMD2_BK1_VCOM);
ST7701S_write_data(ctx,0x6F);/*1.4v 原来68*/
//-----------------------------------------End Vcom Setting-----------------------------------------------//
ST7701S_write_command(ctx,ST7701S_CMD2_BK1_VGHSS);
ST7701S_write_data(ctx,0x09);//Gate门高电压15v,原来0x07
ST7701S_write_command(ctx,ST7701S_CMD2_BK1_TESTCMD);
//ST7701S_write_data(ctx,0x80);//test command
ST7701S_write_command(ctx,ST7701S_CMD2_BK1_VGLS);
ST7701S_write_data(ctx,0x47);//gate 门低电压-9.51
ST7701S_write_command(ctx,ST7701S_CMD2_BK1_PWCTRL1);
ST7701S_write_data(ctx,0x8a);
ST7701S_write_command(ctx,ST7701S_CMD2_BK1_PWCTRL2);
ST7701S_write_data(ctx,0x21);//AVDD 6.6v AVCL -4.6v
ST7701S_write_command(ctx,ST7701S_CMD2_BK1_SPD1);
ST7701S_write_data(ctx,0x78);//source pre_drive timing setting.
ST7701S_write_command(ctx,ST7701S_CMD2_BK1_SPD2);
ST7701S_write_data(ctx,0x78);//source pre_drive timing setting
ST7701S_write_command(ctx,ST7701S_CMD2_BK1_MIPISET1);
ST7701S_write_data(ctx,0x88);//enable eotp report error
//---------------------------------End Power Control Registers Initial -------------------------------//
mdelay(100);
//---------------------------------------------GIP Setting----------------------------------------------------//
ST7701S_write_command(ctx,0xE0);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x02);
ST7701S_write_command(ctx,0xE1);
ST7701S_write_data(ctx,0x08);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x0A);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x07);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x09);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x33);
ST7701S_write_data(ctx,0x33);
ST7701S_write_command(ctx,0xE2);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_command(ctx,0xE3);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x33);
ST7701S_write_data(ctx,0x33);
ST7701S_write_command(ctx,0xE4);
ST7701S_write_data(ctx,0x44);
ST7701S_write_data(ctx,0x44);

ST7701S_write_command(ctx,0xE5);
ST7701S_write_data(ctx,0x0E);
ST7701S_write_data(ctx,0x2D);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0x10);
ST7701S_write_data(ctx,0x2D);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0x0A);
ST7701S_write_data(ctx,0x2D);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0x0C);
ST7701S_write_data(ctx,0x2D);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_command(ctx,0xE6);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x33);
ST7701S_write_data(ctx,0x33);
ST7701S_write_command(ctx,0xE7);
ST7701S_write_data(ctx,0x44);
ST7701S_write_data(ctx,0x44);
ST7701S_write_command(ctx,0xE8);
ST7701S_write_data(ctx,0x0D);
ST7701S_write_data(ctx,0x2D);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0x0F);
ST7701S_write_data(ctx,0x2D);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0x09);
ST7701S_write_data(ctx,0x2D);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0x0B);
ST7701S_write_data(ctx,0x2D);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_data(ctx,0xA0);
ST7701S_write_command(ctx,0xEB);
ST7701S_write_data(ctx,0x02);
ST7701S_write_data(ctx,0x01);
ST7701S_write_data(ctx,0xE4);
ST7701S_write_data(ctx,0xE4);
ST7701S_write_data(ctx,0x44);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x40);
ST7701S_write_command(ctx,0xEC);
ST7701S_write_data(ctx,0x02);
ST7701S_write_data(ctx,0x01);
ST7701S_write_command(ctx,0xED);
ST7701S_write_data(ctx,0xAB);
ST7701S_write_data(ctx,0x89);
ST7701S_write_data(ctx,0x76);
ST7701S_write_data(ctx,0x54);
ST7701S_write_data(ctx,0x01);
ST7701S_write_data(ctx,0xFF);
ST7701S_write_data(ctx,0xFF);
ST7701S_write_data(ctx,0xFF);
ST7701S_write_data(ctx,0xFF);
ST7701S_write_data(ctx,0xFF);
ST7701S_write_data(ctx,0xFF);
ST7701S_write_data(ctx,0x10);
ST7701S_write_data(ctx,0x45);
ST7701S_write_data(ctx,0x67);
ST7701S_write_data(ctx,0x98);
ST7701S_write_data(ctx,0xBA);

ST7701S_write_command(ctx,0xFF);
ST7701S_write_data(ctx,0x77);
ST7701S_write_data(ctx,0x01);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
ST7701S_write_data(ctx,0x00);
	return 0;
}

/*使能电源，复位，开启lcd*/
static int ST7701S_enable(struct drm_panel *panel)
{
	struct ST7701S *ctx = panel_to_ST7701S(panel);
	return ST7701S_write_command(ctx,ST7701S_DISPLAY_ON);
}

static int ST7701S_disable(struct drm_panel *panel)
{
	struct ST7701S *ctx = panel_to_ST7701S(panel);
	int ret;
	ret=ST7701S_write_command(ctx,ST7701S_DISPLAY_OFF);
	return 0;
}

static int ST7701S_unprepare(struct drm_panel *panel)
{
	struct ST7701S *ctx = panel_to_ST7701S(panel);
	ST7701S_write_command(ctx, ST7701S_SLEEP_IN);

	return 0;
}

static const struct drm_panel_funcs ST7701S_drm_funcs = {
	.disable	= ST7701S_disable,
	.unprepare	= ST7701S_unprepare,
	.prepare	= ST7701S_prepare,
	.enable		= ST7701S_enable,
	.get_modes	= ST7701S_get_modes,

};

static int ST7701S_probe(struct spi_device *spi)
{
	struct device_node *backlight;
	struct ST7701S *ctx;
	struct device_node *reset_gpio_node=spi->dev.of_node;
	enum of_gpio_flags flag;
	int ret,gpio,err;
	u16 *rdata;
	ctx = devm_kzalloc(&spi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	spi_set_drvdata(spi, ctx);
	ctx->spi = spi;
	ctx->spi->bits_per_word = 9;
	ctx->spi->mode = SPI_MODE_3 | SPI_CS_HIGH ;
	ctx->spi->mode &= ~SPI_LSB_FIRST;
	err = spi_setup(spi);
	printk( "setup mode %d, %s%s%s%s%u bits/w, %u Hz max --> %d\n",
			(int) (spi->mode & (SPI_CPOL | SPI_CPHA)),
			(spi->mode & SPI_CS_HIGH) ? "cs_high, " : "",
			(spi->mode & SPI_LSB_FIRST) ? "lsb, " : "",
			(spi->mode & SPI_3WIRE) ? "3wire, " : "",
			(spi->mode & SPI_LOOP) ? "loopback, " : "",
			spi->bits_per_word, spi->max_speed_hz
			);
    	
    ctx->reset_gpio = of_get_named_gpio_flags(reset_gpio_node, "reset-gpio", 0, &flag);
	if (!gpio_is_valid(ctx->reset_gpio)) {
    	printk("gpio: %d is invalid\n",ctx->reset_gpio); return -ENODEV;
        }
	if (gpio_request(ctx->reset_gpio, "reset-gpio")) {
        printk("gpio %d request failed!\n",ctx->reset_gpio);
        gpio_free(ctx->reset_gpio);
        return -ENODEV;
        }
           ctx->gpio_enable_value = (flag == OF_GPIO_ACTIVE_LOW) ? 0:1;
   	gpio_direction_output(ctx->reset_gpio,ctx->gpio_enable_value);
    	printk(" reset_gpio putout %d\n",ctx->gpio_enable_value);

/*从设备树获取复位引脚*/
	drm_panel_init(&ctx->my_panel);
	ctx->my_panel.funcs = &ST7701S_drm_funcs;
	ctx->my_panel.dev = &spi->dev;
	ret = drm_panel_add(&ctx->my_panel);
	if (ret < 0)
		goto err_free_backlight;
	return 0;

err_free_backlight:
	if (ctx->backlight)
		put_device(&ctx->backlight->dev);

	return ret;
}

static int ST7701S_remove(struct spi_device *spi)
{
	struct ST7701S *ctx = spi_get_drvdata(spi);

	drm_panel_remove(&ctx->my_panel);
	gpio_free(ctx->reset_gpio);
	return 0;
}

static const struct of_device_id ST7701S_of_match[] = {
	{ .compatible = "sitronix,ST7701S" },
	{ }
};
MODULE_DEVICE_TABLE(of, ST7701S_of_match);

static struct spi_driver ST7701S_driver = {
	.probe = ST7701S_probe,
	.remove = ST7701S_remove,
	.driver = {
		.name = "ST7701S",
		.of_match_table = ST7701S_of_match,
	},
};
module_spi_driver(ST7701S_driver);

MODULE_AUTHOR("miller.wu@myirtech.com");
MODULE_DESCRIPTION("Sitronix ST7701S SPI_LCD Driver");
MODULE_LICENSE("GPL v2");
