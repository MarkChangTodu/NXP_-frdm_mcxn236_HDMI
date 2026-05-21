/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Phase 0+1 bring-up: powers up the HDMIMIX/HDMI_PHY domains via raw GPC
 * register access (sub-goal 1.1), maps the four HDMI MMIO windows (1.2),
 * enables the HDMI BLK_CTRL level-1/level-2 clock gates (1.3) and deasserts
 * the per-block resets (1.4), then reads back known reset values from the
 * HDMIMIX slaves to verify the path is alive. No display output yet.
 */

#define DT_DRV_COMPAT nxp_imx8mp_hdmi

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(imx8mp_hdmi, CONFIG_DISPLAY_LOG_LEVEL);

/* CCM: i.MX8MP Clock Controller Module (IMX8MPRM §5.1).
 * HDMI_APB_CLK_ROOT and HDMI_AXI_CLK_ROOT must have ENABLE=1 (bit 28) before
 * the HDMI TX APB slave (0x32FD8000) will respond. POR value is 0x10000000
 * (ENABLE=1, MUX=0, no divider), but U-Boot may gate them. We enable them
 * explicitly before touching BLK_CTRL clocks.
 *   HDMI_APB_CLK_ROOT CCM+0x8B00  (200 MHz, docs/source/IMX8MPRM.md:L9406)
 *   HDMI_AXI_CLK_ROOT CCM+0x8B80  (500 MHz, docs/source/IMX8MPRM.md:L9414)
 */
#define CCM_BASE                  0x30380000UL
#define CCM_REG_SIZE              KB(64)
#define CCM_HDMI_APB_CLK_ROOT_OFF 0x8B00
#define CCM_HDMI_AXI_CLK_ROOT_OFF 0x8B80
#define CCM_CLK_ROOT_ENABLE       BIT(28)

/* GPC: i.MX8MP General Power Controller (IMX8MPRM §5.2). */
#define GPC_BASE                  0x303a0000UL
#define GPC_REG_SIZE              KB(64)
#define GPC_PU_PGC_SW_PUP_REQ_OFF 0xd8
#define GPC_PU_PGC_PUP_STATUS_OFF 0xdc /* placeholder; spec pages confirm via §5.2 */
/* GPC_PU_PGC_SW_PUP_REQ bit assignments, IMX8MPRM §5.2.10.37
 * (docs/source/IMX8MPRM.md:L26286-L26342). Reserved [31:20] -- writing
 * BIT(30)/BIT(31) is a no-op, which is why our earlier "power-up" appeared
 * to ack but actually did nothing for HDMI_PHY.
 *   bit 14 = HDMIMIX_SW_PUP_REQ
 *   bit 15 = HDMI_PHY_SW_PUP_REQ
 */
#define GPC_HDMIMIX_BIT           BIT(14)
#define GPC_HDMI_PHY_BIT          BIT(15)

/* Reset value of HTX_PVI_CTRL (IMX8MPRM §13.13.3.1.1). Used as a golden
 * value to verify MMU mapping + power-up: if we read this back it means
 * everything is wired correctly.
 */
#define HTX_PVI_CTRL_RESET_VAL    0x00377000UL

/* Reset value of HDMI_TX_PHY PHY_REG1 (IMX8MPRM §13.10.3.1.1). 8-bit
 * register at offset 0x4.
 */
#define HDMI_PHY_REG1_RESET_VAL   0xd1

/*
 * HDMI TX BLK_CTRL register map (IMX8MPRM §13.11.3 / IMX8MPRM.md
 * L240145-L240475). Base = 0x32FC_0000, accessible regardless of clock
 * gating ("blk_ctl is required to be accessible at all times").
 */
#define BLK_CTRL_RTX_RESET_CTL0_SET 0x24
#define BLK_CTRL_RTX_CLK_CTL0_SET   0x44
#define BLK_CTRL_RTX_CLK_CTL1_SET   0x54
#define BLK_CTRL_TX_CONTROL0        0x200
#define BLK_CTRL_TX_CONTROL0_CLR    0x208
#define BLK_CTRL_TX_CONTROL0_RESET  0x00000018UL

/* TX_CONTROL0 bit 3 = TX_PHY_PDOWN (drives PHY_PDOWN input of TX PHY).
 * Out of POR this bit is SET, i.e. PHY is powered-down and its CSR slave
 * SLVERRs. IMX8MPRM §13.11.3.1.13 (docs/source/IMX8MPRM.md:L241025-L241028).
 */
#define TX_CONTROL0_TX_PHY_PDOWN    BIT(3)

/* RTX_CLK_CTL0 level-1 (global) clock enables, IMX8MPRM §13.11.3.1.5. */
#define CLK_CTL0_GLOBAL_APB_CLK_EN     BIT(0)
#define CLK_CTL0_GLOBAL_B_CLK_EN       BIT(1)
#define CLK_CTL0_GLOBAL_REF266M_CLK_EN BIT(2)
#define CLK_CTL0_GLOBAL_XTAL24M_CLK_EN BIT(4)
/* NOC_HDMI_CLK_EN (bit 10): bus_clk for the HDMIMIX internal NOC.
 * The HDMI TX controller (DWC HDMI TX, PA=0x32FD8000) is accessed via AXI
 * routed through this NOC. Without this bit the NOC has no clock and every
 * AXI transaction to the HDMI TX slave returns SLVERR. BLK_CTRL and the
 * PHY have dedicated APB ports that bypass the NOC, which is why they work
 * even when this bit is clear. (IMX8MPRM §13.11.3.1.5, field NOC_HDMI_CLK_EN)
 */
#define CLK_CTL0_NOC_HDMI_CLK_EN       BIT(10)

/* RTX_CLK_CTL1 level-2 HDMI TX / PHY APB gates, IMX8MPRM §13.11.3.1.6.
 * The DWC HDMI TX 2.0 uses the HPI (Host Programming Interface) as its
 * main register access port. TX_HPI_CLK_EN (bit13) must be enabled or the
 * HPI state machine has no clock and every APB read to the TX controller
 * (0x32FD8000) returns SLVERR. TX_APB_CLK_EN (bit14) alone is not enough.
 * All other functional clocks (SFR, GPA, etc.) are also enabled so the
 * IP is ready for sub-goal 2.x register access (phy_stat0, ih_i2cm, ...).
 */
#define CLK_CTL1_TX_HPI_CLK_EN     BIT(13)  /* HPI bus clock -- REQUIRED */
#define CLK_CTL1_TX_APB_CLK_EN     BIT(14)
#define CLK_CTL1_TX_CEC_CLK_EN     BIT(15)
#define CLK_CTL1_TX_ESM_CLK_EN     BIT(16)
#define CLK_CTL1_TX_GPA_CLK_EN     BIT(17)
#define CLK_CTL1_TX_SFR_CLK_EN     BIT(19)
#define CLK_CTL1_TX_PHY_APB_CLK_EN BIT(22)
#define CLK_CTL1_TX_PHY_INT_CLK_EN BIT(24)
#define CLK_CTL1_TX_VID_LINK_PIX_CLK_EN BIT(28)

/*
 * RTX_RESET_CTL0 active-low resets, IMX8MPRM §13.11.3.1.4. Reset value is
 * 0 -> every reset is asserted out of POR; writing 1 to RTX_RESET_CTL0_SET
 * deasserts the corresponding reset.
 */
#define RESET_CTL0_NOC_RESET_N         BIT(0)
#define RESET_CTL0_TX_RSTZ             BIT(10)
#define RESET_CTL0_TX_APBRSTZ          BIT(11)
#define RESET_CTL0_TX_PHY_PRESETN      BIT(12)
#define RESET_CTL0_VID_LINK_SLV_RESETN BIT(22)

/* Reg-name indices, mirroring binding order. */
enum {
	HDMI_REG_BLK_CTRL,
	HDMI_REG_HTX_PVI,
	HDMI_REG_HDMI_TX,
	HDMI_REG_HDMI_PHY,
	HDMI_REG_COUNT,
};

struct imx8mp_hdmi_config {
	DEVICE_MMIO_NAMED_ROM(blk_ctrl);
	DEVICE_MMIO_NAMED_ROM(htx_pvi);
	DEVICE_MMIO_NAMED_ROM(hdmi_tx);
	DEVICE_MMIO_NAMED_ROM(hdmi_phy);
	const struct pinctrl_dev_config *pcfg;
};

/* IOMUXC SW_MUX_CTL_PAD_HDMI_* registers (IMX8MPRM §8.2.4.140-143,
 * docs/subsystems/iomux_controller_iomuxc_8_2.md:L10341-L10531). For all
 * four HDMI pads MUX_MODE=000 selects ALT0 = HDMIMIX_HDMI_{SCL,SDA,CEC,HPD},
 * which routes the pad to the HDMI TX controller's own I2C/CEC/HPD logic.
 */
#define IOMUXC_BASE                       0x30330000UL
#define IOMUXC_REG_SIZE                   KB(64)
#define IOMUXC_SW_MUX_HDMI_DDC_SCL_OFF    0x240
#define IOMUXC_SW_MUX_HDMI_DDC_SDA_OFF    0x244
#define IOMUXC_SW_MUX_HDMI_CEC_OFF        0x248
#define IOMUXC_SW_MUX_HDMI_HPD_OFF        0x24c
#define IOMUXC_MUX_MODE_MASK              0x7UL

/* HDMI TX (Synopsys DesignWare) PHY status block (IMX8MPRM §13.9.3.6,
 * docs/subsystems/hdmi_tx_controller_13_9.md:L6685-L6950). Registers are
 * byte-addressed inside the 32 KiB TX window. phy_conf0.enhpdrxsense is
 * already set out of reset (phy_conf0 POR = 0x06), so HPD detection works
 * without further PHY programming.
 *   0x3004 phy_stat0  bit1 = HPD, bit0 = TX_PHY_LOCK, bits[7:4] = RXSENSE
 */
#define HDMI_TX_PHY_STAT0_OFF             0x3004
#define HDMI_TX_PHY_STAT0_HPD             BIT(1)

/* E-DDC I2C master (for EDID reads). IMX8MPRM §13.9.3.12 (L237109-L237175).
 * Base of EDDC register block is HDMI_TX + 0x7E00.
 * All registers are 8-bit; accessed via sys_read8/sys_write8.
 *
 * Interrupt status: ih_i2cm_stat0 at HDMI_TX + 0x5 (IMX8MPRM §13.9.3.2.7,
 * L228227-L228259). bit1 = done (W1C), bit0 = error (W1C).
 */
#define HDMI_TX_IH_I2CM_STAT0_OFF   0x0005  /* interrupt status (W1C) */
#define HDMI_TX_I2CM_BASE           0x7E00
#define I2CM_OFF(r)                 (HDMI_TX_I2CM_BASE + (r))
#define I2CM_SLAVE_OFF              I2CM_OFF(0x00)  /* slave addr (7-bit) */
#define I2CM_ADDRESS_OFF            I2CM_OFF(0x01)  /* register/byte addr */
#define I2CM_DATAI_OFF              I2CM_OFF(0x03)  /* read data (RO) */
#define I2CM_OPERATION_OFF          I2CM_OFF(0x04)  /* 0x01=single, 0x04=burst8 */
#define I2CM_INT_OFF                I2CM_OFF(0x05)  /* done int config (reset=0x40) */
#define I2CM_CTLINT_OFF             I2CM_OFF(0x06)  /* error int config */
#define I2CM_DIV_OFF                I2CM_OFF(0x07)  /* clock divisor (reset=0x0B) */
#define I2CM_SOFTRSTZ_OFF           I2CM_OFF(0x09)  /* soft reset: 0=rst, 1=run */
/* SS SCL timing defaults give ~100kHz with sfr_clk. */
#define I2CM_SS_SCL_HCNT_0_OFF     I2CM_OFF(0x0C)  /* reset=0x6C (108 cycles) */
#define I2CM_SS_SCL_LCNT_0_OFF     I2CM_OFF(0x0E)  /* reset=0x7F (127 cycles) */
/* 8-byte burst read result buffers (i2cm_read_buff0-7). */
#define I2CM_READ_BUFF_BASE         I2CM_OFF(0x20)

/* DDC slave addresses (EDID spec). */
#define DDC_ADDR_EDID               0x50    /* EDID slave */
#define EDID_BLOCK_LEN              128
#define EDID_HEADER_MAGIC0          0x00
#define EDID_HEADER_MAGIC1          0xFF

/* ih_i2cm_stat0 bits */
#define IH_I2CM_DONE                BIT(1)
#define IH_I2CM_ERROR               BIT(0)

/*
 * HDMI TX PHY register offsets (relative to hdmi_phy VA, base=0x32FDFF00).
 * All registers are 8-bit; accessed via sys_read8/sys_write8.
 * IMX8MPRM §13.10.3.1 (docs/subsystems/hdmi_tx_phy_13_10.md).
 */
#define HDMI_PHY_REG1_OFF    0x04  /* PMS_P pre-div, clock sel        reset=0xD1 */
#define HDMI_PHY_REG2_OFF    0x08  /* PMS_M main-div [7:0]             reset=0x3E */
#define HDMI_PHY_REG3_OFF    0x0C  /* PMS_S post-div, SDC_N            reset=0x15 */
#define HDMI_PHY_REG4_OFF    0x10  /* SDM_K_EN, SDM_LC denominator     reset=0xC0 */
#define HDMI_PHY_REG5_OFF    0x14  /* SDM_K_SIGN, SDM_K_CODE numerator reset=0x90 */
#define HDMI_PHY_REG27_OFF   0x6C  /* sub-block RESETn (all active-HIGH) reset=0x00 */
#define HDMI_PHY_REG28_OFF   0x70  /* power-down controls               reset=0x00 */
#define HDMI_PHY_REG33_OFF   0x84  /* MODE_SET_DONE (bit7)              reset=0x80 */
#define HDMI_PHY_REG34_OFF   0x88  /* status (RO)                       reset=0x00 */

/* PHY_REG27 sub-block RESETn bits (1=run, 0=in reset). POR=0x00. */
#define PHY_REG27_FLD_RSTN      BIT(7)
#define PHY_REG27_SDC_RSTN      BIT(6)
#define PHY_REG27_SDM_RSTN      BIT(5)
#define PHY_REG27_NDIV_RSTN     BIT(4)
#define PHY_REG27_MDIV_RSTN     BIT(3)
#define PHY_REG27_PDIV_RSTN     BIT(2)
#define PHY_REG27_AFC_PLL_RSTN  BIT(1)
#define PHY_REG27_AFC_INIT_RSTN BIT(0)

/* PHY_REG34 status bits (read-only). */
#define PHY_REG34_PHY_READY   BIT(7)
#define PHY_REG34_PLL_LOCK    BIT(6)
#define PHY_REG34_PHY_CLK_RDY BIT(5)
#define PHY_REG34_AFC_DONE    BIT(4)

struct imx8mp_hdmi_data {
	DEVICE_MMIO_NAMED_RAM(blk_ctrl);
	DEVICE_MMIO_NAMED_RAM(htx_pvi);
	DEVICE_MMIO_NAMED_RAM(hdmi_tx);
	DEVICE_MMIO_NAMED_RAM(hdmi_phy);
};

#define DEV_DATA(dev) ((struct imx8mp_hdmi_data *)(dev)->data)
#define DEV_CFG(dev)  ((const struct imx8mp_hdmi_config *)(dev)->config)

static int imx8mp_hdmi_gpc_power_up(void)
{
	mm_reg_t gpc;
	uint32_t val;

	device_map(&gpc, GPC_BASE, GPC_REG_SIZE, K_MEM_CACHE_NONE);

	val = sys_read32(gpc + GPC_PU_PGC_SW_PUP_REQ_OFF);
	sys_write32(val | GPC_HDMIMIX_BIT | GPC_HDMI_PHY_BIT,
		    gpc + GPC_PU_PGC_SW_PUP_REQ_OFF);

	/* Bits self-clear when the PGC accepts the request. Poll briefly. */
	for (int i = 0; i < 1000; i++) {
		val = sys_read32(gpc + GPC_PU_PGC_SW_PUP_REQ_OFF);
		if ((val & (GPC_HDMIMIX_BIT | GPC_HDMI_PHY_BIT)) == 0) {
			LOG_INF("GPC power-up acknowledged in %d iterations", i);
			return 0;
		}
		k_busy_wait(10);
	}

	LOG_ERR("GPC power-up timed out (SW_PUP_REQ=0x%08x)", val);
	return -ETIMEDOUT;
}

/*
 * Sub-goals 1.3 + 1.4: ungate HDMI BLK_CTRL clocks and deassert per-block
 * resets so that AXI accesses to the HDMIMIX slaves no longer SLVERR.
 *
 * BLK_CTRL itself is always accessible (IMX8MPRM §13.11.3.1.5 note on
 * GLOBAL_APB_CLK_EN: "blk_ctl is required to be accessible at all times"),
 * so we can hit RTX_CLK_CTL0/1 and RTX_RESET_CTL0 directly via the mapped
 * VA without any preconditions besides GPC power-up.
 *
 * CCM clock roots HDMI_APB_CLK_ROOT (0x8B00) and HDMI_AXI_CLK_ROOT (0x8B80)
 * have ENABLE=1 in their POR-reset value but U-Boot may gate them. We enable
 * them explicitly before touching BLK_CTRL clocks.
 */
static int imx8mp_hdmi_clocks_reset(const struct device *dev)
{
	mm_reg_t blk_ctrl = DEVICE_MMIO_NAMED_GET(dev, blk_ctrl);
	mm_reg_t ccm;
	uint32_t tx_ctrl0, val;

	/* (a-pre) Ensure CCM HDMI clock roots are ungated. HDMI TX APB slave
	 * (0x32FD8000) will SLVERR if either root is gated at the CCM level,
	 * regardless of BLK_CTRL state. We set ENABLE (bit 28) without touching
	 * the source MUX or divider fields.
	 */
	device_map(&ccm, CCM_BASE, CCM_REG_SIZE, K_MEM_CACHE_NONE);
	val = sys_read32(ccm + CCM_HDMI_APB_CLK_ROOT_OFF);
	LOG_INF("CCM HDMI_APB_CLK_ROOT before = 0x%08x", val);
	sys_write32(val | CCM_CLK_ROOT_ENABLE, ccm + CCM_HDMI_APB_CLK_ROOT_OFF);
	val = sys_read32(ccm + CCM_HDMI_AXI_CLK_ROOT_OFF);
	LOG_INF("CCM HDMI_AXI_CLK_ROOT before = 0x%08x", val);
	sys_write32(val | CCM_CLK_ROOT_ENABLE, ccm + CCM_HDMI_AXI_CLK_ROOT_OFF);
	k_busy_wait(10);

	/* (a) Sanity: BLK_CTRL is always accessible. TX_CONTROL0 reset = 0x18. */
	tx_ctrl0 = sys_read32(blk_ctrl + BLK_CTRL_TX_CONTROL0);
	LOG_INF("BLK_CTRL TX_CONTROL0 = 0x%08x (expect 0x%08x)",
		tx_ctrl0, (uint32_t)BLK_CTRL_TX_CONTROL0_RESET);

	/* (b) Level-1 global clocks: APB + bus + 266MHz ref + XTAL24M + NOC. */
	sys_write32(CLK_CTL0_GLOBAL_APB_CLK_EN |
		    CLK_CTL0_GLOBAL_B_CLK_EN |
		    CLK_CTL0_GLOBAL_REF266M_CLK_EN |
		    CLK_CTL0_GLOBAL_XTAL24M_CLK_EN |
		    CLK_CTL0_NOC_HDMI_CLK_EN,
		    blk_ctrl + BLK_CTRL_RTX_CLK_CTL0_SET);

	/* (c) Level-2 per-block APB gates.
	 * TX_HPI_CLK_EN (bit13): REQUIRED -- clocks the DWC HPI state machine.
	 *   Without it any APB read to the TX controller returns SLVERR.
	 * TX_APB_CLK_EN (bit14): APB bus clock for TX IP synchroniser.
	 * TX_SFR/GPA/ESM/CEC (bits 19,17,16,15): functional clocks; safe to
	 *   enable early since their sources are gated if not configured.
	 * TX_PHY_APB + TX_PHY_INT: PHY CSR access (proven required by goal 1.4).
	 * TX_VID_LINK_PIX: PVI pixel-clock path.
	 */
	sys_write32(CLK_CTL1_TX_HPI_CLK_EN |
		    CLK_CTL1_TX_APB_CLK_EN |
		    CLK_CTL1_TX_CEC_CLK_EN |
		    CLK_CTL1_TX_ESM_CLK_EN |
		    CLK_CTL1_TX_GPA_CLK_EN |
		    CLK_CTL1_TX_SFR_CLK_EN |
		    CLK_CTL1_TX_PHY_APB_CLK_EN |
		    CLK_CTL1_TX_PHY_INT_CLK_EN |
		    CLK_CTL1_TX_VID_LINK_PIX_CLK_EN,
		    blk_ctrl + BLK_CTRL_RTX_CLK_CTL1_SET);

	/* Give the gates a moment to propagate before deasserting resets. */
	k_busy_wait(1);

	/* (d) Deassert NOC, TX controller, TX PHY APB and PVI slave resets. */
	sys_write32(RESET_CTL0_NOC_RESET_N |
		    RESET_CTL0_TX_RSTZ |
		    RESET_CTL0_TX_APBRSTZ |
		    RESET_CTL0_TX_PHY_PRESETN |
		    RESET_CTL0_VID_LINK_SLV_RESETN,
		    blk_ctrl + BLK_CTRL_RTX_RESET_CTL0_SET);

	/* Resets take a few cycles to propagate to each block. */
	k_busy_wait(10);

	/* (e) Power up the HDMI TX PHY: clear TX_CONTROL0.TX_PHY_PDOWN (bit 3).
	 * POR value of TX_CONTROL0 = 0x18 has bit 3 set -> PHY held in PDOWN
	 * and its CSR slave SLVERRs. We clear it via TX_CONTROL0_CLR (0x208).
	 */
	sys_write32(TX_CONTROL0_TX_PHY_PDOWN, blk_ctrl + BLK_CTRL_TX_CONTROL0_CLR);
	k_busy_wait(10);

	return 0;
}

/*
 * Sub-goal 1.4 verification: read known reset values back from the slaves
 * that only need an APB clock + reset deassert (BLK_CTRL itself and the
 * HDMI_PHY, which lives in its own power/clock domain).
 *
 * HTX_PVI is deliberately NOT read here: per IMX8MPRM §13.13.1.1 figure
 * 13-85 (docs/source/IMX8MPRM.md:L241611-L241619) HTX_PVI is a single-
 * clock-domain block clocked by htx_p_clk (pixel clock). Its CSR slave
 * port therefore needs a live pixel-clock source -- which in turn needs
 * either the HDMI_PHY's PIXEL_CLK_OUT (sub-goal 2.x) or VIDEO_PLL via
 * HTX_PIPE_CLK_SEL/HTXPHY_CLK_SEL. That setup belongs to sub-goal 1.5.
 *
 * The previous attempt to read HTX_PVI+0x0 here SLVERR'd with
 *   ESR=0x96000210 / FAR=0xC07FB000 (mapped VA of 0x32FC_4000).
 */
static void imx8mp_hdmi_slave_readback(const struct device *dev)
{
	mm_reg_t blk_ctrl = DEVICE_MMIO_NAMED_GET(dev, blk_ctrl);
	mm_reg_t hdmi_tx  = DEVICE_MMIO_NAMED_GET(dev, hdmi_tx);
	mm_reg_t hdmi_phy = DEVICE_MMIO_NAMED_GET(dev, hdmi_phy);
	uint8_t phy_reg1;

	phy_reg1 = sys_read8(hdmi_phy + 0x4);
	LOG_INF("HDMI_PHY_REG1   = 0x%02x       (expect 0x%02x)",
		phy_reg1, (uint32_t)HDMI_PHY_REG1_RESET_VAL);

	/* Diagnostic: read CCM root state after enabling */
	{
		mm_reg_t ccm;

		device_map(&ccm, CCM_BASE, CCM_REG_SIZE, K_MEM_CACHE_NONE);
		LOG_INF("CCM HDMI_APB_CLK_ROOT after= 0x%08x",
			sys_read32(ccm + CCM_HDMI_APB_CLK_ROOT_OFF));
		LOG_INF("CCM HDMI_AXI_CLK_ROOT after= 0x%08x",
			sys_read32(ccm + CCM_HDMI_AXI_CLK_ROOT_OFF));
	}

	/* Diagnostic: read RTX_CLK_CTL0/1 and RESET_CTL0 to confirm state */
	LOG_INF("BLK_CTRL CLK_CTL0  = 0x%08x", sys_read32(blk_ctrl + 0x40));
	LOG_INF("BLK_CTRL CLK_CTL1  = 0x%08x", sys_read32(blk_ctrl + 0x50));
	LOG_INF("BLK_CTRL RESET_CTL0= 0x%08x", sys_read32(blk_ctrl + 0x20));

	/* Diagnostic: try reading HDMI TX offset 0 from init context. */
	LOG_INF("HDMI_TX[0] (init ctx) = 0x%08x", sys_read32(hdmi_tx + 0));
}

/* Sub-goal 2.1 verification: confirm pinctrl actually muxed the 4 HDMI
 * pads to ALT0 (MUX_MODE field [2:0] = 0). */
static void imx8mp_hdmi_pinmux_readback(void)
{
	mm_reg_t iomuxc;
	struct { const char *name; uint32_t off; } pins[] = {
		{"HDMI_DDC_SCL", IOMUXC_SW_MUX_HDMI_DDC_SCL_OFF},
		{"HDMI_DDC_SDA", IOMUXC_SW_MUX_HDMI_DDC_SDA_OFF},
		{"HDMI_CEC    ", IOMUXC_SW_MUX_HDMI_CEC_OFF},
		{"HDMI_HPD    ", IOMUXC_SW_MUX_HDMI_HPD_OFF},
	};

	device_map(&iomuxc, IOMUXC_BASE, IOMUXC_REG_SIZE, K_MEM_CACHE_NONE);

	for (size_t i = 0; i < ARRAY_SIZE(pins); i++) {
		uint32_t val = sys_read32(iomuxc + pins[i].off);
		uint32_t mux = val & IOMUXC_MUX_MODE_MASK;
		LOG_INF("IOMUXC %s MUX_CTL = 0x%08x  MUX_MODE=%u (%s)",
			pins[i].name, val, mux,
			mux == 0 ? "ALT0 HDMIMIX OK" : "!! NOT ALT0");
	}
}

/*
 * Sub-goal 3.1: configure the HDMI TX PHY PLL for 148.5 MHz TMDS (1080p60)
 * and wait for PLL_LOCK.
 *
 * PLL formula (integer + fractional-K):
 *   F_PLL = F_REF × (M + K/LC) / (P × S)
 * With F_REF=24 MHz, P=1 (REG1[3:0]=1), M=62 (REG2=0x3E),
 *      S=2 (REG3[7:4]=0001), SDM_K_EN=1 (REG4[7]=1), LC=64 (REG4[6:0]=64):
 *   K=0:   24 × 62 / 2 = 744.0 MHz → TMDS = 148.8 MHz (high by 0.2%)
 *   K=-8:  24 × (62 - 8/64) / 2 = 742.5 MHz → TMDS = 148.5 MHz  ✓
 *   K=-16: 24 × (62 - 16/64) / 2 = 741.0 MHz → TMDS = 148.2 MHz (default)
 *
 * Only REG5 differs from reset: SDM_K_SIGN=1(neg), SDM_K_CODE=8 → 0x88.
 * (Default 0x90 = K=-16 → 148.2 MHz; correct value is 0x88 = K=-8.)
 *
 * After setting dividers, de-assert all sub-block resets via REG27=0xFF,
 * then poll REG34 for PLL_LOCK (bit6) within 5 ms.
 */
static int imx8mp_hdmi_phy_pll_init(const struct device *dev)
{
	mm_reg_t hdmi_phy = DEVICE_MMIO_NAMED_GET(dev, hdmi_phy);
	uint8_t val;
	int i;

	/* Read back current PLL config for diagnostics. */
	printk("[hdmi_phy] REG1=0x%02x REG2=0x%02x REG3=0x%02x REG4=0x%02x REG5=0x%02x\n",
	       sys_read8(hdmi_phy + HDMI_PHY_REG1_OFF),
	       sys_read8(hdmi_phy + HDMI_PHY_REG2_OFF),
	       sys_read8(hdmi_phy + HDMI_PHY_REG3_OFF),
	       sys_read8(hdmi_phy + HDMI_PHY_REG4_OFF),
	       sys_read8(hdmi_phy + HDMI_PHY_REG5_OFF));

	/*
	 * Correct fractional K for exact 148.5 MHz:
	 * REG5 = 0x88: SDM_K_SIGN=1(negative), SDM_K_CODE=8
	 * This gives K/LC = -8/64 = -0.125, F_PLL = 742.5 MHz.
	 */
	sys_write8(0x88, hdmi_phy + HDMI_PHY_REG5_OFF);

	/*
	 * De-assert all PHY PLL sub-block resets.
	 * REG27 POR=0x00 (all in reset). Write 0xFF to release:
	 * FLD(7)+SDC(6)+SDM(5)+NDIV(4)+MDIV(3)+PDIV(2)+AFC_PLL(1)+AFC_INIT(0).
	 */
	sys_write8(0xFF, hdmi_phy + HDMI_PHY_REG27_OFF);

	/*
	 * Poll REG34 for PLL_LOCK (bit6).
	 * AFC typically completes within 500 µs; allow up to 5 ms (500 × 10 µs).
	 */
	for (i = 0; i < 500; i++) {
		val = sys_read8(hdmi_phy + HDMI_PHY_REG34_OFF);
		if (val & PHY_REG34_PLL_LOCK) {
			break;
		}
		k_busy_wait(10);
	}

	printk("[hdmi_phy] REG34=0x%02x after %d×10µs:"
	       " PHY_READY=%d PLL_LOCK=%d PHY_CLK_RDY=%d AFC_DONE=%d\n",
	       val, i,
	       !!(val & PHY_REG34_PHY_READY),
	       !!(val & PHY_REG34_PLL_LOCK),
	       !!(val & PHY_REG34_PHY_CLK_RDY),
	       !!(val & PHY_REG34_AFC_DONE));

	if (!(val & PHY_REG34_PLL_LOCK)) {
		LOG_ERR("Goal 3.1 FAIL: PHY PLL did not lock (REG34=0x%02x)", val);
		return -ETIMEDOUT;
	}

	printk("[hdmi_phy] Goal 3.1 PASS: PLL locked (148.5 MHz TMDS, 1080p60)\n");
	return 0;
}

/* Sub-goal 2.2: HPD detect via phy_stat0[1]. Polled in a low-priority
 * thread so cable plug/unplug events are visible without interrupts.
 */
#define HDMI_HPD_POLL_MS         250
#define HDMI_HPD_THREAD_STACKSZ  4096
#define HDMI_HPD_THREAD_PRIO     10

static K_THREAD_STACK_DEFINE(hdmi_hpd_stack, HDMI_HPD_THREAD_STACKSZ);
static struct k_thread hdmi_hpd_thread;

static bool imx8mp_hdmi_hpd_state(const struct device *dev)
{
	mm_reg_t hdmi_tx = DEVICE_MMIO_NAMED_GET(dev, hdmi_tx);
	uint32_t stat0;

	/* phy_stat0 is an 8-bit register at PHY config base+4h (=HDMI_TX+3004h).
	 * Read as 32-bit to avoid APB byte-enable issues; HPD is bit 1.
	 * (IMX8MPRM §13.9.3.6.6, hdmi_tx_controller_13_9.md:L6902-L6950)
	 */
	stat0 = sys_read32(hdmi_tx + HDMI_TX_PHY_STAT0_OFF);
	return (stat0 & HDMI_TX_PHY_STAT0_HPD) != 0;
}

/*
 * Sub-goal 2.3: Read 128-byte EDID block 0 from HDMI sink via the DWC HDMI TX
 * built-in E-DDC I2C master (IMX8MPRM §13.9.3.12, L237109-L237175).
 *
 * Protocol:
 *  1. Soft-reset + configure I2C master.
 *  2. For each 8-byte chunk (16 chunks × 8 = 128 bytes):
 *       a. Write i2cm_address = byte offset.
 *       b. Write i2cm_operation = 0x04 (burst-8 read / rd8).
 *       c. Wait 3 ms for I2C transaction to complete.
 *          NOTE: ih_i2cm_stat0 is level-triggered, not W1C-pollable; use
 *          a fixed delay instead. At 100 kHz, 8 bytes ≈ 1 ms; 3 ms gives
 *          3× safety margin.
 *       d. Copy i2cm_read_buff0-7 to output buffer.
 *  3. Verify EDID checksum (sum of all 128 bytes must be 0x00).
 *
 * Returns 0 on success, negative errno on error.
 */
static int imx8mp_hdmi_edid_read(const struct device *dev, uint8_t *buf)
{
	mm_reg_t hdmi_tx = DEVICE_MMIO_NAMED_GET(dev, hdmi_tx);
	int i;

	/* (a) Soft-reset the I2C master: write 0 then 1 to i2cm_softrstz. */
	sys_write8(0x00, hdmi_tx + I2CM_SOFTRSTZ_OFF);
	k_busy_wait(100);
	sys_write8(0x01, hdmi_tx + I2CM_SOFTRSTZ_OFF);
	k_busy_wait(100);

	/* (b) Clear i2cm_int and i2cm_ctlint masks. */
	sys_write8(0x00, hdmi_tx + I2CM_INT_OFF);
	sys_write8(0x00, hdmi_tx + I2CM_CTLINT_OFF);

	/* (c) Set DDC slave address = 0x50. */
	sys_write8(DDC_ADDR_EDID, hdmi_tx + I2CM_SLAVE_OFF);

	/*
	 * (d) Read 128 bytes as 16 × 8-byte bursts.
	 *
	 * ih_i2cm_stat0 is level-triggered: the done bit stays asserted as
	 * long as the I2C master holds its internal done signal high, so
	 * W1C polling does not work reliably. Use a fixed delay instead.
	 * At 100 kHz standard mode, 8 bytes (≈100 I2C bits) takes ~1 ms;
	 * 3 ms gives 3× safety margin and works for fast-mode too.
	 */
	for (i = 0; i < EDID_BLOCK_LEN; i += 8) {
		sys_write8((uint8_t)i, hdmi_tx + I2CM_ADDRESS_OFF);
		sys_write8(0x04, hdmi_tx + I2CM_OPERATION_OFF); /* rd8 */

		k_busy_wait(3000); /* 3 ms — let I2C transaction complete */

		/* Copy 8 burst-read bytes from i2cm_read_buff0-7. */
		for (int j = 0; j < 8; j++) {
			buf[i + j] = sys_read8(hdmi_tx + I2CM_READ_BUFF_BASE + j);
		}

		/* Diagnostic: first 2 bursts (header bytes). */
		if (i < 16) {
			printk("[hdmi_edid] byte %3d: %02x %02x %02x %02x %02x %02x %02x %02x\n",
			       i,
			       buf[i+0], buf[i+1], buf[i+2], buf[i+3],
			       buf[i+4], buf[i+5], buf[i+6], buf[i+7]);
		}
	}

	/* (e) Verify EDID checksum: sum of all 128 bytes must be 0x00. */
	uint8_t sum = 0;

	for (i = 0; i < EDID_BLOCK_LEN; i++) {
		sum += buf[i];
	}
	if (sum != 0) {
		LOG_WRN("EDID checksum bad (sum=0x%02x)", sum);
		return -EBADMSG;
	}

	return 0;
}

static void imx8mp_hdmi_hpd_thread_fn(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	uint8_t edid[EDID_BLOCK_LEN];
	bool last;
	int ret;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("[hdmi_hpd] thread started\n");

	last = imx8mp_hdmi_hpd_state(dev);
	LOG_INF("HPD initial state: %s", last ? "on (cable connected)" : "off");

	if (last) {
		ret = imx8mp_hdmi_edid_read(dev, edid);
		if (ret == 0) {
			/* Goal 2.4: decode PnP manufacturer ID (bytes 8-9).
			 * Two bytes in big-endian; each letter = 5 bits, A=1..Z=26.
			 */
			uint16_t mfr_raw = ((uint16_t)edid[8] << 8) | edid[9];
			char mfr[4] = {
				'A' + ((mfr_raw >> 10) & 0x1F) - 1,
				'A' + ((mfr_raw >>  5) & 0x1F) - 1,
				'A' + ((mfr_raw >>  0) & 0x1F) - 1,
				'\0'
			};
			uint16_t product = ((uint16_t)edid[11] << 8) | edid[10];

			/* Goal 2.4: parse preferred timing from first DTD
			 * (Detailed Timing Descriptor, EDID bytes 54-71).
			 * Pixel clock: bytes 54-55, little-endian, unit = 10 kHz.
			 */
			uint32_t pclk_10khz = ((uint16_t)edid[55] << 8) | edid[54];
			uint16_t h_active   = ((uint16_t)(edid[58] >> 4) << 8) | edid[56];
			uint16_t h_blank    = ((uint16_t)(edid[58] & 0x0F) << 8) | edid[57];
			uint16_t v_active   = ((uint16_t)(edid[61] >> 4) << 8) | edid[59];
			uint16_t v_blank    = ((uint16_t)(edid[61] & 0x0F) << 8) | edid[60];
			uint16_t h_total    = h_active + h_blank;
			uint16_t v_total    = v_active + v_blank;
			/* refresh = pclk_10khz * 10000 / (h_total * v_total) */
			uint32_t refresh    = (h_total && v_total)
				? (pclk_10khz * 10000U / ((uint32_t)h_total * v_total))
				: 0;

			printk("[hdmi_hpd] Goal 2.3 PASS: EDID read OK\n");
			printk("[hdmi_hpd] Manufacturer : %s  Product: 0x%04x\n",
			       mfr, product);
			printk("[hdmi_hpd] Goal 2.4: preferred timing %u x %u @ %u Hz"
			       "  (pclk=%u kHz)\n",
			       h_active, v_active, refresh, pclk_10khz * 10);
		} else {
			printk("[hdmi_hpd] Goal 2.3 FAIL: EDID read error %d\n", ret);
		}
	}

	while (true) {
		k_msleep(HDMI_HPD_POLL_MS);
		bool now = imx8mp_hdmi_hpd_state(dev);

		if (now != last) {
			mm_reg_t hdmi_tx = DEVICE_MMIO_NAMED_GET(dev, hdmi_tx);
			uint32_t stat0 = sys_read32(hdmi_tx + HDMI_TX_PHY_STAT0_OFF);

			printk("\n[hdmi_hpd] *** HDMI CABLE %s *** (phy_stat0=0x%02x)\n",
			       now ? "CONNECTED" : "DISCONNECTED", stat0 & 0xff);
			LOG_INF("HPD event: %s", now ? "on (cable connected)" : "off (cable removed)");
			last = now;

			if (now) {
				ret = imx8mp_hdmi_edid_read(dev, edid);
				if (ret == 0) {
					uint16_t mfr_raw = ((uint16_t)edid[8] << 8) | edid[9];
					char mfr[4] = {
						'A' + ((mfr_raw >> 10) & 0x1F) - 1,
						'A' + ((mfr_raw >>  5) & 0x1F) - 1,
						'A' + ((mfr_raw >>  0) & 0x1F) - 1,
						'\0'
					};
					uint16_t product = ((uint16_t)edid[11] << 8) | edid[10];
					uint32_t pclk_10khz = ((uint16_t)edid[55] << 8) | edid[54];
					uint16_t h_active   = ((uint16_t)(edid[58] >> 4) << 8) | edid[56];
					uint16_t v_active   = ((uint16_t)(edid[61] >> 4) << 8) | edid[59];
					uint16_t h_blank    = ((uint16_t)(edid[58] & 0x0F) << 8) | edid[57];
					uint16_t v_blank    = ((uint16_t)(edid[61] & 0x0F) << 8) | edid[60];
					uint32_t refresh    = (h_active + h_blank) && (v_active + v_blank)
						? (pclk_10khz * 10000U /
						   ((uint32_t)(h_active + h_blank) * (v_active + v_blank)))
						: 0;

					printk("[hdmi_hpd] EDID re-read OK after reconnect\n");
					printk("[hdmi_hpd] Manufacturer : %s  Product: 0x%04x\n",
					       mfr, product);
					printk("[hdmi_hpd] Preferred timing: %u x %u @ %u Hz"
					       "  (pclk=%u kHz)\n",
					       h_active, v_active, refresh, pclk_10khz * 10);
				} else {
					printk("[hdmi_hpd] EDID re-read error %d\n", ret);
				}
			}
		}
	}
}

static int imx8mp_hdmi_init(const struct device *dev)
{
	const struct imx8mp_hdmi_config *cfg = DEV_CFG(dev);
	int ret;

	DEVICE_MMIO_NAMED_MAP(dev, blk_ctrl, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, htx_pvi, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, hdmi_tx, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, hdmi_phy, K_MEM_CACHE_NONE);

	/* Sub-goal 2.1: claim HDMI_DDC_SCL/SDA/CEC/HPD pads in ALT0 mode so
	 * the HDMI TX controller's built-in I2C master + HPD logic are wired
	 * to the SoC pins (board DTS puts them in ALT5/GPIO by default).
	 */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl_apply_state failed: %d", ret);
		return ret;
	}

	ret = imx8mp_hdmi_gpc_power_up();
	if (ret) {
		return ret;
	}

	ret = imx8mp_hdmi_clocks_reset(dev);
	if (ret) {
		return ret;
	}

	imx8mp_hdmi_slave_readback(dev);
	imx8mp_hdmi_pinmux_readback();

	/* Sub-goal 3.1: configure PHY PLL for 148.5 MHz TMDS. */
	ret = imx8mp_hdmi_phy_pll_init(dev);
	if (ret) {
		return ret;
	}

	/* Sub-goal 2.2: kick off HPD polling thread. */
	k_thread_create(&hdmi_hpd_thread, hdmi_hpd_stack,
			K_THREAD_STACK_SIZEOF(hdmi_hpd_stack),
			imx8mp_hdmi_hpd_thread_fn, (void *)dev, NULL, NULL,
			HDMI_HPD_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&hdmi_hpd_thread, "hdmi_hpd");

	LOG_INF("imx8mp_hdmi: probe ok (phase 0+1, goals 1.1-2.2)");
	return 0;
}

static int imx8mp_hdmi_write(const struct device *dev, const uint16_t x,
			     const uint16_t y,
			     const struct display_buffer_descriptor *desc,
			     const void *buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(x);
	ARG_UNUSED(y);
	ARG_UNUSED(desc);
	ARG_UNUSED(buf);
	return -ENOSYS;
}

static void imx8mp_hdmi_get_capabilities(const struct device *dev,
					 struct display_capabilities *caps)
{
	ARG_UNUSED(dev);
	memset(caps, 0, sizeof(*caps));
}

static int imx8mp_hdmi_set_pixel_format(const struct device *dev,
					const enum display_pixel_format pf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pf);
	return -ENOSYS;
}

static DEVICE_API(display, imx8mp_hdmi_api) = {
	.write = imx8mp_hdmi_write,
	.get_capabilities = imx8mp_hdmi_get_capabilities,
	.set_pixel_format = imx8mp_hdmi_set_pixel_format,
};

#define IMX8MP_HDMI_INIT(n)                                                                        \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct imx8mp_hdmi_config imx8mp_hdmi_config_##n = {                          \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(blk_ctrl, DT_DRV_INST(n)),                      \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(htx_pvi,  DT_DRV_INST(n)),                      \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(hdmi_tx,  DT_DRV_INST(n)),                      \
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(hdmi_phy, DT_DRV_INST(n)),                      \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
	};                                                                                         \
	static struct imx8mp_hdmi_data imx8mp_hdmi_data_##n;                                       \
	DEVICE_DT_INST_DEFINE(n, imx8mp_hdmi_init, NULL,                                           \
			      &imx8mp_hdmi_data_##n, &imx8mp_hdmi_config_##n,                      \
			      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,                           \
			      &imx8mp_hdmi_api);

DT_INST_FOREACH_STATUS_OKAY(IMX8MP_HDMI_INIT)
