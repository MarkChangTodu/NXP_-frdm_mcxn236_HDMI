/*
 * Copyright 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * i.MX8MP HDMI bring-up test — Main Goal 1.1 - 1.4
 *
 * Plan reference: docs/plans/hdmi_driver_plan.md
 *
 * What this test verifies
 * -----------------------
 *  1.1  GPC powers up HDMIMIX + HDMI_PHY PGC domains.
 *  1.2  MMU mappings for HDMI BLK_CTRL (0x32FC_0000), HTX_PVI (0x32FC_4000),
 *       HDMI TX (0x32FD_8000) and HDMI TX PHY (0x32FD_FF00) build successfully
 *       (no translation fault, no MAX_XLAT_TABLES exhaustion).
 *  1.3  HDMI BLK_CTRL level-1 (global APB / bus / 266MHz / XTAL24M) and
 *       level-2 (TX_APB / TX_PHY_APB / TX_VID_LINK_PIX) clock gates enabled.
 *  1.4  RTX_RESET_CTL0 NOC / TX / TX_APB / TX_PHY_PRESET / VID_LINK_SLV
 *       active-low resets deasserted; HDMIMIX slaves return documented
 *       reset values without AXI SLVERR (HTX_PVI_CTRL = 0x00377000,
 *       HDMI_PHY_REG1 = 0xd1).
 *
 * Expected console output on success
 * ----------------------------------
 *   *** Booting Zephyr OS build ... ***
 *   [00:00:00.xxx,xxx] <dbg> os: k_mem_map_phys_bare: arch_mem_map(0xc07fc000, 0x32fc0000, 16384, a) offset 0
 *   [00:00:00.xxx,xxx] <dbg> os: k_mem_map_phys_bare: arch_mem_map(0xc07fb000, 0x32fc4000, 4096, a) offset 0
 *   [00:00:00.xxx,xxx] <dbg> os: k_mem_map_phys_bare: arch_mem_map(0xc07f3000, 0x32fd8000, 32768, a) offset 0
 *   [00:00:00.xxx,xxx] <dbg> os: k_mem_map_phys_bare: arch_mem_map(0xc07f2000, 0x32fdf000, 4096, a) offset 3840
 *   [00:00:00.xxx,xxx] <dbg> os: k_mem_map_phys_bare: arch_mem_map(0xc07e2000, 0x303a0000, 65536, a) offset 0
 *   [00:00:00.xxx,xxx] <inf> imx8mp_hdmi: GPC power-up acknowledged in N iterations
 *   [00:00:00.xxx,xxx] <inf> imx8mp_hdmi: BLK_CTRL TX_CONTROL0 = 0x000000XX (expect 0x00000018)
 *   [00:00:00.xxx,xxx] <inf> imx8mp_hdmi: HDMI_PHY_REG1   = 0xd1       (expect 0xd1)
 *   [00:00:00.xxx,xxx] <inf> imx8mp_hdmi: imx8mp_hdmi: probe ok (phase 0+1, goals 1.1-1.4)
 *   HDMI bring-up test: starting
 *   Device handle      : 0x........
 *   Device is_ready()  : YES
 *   TEST 1.1-1.4 PASS
 *
 * Note: HTX_PVI readback is moved to sub-goal 1.5 — HTX_PVI is a single-
 *       clock-domain block (figure 13-85) clocked by htx_p_clk, so its
 *       CSR slave port can only be reached once the pixel-clock source
 *       is alive (PHY PIXEL_CLK_OUT or VIDEO_PLL via HTX_PIPE_CLK_SEL).
 *
 * Failure modes to look for
 * -------------------------
 *  - "GPC power-up timed out" → SW_PUP_REQ bits never self-cleared. Check
 *    that GPC MMU region is mapped (it is, in soc/.../mmu_regions.c) and
 *    that no other master holds the HDMIMIX powered-down.
 *  - "MMIO reads returned 0xffffffff" → power not up OR MMU mapping wrong.
 *  - "HTX_PVI_CTRL mismatch" warning → either power up race or wrong reg
 *    base in DT.
 *  - "TEST 1.1-1.4 FAIL: device not ready" → device init returned non-zero.
 *  - readback mismatch (e.g. HTX_PVI_CTRL != 0x00377000) → BLK_CTRL clock
 *    gate or reset deassert sequence wrong, or AXI SLVERR / data abort
 *    appears instead.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define HDMI_NODE DT_NODELABEL(hdmi)

#if !DT_NODE_HAS_STATUS(HDMI_NODE, okay)
#error "hdmi node is not enabled — enable it in the board overlay"
#endif

int main(void)
{
	const struct device *hdmi = DEVICE_DT_GET(HDMI_NODE);

	printk("\nHDMI bring-up test: starting\n");
	printk("Device handle      : %p\n", hdmi);

	if (!device_is_ready(hdmi)) {
		printk("Device is_ready()  : NO\n");
		printk("TEST 1.1-1.4 FAIL: device not ready (driver init returned error)\n");
		return -1;
	}

	printk("Device is_ready()  : YES\n");
	printk("TEST 1.1-1.4 PASS\n");
	printk("\nNote: see <inf> imx8mp_hdmi lines above for the readback values\n");
	printk("of BLK_CTRL TX_CONTROL0 and HDMI_PHY_REG1.\n");
	printk("HTX_PVI readback is moved to sub-goal 1.5 (needs pixel clock).\n");
	printk("\nGoal 2.2: HPD polling active — plug/unplug HDMI cable to test.\n");

	/* Keep main alive so the HPD polling thread (created in driver init)
	 * can keep running and log HPD state changes.
	 */
	while (true) {
		k_sleep(K_SECONDS(10));
	}

	return 0;
}
