/*
 * Copyright 2020-2022,2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm64/arm_mmu.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

static const struct arm_mmu_region mmu_regions[] = {

	MMU_REGION_FLAT_ENTRY("GIC",
			      DT_REG_ADDR_BY_IDX(DT_NODELABEL(gic), 0),
			      DT_REG_SIZE_BY_IDX(DT_NODELABEL(gic), 0),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_NS),

	MMU_REGION_FLAT_ENTRY("GIC",
			      DT_REG_ADDR_BY_IDX(DT_NODELABEL(gic), 1),
			      DT_REG_SIZE_BY_IDX(DT_NODELABEL(gic), 1),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_NS),

	MMU_REGION_FLAT_ENTRY("CCM",
			      DT_REG_ADDR(DT_NODELABEL(ccm)),
			      DT_REG_SIZE(DT_NODELABEL(ccm)),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_NS),

	MMU_REGION_FLAT_ENTRY("ANA_PLL",
			      DT_REG_ADDR(DT_NODELABEL(ana_pll)),
			      DT_REG_SIZE(DT_NODELABEL(ana_pll)),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_NS),

	MMU_REGION_FLAT_ENTRY("IOMUXC",
			      DT_REG_ADDR(DT_NODELABEL(iomuxc)),
			      DT_REG_SIZE(DT_NODELABEL(iomuxc)),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_NS),

	MMU_REGION_FLAT_ENTRY("RDC",
			      DT_REG_ADDR(DT_NODELABEL(rdc)),
			      DT_REG_SIZE(DT_NODELABEL(rdc)),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_NS),

	/* GPC: needed for raw HDMIMIX/HDMI_PHY power-up before driver init.
	 * No DT node — accessed only by HDMI driver bring-up code.
	 */
	MMU_REGION_FLAT_ENTRY("GPC",
			      0x303a0000, KB(64),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_NS),

	/* HDMIMIX: covers HDMI BLK_CTRL (0x32FC_0000) through HDMI TX PHY
	 * (0x32FD_FFFF). 128KB single mapping serves all four HDMI MMIO regions.
	 */
	MMU_REGION_FLAT_ENTRY("HDMIMIX",
			      0x32fc0000, KB(128),
			      MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_NS),

	MMU_REGION_DT_COMPAT_FOREACH_FLAT_ENTRY(nxp_imx_iuart,
				  (MT_DEVICE_nGnRnE | MT_P_RW_U_NA | MT_NS))
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
