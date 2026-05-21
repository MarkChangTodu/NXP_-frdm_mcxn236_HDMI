.. zephyr:code-sample:: imx8mp-hdmi-bringup
   :name: i.MX8MP HDMI bring-up test (Main Goal 1.1 + 1.2)

   Verify that the i.MX8MP HDMI driver can power up HDMIMIX/HDMI_PHY via
   GPC and that MMU mappings for the four HDMI MMIO regions return their
   documented reset values.

Overview
********

This sample is the first hardware test in the HDMI driver development plan
at ``docs/plans/hdmi_driver_plan.md``. It corresponds to plan sub-goals
**1.1** (GPC power-up of HDMIMIX + HDMI_PHY) and **1.2** (MMU mapping of
the four HDMI register windows).

Both of these steps are already implemented inside the driver init function
``imx8mp_hdmi_init()`` in
``zephyr/drivers/display/display_imx8mp_hdmi.c``. This sample exists only
to give you a single bin you can flash to the EVK and confirm PASS/FAIL on
the UART4 console.

Requirements
************

- ``imx8mp_evk/mimx8ml8/a53`` board, booted from U-Boot at ``0xC0000000``.
- UART4 serial console at 115200 8N1.
- HDMI cable is **not** required for this test — neither HPD nor EDID is
  checked. We are only validating power, clocks, MMU, and reset values.

Building and running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/display/imx8mp_hdmi_bringup
   :board: imx8mp_evk/mimx8ml8/a53
   :goals: build
   :compact:

Flash ``zephyr.bin`` to the EVK via TFTP/MMC and ``go 0xC0000000`` as
documented in ``zephyr/boards/nxp/imx8mp_evk/doc/index.rst``.

Expected console output
***********************

On success (regex anchors used by ``sample.yaml`` harness):

.. code-block:: text

   *** Booting Zephyr OS build ... ***
   [00:00:00.xxx,xxx] <inf> imx8mp_hdmi: GPC power-up acknowledged in N iterations
   [00:00:00.xxx,xxx] <inf> imx8mp_hdmi: HDMI BLK_CTRL[0x00] = 0x........
   [00:00:00.xxx,xxx] <inf> imx8mp_hdmi: HTX_PVI[0x00]       = 0x00377000 (expect 0x00377000 reset)
   [00:00:00.xxx,xxx] <inf> imx8mp_hdmi: HDMI_TX[0x00]       = 0x........
   [00:00:00.xxx,xxx] <inf> imx8mp_hdmi: HDMI_PHY PHY_REG1   = 0xd1 (expect 0xd1 reset)
   [00:00:00.xxx,xxx] <inf> imx8mp_hdmi: imx8mp_hdmi: probe ok
   HDMI bring-up test: starting
   Device handle      : 0x........
   Device is_ready()  : YES
   TEST 1.1+1.2 PASS

PASS criteria
*************

#. ``TEST 1.1+1.2 PASS`` line is printed.
#. ``HTX_PVI[0x00]`` reads back ``0x00377000`` (the reset value documented
   in IMX8MPRM §13.13).
#. ``HDMI_PHY PHY_REG1`` reads back ``0xd1`` (the reset value documented in
   IMX8MPRM §13.10).
#. ``GPC power-up acknowledged`` is logged (no timeout).

FAIL signatures and remediation
*******************************

``GPC power-up timed out``
   ``SW_PUP_REQ`` bits never self-cleared. Verify the GPC MMU region in
   ``zephyr/soc/nxp/imx/imx8m/a53/mmu_regions.c`` is mapped and that no
   other master holds the HDMIMIX powered down. Re-check
   ``GPC_PU_PGC_SW_PUP_REQ_OFF`` against IMX8MPRM §5.2.

``MMIO reads returned 0xffffffff``
   Either the power-up did not complete or the MMU mapping is wrong. Inspect
   the ``HDMIMIX`` and ``GPC`` ``MMU_REGION_FLAT_ENTRY`` definitions.

``HTX_PVI_CTRL mismatch (got 0x........, expected 0x00377000)``
   Power up may have raced. Add a small ``k_msleep(1)`` after GPC ack and
   retry; if still mismatched, the reg base in the DT node is wrong.

``TEST 1.1+1.2 FAIL: device not ready``
   The driver ``init`` returned non-zero. Look at the most recent
   ``<err> imx8mp_hdmi:`` line for the cause.

Plan reference
**************

See ``docs/plans/hdmi_driver_plan.md`` "Main Goal 1 — Power / Clock / Reset
bring-up". After this test passes, proceed to **sub-goal 1.3** (enable CCM
root clocks ``HDMI_APB_CLK_ROOT``, ``HDMI_AXI_CLK_ROOT``,
``HDMI_REF_266M_CLK_RO``, ``HDMI_FDCC_TST_CLK_RO``).
