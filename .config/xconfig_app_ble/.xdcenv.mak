#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /home/keenan/ti/simplelink_cc13x0_sdk_2_30_00_20/source;/home/keenan/ti/simplelink_cc13x0_sdk_2_30_00_20/kernel/tirtos/packages;/home/keenan/ti/tirtos_cc13xx_cc26xx_2_21_00_06/packages;/home/keenan/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/tidrivers_cc13xx_cc26xx_2_21_00_04/packages;/home/keenan/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/bios_6_46_01_37/packages;/home/keenan/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/uia_2_01_00_01/packages;/home/keenan/workspace_v8/simple_peripheral_cc1350lp_app_FlashROM/.config
override XDCROOT = /home/keenan/ti/xdctools_3_50_08_24_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /home/keenan/ti/simplelink_cc13x0_sdk_2_30_00_20/source;/home/keenan/ti/simplelink_cc13x0_sdk_2_30_00_20/kernel/tirtos/packages;/home/keenan/ti/tirtos_cc13xx_cc26xx_2_21_00_06/packages;/home/keenan/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/tidrivers_cc13xx_cc26xx_2_21_00_04/packages;/home/keenan/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/bios_6_46_01_37/packages;/home/keenan/ti/tirtos_cc13xx_cc26xx_2_21_00_06/products/uia_2_01_00_01/packages;/home/keenan/workspace_v8/simple_peripheral_cc1350lp_app_FlashROM/.config;/home/keenan/ti/xdctools_3_50_08_24_core/packages;..
HOSTOS = Linux
endif
