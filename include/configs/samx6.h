/*
 * Configuration settings for the Kontron SAMX6I QUAD
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"
#undef  CONFIG_LDO_BYPASS_CHECK
#define CONFIG_MX6

#ifdef CONFIG_SECURE_BOOT
#ifndef CONFIG_CSF_SIZE
#define CONFIG_CSF_SIZE 0x4000
#endif
#endif

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_MACH_TYPE	0x10E9

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_SYS_GENERIC_BOARD

#define SZ_1K   (1024)
#define SZ_1M	(1024*SZ_1K)
/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MISC_INIT_R
#define CONFIG_MXC_GPIO
#define CONFIG_CMD_GPIO
#define CONFIG_CI_UDC
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_ETHER
#define CONFIG_USB_ETH_CDC
#define CONFIG_NETCONSOLE

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE
#define CONFIG_CONSOLE_DEV			"ttymxc0"

#define CONFIG_CMD_SF
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_WINBOND
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  3
#define CONFIG_SF_DEFAULT_CS   2
#define CONFIG_SF_DEFAULT_SPEED 25000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0
#define CONFIG_SYS_FSL_USDHC_NUM       3

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_EXT4_WRITE
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC spefific */

#ifdef CONFIG_MX6Q
#define CONFIG_CMD_SATA
#endif

/*
 * SATA Configs
 */
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1
#define CONFIG_PHYLIB
#define CONFIG_PHY_BROADCOM
#define CONFIG_FEC_DMA_MINALIGN		64

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_CMD_FAT
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_ETHER_MCS7830
#define CONFIG_USB_ETHER_SMSC95XX
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	/* For OTG port */
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	0
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL_VIA_CONTROL_EP

/* Miscellaneous commands */
#define CONFIG_CMD_BMODE
#define CONFIG_CMD_SETEXPR

/* Framebuffer and LCD */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_BMP_16BPP
#ifdef CONFIG_MX6DL
#define CONFIG_IPUV3_CLK 198000000
#else
#define CONFIG_IPUV3_CLK 264000000
#endif
#define CONFIG_CMD_HDMIDETECT
#define CONFIG_CONSOLE_MUX
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX	       1
#define CONFIG_BAUDRATE			       115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY	       1

#define CONFIG_PREBOOT                 ""

#define CONFIG_LOADADDR			       0x10800000
#define CONFIG_SYS_TEXT_BASE	       0x17800000

#ifdef CONFIG_CMD_SATA
#define CONFIG_DRIVE_SATA "sata "
#else
#define CONFIG_DRIVE_SATA
#endif

#ifdef CONFIG_CMD_MMC
#define CONFIG_DRIVE_MMC "mmc "
#else
#define CONFIG_DRIVE_MMC
#endif

#ifdef CONFIG_USB_STORAGE
#define CONFIG_DRIVE_USB "usb "
#else
#define CONFIG_DRIVE_USB
#endif

#define CONFIG_DRIVE_TYPES CONFIG_DRIVE_SATA CONFIG_DRIVE_MMC CONFIG_DRIVE_USB
#define CONFIG_UMSDEVS CONFIG_DRIVE_SATA CONFIG_DRIVE_MMC

#define CONFIG_DEFAULT_FDT_FILE "smx6.dtb"
#define CONFIG_HOSTNAME		"SMARC-sAMX6"
#define CONFIG_SYS_AUTOLOAD "no"

#define CONFIG_SYS_MMCDEV   2
#define CONFIG_SYS_MMCROOT  "/dev/mmcblk0p1 rw"
#define CONFIG_SYS_MMCPART  1
#define CONFIG_BOOTIMAGE    "uImage"

#define CONFIG_BOOTARGS \
	"console=" CONFIG_CONSOLE_DEV "," __stringify(CONFIG_BAUDRATE) " " \
    "root=" CONFIG_SYS_MMCROOT " " \
    "video=mxcfb0:dev=ldb,1280x800M@50,if=RGB24 ldb=sin0"

#define CONFIG_BOOTCOMMAND \
		"ext2load mmc " __stringify(CONFIG_SYS_MMCDEV) ":" __stringify(CONFIG_SYS_MMCPART) " " \
	    __stringify(CONFIG_LOADADDR) " " \
        CONFIG_BOOTIMAGE " && " \
        "bootm " __stringify(CONFIG_LOADADDR)

#define CONFIG_EXTRA_ENV_SETTINGS \
	"cleanenv=" \
	   "sf probe;  " \
       "sf erase 0xC0000 0x10000; " \
       "reset;\0" \
    "upgrade_uboot=" \
       "mmc dev 1; " \
       "fatload mmc 1:0 " __stringify(CONFIG_LOADADDR) " u-boot.imx; " \
       "sf probe; " \
       "sf erase 0 0xC0000; " \
	   "sf write " __stringify(CONFIG_LOADADDR) " 0x400 ${filesize}; " \
	   "reset;\0" \
	"bootsel_script=sdboot\0" \
	"uimage=uImage\0" \
    "ext2load=mmc 2:1" __stringify(CONFIG_LOADADDR) " uImage \0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"initrdaddr=0x12700000\0" \
    "initrdfile=initrd\0" \
    "load_splash_img=sf probe 2; sf read $loadaddr $splash_img_addr $splash_img_size;\0" \
	"fdtfile=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"fdtaddr=0x11000000\0" \
    "loadfdt=ext2load mmc ${mmcdev}:${mmcpart} ${fdtaddr} ${fdtfile}\0" \
    "loadinitrd=fatload usb 0:1 ${initrdaddr} ${initrdfile} && fatload usb 0:1 ${loadaddr} ${uimage}\0" \
	"loads_echo=1\0" \
	"loaduimage=ext2load mmc ${bootdev}:${bootpart} ${loadaddr} ${uimage}\0" \
	"machid=" __stringify(CONFIG_MACH_TYPE) "\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} root=${mmcroot}\0" \
    "mmcboot=setenv bootdev ${mmcdev}; " \
	   "setenv bootpart ${mmcpart}; " \
	   "mmc dev ${mmcdev}; " \
       "if mmc rescan ${bootdev}; then " \
          "if run loaduimage; then " \
             "echo Booting from MMC ...; " \
             "setenv bootargs console=${consoledev},${baudrate} root=${mmcroot}; bootm ${loadaddr}; " \
          "fi; " \
       "fi;\0" \
    "mmcdev=2\0" \
    "mmcpart=1\0" \
    "mmcroot=/dev/mmcblk0p1 rw\0" \
    "netargs=setenv bootargs console=${console},${baudrate} root=/dev/nfs ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
    "netboot=echo Booting from net ...; tftp ${fdtaddr} ${fdtfile};tftp ${loadaddr} ${uimage};bootm ${loadaddr} - ${fdtaddr}\0" \
    "netdev=eth0\0" \
	"netupdate=" \
       "if tftp $loadaddr $updfile; then " \
          "setenv loader tftp; " \
          "source $loadaddr; " \
       "else "\
          "run updFal; "\
       "fi\0" \
    "print_splash_img=run load_splash_img; " \
       "if bmp info $load_addr; then " \
          "bmp display $loadaddr; " \
       "else " \
          "echo No Splash Image found; " \
       "fi\0" \
    "sdboot=" \
       "setenv bootdev ${sddev}; setenv bootpart ${sdpart}; " \
       "mmc dev ${sddev}; "\
       "if mmc rescan ${bootdev}; then " \
          "if run loaduimage; then " \
             "echo Booting from SD Card ...;" \
             "setenv bootargs console=${consoledev},${baudrate} root=${sdroot};" \
             "bootm ${loadaddr};" \
          "fi;" \
       "fi;\0" \
    "sddev=1\0" \
    "sdpart=1\0" \
    "sdroot=/dev/mmcblk1p1 rootwait rw\0" \
    "serial#=UpdateMe\0" \
    "splash_img_addr=100000\0" \
    "splash_img_size=80000\0" \
    "splashimage=10900000\0" \
    "uimage=uImage\0" \
    "updFal=echo update failed\0" \
    "updMmcAddExt=mmc dev 0 && ext2load mmc 0 $loadaddr $updfile && setenv loader ext2load mmc 0 && source $loadaddr && true;\0" \
    "updMmcAddFat=mmc dev 0 && fatload mmc 0 $loadaddr $updfile && setenv loader fatload mmc 0 && source $loadaddr && true;\0" \
    "updMmcOnbExt=mmc dev 2 && ext2load mmc 2 $loadaddr $updfile && setenv loader ext2load mmc 2 && source $loadaddr && true;\0" \
    "updMmcOnbFat=mmc dev 2 && fatload mmc 2 $loadaddr $updfile && setenv loader fatload mmc 2 && source $loadaddr && true;\0" \
    "updSdExt=mmc dev 1 && ext2load mmc 1 $loadaddr $updfile && setenv loader ext2load mmc 1 && source $loadaddr && true;\0" \
    "updSdFat=mmc dev 1 && fatload mmc 1 $loadaddr $updfile && setenv loader fatload mmc 1 && source $loadaddr && true;\0" \
    "updUsbExt=usb start && usb dev 0 && ext2load usb 0:1 $loadaddr $updfile && setenv loader ext2load usb 0:1 && source $loadaddr && true;\0" \
    "updUsbFat=usb start && usb dev 0 && fatload usb 0:1 $loadaddr $updfile && setenv loader fatload usb 0:1 && source $loadaddr && true;\0" \
    "update=run updUsbExt || run updUsbFat || run updSdExt || run updSdFat || run updMmcAddExt || run updMmcAddFat || run updMmcOnbExt || run updMmcOnbFat || run updFal;\0" \
    "updfile=update_smx6/update\0" \
    "usbboot=" \
       "usb start;" \
       "if usb dev 0; then " \
          "if run loadinitrd; then " \
             "bootm ${loadaddr} ${initrdaddr};" \
          "fi;" \
       "fi;\0" \
    "wince_kitl=disabled\0"

#define CONFIG_LOADS_ECHO  1

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT	       "U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE	       1024
#define CONFIG_SYS_MAXARGS	       48
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END	       0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

#define CONFIG_SYS_LOAD_ADDR	       CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS	       1
#define PHYS_SDRAM		       MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE	       PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(32 * SZ_1K)

#ifndef CONFIG_SYS_NOSMP
#define CONFIG_SYS_NOSMP
#endif

#define CONFIG_SYS_USE_SPINOR
#define CONFIG_ENV_IS_IN_SPI_FLASH

#define CONFIG_ENV_OFFSET		(768 * SZ_1K)
#define CONFIG_ENV_SECT_SIZE	(64 * SZ_1K)
#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_ENV_OFFSET_REDUND  (CONFIG_ENV_OFFSET + CONFIG_ENV_SIZE)
#define CONFIG_ENV_SIZE_REDUN   (CONFIG_ENV_SIZE)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED

#define CONFIG_OF_LIBFDT

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#define CONFIG_CMD_BMP

#define CONFIG_CMD_TIME
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_ALT_MEMTEST

#define CONFIG_CMD_BOOTZ
#define CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_CMD_FS_GENERIC

/*
 * PCI express
 */
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI
#define CONFIG_PCI_PNP
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#endif

#define CONFIG_CMD_ELF

#define CONFIG_USB_GADGET
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USBDOWNLOAD_GADGET
#define CONFIG_USB_GADGET_VBUS_DRAW	2

#define CONFIG_G_DNL_VENDOR_NUM		0x18d1
#define CONFIG_G_DNL_PRODUCT_NUM	0x0d02
#define CONFIG_G_DNL_MANUFACTURER	"FSL"

#define CONFIG_CMD_FASTBOOT
#define CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_USB_FASTBOOT_BUF_ADDR   CONFIG_SYS_LOAD_ADDR
#define CONFIG_USB_FASTBOOT_BUF_SIZE   0x07000000

#endif	       /* __CONFIG_H */
