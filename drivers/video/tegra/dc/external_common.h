
#ifndef __EXTERNAL_COMMON_H__
#define __EXTERNAL_COMMON_H__

#include <mach/cable_detect.h>

#ifdef CONFIG_TEGRA_HDMI_MHL
extern bool IsD0Mode(void);
extern void update_mhl_status(bool isMHL, enum usb_connect_type statMHL);
#endif

#ifdef CONFIG_TEGRA_HDMI_MHL_SUPERDEMO
typedef unsigned char uint8;
typedef int uint16;
typedef unsigned int uint32;

extern bool g_bDemoTvFound;

/* 1. Remember to use little-endian convention.
   2. keep reserved bits be zeros for compatibility */

#define DEMOTV_DESC_FLAG0			0
#define DEMOTV_DESC_FLAG1			2
#define DEMOTV_DESC_CSTM			3
#define DEMOTV_DESC_FLAG2			4
#define DEMOTV_DESC_DATA			5
#define DEMOTV_DESC_VERSION			5 /* all versions have the same VERSION offset */
#define DEMOTV_DESC_MAGIC			6 /* all versions have the same MAGIC offset */
#define DEMOTV_DESC_TV_ID			7 /* definitions below are just for reference of version 1 */
#define DEMOTV_DESC_1ST_TIMING		9
#define DEMOTV_DESC_2ND_TIMING		11
#define DEMOTV_DESC_FEATURES		13
	#define DEMOTV_BIT_OPTICAL			0x0001
	#define DEMOTV_BIT_RCP				0x0002
	#define DEMOTV_BIT_PROXIMITY		0x0004
	#define DEMOTV_BIT_SONAR			0x0008
#define DEMOTV_DESC_RESERVED		15
#define DEMOTV_DESC_CHECKSUM		17

/* dummy */
struct st_demotv_data_v0 {
	uint8 version;
	uint8 magic;
	uint8 data[11];
};

struct st_demotv_data_v1 {
	uint8 version;
	uint8 magic;
	uint16 tv_id;
	uint16 timing_1st;
	uint16 timing_2nd;
	uint16 features;
	uint16 reserved;
	uint8 checksum;
};

struct st_demotv_patterns {
	const char vendor_id[4];
};

#endif /* CONFIG_FB_MSM_HDMI_MHL_SUPERDEMO */

#endif
