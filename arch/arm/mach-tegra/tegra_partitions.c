/*
 * arch/arm/mach-tegra/tegra_partitions.c
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <asm/mach/flash.h>
#include <asm/setup.h>

#define ATAG_TEGRA_PARTITION 0x4d534D70
#define TEGRA_MAX_PARTITIONS 32

#define FLASH_PART_MAGIC1     0x55EE73AA
#define FLASH_PART_MAGIC2     0xE35EBDDB
#define FLASH_PARTITION_VERSION   0x3

#define LINUX_FS_PARTITION_NAME  "0:EFS2APPS"

struct tegra_ptbl_entry {
	char name[16];
	__u32 offset;
	__u32 size;
	__u32 flags;
};

static struct mtd_partition tegra_nand_partitions[TEGRA_MAX_PARTITIONS];
static char tegra_nand_names[TEGRA_MAX_PARTITIONS * 16];

struct flash_platform_data tegra_nand_data;

int emmc_partition_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data)
{
	struct mtd_partition *ptn = tegra_nand_partitions;
	char *p = page;
	int i;
	uint64_t offset;
	uint64_t size;

	p += sprintf(p, "dev:        size     erasesize name\n");
	for (i = 0; i < TEGRA_MAX_PARTITIONS && ptn->name; i++, ptn++) {
		offset = ptn->offset;
		size = ptn->size;
		p += sprintf(p, "mmcblk0p%llu: %08llx %08x \"%s\"\n", offset, size * 4096, 4096, ptn->name);
	}

	return p - page;
}

static int __init parse_tag_tegra_partition(const struct tag *tag)
{
	struct mtd_partition *ptn = tegra_nand_partitions;
	char *name = tegra_nand_names;
	struct tegra_ptbl_entry *entry = (void *) &tag->u;
	unsigned count, n;

	count = (tag->hdr.size - 2) /
		(sizeof(struct tegra_ptbl_entry) / sizeof(__u32));

	printk("[emmc] (sizeof(struct tegra_ptbl_entry) / sizeof(__u32)) %d\n",(sizeof(struct tegra_ptbl_entry) / sizeof(__u32))); 
	printk("[emmc] header->size %d\n",tag->hdr.size); 
    	printk("[emmc] count %d\n",count);
	if (count > TEGRA_MAX_PARTITIONS)
		count = TEGRA_MAX_PARTITIONS;

	for (n = 0; n < count; n++) {
		memcpy(name, entry->name, 15);
		name[15] = 0;

		ptn->name = name;
		ptn->offset = entry->offset;
		ptn->size = entry->size;

		printk(KERN_INFO "Partition (from atag) %s "
				"-- Offset:%llx Size:%llx\n",
				ptn->name, ptn->offset, ptn->size);

		name += 16;
		entry++;
		ptn++;
	}

	tegra_nand_data.nr_parts = count;
	tegra_nand_data.parts = tegra_nand_partitions;

	return 0;
}

__tagtable(ATAG_TEGRA_PARTITION, parse_tag_tegra_partition);


struct flash_partition_entry {
	char name[16];
	u32 offset;	/* Offset in blocks from beginning of device */
	u32 length;	/* Length of the partition in blocks */
	u8 attrib1;
	u8 attrib2;
	u8 attrib3;
	u8 which_flash;	/* Numeric ID (first = 0, second = 1) */
};
struct flash_partition_table {
	u32 magic1;
	u32 magic2;
	u32 version;
	u32 numparts;
	struct flash_partition_entry part_entry[16];
};

static int get_nand_partitions(void)
{
	struct flash_partition_table *partition_table;
	struct flash_partition_entry *part_entry;
	struct mtd_partition *ptn = tegra_nand_partitions;
	char *name = tegra_nand_names;
	int part;

	if (tegra_nand_data.nr_parts)
		return 0;

	partition_table = (struct flash_partition_table *)
	    kzalloc(
		       sizeof(struct flash_partition_table),GFP_KERNEL);

	if (!partition_table) {
		printk(KERN_WARNING "%s: no flash partition table in shared "
		       "memory\n", __func__);
		return -ENOENT;
	}

	if ((partition_table->magic1 != (u32) FLASH_PART_MAGIC1) ||
	    (partition_table->magic2 != (u32) FLASH_PART_MAGIC2) ||
	    (partition_table->version != (u32) FLASH_PARTITION_VERSION)) {
		printk(KERN_WARNING "%s: version mismatch -- magic1=%#x, "
		       "magic2=%#x, version=%#x\n", __func__,
		       partition_table->magic1,
		       partition_table->magic2,
		       partition_table->version);
		return -EFAULT;
	}

	tegra_nand_data.nr_parts = 0;

	/* Get the LINUX FS partition info */
	for (part = 0; part < partition_table->numparts; part++) {
		part_entry = &partition_table->part_entry[part];

		/* Find a match for the Linux file system partition */
		if (strcmp(part_entry->name, LINUX_FS_PARTITION_NAME) == 0) {
			strcpy(name, part_entry->name);
			ptn->name = name;

			/*TODO: Get block count and size info */
			ptn->offset = part_entry->offset;

			/* For SMEM, -1 indicates remaining space in flash,
			 * but for MTD it is 0
			 */
			if (part_entry->length == (u32)-1)
				ptn->size = 0;
			else
				ptn->size = part_entry->length;

			tegra_nand_data.nr_parts = 1;
			tegra_nand_data.parts = tegra_nand_partitions;

			printk(KERN_INFO "Partition(from smem) %s "
					"-- Offset:%llx Size:%llx\n",
					ptn->name, ptn->offset, ptn->size);

			return 0;
		}
	}
	printk(KERN_WARNING "%s: no partition table found!", __func__);

	return -ENODEV;
}
device_initcall(get_nand_partitions);
