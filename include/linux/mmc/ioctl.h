#ifndef LINUX_MMC_IOCTL_H
#define LINUX_MMC_IOCTL_H

#define MMCBOOTMULT     _IOR('M', 1, uint32_t)
#define MMCBOOTSEL      _IOR('M', 2, uint32_t)

#define	MMCIO_WRITE_PROT_INFO	_IOR('M', 10, mmc_write_prot_info_t)
#define	MMCIO_GET_WRITE_PROT	_IOWR('M', 11, mmc_get_write_prot_t)
#define	MMCIO_SET_WRITE_PROT	_IOW('M', 12, mmc_set_write_prot_t)
#define	MMCIO_EN_PERM_WRITE_PROT _IO('M', 14)
#define	MMCIO_GET_EXT_CSD	_IOR('M', 15, mmcio_ext_csd_t)


typedef struct mmc_write_prot_info {
	uint32_t	group_size;
} mmc_write_prot_info_t;

typedef struct mmc_get_write_prot {
	uint32_t	start;		/* input: start, in sectors */
	uint32_t	protect;	/* output: protect flags for start and
					   next 31 blocks. */
	uint8_t		types[8];	/* output: protect types for start and
					   next 31 blocks. */
} mmc_get_write_prot_t;


#define MMC_WRITE_PROT_OFF          0
#define MMC_WRITE_PROT_TEMPORARY    1
#define MMC_WRITE_PROT_POWERON      2
#define MMC_WRITE_PROT_PERMANENT    3

typedef struct mmc_set_write_prot {
	uint32_t	start;		/* Sectors */
	uint32_t	len;		/* number of wp groups to be affected */
	uint32_t	type;		/* 0 - off, 1 temp, 2 poweron, 3 permanent */
} mmc_set_write_prot_t;

typedef struct mmcio_ext_csd {
	uint8_t	ext_csd[512];
} mmcio_ext_csd_t;

#endif
