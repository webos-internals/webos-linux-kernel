#ifndef _LINUX_SPI_ACX567AKM_H
#define _LINUX_SPI_ACX567AKM_H

#include <linux/device.h>
#include <linux/spi/spi.h>

extern int acx567akm_spi_read_u8(struct spi_device *spi, u8 reg, u8* val);
extern int acx567akm_spi_write_u8(struct spi_device *spi, u8 reg, u8 val);

#ifdef CONFIG_SPI_ACX567AKM_DBG
extern int acx567akm_dbg_register_spi_device(struct spi_device *);
extern void acx567akm_dbg_unregister_spi_device(struct spi_device *);
#else // !CONFIG_SPI_ACX567AKM_DBG
static inline int acx567akm_dbg_register_spi_device(struct spi_device *spi)
{
	return (0);
}

static inline void acx567akm_dbg_unregister_spi_device(struct spi_device *spi)
{
}
#endif /* CONFIG_SPI_ACX567AKM_DBG */
#endif /* _LINUX_SPI_ACX567AKM_H */
