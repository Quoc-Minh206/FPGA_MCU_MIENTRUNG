#ifndef __FATFS_SD_H
#define __FATFS_SD_H

#include "stm32f1xx_hal.h"
#include "integer.h"
#include "diskio.h"

/* Định nghĩa chân CS kết nối với PA4 */
#define SD_CS_PORT          GPIOA
#define SD_CS_PIN           GPIO_PIN_4

/* Sử dụng SPI1 */
extern SPI_HandleTypeDef    hspi1;
#define HSPI_SDCARD         &hspi1

/* Khai báo các hàm giao tiếp */
DSTATUS SD_disk_initialize(BYTE pdrv);
DSTATUS SD_disk_status(BYTE pdrv);
DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void* buff);

#endif
