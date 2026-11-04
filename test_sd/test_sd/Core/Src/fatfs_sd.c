#include "fatfs_sd.h"
#include <stdio.h>

/* Khai bao ham in UART tu main.c */
extern void UART_Printf(const char *format, ...);

#define SD_TYPE_MMC     1
#define SD_TYPE_V1      2
#define SD_TYPE_V2      4
#define SD_TYPE_BLOCK   8

#define CMD0    (0)             /* GO_IDLE_STATE */
#define CMD1    (1)             /* SEND_OP_COND */
#define ACMD41  (0x80+41)       /* SEND_OP_COND (SDC) */
#define CMD8    (8)             /* SEND_IF_COND */
#define CMD9    (9)             /* SEND_CSD */
#define CMD10   (10)            /* SEND_CID */
#define CMD12   (12)            /* STOP_TRANSMISSION */
#define ACMD13  (0x80+13)       /* SD_STATUS (SDC) */
#define CMD16   (16)            /* SET_BLOCKLEN */
#define CMD17   (17)            /* READ_SINGLE_BLOCK */
#define CMD18   (18)            /* READ_MULTIPLE_BLOCK */
#define CMD23   (23)            /* SET_BLOCK_COUNT */
#define ACMD23  (0x80+23)       /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24   (24)            /* WRITE_BLOCK */
#define CMD25   (25)            /* WRITE_MULTIPLE_BLOCK */
#define CMD41   (41)            /* SEND_OP_COND (ACMD) */
#define CMD55   (55)            /* APP_CMD */
#define CMD58   (58)            /* READ_OCR */

static volatile DSTATUS Stat = STA_NOINIT;
static BYTE CardType;

static void SPI_SetSlowSpeed(void) {
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; /* 72MHz / 256 = ~281 kHz */
    HAL_SPI_Init(&hspi1);
}

static void SPI_SetFastSpeed(void) {
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;   /* 72MHz / 8 = 9 MHz */
    HAL_SPI_Init(&hspi1);
}

static void SELECT(void) {
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
}

static void DESELECT(void) {
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
}

/* Ham truyen nhan 1 byte dong thoi de tranh tran bo dem OVR */
static BYTE SPI_TxRxByte(BYTE tx_data) {
    BYTE rx_data = 0xFF;
    HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, 100);
    return rx_data;
}

static BYTE SPI_RxByte(void) {
    return SPI_TxRxByte(0xFF);
}

static void SPI_TxByte(BYTE data) {
    SPI_TxRxByte(data);
}

static BYTE wait_ready(UINT wt) {
    BYTE d;
    uint32_t tickstart = HAL_GetTick();
    do {
        d = SPI_RxByte();
        if (d == 0xFF) return 1;
    } while ((HAL_GetTick() - tickstart) < wt);
    return 0;
}

static void deselect(void) {
    DESELECT();
    SPI_RxByte(); /* Gui xung dummy de the giai phong bus MISO */
}

static int select_card(void) {
    SELECT();
    SPI_RxByte();
    if (wait_ready(500)) return 1;
    deselect();
    return 0;
}

static BYTE send_cmd(BYTE cmd, DWORD arg) {
    BYTE n, res;

    if (cmd & 0x80) {
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);
        if (res > 1) return res;
    }

    if (cmd != CMD12) {
        deselect();
        if (cmd == CMD0) {
            SELECT();
            SPI_RxByte();
        } else {
            if (!select_card()) return 0xFF;
        }
    }

    /* Gui goi tin Command 6 bytes */
    SPI_TxByte(0x40 | cmd);
    SPI_TxByte((BYTE)(arg >> 24));
    SPI_TxByte((BYTE)(arg >> 16));
    SPI_TxByte((BYTE)(arg >> 8));
    SPI_TxByte((BYTE)arg);

    n = 0x01;
    if (cmd == CMD0) n = 0x95;
    if (cmd == CMD8) n = 0x87;
    SPI_TxByte(n);

    if (cmd == CMD12) SPI_RxByte();

    /* Cho phan hoi (R1): bit 7 phai bang 0 */
    n = 100;
    do {
        res = SPI_RxByte();
    } while ((res & 0x80) && --n);

    return res;
}

static int rcvr_datablock(BYTE *buff, UINT btr) {
    BYTE token;
    uint32_t tickstart = HAL_GetTick();

    do {
        token = SPI_RxByte();
    } while ((token == 0xFF) && (HAL_GetTick() - tickstart < 200));

    if (token != 0xFE) return 0;

    for (UINT i = 0; i < btr; i++) {
        buff[i] = SPI_RxByte();
    }
    SPI_RxByte(); /* CRC byte 1 */
    SPI_RxByte(); /* CRC byte 2 */
    return 1;
}

static int xmit_datablock(const BYTE *buff, BYTE token) {
    BYTE resp;
    if (!wait_ready(500)) return 0;

    SPI_TxByte(token);
    if (token != 0xFD) {
        for (UINT i = 0; i < 512; i++) {
            SPI_TxByte(buff[i]);
        }
        SPI_RxByte();
        SPI_RxByte();

        resp = SPI_RxByte();
        if ((resp & 0x1F) != 0x05) return 0;
    }
    return 1;
}

DSTATUS SD_disk_initialize(BYTE pdrv) {
    BYTE n, ty, ocr[4];

    if (pdrv) return STA_NOINIT;
    if (Stat & STA_NODISK) return Stat;

    UART_Printf("  [SD_Init] 1. Ha toc do SPI xuong ~281kHz (Prescaler 256)...\r\n");
    SPI_SetSlowSpeed();

    HAL_Delay(100);

    UART_Printf("  [SD_Init] 2. Phat 160 xung clock (20 bytes 0xFF) voi CS=HIGH...\r\n");
    DESELECT();
    for (n = 20; n; n--) SPI_RxByte();

    ty = 0;
    UART_Printf("  [SD_Init] 3. Gui CMD0 vao SPI mode (Cho toi da 1 giay)...\r\n");
    uint32_t tickstart = HAL_GetTick();
    BYTE cmd0_res = 0xFF;
    int attempt = 0;

    while ((HAL_GetTick() - tickstart < 1000) && (cmd0_res != 1)) {
        attempt++;
        cmd0_res = send_cmd(CMD0, 0);
        deselect();
        if (cmd0_res != 1) {
            HAL_Delay(20);
        }
    }

    UART_Printf("  [SD_Init] -> CMD0 ket qua: 0x%02X (sau %d lan thu)\r\n", cmd0_res, attempt);

    if (cmd0_res == 1) {
        UART_Printf("  [SD_Init] -> The da vao SPI Mode thanh cong!\r\n");

        BYTE cmd8_res = send_cmd(CMD8, 0x1AA);
        UART_Printf("  [SD_Init] 4. Gui CMD8 kiem tra phien ban SDC: 0x%02X\r\n", cmd8_res);

        if (cmd8_res == 1) {
            /* SDC V2 */
            for (n = 0; n < 4; n++) ocr[n] = SPI_RxByte();
            deselect();
            UART_Printf("  [SD_Init] -> SDC V2! OCR: 0x%02X 0x%02X 0x%02X 0x%02X\r\n", ocr[0], ocr[1], ocr[2], ocr[3]);

            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                UART_Printf("  [SD_Init] 5. Gui ACMD41 khoi dong the (cho toi da 1s)...\r\n");
                tickstart = HAL_GetTick();
                BYTE acmd41_res = 0xFF;
                while ((HAL_GetTick() - tickstart < 1000)) {
                    acmd41_res = send_cmd(ACMD41, 1UL << 30);
                    deselect();
                    if (acmd41_res == 0) break;
                    HAL_Delay(10);
                }
                UART_Printf("  [SD_Init] -> ACMD41 ket qua: 0x%02X\r\n", acmd41_res);

                if (acmd41_res == 0) {
                    if (send_cmd(CMD58, 0) == 0) {
                        for (n = 0; n < 4; n++) ocr[n] = SPI_RxByte();
                        deselect();
                        ty = (ocr[0] & 0x40) ? SD_TYPE_V2 | SD_TYPE_BLOCK : SD_TYPE_V2;
                        UART_Printf("  [SD_Init] -> Loai the: %s\r\n", (ty & SD_TYPE_BLOCK) ? "SDHC/SDXC (Block-addressed)" : "SDSC (Byte-addressed)");
                    }
                }
            }
        } else {
            /* SDC V1 hoac MMC */
            deselect();
            UART_Printf("  [SD_Init] -> The SDC V1 hoac MMC\r\n");
            if (send_cmd(ACMD41, 0) <= 1) {
                ty = SD_TYPE_V1;
                tickstart = HAL_GetTick();
                while ((HAL_GetTick() - tickstart < 1000) && send_cmd(ACMD41, 0)) {
                    deselect();
                    HAL_Delay(10);
                }
            } else {
                ty = SD_TYPE_MMC;
                tickstart = HAL_GetTick();
                while ((HAL_GetTick() - tickstart < 1000) && send_cmd(CMD1, 0)) {
                    deselect();
                    HAL_Delay(10);
                }
            }
            deselect();
            if ((send_cmd(CMD16, 512) != 0)) ty = 0;
            deselect();
        }
    } else {
        deselect();
        UART_Printf("  [SD_Init] [LOI] The khong phan hoi CMD0 (0x%02X != 0x01)!\r\n", cmd0_res);
    }

    CardType = ty;
    deselect();

    if (ty) {
        Stat &= ~STA_NOINIT;
        UART_Printf("  [SD_Init] 6. Tang toc do SPI len 9MHz (Prescaler 8) de doc ghi nhanh.\r\n");
        SPI_SetFastSpeed();
        UART_Printf("  [SD_Init] -> Khoi tao the SD THANH CONG!\r\n");
    } else {
        Stat = STA_NOINIT;
        UART_Printf("  [SD_Init] -> Khoi tao the SD THAT BAI!\r\n");
    }

    return Stat;
}

DSTATUS SD_disk_status(BYTE pdrv) {
    if (pdrv) return STA_NOINIT;
    return Stat;
}

DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & SD_TYPE_BLOCK)) sector *= 512;

    if (count == 1) {
        if ((send_cmd(CMD17, sector) == 0) && rcvr_datablock(buff, 512))
            count = 0;
    } else {
        if (send_cmd(CMD18, sector) == 0) {
            do {
                if (!rcvr_datablock(buff, 512)) break;
                buff += 512;
            } while (--count);
            send_cmd(CMD12, 0);
        }
    }
    deselect();
    return count ? RES_ERROR : RES_OK;
}

DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & SD_TYPE_BLOCK)) sector *= 512;

    if (count == 1) {
        if ((send_cmd(CMD24, sector) == 0) && xmit_datablock(buff, 0xFE))
            count = 0;
    } else {
        if (CardType & (SD_TYPE_V1 | SD_TYPE_V2)) send_cmd(ACMD23, count);
        if (send_cmd(CMD25, sector) == 0) {
            do {
                if (!xmit_datablock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);
            if (!xmit_datablock(0, 0xFD)) count = 1;
        }
    }
    deselect();
    return count ? RES_ERROR : RES_OK;
}

DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
    if (pdrv) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    DRESULT res = RES_ERROR;
    switch (cmd) {
        case CTRL_SYNC:
            if (select_card()) {
                deselect();
                res = RES_OK;
            }
            break;
        case GET_SECTOR_SIZE:
            *(WORD*)buff = 512;
            res = RES_OK;
            break;
        case GET_BLOCK_SIZE:
            *(DWORD*)buff = 1;
            res = RES_OK;
            break;
        case GET_SECTOR_COUNT:
            *(DWORD*)buff = 1024 * 1024;
            res = RES_OK;
            break;
        default:
            res = RES_PARERR;
            break;
    }
    return res;
}
