/******************************************************************************
 * gps.h  -  Driver GPS NEO-6M qua UART0 cho SONiX SN32F407 EVK
 *
 * Ket noi phan cung (cum J10 - DB_UART tren EVK):
 *      GPS VCC  -> 3.3V   (hoac 5V tuy module)
 *      GPS GND  -> GND
 *      GPS TX   -> P3.2  (URXD0)
 *      GPS RX   -> P3.1  (UTXD0)
 *  UART0: 9600 baud, 8N1 (mac dinh cua u-blox NEO-6M)
 *
 * Bo tach NMEA viet theo kieu may trang thai, KHONG dung <string.h> / <stdio.h>
 * de tiet kiem ROM 32KB (theo yeu cau trong tai lieu phan chia nhiem vu).
 *****************************************************************************/
#ifndef GPS_H
#define GPS_H

#include <stdint.h>

#define GPS_LINE_MAX        96      /* Do dai toi da 1 cau NMEA (chuan = 82) */

/* Toan bo thong tin GPS - xem truc tiep trong cua so Watch cua Keil */
typedef struct
{
    /* --- Thong ke duong truyen (de biet GPS co gui du lieu hay khong) --- */
    uint32_t rx_bytes;          /* Tong so byte nhan duoc tu GPS            */
    uint32_t sentences_ok;      /* So cau NMEA dung checksum                */
    uint32_t checksum_err;      /* So cau sai checksum                      */
    uint32_t uart_err;          /* Loi khung / overrun UART                 */
    uint32_t gga_count;         /* So cau GGA da tach                       */
    uint32_t rmc_count;         /* So cau RMC da tach                       */

    /* --- Ket qua tach --- */
    uint8_t  fix_valid;         /* 1 = da khoa ve tinh (RMC status = 'A')   */
    uint8_t  fix_quality;       /* GGA: 0=no fix, 1=GPS, 2=DGPS             */
    uint8_t  satellites;        /* So ve tinh dang dung de Fix (tu GGA)     */
    uint8_t  sats_in_view;      /* So ve tinh ang-ten dang nhin thay (GSV)  */
    uint8_t  max_snr;           /* Cuong do song ve tinh manh nhat (dB-Hz)  */
    char     time_utc[9];       /* "hh:mm:ss" (UTC)                         */
    char     time_vn[9];        /* "hh:mm:ss" (gio Viet Nam UTC+7)          */
    char     date[9];           /* "dd/mm/yy"                               */
    char     latitude[16];      /* chuoi NMEA goc, vd "1603.25926N"         */
    char     longitude[16];     /* chuoi NMEA goc, vd "10812.34567E"        */
    int32_t  lat_e6;            /* vi do  x 1e6 (do thap phan)              */
    int32_t  lon_e6;            /* kinh do x 1e6 (do thap phan)             */
    char     altitude[10];      /* do cao (m), chuoi                        */
    char     speed_knots[10];   /* toc do (knot), chuoi                     */
    char     hdop[8];           /* HDOP, chuoi                              */
} gps_info_t;

extern volatile gps_info_t g_gps;

/* Cau NMEA hop le gan nhat (bat ky loai nao) + cau GGA/RMC/GSV gan nhat */
extern char g_gps_last_sentence[GPS_LINE_MAX];
extern char g_gps_last_gga[GPS_LINE_MAX];
extern char g_gps_last_rmc[GPS_LINE_MAX];
extern char g_gps_last_gsv[GPS_LINE_MAX];

/* Toa do dang "16.054321,108.123456" (ten bien theo system_shared.h)      */
extern char g_gps_coord[32];
/* Link Google Maps de gui SMS sau nay                                     */
extern char g_gps_maps_link[64];

void    GPS_Init(void);          /* Cau hinh PFPA + UART0 9600 + ngat RX     */
void    GPS_Process(void);       /* Goi lien tuc trong while(1)             */
uint8_t GPS_NewSentence(void);   /* Tra ve 1 (va xoa co) neu co cau moi     */

/* Che do Demo/Test Fix khi o trong nha mai ton (bam nut tren EVK de bat/tat) */
extern volatile uint8_t g_gps_sim_mode;
void    GPS_SimToggle(void);     /* Bat / Tat che do gia lap da Fix ve tinh */
void    GPS_SimStep1s(void);     /* Goi moi 1 giay khi g_gps_sim_mode == 1  */

#endif /* GPS_H */
