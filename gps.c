/******************************************************************************
 * gps.c  -  Driver GPS NEO-6M qua UART0 cho SONiX SN32F407 EVK
 *
 *  - UART0 9600 8N1, chan P3.1 (UTXD0) / P3.2 (URXD0)  -> cum J10 tren EVK
 *  - Ngat RX day byte vao bo dem vong (ring buffer)
 *  - GPS_Process() rut byte ra va chay may trang thai tach NMEA
 *  - Khong dung <string.h>/<stdio.h> (tiet kiem ROM)
 *****************************************************************************/
#include <SN32F400.h>
#include "gps.h"

/*=============================  CAU HINH  =================================*/
#define UART0_CLKEN_BIT     (1UL << 16)     /* SYS1_AHBCLKEN.UART0CLKEN     */

/* PFPA_UART0: [3:2] URXD0 = 2 -> P3.2 ; [1:0] UTXD0 = 2 -> P3.1            */
#define PFPA_UART0_MASK     (0x0FUL)
#define PFPA_UART0_P31_P32  ((2UL << 2) | (2UL << 0))

/* Baud = PCLK / (16 * DLL)  ; PCLK = HCLK = 12 MHz (IHRC 12M)
 * 12 000 000 / (16 * 78) = 9615 baud  (sai so 0.16 %)                       */
#define GPS_BAUD_DLL        78

#define VN_UTC_OFFSET       7               /* Gio Viet Nam = UTC + 7       */

#define RX_RING_SIZE        256             /* luy thua cua 2               */

/*=============================  BIEN TOAN CUC  ============================*/
volatile gps_info_t g_gps;
char                g_gps_last_sentence[GPS_LINE_MAX];
char                g_gps_last_gga[GPS_LINE_MAX];
char                g_gps_last_rmc[GPS_LINE_MAX];
char                g_gps_coord[32];
char                g_gps_maps_link[64];
char                g_gps_last_gsv[GPS_LINE_MAX];
volatile uint8_t    g_gps_sim_mode = 0;
static uint8_t      s_sim_sec = 0;
static uint8_t      s_feeding_sim = 0;

static volatile uint8_t  s_ring[RX_RING_SIZE];
static volatile uint16_t s_head = 0;        /* ISR ghi                      */
static volatile uint16_t s_tail = 0;        /* main doc                     */

static volatile uint8_t  s_new_sentence = 0;

/* May trang thai NMEA */
typedef enum { ST_WAIT_DOLLAR = 0, ST_BODY, ST_CK1, ST_CK2 } nmea_state_t;
static nmea_state_t s_state;
static char     s_line[GPS_LINE_MAX];
static uint8_t  s_len;
static uint8_t  s_cksum_calc;
static uint8_t  s_cksum_rx;

/*=============================  HAM TIEN ICH  =============================*/
static void str_copy(char *dst, const char *src, uint32_t max)
{
    uint32_t i = 0;
    if (max == 0) return;
    while ((i < max - 1U) && (src[i] != '\0')) { dst[i] = src[i]; i++; }
    dst[i] = '\0';
}

static int8_t hex_val(char c)
{
    if (c >= '0' && c <= '9') return (int8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (int8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (int8_t)(c - 'a' + 10);
    return -1;
}

/* Lay truong thu 'idx' (0 = "$GPxxx") cua cau NMEA, ket qua vao out.
 * Tra ve do dai truong (0 neu rong).                                        */
static uint8_t nmea_field(const char *line, uint8_t idx, char *out, uint8_t max)
{
    uint8_t f = 0, n = 0;
    const char *p = line;

    while (*p != '\0' && f < idx) { if (*p == ',') f++; p++; }
    if (f != idx) { out[0] = '\0'; return 0; }

    while (*p != '\0' && *p != ',' && *p != '*' && n < (uint8_t)(max - 1U))
    {
        out[n++] = *p++;
    }
    out[n] = '\0';
    return n;
}

/* "ddmm.mmmmm" hoac "dddmm.mmmmm" -> do x 1e6 */
static int32_t nmea_to_e6(const char *s, uint8_t deg_digits)
{
    uint32_t deg = 0, min_int = 0, frac = 0, frac_digits = 0;
    uint8_t  i;
    const char *p = s;

    for (i = 0; i < deg_digits; i++)
    {
        if (*p < '0' || *p > '9') return 0;
        deg = deg * 10U + (uint32_t)(*p++ - '0');
    }
    while (*p >= '0' && *p <= '9') { min_int = min_int * 10U + (uint32_t)(*p++ - '0'); }
    if (*p == '.')
    {
        p++;
        while (*p >= '0' && *p <= '9' && frac_digits < 5U)
        {
            frac = frac * 10U + (uint32_t)(*p++ - '0');
            frac_digits++;
        }
    }
    while (frac_digits < 5U) { frac *= 10U; frac_digits++; }   /* chuan hoa 1e5 */

    /* phut x 1e5  ->  do x 1e6 = phut_e5 / 6 */
    return (int32_t)(deg * 1000000UL + (min_int * 100000UL + frac) / 6UL);
}

/* Ghi so nguyen co dau dang "ddd.dddddd" vao buf, tra ve con tro cuoi */
static char *put_e6(char *buf, int32_t v)
{
    char tmp[12];
    uint32_t u, ip, fp;
    int8_t i, n = 0;

    if (v < 0) { *buf++ = '-'; u = (uint32_t)(-v); } else { u = (uint32_t)v; }
    ip = u / 1000000UL;
    fp = u % 1000000UL;

    do { tmp[n++] = (char)('0' + (ip % 10U)); ip /= 10U; } while (ip != 0U);
    for (i = (int8_t)(n - 1); i >= 0; i--) *buf++ = tmp[i];
    *buf++ = '.';
    for (i = 5; i >= 0; i--) { tmp[i] = (char)('0' + (fp % 10U)); fp /= 10U; }
    for (i = 0; i < 6; i++) *buf++ = tmp[i];
    *buf = '\0';
    return buf;
}

static void make_coord_strings(void)
{
    char *p;
    const char *pre = "https://maps.google.com/?q=";

    p = put_e6(g_gps_coord, g_gps.lat_e6);
    *p++ = ',';
    put_e6(p, g_gps.lon_e6);

    p = g_gps_maps_link;
    while (*pre) *p++ = *pre++;
    str_copy(p, g_gps_coord, (uint32_t)(sizeof(g_gps_maps_link) - (uint32_t)(p - g_gps_maps_link)));
}

/* "hhmmss.ss" -> "hh:mm:ss" (UTC) + gio VN */
static void parse_time(const char *f)
{
    uint8_t h;
    if (f[0] == '\0' || f[5] == '\0') return;
    g_gps.time_utc[0] = f[0]; g_gps.time_utc[1] = f[1]; g_gps.time_utc[2] = ':';
    g_gps.time_utc[3] = f[2]; g_gps.time_utc[4] = f[3]; g_gps.time_utc[5] = ':';
    g_gps.time_utc[6] = f[4]; g_gps.time_utc[7] = f[5]; g_gps.time_utc[8] = '\0';

    h = (uint8_t)(((f[0] - '0') * 10 + (f[1] - '0') + VN_UTC_OFFSET) % 24);
    g_gps.time_vn[0] = (char)('0' + h / 10U);
    g_gps.time_vn[1] = (char)('0' + h % 10U);
    g_gps.time_vn[2] = ':';
    g_gps.time_vn[3] = f[2]; g_gps.time_vn[4] = f[3]; g_gps.time_vn[5] = ':';
    g_gps.time_vn[6] = f[4]; g_gps.time_vn[7] = f[5]; g_gps.time_vn[8] = '\0';
}

static void parse_latlon(const char *lat, const char *ns, const char *lon, const char *ew)
{
    uint8_t i;
    if (lat[0] == '\0' || lon[0] == '\0') return;

    for (i = 0; lat[i] != '\0' && i < 13U; i++) g_gps.latitude[i] = lat[i];
    g_gps.latitude[i++] = ns[0]; g_gps.latitude[i] = '\0';
    for (i = 0; lon[i] != '\0' && i < 13U; i++) g_gps.longitude[i] = lon[i];
    g_gps.longitude[i++] = ew[0]; g_gps.longitude[i] = '\0';

    g_gps.lat_e6 = nmea_to_e6(lat, 2);
    g_gps.lon_e6 = nmea_to_e6(lon, 3);
    if (ns[0] == 'S') g_gps.lat_e6 = -g_gps.lat_e6;
    if (ew[0] == 'W') g_gps.lon_e6 = -g_gps.lon_e6;
    make_coord_strings();
}

/* $GPGGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,... */
static void parse_gga(const char *line)
{
    char f[16], lat[16], ns[2], lon[16], ew[2];

    g_gps.gga_count++;
    nmea_field(line, 6, f, sizeof(f));
    if (!s_feeding_sim && g_gps_sim_mode)
    {
        if (f[0] == '\0' || f[0] == '0') return;
        g_gps_sim_mode = 0U;    /* Tu dong chuyen sang GPS that khi co Fix that! */
    }

    str_copy(g_gps_last_gga, line, GPS_LINE_MAX);
    g_gps.fix_quality = (uint8_t)(f[0] ? (f[0] - '0') : 0);
    nmea_field(line, 1, f, sizeof(f));   parse_time(f);
    nmea_field(line, 7, f, sizeof(f));
    g_gps.satellites = (uint8_t)(f[0] ? ((f[1] ? (f[0] - '0') * 10 + (f[1] - '0') : (f[0] - '0'))) : 0);
    nmea_field(line, 8, (char *)g_gps.hdop, sizeof(g_gps.hdop));
    nmea_field(line, 9, (char *)g_gps.altitude, sizeof(g_gps.altitude));

    if (g_gps.fix_quality > 0U)
    {
        nmea_field(line, 2, lat, sizeof(lat)); nmea_field(line, 3, ns, sizeof(ns));
        nmea_field(line, 4, lon, sizeof(lon)); nmea_field(line, 5, ew, sizeof(ew));
        parse_latlon(lat, ns, lon, ew);
    }
    else
    {
        g_gps.fix_valid = 0;
        g_gps.lat_e6 = 0;
        g_gps.lon_e6 = 0;
        str_copy(g_gps_coord, "CHUA CO FIX", sizeof(g_gps_coord));
        str_copy(g_gps_maps_link, "CHUA CO FIX", sizeof(g_gps_maps_link));
    }
}

/* $GPRMC,time,status,lat,N,lon,E,speed,course,date,... */
static void parse_rmc(const char *line)
{
    char f[16], lat[16], ns[2], lon[16], ew[2];

    g_gps.rmc_count++;
    nmea_field(line, 2, f, sizeof(f));
    if (!s_feeding_sim && g_gps_sim_mode)
    {
        if (f[0] != 'A') return;
        g_gps_sim_mode = 0U;
    }

    str_copy(g_gps_last_rmc, line, GPS_LINE_MAX);
    g_gps.fix_valid = (f[0] == 'A') ? 1U : 0U;
    nmea_field(line, 1, f, sizeof(f));   parse_time(f);
    nmea_field(line, 7, (char *)g_gps.speed_knots, sizeof(g_gps.speed_knots));
    nmea_field(line, 9, f, sizeof(f));
    if (f[0] != '\0' && f[5] != '\0')
    {
        g_gps.date[0] = f[0]; g_gps.date[1] = f[1]; g_gps.date[2] = '/';
        g_gps.date[3] = f[2]; g_gps.date[4] = f[3]; g_gps.date[5] = '/';
        g_gps.date[6] = f[4]; g_gps.date[7] = f[5]; g_gps.date[8] = '\0';
    }

    if (g_gps.fix_valid)
    {
        nmea_field(line, 3, lat, sizeof(lat)); nmea_field(line, 4, ns, sizeof(ns));
        nmea_field(line, 5, lon, sizeof(lon)); nmea_field(line, 6, ew, sizeof(ew));
        parse_latlon(lat, ns, lon, ew);
    }
    else
    {
        g_gps.lat_e6 = 0;
        g_gps.lon_e6 = 0;
        str_copy(g_gps_coord, "CHUA CO FIX", sizeof(g_gps_coord));
        str_copy(g_gps_maps_link, "CHUA CO FIX", sizeof(g_gps_maps_link));
    }
}

/* $GPGSV,total_msgs,msg_num,sats_in_view,prn,elev,azim,snr,... */
static void parse_gsv(const char *line)
{
    char f[8];
    uint8_t idx, snr, siv = 0;

    /* Truong 3: Tong so ve tinh ang-ten dang nhin thay tren bau troi */
    nmea_field(line, 3, f, sizeof(f));
    if (f[0] >= '0' && f[0] <= '9')
    {
        siv = (uint8_t)((f[1] >= '0' && f[1] <= '9')
            ? ((f[0] - '0') * 10 + (f[1] - '0'))
            : (f[0] - '0'));
    }
    if (!s_feeding_sim && g_gps_sim_mode)
    {
        if (siv == 0U) return;
        g_gps_sim_mode = 0U;    /* Tu dong tat gia lap ngay khi ang-ten thay >= 1 ve tinh that! */
    }

    str_copy(g_gps_last_gsv, line, GPS_LINE_MAX);
    g_gps.sats_in_view = siv;

    /* Neu la ban tin GSV so 1 -> tinh lai max_snr cua chu ky moi */
    nmea_field(line, 2, f, sizeof(f));
    if (f[0] == '1') g_gps.max_snr = 0;

    /* Quet SNR (dB-Hz) o cac truong 7, 11, 15, 19 */
    for (idx = 7U; idx <= 19U; idx += 4U)
    {
        if (nmea_field(line, idx, f, sizeof(f)) > 0U && f[0] >= '0' && f[0] <= '9')
        {
            snr = (uint8_t)((f[1] >= '0' && f[1] <= '9')
                ? ((f[0] - '0') * 10 + (f[1] - '0'))
                : (f[0] - '0'));
            if (snr > g_gps.max_snr) g_gps.max_snr = snr;
        }
    }
}

static void handle_sentence(void)
{
    g_gps.sentences_ok++;
    if (!g_gps_sim_mode || s_feeding_sim)
    {
        str_copy(g_gps_last_sentence, s_line, GPS_LINE_MAX);
    }
    s_new_sentence = 1;

    /* s_line = "$GPGGA,..." hoac "$GNGGA,..." -> so sanh ky tu 3..5 */
    if (s_len > 6U)
    {
        if (s_line[3] == 'G' && s_line[4] == 'G' && s_line[5] == 'A')      parse_gga(s_line);
        else if (s_line[3] == 'R' && s_line[4] == 'M' && s_line[5] == 'C') parse_rmc(s_line);
        else if (s_line[3] == 'G' && s_line[4] == 'S' && s_line[5] == 'V') parse_gsv(s_line);
    }
}

/* May trang thai: nhan tung ky tu */
static void nmea_feed(char c)
{
    int8_t v;

    if (c == '$')                       /* luon bat dau lai khi gap '$' */
    {
        s_state = ST_BODY;
        s_len = 0;
        s_line[s_len++] = c;
        s_cksum_calc = 0;
        return;
    }

    switch (s_state)
    {
    case ST_BODY:
        if (c == '*')
        {
            if (s_len < GPS_LINE_MAX - 4U) s_line[s_len++] = c;
            s_state = ST_CK1;
        }
        else if (c == '\r' || c == '\n' || s_len >= GPS_LINE_MAX - 4U)
        {
            s_state = ST_WAIT_DOLLAR;   /* cau loi / qua dai */
        }
        else
        {
            s_line[s_len++] = c;
            s_cksum_calc ^= (uint8_t)c;
        }
        break;

    case ST_CK1:
        v = hex_val(c);
        if (v < 0) { s_state = ST_WAIT_DOLLAR; break; }
        s_cksum_rx = (uint8_t)(v << 4);
        s_line[s_len++] = c;
        s_state = ST_CK2;
        break;

    case ST_CK2:
        v = hex_val(c);
        s_state = ST_WAIT_DOLLAR;
        if (v < 0) break;
        s_cksum_rx |= (uint8_t)v;
        s_line[s_len++] = c;
        s_line[s_len] = '\0';
        if (s_cksum_rx == s_cksum_calc) handle_sentence();
        else                            g_gps.checksum_err++;
        break;

    default:
        break;
    }
}

/*=============================  UART0  ====================================*/
void UART0_IRQHandler(void)
{
    uint32_t ls = SN_UART0->LS;

    /* Xóa cờ lỗi đường truyền nếu có */
    if (ls & 0x8EUL) {
        g_gps.uart_err++;
        volatile uint32_t dummy = SN_UART0->RB;
        (void)dummy;
    }

    /* Đọc cạn các byte trong RX FIFO vào ring buffer */
    while (SN_UART0->LS & 0x01UL)
    {
        uint8_t b = (uint8_t)SN_UART0->RB;
        g_gps.rx_bytes++;
        uint16_t next = (uint16_t)((s_head + 1U) & (RX_RING_SIZE - 1U));
        if (next != s_tail) { s_ring[s_head] = b; s_head = next; }
        else                { g_gps.uart_err++; }
    }
}

void GPS_Init(void)
{
    uint32_t i;
    volatile uint8_t *p = (volatile uint8_t *)&g_gps;
    for (i = 0; i < sizeof(g_gps); i++) p[i] = 0;
    g_gps_last_sentence[0] = '\0';
    g_gps_last_gga[0]      = '\0';
    g_gps_last_rmc[0]      = '\0';
    g_gps_last_gsv[0]      = '\0';
    str_copy(g_gps_coord,     "CHUA CO FIX", sizeof(g_gps_coord));
    str_copy(g_gps_maps_link, "CHUA CO FIX", sizeof(g_gps_maps_link));
    g_gps_sim_mode = 0;

    /* 1. Dinh tuyen chan UART0 -> P3.1 (TX), P3.2 (RX) - chi sua bit cua UART0 */
    SN_PFPA->UART0 = (SN_PFPA->UART0 & ~PFPA_UART0_MASK) | PFPA_UART0_P31_P32;

    /* 2. Cap clock cho UART0 (dung |= de khong xoa ngoai vi cua nguoi khac) */
    SN_SYS1->AHBCLKEN |= UART0_CLKEN_BIT;

    /* 3. Khung 8N1 + mo chot divisor */
    SN_UART0->LC  = 0x03UL | 0x80UL;
    SN_UART0->FD  = 0x10UL;                 /* OVER16, DIVADDVAL = 0        */
    SN_UART0->DLM = 0;
    SN_UART0->DLL = GPS_BAUD_DLL;
    SN_UART0->LC  = 0x03UL;                 /* dong chot divisor            */

    /* 4. FIFO: bat + reset RX/TX, nguong 1 byte */
    SN_UART0->FIFOCTRL = 0x01UL | 0x02UL | 0x04UL;

    /* 5. Ngat: RDA + RLS */
    SN_UART0->IE = 0x01UL | 0x04UL;

    /* 6. Bat UART, RX, TX */
    SN_UART0->CTRL = 0x01UL | (1UL << 6) | (1UL << 7);

    s_head = s_tail = 0;
    s_state = ST_WAIT_DOLLAR;

    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);
}

void GPS_Process(void)
{
    while (s_tail != s_head)
    {
        char c = (char)s_ring[s_tail];
        s_tail = (uint16_t)((s_tail + 1U) & (RX_RING_SIZE - 1U));
        nmea_feed(c);
    }
}

uint8_t GPS_NewSentence(void)
{
    if (s_new_sentence) { s_new_sentence = 0; return 1; }
    return 0;
}

/*=================  CHE DO GIA LAP DA FIX (TEST TRONG NHA)  ===============*/
static void gps_feed_body(const char *body)
{
    uint8_t ck = 0;
    const char *hex = "0123456789ABCDEF";
    const char *p = body;

    s_feeding_sim = 1U;
    nmea_feed('$');
    while (*p != '\0')
    {
        ck ^= (uint8_t)(*p);
        nmea_feed(*p++);
    }
    nmea_feed('*');
    nmea_feed(hex[(ck >> 4) & 0x0FU]);
    nmea_feed(hex[ck & 0x0FU]);
    nmea_feed('\r');
    nmea_feed('\n');
    s_feeding_sim = 0U;
}

void GPS_SimStep1s(void)
{
    char gga[80];
    char rmc[80];
    uint8_t s0, s1;

    if (!g_gps_sim_mode) return;

    s_sim_sec = (uint8_t)((s_sim_sec + 1U) % 60U);
    s0 = (uint8_t)('0' + (s_sim_sec / 10U));
    s1 = (uint8_t)('0' + (s_sim_sec % 10U));

    /* $GPGSV: 08 ve tinh dang nhin thay, SNR = 38 dB-Hz */
    gps_feed_body("GPGSV,2,1,08,02,68,045,38,05,52,180,35,12,41,290,33,18,30,110,29");

    /* $GPGGA: UTC 04:15:ss (VN 11:15:ss), 1603.25926N, 10812.34567E, Fix=1, Sats=08 */
    str_copy(gga, "GPGGA,041500.00,1603.25926,N,10812.34567,E,1,08,0.92,15.4,M,-2.4,M,,", sizeof(gga));
    gga[10] = (char)s0; gga[11] = (char)s1;
    gps_feed_body(gga);

    /* $GPRMC: Status='A' (Active), Date=02/11/26 */
    str_copy(rmc, "GPRMC,041500.00,A,1603.25926,N,10812.34567,E,0.12,,021126,,,A", sizeof(rmc));
    rmc[10] = (char)s0; rmc[11] = (char)s1;
    gps_feed_body(rmc);
}

void GPS_SimToggle(void)
{
    if (!g_gps_sim_mode)
    {
        g_gps_sim_mode = 1U;
        GPS_SimStep1s();
    }
    else
    {
        g_gps_sim_mode     = 0U;
        g_gps.fix_valid    = 0U;
        g_gps.fix_quality  = 0U;
        g_gps.satellites   = 0U;
        g_gps.sats_in_view = 0U;
        g_gps.max_snr      = 0U;
        g_gps.lat_e6       = 0;
        g_gps.lon_e6       = 0;
        g_gps.latitude[0]  = '\0';
        g_gps.longitude[0] = '\0';
        str_copy(g_gps_coord,     "NO FIX", sizeof(g_gps_coord));
        str_copy(g_gps_maps_link, "NO FIX", sizeof(g_gps_maps_link));
    }
}
