/* Memobus settings Omron Varispeed E7 */
#define MEMOBUS_SLAVE_ID     0x01    /* H5-01 */
#define MEMOBUS_BITRATE      9600    /* H5-02 */
#define MEMOBUS_WAIT_TIME    (5 + 3) /* H5-06 + (24 bit / bitrate * 1000) ~ 3 */
#define MEMOBUS_CE_TIME      2000    /* H5-09 */
#define MEMOBUS_TIMEOUT      250     /* max duration of read */
#define MEMOBUS_LOOPBACK     0       /* Enable for loopback test */

#define RS485_RTS            7 /* RS485 Direction control*/
#define RS485_RX             2 /* RS485 TX pin for SoftwareSerial */
#define RS485_TX             3 /* RS485 RX pin for SoftwareSerial */
