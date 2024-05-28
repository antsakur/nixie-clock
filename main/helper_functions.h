#include "defines.h"

#define HOUR_HIGH_LD (1ULL << 47)
#define HOUR_HIGH_1  (1ULL << 46)
#define HOUR_HIGH_2  (1ULL << 45)
#define HOUR_HIGH_3  (1ULL << 44)
#define HOUR_HIGH_4  (1ULL << 43)
#define HOUR_HIGH_5  (1ULL << 42)
#define HOUR_HIGH_6  (1ULL << 41)
#define HOUR_HIGH_7  (1ULL << 40)
#define HOUR_HIGH_8  (1ULL << 39)
#define HOUR_HIGH_9  (1ULL << 38)
#define HOUR_HIGH_0  (1ULL << 37)
#define HOUR_HIGH_RD (1ULL << 36)

#define HOUR_LOW_LD (1ULL << 35)
#define HOUR_LOW_1  (1ULL << 34)
#define HOUR_LOW_2  (1ULL << 33)
#define HOUR_LOW_3  (1ULL << 32)
#define HOUR_LOW_4  (1ULL << 31)
#define HOUR_LOW_5  (1ULL << 30)
#define HOUR_LOW_6  (1ULL << 29)
#define HOUR_LOW_7  (1ULL << 28)
#define HOUR_LOW_8  (1ULL << 27)
#define HOUR_LOW_9  (1ULL << 26)
#define HOUR_LOW_0  (1ULL << 25)
#define HOUR_LOW_RD (1ULL << 24)

#define MINUTE_HIGH_LD (1ULL << 23)
#define MINUTE_HIGH_1  (1ULL << 22)
#define MINUTE_HIGH_2  (1ULL << 21)
#define MINUTE_HIGH_3  (1ULL << 20)
#define MINUTE_HIGH_4  (1ULL << 19)
#define MINUTE_HIGH_5  (1ULL << 18)
#define MINUTE_HIGH_6  (1ULL << 17)
#define MINUTE_HIGH_7  (1ULL << 16)
#define MINUTE_HIGH_8  (1ULL << 15)
#define MINUTE_HIGH_9  (1ULL << 14)
#define MINUTE_HIGH_0  (1ULL << 13)
#define MINUTE_HIGH_RD (1ULL << 12)

#define MINUTE_LOW_LD (1ULL << 11)
#define MINUTE_LOW_1  (1ULL << 10)
#define MINUTE_LOW_2  (1ULL << 9)
#define MINUTE_LOW_3  (1ULL << 8)
#define MINUTE_LOW_4  (1ULL << 7)
#define MINUTE_LOW_5  (1ULL << 6)
#define MINUTE_LOW_6  (1ULL << 5)
#define MINUTE_LOW_7  (1ULL << 4)
#define MINUTE_LOW_8  (1ULL << 3)
#define MINUTE_LOW_9  (1ULL << 2)
#define MINUTE_LOW_0  (1ULL << 1)
#define MINUTE_LOW_RD (1ULL << 0)

// Look up table for dot animation
uint64_t dot_animation_LUT[8] = {   MINUTE_LOW_RD,
                                    MINUTE_LOW_LD,
                                    MINUTE_HIGH_RD,
                                    MINUTE_HIGH_LD,
                                    HOUR_LOW_RD,
                                    HOUR_LOW_LD,
                                    HOUR_HIGH_RD,
                                    HOUR_HIGH_LD
                                    };

/***********************************************************/
/* Format serial data to be written to the shift registers */
/* Hours and minutes are extracted from timeinfo struct    */
/***********************************************************/
uint64_t formatTime(struct tm timeinfo)
{
    uint64_t ser_tm_data = 0;

    // Divide to get first digit of hours/minutes
    // Modulo to get second digit of hours/minutes
    uint64_t minute_high = (timeinfo.tm_min / 10 == 0) ? MINUTE_HIGH_0 : (uint64_t)1 << (13 + (10 - (timeinfo.tm_min / 10)));
    uint64_t minute_low  = (timeinfo.tm_min % 10 == 0) ? MINUTE_LOW_0 : (uint64_t)1 << (1 + (10 - (timeinfo.tm_min % 10)));

    uint64_t hour_high = (timeinfo.tm_hour / 10 == 0) ? HOUR_HIGH_0 : (uint64_t)1 << (37 + (10 - (timeinfo.tm_hour / 10)));
    uint64_t hour_low  = (timeinfo.tm_hour % 10 == 0) ? HOUR_LOW_0 : (uint64_t)1 << (25 + (10 - (timeinfo.tm_hour % 10)));

    // Bit positon 10 -> 0-digit, bit position 1 --> 9-digit
    return ser_tm_data = hour_high + hour_low + minute_high + minute_low;
}