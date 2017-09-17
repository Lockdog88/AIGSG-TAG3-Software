#ifndef RTC_H_
#define RTC_H_

#include "qm_rtc.h"
#include "qm_isr.h"
#include "qm_interrupt.h"
#include "debug.h"
#include "power_states.h"

#define ALARM_SEC  (QM_RTC_ALARM_SECOND)
#define ALARM_MIN  (QM_RTC_ALARM_MINUTE)
#define ALARM_HOUR (QM_RTC_ALARM_HOUR)

typedef enum {
	RTC_CLEAR,
	RTC_SET,
} rtc_setup;

void rtc_wait_sleep(uint32_t time);
void rtc_wait(uint32_t time);
void rtc_init(rtc_setup state, uint32_t time);
void rtc_hal_init(void);

#endif /* RTC_H_ */
