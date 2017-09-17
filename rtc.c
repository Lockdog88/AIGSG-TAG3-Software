#include "rtc.h"

void rtc_wait_sleep(uint32_t time)
{
	debug_str("RTC Routine START");
	rtc_init(RTC_SET, time);
	power_soc_deep_sleep(POWER_WAKE_FROM_RTC);
	//rtc_init(RTC_CLEAR, 0);
	debug_str("RTC Routine END");
}

void rtc_wait(uint32_t time)
{
	uint32_t start = QM_RTC[QM_RTC_0].rtc_ccvr;
	while(QM_RTC[QM_RTC_0].rtc_ccvr < (start+time)){}
}

void rtc_init(rtc_setup state, uint32_t time)
{
	if(state) qm_rtc_set_alarm(QM_IRQ_RTC_0, QM_RTC[QM_RTC_0].rtc_ccvr+time);
	/*qm_rtc_config_t rtc;
	if (state)
	{
		clk_periph_enable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);

		rtc.init_val = 0;
		rtc.alarm_en = true;
		rtc.alarm_val = time;
		rtc.callback = NULL;
		rtc.callback_data = NULL;
		qm_rtc_set_config(QM_RTC_0, &rtc);

		qm_irq_request(QM_IRQ_RTC_0, qm_rtc_isr_0);

		debug_str("RTC Set");
	} else
	{
		rtc.init_val = 0;
		rtc.alarm_en = false;
		rtc.alarm_val = 0;
		rtc.callback = NULL;
		rtc.callback_data = NULL;
		qm_rtc_set_config(QM_RTC_0, &rtc);

		debug_str("RTC Clear");
	}*/
}

void rtc_hal_init(void)
{
	qm_rtc_config_t rtc;
	clk_periph_enable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);

	rtc.init_val = 0;
	rtc.alarm_en = true;
	rtc.alarm_val = 0;
	rtc.callback = NULL;
	rtc.callback_data = NULL;
	qm_rtc_set_config(QM_RTC_0, &rtc);

	qm_irq_request(QM_IRQ_RTC_0, qm_rtc_isr_0);

}
