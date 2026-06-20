
#include <zephyr/sys/poweroff.h>

#include <register.h>

void z_sys_poweroff(void)
{
	extern void HAL_PMU_EnterHibernate(void);

	__builtin_unreachable();
	// HAL_PMU_EnterHibernate();
}
