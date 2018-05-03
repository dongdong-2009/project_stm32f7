/*
 * File      : drv_led.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-08-01     xiaonong     the first version
 */
#include <rtthread.h>
#ifdef RT_USING_STEMWIN


#include <board.h>
#include <drv_ADC.h>
#include "drv_touch.h"

static void Touch_thread_entry(void *parameter)
{
	while (1)
    {
		k_TouchUpdate();
        rt_thread_delay(RT_TICK_PER_SECOND/100);
    }
}

int Touch_hw_init(void)
{
	return 0;
}
INIT_BOARD_EXPORT(Touch_hw_init);

int Touch_init(void)
{
    rt_thread_t tid;
	BSP_TS_Init(700, 384);

    tid = rt_thread_create("Touch",
                           Touch_thread_entry, RT_NULL,
                           800, RT_THREAD_PRIORITY_MAX-2, 5);

    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return 0;
}
INIT_APP_EXPORT(Touch_init);
#endif
