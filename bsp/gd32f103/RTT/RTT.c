#include "rtthread.h"
#include "SEGGER_RTT.H"

struct rt_device rtt_console;
rt_size_t rtt_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size);
rt_size_t rtt_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size);

void rtt_thread_entry(void* argu)
{
	while(1)
	{
		if(SEGGER_RTT_HasData(0))
		{
			if(rtt_console.rx_indicate != RT_NULL)
			{
				rt_size_t rx_length = SEGGER_RTT_HasData(0);
				rtt_console.rx_indicate(&rtt_console, rx_length);
			}
		}
		rt_thread_delay(1);
	}
}

int rtt_init(void)
{
	SEGGER_RTT_Init();
	SEGGER_RTT_ConfigUpBuffer(0, 0, 0, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
	rtt_console.type = RT_Device_Class_Miscellaneous;
	rtt_console.device_id = 0;
	rtt_console.write = rtt_write;
	rtt_console.read = rtt_read;
	rt_device_register(&rtt_console, "rtt", 0);
	return 0;
}
INIT_BOARD_EXPORT(rtt_init);

int rtt_init_t(void)
{
	rt_thread_t tid;
	tid = rt_thread_create("rtt_thread",
													rtt_thread_entry,
													RT_NULL,
													512,
													7,
													5);
	if(tid != RT_NULL)
	{
		rt_thread_startup(tid);
		return 0;
	}
	return -1;
}
INIT_APP_EXPORT(rtt_init_t);

rt_size_t rtt_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
	SEGGER_RTT_WriteString(0, buffer);
	return 0;
}

rt_size_t rtt_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
	return SEGGER_RTT_Read(0, buffer, size);
}

