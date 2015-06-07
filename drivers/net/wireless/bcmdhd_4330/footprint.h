#ifndef _bcm_foot_print_
#define _bcm_foot_print_

#include <mach/msm_iomap.h>
#define WIFI_FOOT_PRINT_MAGIC                            0x12233400
#define WIFI_FOOT_PRINT_BASE_CPU0_VIRT           (MSM_KERNEL_FOOTPRINT_BASE + 0x0)

static void set_wifi_foot_print(unsigned state, unsigned int offset)
{
       unsigned *status = (unsigned *)(WIFI_FOOT_PRINT_BASE_CPU0_VIRT + offset);
       *status = (WIFI_FOOT_PRINT_MAGIC | state);
       mb();
}

#endif 
