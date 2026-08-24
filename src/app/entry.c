#include "entry.h"

#include "chassis_service.h"

void entry_init(void)
{
    (void)chassis_service_init();
}

void entry_loop(void)
{
    (void)chassis_service_update();
}
