#include "NutsLib.h"

void app_main(void)
{
    // timer watchdog and task watchdog disabled in menuconfig
    Nut_Init();

    while (1)
    {
        Nut_Loop();
    }
}
