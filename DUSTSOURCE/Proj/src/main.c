#include "DustSensor.h"

int main(void)
{
    DustSensor_Init();
    
    while(1)
    {
        DustSensor_Process();
    }
}


