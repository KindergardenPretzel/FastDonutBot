#include <cmath>
#include "vex.h"


namespace toolbox {

    float fround(float number){
        float value = (int)(number * 100);
        return (float)value / 100;
    }

unsigned long long int highResTimerMs(){
    return (vex::timer::systemHighResolution())/1000;
}
}