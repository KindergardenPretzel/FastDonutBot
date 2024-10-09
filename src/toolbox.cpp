#include <cmath>

namespace toolbox {

    float fround(float number){
        float value = (int)(number * 100);
        return (float)value / 100;
    }

}