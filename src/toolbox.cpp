#include <cmath>

         float flround(float number){
            float value = (int)(number * 100);
            return (float)value / 100;
        }

class toolbox
{
    public:
        static float round(float number){
            float value = (int)(number * 100);
            return (float)value / 100;
        }
};