#include "Utils.h"

short scale(float in)
{
    if (in > 90)
    {
        in = 90;
    }
    if (in < -90)
    {
        in = -90;
    }
    return in * 364.0888;
}

short scale_xyz(float in)
{
    if (in > 3000)
    {
        in = 3000;
    }
    if (in < -3000)
    {
        in = -3000;
    }
    return in * 10;
}