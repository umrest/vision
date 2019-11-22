#pragma once

#include <memory>
#include "TagPosition.h"

class VisionData
{
public:
    VisionData(TagPosition t_in) : t(t_in), data(new char[128])
    {
    }

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
    char *Serialize()
    {

        data[0] = type;

        short x_scaled = scale_xyz(t.x);
        short y_scaled = scale_xyz(t.y);
        short z_scaled = scale_xyz(t.z);

        char *X_data = static_cast<char *>(static_cast<void *>(&x_scaled));
        data[X_OFFSET] = X_data[0];
        data[X_OFFSET + 1] = X_data[1];

        char *Y_data = static_cast<char *>(static_cast<void *>(&y_scaled));
        data[Y_OFFSET] = Y_data[0];
        data[Y_OFFSET + 1] = Y_data[1];

        char *Z_data = static_cast<char *>(static_cast<void *>(&z_scaled));
        data[Z_OFFSET] = Z_data[0];
        data[Z_OFFSET + 1] = Z_data[1];

        short yaw_scaled = scale(t.yaw);
        short pitch_scaled = scale(t.pitch);
        short roll_scaled = scale(t.roll);

        char *YAW_data = static_cast<char *>(static_cast<void *>(&yaw_scaled));
        data[YAW_OFFSET] = YAW_data[0];
        data[YAW_OFFSET + 1] = YAW_data[1];

        char *PITCH_data = static_cast<char *>(static_cast<void *>(&pitch_scaled));
        data[PITCH_OFFSET] = PITCH_data[0];
        data[PITCH_OFFSET + 1] = PITCH_data[1];

        char *ROLL_data = static_cast<char *>(static_cast<void *>(&roll_scaled));
        data[ROLL_OFFSET] = ROLL_data[0];
        data[ROLL_OFFSET + 1] = ROLL_data[1];

        return data.get();
    }

private:
    TagPosition t;

    std::unique_ptr<char[]> data;

    const char type = 2;

    // Offsets
    const int YAW_OFFSET = 1;
    const int PITCH_OFFSET = 3;
    const int ROLL_OFFSET = 5;
    const int X_OFFSET = 7;
    const int Y_OFFSET = 9;
    const int Z_OFFSET = 11;
};