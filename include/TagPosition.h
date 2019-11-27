#pragma once
#include <iostream>

struct TagPosition
{
	TagPosition& operator=(TagPosition& other){
		x = other.x;
		y = other.y;
		z = other.z;
		roll = other.roll;
		pitch = other.pitch;
		yaw = other.yaw;
		return *this;
	}

	float x = 0;
	float y = 0;
	float z = 0;

	float roll = 0;
	float pitch = 0;
	float yaw = 0;

    void reset(){
        x = 0;
	    y = 0;
	    z = 0;

	    roll = 0;
	    pitch = 0;
	    yaw = 0;
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

	void Serialize(char data[12]){
		short x_scaled = scale_xyz(x);
        short y_scaled = scale_xyz(y);
        short z_scaled = scale_xyz(z);

        char *X_data = static_cast<char *>(static_cast<void *>(&x_scaled));
        data[X_OFFSET] = X_data[0];
        data[X_OFFSET + 1] = X_data[1];

        char *Y_data = static_cast<char *>(static_cast<void *>(&y_scaled));
        data[Y_OFFSET] = Y_data[0];
        data[Y_OFFSET + 1] = Y_data[1];

        char *Z_data = static_cast<char *>(static_cast<void *>(&z_scaled));
        data[Z_OFFSET] = Z_data[0];
        data[Z_OFFSET + 1] = Z_data[1];

        short yaw_scaled = scale(yaw);
        short pitch_scaled = scale(pitch);
        short roll_scaled = scale(roll);

        char *YAW_data = static_cast<char *>(static_cast<void *>(&yaw_scaled));
        data[YAW_OFFSET] = YAW_data[0];
        data[YAW_OFFSET + 1] = YAW_data[1];

        char *PITCH_data = static_cast<char *>(static_cast<void *>(&pitch_scaled));
        data[PITCH_OFFSET] = PITCH_data[0];
        data[PITCH_OFFSET + 1] = PITCH_data[1];

        char *ROLL_data = static_cast<char *>(static_cast<void *>(&roll_scaled));
        data[ROLL_OFFSET] = ROLL_data[0];
        data[ROLL_OFFSET + 1] = ROLL_data[1];
	}

	    // Offsets
    const int YAW_OFFSET = 0;
    const int PITCH_OFFSET = 2;
    const int ROLL_OFFSET = 4;
    const int X_OFFSET = 6;
    const int Y_OFFSET = 8;
    const int Z_OFFSET = 10;
};


std::ostream& operator<<(std::ostream& os, const TagPosition& t);