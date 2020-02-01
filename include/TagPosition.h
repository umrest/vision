#pragma once
#include <iostream>
#include <math.h>

#include "Utils.h"

#define PI 3.14159265

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

struct FieldPosition
{
    private:
    // TAG 1 PARAMETERS
    const double FIELD_SIZE_Y = 3.6;
    const double FIELD_SIZE_X = 5.4;

    // Width of the Sieve
    const double T1_Y_OFFSET = .3; // m
    // From center of sieve to corner of field
    const double T1_X_OFFSET = 1.5; // m

    const double CAMERA_X_OFFSET = 0;
    const double CAMERA_Y_OFFSET = .4;
    public:

    FieldPosition(TagPosition& t1, TagPosition & t2){
        double r = sqrt(t1.z * t1.z + t1.x * t1.x);

        double theta = t1.yaw / 180 * PI + atan2(t1.z, t1.x);

        // Only using tag0 for now
        y = r*sin(theta) / 39.37; // in m
        x = r*cos(theta) / 39.37; // in m

        yaw = t1.yaw; // in deg
        
        

    }

    FieldPosition& operator=(FieldPosition& other){
		x = other.x;
		y = other.y;
		yaw = other.yaw;
		return *this;
	}

	double x = 0;
	double y = 0;

    double yaw = 0;

    void Serialize(char data[12]){
        short x_scaled = scale_xyz(x);
        short y_scaled = scale_xyz(y);
        
		char *X_data = static_cast<char *>(static_cast<void *>(&x_scaled));
        data[X_OFFSET] = X_data[0];
        data[X_OFFSET + 1] = X_data[1];

        char *Y_data = static_cast<char *>(static_cast<void *>(&y_scaled));
        data[Y_OFFSET] = Y_data[0];
        data[Y_OFFSET + 1] = Y_data[1];

        short yaw_scaled = scale(yaw);

        char *YAW_data = static_cast<char *>(static_cast<void *>(&yaw_scaled));
        data[YAW_OFFSET] = YAW_data[0];
        data[YAW_OFFSET + 1] = YAW_data[1];

        
        std::cout << x_scaled << " " << y_scaled << " " << yaw_scaled << std::endl;
	}
    

    const int YAW_OFFSET = 0;
    const int X_OFFSET = 2;
    const int Y_OFFSET = 4;
};

std::ostream& operator<<(std::ostream& os, const TagPosition& t);