#pragma once

#include <memory>
#include "TagPosition.h"

class VisionData {
    public:
    VisionData(TagPosition t_in) : t(t_in), data(new char[128]){

    }
    char* Serialize(){
		
		data[0] = type;

		char* X_data = static_cast<char*>(static_cast<void*>(&t.x));
		data[X_OFFSET] = X_data[0];
		data[X_OFFSET + 1] = X_data[1];

        char* Y_data = static_cast<char*>(static_cast<void*>(&t.y));
		data[Y_OFFSET] = Y_data[0];
		data[Y_OFFSET + 1] = Y_data[1];

        char* Z_data = static_cast<char*>(static_cast<void*>(&t.z));
		data[Z_OFFSET] = Z_data[0];
		data[Z_OFFSET + 1] = Z_data[1];

        char* YAW_data = static_cast<char*>(static_cast<void*>(&t.yaw));
		data[YAW_OFFSET] = YAW_data[0];
		data[YAW_OFFSET + 1] = YAW_data[1];

        char* PITCH_data = static_cast<char*>(static_cast<void*>(&t.pitch));
		data[PITCH_OFFSET] = PITCH_data[0];
		data[PITCH_OFFSET + 1] = PITCH_data[1];

        char* ROLL_data = static_cast<char*>(static_cast<void*>(&t.roll));
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