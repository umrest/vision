#pragma once

#include <memory>
#include "TagPosition.h"

class VisionData
{
public:
    VisionData(TagPosition t1_in, TagPosition t2_in) : t1(t1_in), t2(t2_in), data(new char[128])
    {
    }

  
    char *Serialize()
    {

        data[0] = type;

        char tagdata[12];

        t1.Serialize(tagdata);
        for(size_t i = 0; i < 12; i++){
            data[TAG_1_OFFSET + i] = tagdata[i];  
        }

        t2.Serialize(tagdata);
        for(size_t i = 0; i < 12; i++){
            data[TAG_2_OFFSET + i] = tagdata[i];   
                 }
        

        return data.get();
    }

private:
    TagPosition t1;
    TagPosition t2;

    std::unique_ptr<char[]> data;

    const char type = 2;

    const int TAG_1_OFFSET = 1;
    const int TAG_2_OFFSET = 13;

};