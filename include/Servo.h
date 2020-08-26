#include <wiringPi.h>


   

class Servo {
    public:
       double cur_angle = 0;
       int pin = 11;

       double ms(){
           return (cur_angle / 180) * 200;
       }
    Servo(){
        
       wiringPiSetupGpio();

        pinMode (pin, PWM_OUTPUT) ;
        pwmSetMode (PWM_MODE_MS);
        pwmSetRange (2000);
        pwmSetClock (192);
    }

    void write(double angle){
        cur_angle = angle;
        pwmWrite(pin,ms() / 10);
        
        
    }


};