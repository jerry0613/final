#include "mbed.h"
#include "bbcar.h"
#include "mbed_rpc.h"

Ticker servo_ticker;
PwmOut pin9(D9), pin8(D8);
BBCar car(pin8, pin9, servo_ticker);
DigitalInOut ping(D10);
RawSerial xbee(D12, D11);
Serial pc(USBTX,USBRX); //tx,rx
Serial uart(D1,D0); //tx,rx
Timer t;
float val;
int tmp = 0;
float object[3];

void rotate_r() {
   car.goStraight(46, -46); // rotate right
   wait(1);
   car.stop();
   wait(1); 
   xbee.printf("rotate right\r\n");
}
void rotate_l() {
    car.goStraight(-45, 45); // rotate left
    wait(1);
    car.stop();
    wait(1); 
    xbee.printf("rotate left\r\n");
}
void forward(float d) {
    car.goStraight(-96, -100); // go forward until distance < d
    while(1) {
        ping.output();
        ping = 0; wait_us(200);
        ping = 1; wait_us(5);
        ping = 0; wait_us(5);

        ping.input();
        while(ping.read()==0);
        t.start();
        while(ping.read()==1);
        val = t.read();
        printf("Ping = %lf\r\n", val*17700.4f);
        t.stop();
        t.reset();
        if (val*17700.4f < d)
            tmp++;
        else
            tmp = 0;
    
        if (tmp >= 3)
            break;
    }
    car.stop();
    wait(1);
    xbee.printf("go forward\r\n");
}

void forward2(float d) {
    car.goStraight(-100, -95); // go forward until distance < d
    while(1) {
        ping.output();
        ping = 0; wait_us(200);
        ping = 1; wait_us(5);
        ping = 0; wait_us(5);

        ping.input();
        while(ping.read()==0);
        t.start();
        while(ping.read()==1);
        val = t.read();
        printf("Ping = %lf\r\n", val*17700.4f);
        t.stop();
        t.reset();
        if (val*17700.4f < d)
            tmp++;
        else
            tmp = 0;
    
        if (tmp >= 3)
            break;
    }
    car.stop();
    wait(1);
    xbee.printf("go forward\r\n");
}

void straight(float t) {
    car.goStraight(-100, -95); // go forward in t sec
    wait(t);
    xbee.printf("go forward\r\n");
    car.stop();
    wait(1);
}

void detect(int i) {
    ping.output();
    ping = 0; wait_us(200);
    ping = 1; wait_us(5);
    ping = 0; wait_us(5);

    ping.input();
    while(ping.read()==0);
    t.start();
    while(ping.read()==1);
    val = t.read();
    printf("Ping = %lf\r\n", val*17700.4f);
    object[i] = val*17700.4f;
    t.stop();
    t.reset();
}

int main() {
    
    pc.baud(9600);
    xbee.baud(9600);
    char s[21];
    /*while(1) {
        car.stop();
    }*/
   
    forward(22); // go forward
    
    rotate_l();
    
    xbee.printf("enter mission 1\r\n");
    forward(30);
    rotate_r();

    car.goStraight(100, 85); // go forward until distance < d
    while(1) {
        ping.output();
        ping = 0; wait_us(200);
        ping = 1; wait_us(5);
        ping = 0; wait_us(5);

        ping.input();
        while(ping.read()==0);
        t.start();
        while(ping.read()==1);
        val = t.read();
        printf("Ping = %lf\r\n", val*17700.4f);
        t.stop();
        t.reset();
        if (val*17700.4f > 22)
            tmp++;
        else
            tmp = 0;
    
        if (tmp >= 3)
            break;
    }
    car.stop();
    wait(1);
    xbee.printf("go backward\r\n");

    sprintf(s,"image_classification");
    uart.puts(s);
    xbee.printf("take photo\r\n");
    wait(2);
    char recv = uart.getc();
    xbee.printf("%c\r\n", recv);
    

    forward(22);

    rotate_r();
    straight(1.5);
    rotate_l();

    car.goStraight(100, 85); // go backward
    wait(2);
    xbee.printf("go backward\r\n");
    car.stop();
    wait(1);
    xbee.printf("park\r\n");

    forward(22);
    rotate_r();
    //straight(2.5);
    car.goStraight(-100, -95);
    wait(1);
    forward(35);
    rotate_r();
    xbee.printf("leave mission 1\r\n");

    car.goStraight(-100, -95);
    wait(2);
    forward2(24);
    rotate_r();
    xbee.printf("enter mission 2\r\n");

    straight(2.8);
    rotate_r();
    forward(28);
    
    detect(0);

    car.goStraight(-22, 22); // rotate left
    wait(1);
    car.stop();
    detect(1);
    wait(1); 
    xbee.printf("rotate left\r\n");

    car.goStraight(38, -38); // rotate right
    wait(1);
    car.stop();
    wait(1); 
    xbee.printf("rotate right\r\n");
    detect(2);

    car.goStraight(-20, 20); // rotate left
    wait(1);
    car.stop();
    detect(1);
    wait(1); 
    xbee.printf("rotate left\r\n");
    
    xbee.printf("image detection\r\n");
    
    rotate_l();
    rotate_l();
    forward(20);
    rotate_r();
    car.goStraight(-100, -95);
    wait(1);
    forward(22);
    rotate_r();
    car.goStraight(-96, -100);

    if (object[1] < object[0] && object[0] < object[2])
        xbee.printf("upper triangle\r\n");
    else if (object[0] < object[1] && object[0] < object[2]) {
        if (object[1] - object[0] < 10 && object[2] - object[0] < 10)
            xbee.printf("square\r\n");
        else
        {
            xbee.printf("normal triangle\r\n");
        }
        
    }
    else if (object[0] > object[1] && object[0] > object[2])
        xbee.printf("double triangle\r\n");
    else
    {   
        xbee.printf("normal triangle\r\n");
    }
    // behind: 100 85
    // forward: -98 -87
    // turn right: wait 2, 0 -48
    // turn left: wait 2, -41 0
    // turn backwaerd right: wait 2, 48 0
    // turn backwaerd left: wait 2, 0 44
    
    while(1){    
    }
}
