#include "PinNames.h"
#include "ThisThread.h"
#include "mbed.h"
#include <cmath>
#include "hcsr04.h"
//#define n 10
//#define BLINKING_RATE_MS
//#define BUFFERDSIZZE 5

const double wheel_period = 0.01;
const double wheel_r = 5;

PwmOut motor1_1(PB_3);  // D3
PwmOut motor1_2(PB_4);  // D5
PwmOut motor2_1(PB_10); // D6
PwmOut motor2_2(PC_7);  // D9
PwmOut motor3_1(PB_6);  // D10
PwmOut motor3_2(PA_7);  // D11

HCSR04 sensor(PC_8, PC_9);

I2CSlave slave(D14, D15);

double Kp = 0.025; // 比例係数
double Ki = 0.005; // 積分係数
double Kd = 0.01; // 微分係数

float dis = 0;  // 測定距離
float Target = 35;  // 停止距離(cm)
//float dt, preTime;
float err = 0, integ = 0, differ = 0, pre_err = 0, err_r = 0, integ_r = 0, differ_r = 0, pre_err_r = 0;
volatile double duty, duty_r = 0;//
//int f[n] = {0};
double Kp_r = 0.025; // 比例係数
double Ki_r = 0.005; // 積分係数
double Kd_r = 0.01; // 微分係数

float dis_r = 0;  // 測定距離
float Target_r = 54;  // 停止距離(cm)

/*
  int trig = 3; // 出力ピン
  int echo = 2; // 入力ピン
  int led  = 4;//LEDインジケーター
*/

//static UnbufferedSerial serial_port(PC_10, PC_11);
//static UnbufferedSerial *pc;

// Ticker ticker_robot;

void drive_wheel(double *speed); // speed of x,y,r on the base coordinate system
// of drive base
void drive_robot();

void pid_d();
void pid_r();

double speed_base[3] = {0}; // x, y, r

//int i2c_buf[4] = {0};
//int i2c_buf_old[4] = {0};
//int i2c_counter = 0;

int x_g = 0;


// main() runs in its own thread in the OS
int main() {
  motor1_1.period(wheel_period);
  motor1_2.period(wheel_period);
  motor2_1.period(wheel_period);
  motor2_2.period(wheel_period);
  motor3_1.period(wheel_period);
  motor3_2.period(wheel_period);

  //pc= new UnbufferedSerial(USBTX, USBRX,9600);
  slave.address(0x53);

  int testA = 0;
  //   ticker_robot.attach(&drive_robot, 20ms);
  int caseA = 0;
  while (1) {
    x_g = slave.read();

    //if(x_g!=-1){
    if (caseA == 0) {
      double ddd = (((double)x_g - 127.0) / 127.0);
      if (x_g != -1) {
          /*
        if (ddd > 0.4) {
          ddd = 0.4;
        }
        */

        //for (int j = 0; j < 3; j++) {
          speed_base[0] = ddd;
          speed_base[1] = 0;
          speed_base[2] = 0;
        //}

        printf("x_g:%d", x_g);
        printf("\n");
        printf("speed2:%f", speed_base[2]);
        printf("\n");
        ThisThread::sleep_for(10ms);
      } 
      
      /*
      else {
        for (int j = 0; j < 3; j++) {
          speed_base[0] = 0;
          speed_base[1] = 0;
          speed_base[2] = 0;
        }
      }
      */
      

      if ( x_g == 127) {
        caseA = 1;
      }
    } else if (caseA == 1) {
      sensor.start();
      ThisThread::sleep_for(10ms);
      dis = sensor.get_dist_cm();
      //   printf("%d\n", sensor.get_dist_cm());

      if (dis < 1203.0) {
        printf("dis:%f \n", dis);

        pid_d();
        //for (int j = 0; j < 3; j++) {
          speed_base[0] = 0;
          speed_base[1] = duty;
          speed_base[2] = 0;
        //}


        drive_wheel(speed_base);
        if (dis == Target) {
          caseA = 0;
        }
      }
    } else {
      speed_base[0] = 0;
      speed_base[1] = duty;
      speed_base[2] = 0;
    }

    //}

    //double temp[3] = {0.0, 0.0, 0.1};

    drive_wheel(speed_base);

  }
}



void drive_wheel(double *speed) {

  // printf("%f %f %f \n", speed[0], speed[1], speed[2]);

  double speed_wheel[3] = {
    // the speed of each motor
    1.0 * speed[0] + 0.0 * speed[1] + wheel_r * speed[2],
    -0.5 * speed[0] + sqrt(3) / 2.0 * speed[1] + wheel_r * speed[2],
    -0.5 * speed[0] - sqrt(3) / 2.0 * speed[1] + wheel_r * speed[2]
  };

    // double speed_wheel[3] = {0.3, 0.3, 0.3};

  for (int i = 0; i < 3; i++) {
    if (speed_wheel[i] <= -0.99) {
      speed_wheel[i] = -0.99;
    } else if (speed_wheel[i] >= 0.99) {
      speed_wheel[i] = 0.99;
    }
  }

  if (speed_wheel[0] < 0) {
    motor1_1.write(0);
    motor1_2.write(-speed_wheel[0]);
  } else {
    motor1_1.write(speed_wheel[0]);
    motor1_2.write(0);
  }

  if (speed_wheel[1] < 0) {
    motor2_1.write(0);
    motor2_2.write(-speed_wheel[1]);
  } else {
    motor2_1.write(speed_wheel[1]);
    motor2_2.write(0);
  }

  if (speed_wheel[2] < 0) {
    motor3_1.write(0);
    motor3_2.write(-speed_wheel[2]);
  } else {
    motor3_1.write(speed_wheel[2]);
    motor3_2.write(0);
  }
}

/*
  void drive_robot() {
  drive_wheel(speed_base);
  // todo: more drive functions
  }
*/

void pid_d() {
  //float val;

  err  = dis - Target;
  integ += err;
  differ  = err - pre_err;
  pre_err = err;

  duty = Kp * err + Ki * integ + Kd * differ;

  if (duty > 0.2) {
    duty = 0.2;
  }
  //printf("duty:%f \n", duty);
}