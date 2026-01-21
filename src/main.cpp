#include <Arduino.h>
#include <SimpleFOC.h>

// UART pins (TTL)
HardwareSerial Serialx(PC11, PC10);

// Motor & Driver
BLDCMotor motor = BLDCMotor(10, 0.5, 310); // 10 POLE PAIRS TOTAL / 2, 0.5 RESISTNACE FOR PHASE, 100 KV RATING
BLDCDriver6PWM driver = BLDCDriver6PWM(
  PA10, PB15,
  PA9,  PB14,
  PA8,  PB13
);

float target_velocity = 0;

// Commander
Commander command = Commander(Serialx);

void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}

void doLimitCurrent(char* cmd) {
  command.scalar(&motor.current_limit, cmd);
}

void setup() {
  Serialx.begin(115200);
  SimpleFOCDebug::enable(&Serialx);

  driver.voltage_power_supply = 24; //
  driver.voltage_limit = 36; //
  driver.pwm_frequency = 25000; 

  if (!driver.init()) {
    Serialx.println("Driver init failed!");
    while (1);
  }

  motor.linkDriver(&driver);

  //motor.current_limit = 1; // old one
    motor.current_limit = 2; // set current limit to 2A
    driver.voltage_limit = 12; // limit voltage to 12V
    //motor.controller = MotionControlType::velocity_openloop; // this is the old one it is dangerous
    motor.controller = MotionControlType::angle_openloop; // used angle open loop not velocity open loop

  if (!motor.init()) {
    Serialx.println("Motor init failed!");
    while (1);
  }

  command.add('T', doTarget, "target velocity");
  command.add('C', doLimitCurrent, "current limit");

  
  Serialx.println("Set target velocity [rad/s]");
}

void loop() {
  motor.move(target_velocity);
  command.run();
}
