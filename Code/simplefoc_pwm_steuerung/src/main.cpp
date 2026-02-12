#include <SimpleFOC.h>

//  BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
BLDCDriver6PWM driver = BLDCDriver6PWM(PA10, PB15, PA9, PB14, PA8, PB13);

// instantiate sensor 
void setup() {  
  Serial.begin(115200); // to output the debug information to the serial
  SimpleFOCDebug::enable(&Serial);
  // init sensor

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 25000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
  // dead_zone [0,1]                                    - default 0.02 - 2%
  driver.dead_zone = 0.05;
  // driver init
  Serial.print("Driver init ");
  // init driver
  if (driver.init())  Serial.println("success!");
  else{
    Serial.println("failed!");
    return;
  }
}


void loop() {
  driver.setPwm(6, 6, 6);
  /*_delay(1000);
  driver.setPwm(0, 0, 0);
  _delay(1000);*/
}