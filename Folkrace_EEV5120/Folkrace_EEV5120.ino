#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ezButton.h>
#include <FastLED.h>
#include <bits/stdc++.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Encoder.h>

#define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_NB_TARGET_DETECTED
#define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L5CX_DISABLE_RANGE_SIGMA_MM
#define VL53L5CX_DISABLE_DISTANCE_MM
#define VL53L5CX_DISABLE_TARGET_STATUS
#define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
#define VL53L5CX_DISABLE_MOTION_INDICATOR

#define ENC4 5 //Left motor encoder data pin 1
#define ENC3 6 //Left motor encoder data pin 2 
#define ENC1 7 //Right motor encoder data pin 1
#define ENC2 8 //Right motor encoder data pin 2
#define SERVOPWM 9 //Addressable LED strip data pin
#define RST8 10 //I2C data transfer shutdown for sensor 8
#define RST7 11 //I2C data transfer shutdown for sensor 7
#define RST6 12 //I2C data transfer shutdown for sensor 6
#define RST5 13 //I2C data transfer shutdown for sensor 5
#define Sensorpower 14 //Mosfet pin for turning sensor power on/off
#define SCL1 16 //I2C 1 clock pin
#define SDA1 17 //I2C 1 data pin
#define RST1 20 //I2C data transfer shutdown for sensor 1
#define RST2 21 //I2C data transfer shutdown for sensor 2
#define RST3 22 //I2C data transfer shutdown for sensor 3
#define RST4 23 //I2C data transfer shutdown for sensor 4
#define SCL2 24 //I2C 2 clock pin
#define SDA2 25 //I2C 2 data pin
#define AIN1 26 //REVERSE HIGH Right Motor
#define BIN1 27 //REVERSE HIGH Left Motor
#define PWM2 28 //Speed (0-255) Left Motor
#define PWM1 29 //Speed (0-255) Right Motor
#define EN1 30 //Enabled/Disabled Right Motor
#define EN2 31 //Enabled/Disabled Left Motor
#define BIN2 32 //FORWARD HIGH Left Motor
#define AIN2 37 //FORWARD HIGH Right Motor
#define LED 33 //Servo pwm output
#define RX 34 //Receive data from BT module
#define TX 35 //Transmit data to BT module
#define STATE 36 //BT module connection state
#define AIN2 37 //FORWARD HIGH Right Motor
#define RBTN 38 //Right pushbutton
#define LBTN 39 //Left pushbutton

const int sensor_data_points_per_line = 4; //Näitab mitut kauguspunkti iga anduri puhul kasutame
const int sensor_count = 6; //Kaugusandurite arv
const int led_count = sensor_count * sensor_data_points_per_line; //Kasutuses olevate LEDide arv
const int led_padding = (8*sensor_data_points_per_line - led_count)/2; //Üleliigsete LEDide arv

bool motorON = false; //M
bool steeringON = false;
bool kitMode = false;
bool once = false;

Servo servo;
Encoder LeftMotorEnc(ENC1, ENC2);
Encoder RightMotorEnc(ENC4, ENC3);

class Motor { //motors class
  public:                                     
    int pwr_mode;                   
    int speed_var;                  
    int direction_var;              
    int forward_pin;
    int reverse_pin;
    int speed_pin;
    int pwr_pin;
    long previousEnc;
    
    Motor(int pwr, int s_var, int d_var, int f_pin, int r_pin, int s_pin, int p_pin) { // Constructor with parameters
      pwr_mode = pwr;
      speed_var = s_var;
      direction_var = d_var;
      forward_pin = f_pin;
      reverse_pin = r_pin;
      speed_pin = s_pin;
      pwr_pin = p_pin;
      previousEnc = 0;
      MotorSetup();
    }

    void MotorSetup(){
      if(pwr_mode == 1){
        setPowerUp();
      }
      else{
        setPowerDown();
      }
      if(direction_var == 1){
        setForward();
      }
      else{
        setReverse();
      }
      analogWrite(speed_pin, speed_var);
    }

    void setForward(){
      direction_var = 1;
      digitalWrite(forward_pin, HIGH);
      digitalWrite(reverse_pin, LOW);
    }

    void setReverse(){
      direction_var = 0;
      digitalWrite(forward_pin, LOW);
      digitalWrite(reverse_pin, HIGH);
    }

    void setPowerUp(){
      pwr_mode = 1;
      digitalWrite(pwr_pin, HIGH);
    }

    void setPowerDown(){
      pwr_mode = 0;
      digitalWrite(pwr_pin, LOW);
    }

    void setMotorSpeed(int speed_value, long currentEnc, unsigned long Time){
      if(motorON){
        int speedTreshold = 5;
        if(direction_var == 0){
          currentEnc = currentEnc * -1;
        }
        double distance = ((currentEnc - previousEnc)/464.64)*3.14*6; //distantsi arvutus 
        previousEnc = currentEnc;
        Serial.print("Distance: ");
        Serial.println(distance);
        Serial.print("Time: ");
        Serial.println(Time);
        double currentSpeed = distance/(Time/1000.0);
        Serial.print("Speed: ");
        Serial.println(currentSpeed);
        if (currentSpeed - speedTreshold < speed_value){
          speed_var += 5;
        }
        else if (currentSpeed + speedTreshold > speed_value){
          speed_var -= 5;
        }
  
        if (speed_var < 80){speed_var = 80;}
        if (speed_var > 255){speed_var = 255;}
        Serial.print("Speed var: ");
        Serial.println(speed_var);
        analogWrite(speed_pin, speed_var);
      }
    }


    void setMotorSpeed2(int speed_value){
      speed_var = speed_value;
      analogWrite(speed_pin, speed_var);
    }
};

  //////MOTOR//////
  //Set to drive forward
  float L_motorspeed = 75; //speed in cm/s
  float R_motorspeed = 75; //speed in cm/s
  Motor motorLeft(0, L_motorspeed, 1, BIN2, BIN1, PWM2, EN2); //PowerMode(0 = Off / 1 = On), Speed(0-255), Direction(0 = Reverse / 1 = Forward), forward pin, reverse pin, speed pin, power pin 
  Motor motorRight(0, R_motorspeed, 1, AIN2, AIN1, PWM1, EN1);
  
//Distance sensors data structure
struct Sensor { 
    SparkFun_VL53L5CX sensor;
    int sensorReset;
    int sensorAddress;
    VL53L5CX_ResultsData results;
    TwoWire i2c;
};

//Distance sensors array
Sensor sensors[sensor_count] = { 
          {SparkFun_VL53L5CX(), RST3, 0x28, VL53L5CX_ResultsData(), Wire1}
        , {SparkFun_VL53L5CX(), RST4, 0x27, VL53L5CX_ResultsData(), Wire1}
        , {SparkFun_VL53L5CX(), RST2, 0x26, VL53L5CX_ResultsData(), Wire1}   
        , {SparkFun_VL53L5CX(), RST6, 0x25, VL53L5CX_ResultsData(), Wire1}
        , {SparkFun_VL53L5CX(), RST8, 0x24, VL53L5CX_ResultsData(), Wire1}
        , {SparkFun_VL53L5CX(), RST7, 0x23, VL53L5CX_ResultsData(), Wire1}
};

//Distance sensors setup
void CameraSetup () {

  pinMode(Sensorpower, OUTPUT);
  for (Sensor &sensor : sensors) {
    pinMode(sensor.sensorReset, OUTPUT);
    digitalWrite(sensor.sensorReset, HIGH);
  }
  Serial.println("OFF");
  digitalWrite(Sensorpower, LOW);
  delay(50);
  Serial.println("ON");
  digitalWrite(Sensorpower, HIGH);
  delay(50);

  int i = 0;
  for (Sensor &sensor : sensors) {
      digitalWrite(sensor.sensorReset, LOW);
      delay(100);
      char buf[100];
      if (!sensor.sensor.begin(0x29, sensor.i2c)) {
          sprintf(buf, "Failed to begin sensor %d", i);
          Serial.println(F(buf));
          while (1);
      } else {
          sprintf(buf, "Sensor %d started successfully", i);
          Serial.println(F(buf));
          memset(buf, 0, sizeof buf);
          sprintf(buf, "Sensor %d address: 0x%02X", i, sensor.sensorAddress);
          Serial.print(F(buf));
          if (sensor.sensor.setAddress(sensor.sensorAddress)) {
              sprintf(buf, "Sensor %d address changed to 0x%02X", i, sensor.sensor.getAddress());
              Serial.println(F(buf));
          }
      }
      i++;
  }
}  

void SensorResolutionStuff(SparkFun_VL53L5CX sensor){
  int SensorFrequency = 30; //4*4 max 60, 8*8 max 15
  sensor.setResolution(4 * 4);
  sensor.setRangingFrequency(SensorFrequency);
  //sensor.setWireMaxPacketSize(128);
  //sensor.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::CLOSEST);
  sensor.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
  //sensor.setSharpenerPercent(0);
  sensor.startRanging();
}

void CameraInit(){ 
  for(Sensor &sensor : sensors){ 
    SensorResolutionStuff( sensor.sensor ); 
  } 
}

void CameraGetData() {
  //Poll sensor for new data
  for(Sensor &sensor : sensors){ 
    if (sensor.sensor.isDataReady()) {
      sensor.sensor.getRangingData(&sensor.results);
    }
  }
}

int sensorCounter = 0;
int sensorArray[led_count];
int sensorMemory[5][led_count];
CRGB ledArray[led_count + led_padding];

#define LedBrightness 20
float RedLed = 0;
float GreenLed = 0;

//Distance sensors data processing
void OneDataLineArray(int row) {
  int count = 0;
  for(Sensor &sensor : sensors){
    for(int x = 0; x < sensor_data_points_per_line; x++){
      sensorArray[count] = sensor.results.distance_mm[x + sensor_data_points_per_line * row];
      count++;
    }
  }
  
  LedData();
}

void LedData() {
  for (int i = 0; led_count > i; i++) { 
    
    RedLed = (sensorArray[i]/(2550.0/LedBrightness));
    if(RedLed > 255){
      RedLed = 255;
    }
    GreenLed = LedBrightness - (sensorArray[i]/(2550.0/LedBrightness));
    if(GreenLed > 255){
      GreenLed = 255;
    }
    if(led_padding > 0){
      ledArray[led_padding + i] = CRGB(RedLed, GreenLed, 0);
    }
    else{
      ledArray[i] = CRGB(RedLed, GreenLed, 0); 
    }
  }
  FastLED.show();
}

void PrintNum(int num){
  Serial.print("\t");
  Serial.print(num);
  Serial.print(":");
}
void PrintCameraData(int num, VL53L5CX_ResultsData sensor, int imageWidth){
  for (int x = 0 ; imageWidth > x ; x++) {
    for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth) {
      PrintNum(num);
      Serial.print(sensor.distance_mm[x + y]);
    }
    Serial.println();
  }
  Serial.println();
}
void PrintAllCameraData() {
  int imageResolution = 0; //Used to pretty print output
  int imageWidth = 0; //Used to pretty print output
  imageResolution = sensors[0].sensor.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  for(int i = 0; i < sensor_count; i++){
    //PrintCameraData(i, measurementDatas[i-1]);
    PrintCameraData(i, sensors[i].results, imageWidth);
  }
}

void turn(int turn_degrees){
  if(steeringON){
    int OldMax = 90;
    int OldMin = -90;
    if(turn_degrees > 90){
      turn_degrees = 90;
    }
    if(turn_degrees < -90){
      turn_degrees = -90;
    }
    int NewMax = 166;
    int NewMin = 16;
    int OldRange = (OldMax - OldMin);
    int NewRange = (NewMax - NewMin);  
    int NewValue = (((turn_degrees - OldMin) * NewRange) / OldRange) + NewMin;
    servo.write(NewValue);
    delay(1);
  }
}

const int followWallPixel = 8;
const int maxWallDistance = 350;
const int minWallDistance = 300;
const int maxViewDistance = 1100;
int frontWallDistance = 200;
int minDistance = 0;
int minPixel = 0;
int maxDistance = 0;
int maxPixel = 0;
int cappedSensorLine[led_count];
float steering_angle = 0;
float error = 0;
float lastError = 0;
float P = 7;  //MIN_MAX steeringu puhul 7, teistel 0.7
float D = 2;  //MIN_MAX steeringu puhul 2, teistel 0.1
float P_error = 0;
float D_error = 0;
unsigned long lastTime;
unsigned long elapsedTime;
bool lastDrivingForward = true;
int maxWallZone = 570;
int minWallZone = 300;
int treshold = 90;
int divider = (maxWallZone - minWallZone) / 90;
int leftCenterPixel = 11;
int rightCenterPixel = 12;
int minPixel2 = leftCenterPixel;
int maxPixel2 = rightCenterPixel;
const int minFrontWallDistance = 100;
float scaler = frontWallDistance - minFrontWallDistance;
float lastAngle = 0;
float pixleAngle = 11.25;

void reverse2(int angle, unsigned long reverseTime, int reverseSpeed, unsigned long elapsedTime){
      motorLeft.setReverse();
      motorRight.setReverse();
      motorLeft.setMotorSpeed(reverseSpeed, LeftMotorEnc.read(), elapsedTime);
      motorRight.setMotorSpeed(reverseSpeed, RightMotorEnc.read(), elapsedTime);
      if(lastDrivingForward){
        turn(0);
      }
      delay(reverseTime);
      turn(angle);
      
      delay(2*reverseTime);
      motorLeft.setForward();
      motorRight.setForward();
}

void Steering_easy_if_else(){
  frontWallDistance = 150;
  int L_reverseSpeed = 85; //speed in cm/s
  int R_reverseSpeed = L_reverseSpeed; //speed in cm/s
  int reverseTime = 200;
  float turnMultiplier = 1.0;

  elapsedTime = millis()-lastTime;
  lastTime = millis();
  
  //0 1 2 3 4 5 6 7 8 9 10 (11 12) 13 14 15 16 17 18 19 20 21 22 23
  if (sensorArray[11] < frontWallDistance || sensorArray[12] < frontWallDistance){
    if(lastDrivingForward){
      elapsedTime = 0;
    }
    int angle;
    if (sensorArray[2] > sensorArray[21]){
      reverse2(90, reverseTime, L_reverseSpeed, elapsedTime);
    }
    else {
      reverse2(-90, reverseTime, L_reverseSpeed, elapsedTime);
    }
    lastDrivingForward = false;
  }
  else {
    if(!lastDrivingForward){
      elapsedTime = 0;
    }
    lastDrivingForward = true;
    motorLeft.setMotorSpeed(L_motorspeed, LeftMotorEnc.read(), elapsedTime);
    motorRight.setMotorSpeed(R_motorspeed, RightMotorEnc.read(), elapsedTime);
    for (int i = 0; i < led_count; i++){
      if (sensorArray[i] < maxViewDistance){
        cappedSensorLine[i] = sensorArray[i];
      } 
      else {
        cappedSensorLine[i] = maxViewDistance;
      }
    }

    if(L_motorspeed/100.0 > 1){
      turnMultiplier = 1.8;
    }

    int closest = 4000;
    if (cappedSensorLine[6] < maxWallZone){
      closest = maxWallZone - cappedSensorLine[6];
      steering_angle = ((maxWallZone - cappedSensorLine[6]) / divider) * turnMultiplier;
    }
    else if (cappedSensorLine[17] < maxWallZone && maxWallZone - cappedSensorLine[17] < closest){
      closest = maxWallZone - cappedSensorLine[17];
      steering_angle = -((maxWallZone - cappedSensorLine[17]) / divider) * turnMultiplier;
    }
    else if (cappedSensorLine[9] < maxWallZone && maxWallZone - cappedSensorLine[9] < closest){
      closest = maxWallZone - cappedSensorLine[9];
      steering_angle = ((maxWallZone - cappedSensorLine[9]) / divider) * turnMultiplier;
    }
    else if (cappedSensorLine[14] < maxWallZone && maxWallZone - cappedSensorLine[14] < closest){
      steering_angle = -((maxWallZone - cappedSensorLine[14]) / divider) * turnMultiplier;
    }
    else {
      steering_angle = 0;
    }

    turn(steering_angle);
  }
}

unsigned long date_time = 0;
int counter = 0;

ezButton leftBTN(LBTN);
ezButton rightBTN(RBTN);

int speed_value = 0;
int angle_value = 0;
double P_value = 0;
double I_value = 0;
double D_value = 0;

void kit(bool countdown, long waitTime) {
    int FPS = 10;
    long startTime = 0;
    Serial.println();
    if (countdown){
      startTime = millis();
    }
    while (kitMode) {
        if (countdown){
          Serial.print("countdown: ");
          Serial.print(startTime + waitTime - millis());
          Serial.println();
          if (startTime + waitTime < millis()){
            kitMode = false;
            once = true;
          }
        }     
        
        Serial.print("kitmode: ");
          Serial.print(kitMode);
          Serial.print(!once);
          Serial.println();
        int i = 0;
        if (kitMode == true && !once) {
          Serial.print("KIT");
            if (i == 0) {
                for (i; i < led_count + led_padding*2; i++) {
                    if (i > 2) {
                        ledArray[i - 3] = CRGB(0, 0, 0);
                        ledArray[i - 2] = CRGB(0, 230, 0);
                        ledArray[i - 1] = CRGB(0, 215, 0);
                        ledArray[i] = CRGB(0, 200, 0);
                        FastLED.show();
                        delay(FPS);
                    }
                    else {
                        if (i == 0) {
                            ledArray[i] = CRGB(0, 200, 0);
                            FastLED.show();
                            delay(FPS);
                        }
                        else if (i == 1) {
                            ledArray[i - 1] = CRGB(0, 215, 0);
                            ledArray[i] = CRGB(0, 200, 0);
                            FastLED.show();
                            delay(FPS);
                        }
                        else if (i == 2) {
                            ledArray[i - 2] = CRGB(0, 230, 0);
                            ledArray[i - 1] = CRGB(0, 215, 0);
                            ledArray[i] = CRGB(0, 200, 0);
                            FastLED.show();
                            delay(FPS);
                        }
                    }
                }
            }
            if (i == led_count + led_padding*2) {
                for (i; i > -1; i--) {
                    if (i < led_count + led_padding*2 - 2) {
                        ledArray[i + 3] = CRGB(0, 0, 0);
                        ledArray[i + 2] = CRGB(0, 230, 0);
                        ledArray[i + 1] = CRGB(0, 215, 0);
                        ledArray[i] = CRGB(0, 200, 0);
                        FastLED.show();
                        delay(FPS);
                    }
                    else {
                        if (i == led_count + led_padding*2 - 1) {
                            ledArray[i] = CRGB(0, 200, 0);
                            FastLED.show();
                            delay(FPS);
                        }
                        else if (i == led_count + led_padding*2 - 2) {
                            ledArray[i + 1] = CRGB(0, 215, 0);
                            ledArray[i] = CRGB(0, 200, 0);
                            FastLED.show();
                            delay(FPS);
                        }
                        else if (i == led_count + led_padding*2 - 3) {
                            ledArray[i + 2] = CRGB(0, 230, 0);
                            ledArray[i + 1] = CRGB(0, 215, 0);
                            ledArray[i] = CRGB(0, 200, 0);
                            FastLED.show();
                            delay(FPS);
                        }
                    }
                }
            }
        }
        else { 
          for (int i = 0; led_count > i; i++) {
              ledArray[i] = 0;
            }
          break; 
          }
    }
}

void motors_on_off(){
  if(rightBTN.getCount() % 2 == 0){     
    motorRight.setPowerUp();
    motorLeft.setPowerUp();
    motorON = true;
    steeringON = true;
  }
  else{
    motorRight.setPowerDown();
    motorLeft.setPowerDown();
    motorON = false;
    steeringON = false;
  }
}


void setup() {
  Serial.begin(115200);

  #define I2C_SPEED 1000000           //I2C channel speed defining
  Wire1.begin();                      //I2C channel start
  Wire1.setSCL(SCL1);                 //change the SCL pin
  Wire1.setSDA(SDA1);                 //change the SDA pin
  Wire1.setClock(I2C_SPEED);          //change I2C speed for VL53L5CX max 1000000
  servo.attach(SERVOPWM);             //Servo attaching
  turn(0);                            //Servo turning servo to start position

  //Distance sensors setup
  CameraSetup(); 
  CameraInit();

  //Switch 1 & Switch 2
  pinMode(LBTN, INPUT_PULLUP);
  pinMode(RBTN, INPUT_PULLUP);

  leftBTN.setDebounceTime(50); // set debounce time to 100 milliseconds
  rightBTN.setDebounceTime(50); // set debounce time to 100 milliseconds
  leftBTN.setCountMode(COUNT_FALLING); 
  rightBTN.setCountMode(COUNT_FALLING);
  
  //////LEDS///////
  FastLED.addLeds<WS2812B, LED>(ledArray, led_count + led_padding); //Käivitab LED riba roboti ees andurite info kuvamiseks
  
}

void loop() {
  //Nuppude väärtuste kontrollimine
  leftBTN.loop();
  rightBTN.loop();

  //Parema nupu vajutuste arvu järgi mootorite sisse või välja lülitamine
  motors_on_off();
   
  ////SENSOR DATA POLLING////
  CameraGetData();  //Küsib kõigilt anduritelt järestikku uut infot ja seejärel jätkab muu koodiga

  ////DATA PROCESSING////
  //PrintAllCameraData();
  OneDataLineArray(1);  //Muudab andurite 4x4 array üherealiseks arrayks ja sulgudesse läheb see rida mida edaspidi kasutada on vaja anduritelt

  ////DRIVING LOGIC SELECTION////
  Steering_easy_if_else(); 
}
