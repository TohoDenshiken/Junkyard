#include <Wire.h>
#include <Mouse.h>
#include <Keyboard.h>  // キーボードHID制御用ライブラリの読み出し


#define MPU9250_ADDRESS 0x68
#include <Psx.h>                                          // Includes the Psx Library 
#define dataPin 8
#define cmndPin 9
#define attPin 10
#define clockPin 13

#define LEDPin 12

Psx Psx;                                                  // Initializes the library

unsigned int data = 0;                                    // data stores the controller response


void writeRegister(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void readSensorData(uint8_t address, uint8_t subAddress, int16_t* data, int count) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);

  Wire.requestFrom(address, count * 2, true);
  for (int i = 0; i < count; ++i) {
    data[i] = (Wire.read() << 8) | Wire.read();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU9250
  writeRegister(MPU9250_ADDRESS, 0x6B, 0); // Wake up MPU9250
  writeRegister(MPU9250_ADDRESS, 0x37, 0x02); // Enable magnetometer bypass mode
  writeRegister(MPU9250_ADDRESS, 0x1B, 0x00); // Set gyroscope full-scale range to 250 degrees/s
  writeRegister(MPU9250_ADDRESS, 0x1C, 0x00); // Set accelerometer full-scale range to 2g

  // Additional initialization if needed
  delay(1000);
  Mouse.begin();

  Psx.setupPins(dataPin, cmndPin, attPin, clockPin, 10);  // Defines what each pin is used
                                                          // (Data Pin #, Cmnd Pin #, Att Pin #, Clk Pin #, Delay)
                                                          // Delay measures how long the clock remains at each state,
                                                          // measured in microseconds.
                                                          // too small delay may not work (under 5)
  pinMode(LEDPin, OUTPUT);                                // Establishes LEDPin as an output so the LED can be seen
  Serial.begin(9600);
  
}



void loop() {

  data = Psx.read();
  
  // Read accelerometer and gyroscope data
  int16_t accelData[3];
  readSensorData(MPU9250_ADDRESS, 0x3B, accelData, 3);

  // Convert raw data to m/s^2 for accelerometer and rad/s for gyroscope
  float accelX = accelData[0] / 16384.0; // FS_SEL = 0, sensitivity = +/- 2g
  float accelY = accelData[1] / 16384.0;
  float accelZ = accelData[2] / 16384.0;



  // Calculate roll and pitch
  float roll = atan2(accelY, accelZ) * 180.0 / PI;
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

  // Map roll and pitch to mouse movement
  //int mouseX = map(roll, -90, 90, -10, 10); // Adjust mapping values as needed
  //int mouseY = map(pitch, -90, 90, -10, 10); // Adjust mapping values as needed

  // Move the mouse cursor
  Mouse.move(40*tan(PI*roll/180), 60*tan(PI*(pitch-5)/180));

  if (data & 1024)                                       // If the data anded with a button's hex value is true,
                                                          // it signifies the button is pressed. Hex values for each
                                                          // button can be found in Psx.h
  {
    digitalWrite(LEDPin, HIGH);                           // If button is pressed, turn on the LED
    Mouse.click(); 
  }
  else if(data & 16){

  Keyboard.press('w');
  delay(10);
  Keyboard.releaseAll();

  }

  else if(data & 512){

  Keyboard.press('r');
  delay(10);
  Keyboard.releaseAll();

  }
  
  else
  {
    digitalWrite(LEDPin, LOW);                            // If the button isn't pressed, turn off the LED
  }

  
  delay(5);



  

  //Serial.print("Roll: ");
  //Serial.println(mouseX);
  //Serial.print(", Pitch: ");
  //Serial.println(mouseY);

  delay(10);
}
