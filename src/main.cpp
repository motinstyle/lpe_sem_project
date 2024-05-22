// #include <Arduino.h>

// #define speedPin 2
// #define encoder1 3
// #define encoder2 4
// #define analogError A1

// #define MAX_ROTATIONS_KOEFICIENT 20  // koef for the speed transformation

// void onRotation(void);
// void onEncoder(void);

// unsigned long currentTime = 0;
// unsigned long previousTime = 0;

// volatile bool enc1 = 0;
// volatile bool enc2 = 0;

// int reference = 0;

// volatile int8_t encoderCounter = 0;
// volatile uint8_t rotations = 0;

// void setup() {
//   pinMode(analogError, OUTPUT);
//   pinMode(speedPin, INPUT);
//   pinMode(encoder1, INPUT);
//   pinMode(encoder2, INPUT);

//   analogWrite(analogError, 0);

//   // Serial.begin(9600);
//   Serial.begin(9600);

//   attachInterrupt(digitalPinToInterrupt(speedPin), onRotation, RISING);
//   attachInterrupt(digitalPinToInterrupt(encoder1), onEncoder, CHANGE);
//   // attachInterrupt(0, onRotation, HIGH);
//   // attachInterrupt(1, onEncoder, CHANGE);

//   // currentTime = millis();
// }

// void loop() {
//   reference = encoderCounter * MAX_ROTATIONS_KOEFICIENT;
//   currentTime = millis();

//   if (currentTime - previousTime >= 100) { // Каждую 0.1 секунду
//     float rpm = (rotations * 600.0);// / (currentTime - previousTime) * 1000;
//     // analogWrite(analogError, 0);
//     analogWrite(analogError, map(reference - rpm, 0, 2000, 0, 1023));

//     Serial.print("refernce: ");
//     Serial.print(reference);
//     Serial.print(",");
//     Serial.print("RPM: ");
//     Serial.print(rpm);
//     Serial.print(",");
//     Serial.print("error: ");
//     Serial.println(reference - rpm);
//     // Serial.print(currentTime - previousTime);

//     rotations = 0;
//     previousTime = currentTime;
//   }
// }

// void onRotation(void){
//   // Serial.print("rotations: ");
//   rotations++;
//   // Serial.println(rotations);
//   // Serial.println(digitalRead(speedPin));
// }

// void onEncoder(void) { 
//     // Serial.print("encoder: ");
//     enc1 = digitalRead(encoder1);
//     enc2 = digitalRead(encoder2);
//     if (enc1 != enc2) {if (encoderCounter < 100) encoderCounter++;}
//     else {if (encoderCounter > 0) encoderCounter--;}
//     // Serial.println(encoderCounter);
//     // Serial.println(enc1);
//  }

//---------------------------------------------------------------------------------------------------

// #include <SPI.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 64

// #define OLED_MOSI    11 // Data pin (MOSI)
// #define OLED_CLK     13 // Clock pin (SCK)
// #define OLED_DC      8  // Data/Command pin
// #define OLED_CS      10 // Chip select pin
// #define OLED_RESET   9  // Reset pin

// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);

// #define speedPin 2
// #define encoder1 3
// #define encoder2 4
// #define analogError A1

// #define MAX_ROTATIONS_KOEFICIENT 20  // coefficient for the speed transformation

// void onRotation(void);
// void onEncoder(void);

// unsigned long currentTime = 0;
// unsigned long previousTime = 0;

// volatile bool enc1 = 0;
// volatile bool enc2 = 0;

// int reference = 0;

// volatile int8_t encoderCounter = 0;
// volatile uint8_t rotations = 0;

// void setup() {
//   // Initialize the OLED display
//   if(!display.begin(SSD1306_SWITCHCAPVCC)) {
//     Serial.println(F("SSD1306 allocation failed"));
//     for(;;);
//   }

//   display.display();
//   delay(2000);
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setTextColor(SSD1306_WHITE);
  
//   pinMode(analogError, OUTPUT);
//   analogWrite(analogError, 0);
//   pinMode(speedPin, INPUT);
//   pinMode(encoder1, INPUT);
//   pinMode(encoder2, INPUT);


//   Serial.begin(9600);

//   attachInterrupt(digitalPinToInterrupt(speedPin), onRotation, RISING);
//   attachInterrupt(digitalPinToInterrupt(encoder1), onEncoder, CHANGE);

// }

// void loop() {
//   reference = encoderCounter * MAX_ROTATIONS_KOEFICIENT;
//   currentTime = millis();

//   if (currentTime - previousTime >= 100) { // Every 0.1 seconds
//     float rpm = (rotations * 600.0);
//     analogWrite(analogError, map(reference - rpm, 0, 2000, 0, 1023));

//     Serial.print("reference: ");
//     Serial.print(reference);
//     Serial.print(",");
//     Serial.print("RPM: ");
//     Serial.print(rpm);
//     Serial.print(",");
//     Serial.print("error: ");
//     Serial.println(reference - rpm);

//     // Display on OLED
//     display.clearDisplay();
//     display.setCursor(0, 0);
//     display.print("Reference: ");
//     display.println(reference);
//     display.print("RPM: ");
//     display.println(rpm);
//     display.print("Error: ");
//     display.println(reference - rpm);
//     display.display();

//     rotations = 0;
//     previousTime = currentTime;
//   }
// }

// void onRotation(void){
//   rotations++;
// }

// void onEncoder(void) { 
//   enc1 = digitalRead(encoder1);
//   enc2 = digitalRead(encoder2);
//   if (enc1 != enc2) {
//     if (encoderCounter < 100) encoderCounter++;
//   } else {
//     if (encoderCounter > 0) encoderCounter--;
//   }
// }

//--------------------------------------------------------------------------------------

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define DISCRETISATION_TIME_MS 10

#define OLED_MOSI    11 // Data pin (MOSI)
#define OLED_CLK     13 // Clock pin (SCK)
#define OLED_DC      8  // Data/Command pin
#define OLED_CS      10 // Chip select pin
#define OLED_RESET   9  // Reset pin

#define speedPin 2
#define encoder1 3
#define encoder2 4
#define pwmControlSignal 6

#define MAX_ROTATIONS_KOEFICIENT 48  // coefficient for the speed transformation // last 20
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);

void onRotation(void);
void onEncoder(void);
float PID(float y, const float kp, const float ki, const float kd);

float reference = 0;
float error = 0;
float last_error = 0;
float integrator = 0;
float derivator = 0;
unsigned long pid_time;
unsigned long last_pid_time = millis();
long control_sig = 0;

float rpm = 0;

volatile bool enc1 = 0;
volatile bool enc2 = 0;

volatile int8_t encoderCounter = 0;
volatile bool first_rotation = true;

volatile unsigned long rotation_time = 0;
volatile unsigned long last_rotation_time = 0;

void setup() {

  // Initialize the OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  pinMode(pwmControlSignal, OUTPUT);
  analogWrite(pwmControlSignal, 0);
  pinMode(speedPin, INPUT);
  pinMode(encoder1, INPUT);
  pinMode(encoder2, INPUT);

  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(speedPin), onRotation, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder1), onEncoder, CHANGE);

}

void loop() {
  reference = encoderCounter * MAX_ROTATIONS_KOEFICIENT;
  control_sig = (long)PID(rpm, 0.1, 0.1, 0.1);
  if (micros() - last_rotation_time > 2000000) {
    first_rotation = true;
  }
  float my_rpm = rpm;
  rpm = 0.0;

  analogWrite(pwmControlSignal, control_sig);

  // Serial.print("reference: ");
  // Serial.print(reference);
  // Serial.print(",");
  // Serial.print("RPM: ");
  // Serial.print(rpm);
  // Serial.print(",");
  // Serial.print("error: ");
  // Serial.println(reference - rpm);

  // Display on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Reference: ");
  display.println(reference);
  display.print("RPM: ");
  display.println(my_rpm);
  display.print("Error: ");
  display.println(reference - my_rpm);
  display.print("Control: ");
  display.println(control_sig);
  display.print("delta: ");

  display.println(rotation_time - last_rotation_time);
  display.print("first: ");
  display.println(first_rotation);
  display.display();
}

void onRotation(void){
  if (first_rotation){
    last_rotation_time = micros();
    first_rotation = false;
  }
  else {
    rotation_time = micros();
    if (rotation_time - last_rotation_time > 1000){
      rpm = 60.0/((float)(rotation_time - last_rotation_time)/100000);
      last_rotation_time = rotation_time;
    }
    if (rpm > 4800) rpm = 4800;
  }
}

void onEncoder(void) { 
  enc1 = digitalRead(encoder1);
  enc2 = digitalRead(encoder2);
  if (enc1 != enc2) {
    if (encoderCounter < 100) encoderCounter++;
  } else {
    if (encoderCounter > 0) encoderCounter--;
  }
}

float PID(float y, const float kp, const float ki, const float kd){
  pid_time = millis();
  float time = (float)(pid_time - last_pid_time)/1000;
  error = reference - y; 
  integrator += error * time;
  derivator = (error - last_error) * time;
  last_error = error;
  float u = kp * error + ki * integrator + kd * derivator;
  
  //anti windup
  if (u >= 255.0){
    integrator = 0.0;
    return 255.0;
  }
  else if (u <= 0.0){
    error = 0.0;
    last_error = 0.0;
    return 0.0;
  } 

  return kp * error + ki * integrator + kd * derivator;
}



// void onRotation(void){
//   // Serial.print("rotations: ");
//   rotations++;
//   // Serial.println(rotations);
//   // Serial.println(digitalRead(speedPin));
// }
