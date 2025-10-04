/*
Sketch to drive a 
- 2 wire NEMA 17 with DRV8825 driver or 
- 28BYJ-48 Stepper Motor with ULN2003 Driver
*/

// ********************************* select only NEMA or BYJ
//#define NEMA
#define BYJ
// **********************************
#include <AccelStepper.h>
#include <Encoder.h>

// =====================
// Pin Definitions
// =====================
#define ENC_CLK   2      // Rotary CLK pin 
#define ENC_DT    3      // Rotary DT pin
#define ENC_SW    4      // Rotary SW pin
#define EN_PIN    5      // EN pin
#define DIR_PIN   6      // microstep pin
#define STEP_PIN  7      // microstep pin
#define M0_PIN    8      // microstep pin
#define M1_PIN    9      // microstep pin
#define M2_PIN    10     // microstep pin
#define LED_PIN   13     // Builtin LED

// Motor pin definitions:
#define motorPin1  8      // IN1 on the ULN2003 driver
#define motorPin2  9      // IN2 on the ULN2003 driver
#define motorPin3  10     // IN3 on the ULN2003 driver
#define motorPin4  11     // IN4 on the ULN2003 driver

/*/ Define the AccelStepper interface type: 4 wire motor in half step mode:
	FUNCTION  = 0, ///< Use the functional interface, implementing your own driver functions (internal use only)
	DRIVER    = 1, ///< Stepper Driver, 2 driver pins required
	FULL2WIRE = 2, ///< 2 wire stepper, 2 motor pins required
	FULL3WIRE = 3, ///< 3 wire stepper, such as HDD spindle, 3 motor pins required
  FULL4WIRE = 4, ///< 4 wire full stepper, 4 motor pins required
	HALF3WIRE = 6, ///< 3 wire half stepper, such as HDD spindle, 3 motor pins required
	HALF4WIRE = 8  ///< 4 wire half stepper, 4 motor pins required
*/

                     // ******** Define only one motortype on line 7 or 8 of this sketch ******
                     #ifdef BYJ
                                                                     // Initialize with pin sequence IN1-IN3-IN2-IN4
AccelStepper stepper = AccelStepper(8, motorPin1, motorPin3, motorPin2, motorPin4);
                     #endif
                     #ifdef NEMA
                                                                     // Initialize Stepper Driver, 2 driver pins required
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
                     #endif

Encoder knob(ENC_CLK, ENC_DT);

// =====================
// Variables
// =====================
long oldPosition = 0;
int microstepMode = 0;

float targetSpeed = 0;
const float speedIncrement = 20;
const float maxSpeed = 2000;
const float accel = 500;

// Button handling
static bool lastButtonState = HIGH;
unsigned long buttonPressTime = 0;
const unsigned long longPressTime = 1000;

// LED flicker timing
unsigned long lastLED = 0;
bool ledState = LOW;

// Encoder debounce
unsigned long lastEncoderRead = 0;
const unsigned long encoderDebounce = 100;

// Microstep modes
const int microstepModes[6][3] = { {0,0,0}, {1,0,0}, {0,1,0}, {1,1,0}, {0,0,1}, {1,0,1}};

static uint32_t msTick;                                              // Number of millisecond ticks since we last incremented the second counter
uint32_t  Loopcounter       = 0;

// =====================
// Setup
// =====================
void setup() 
{
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENC_SW, INPUT_PULLUP);

  digitalWrite(EN_PIN, LOW);                                         // always enabled

  Serial.begin(115200);

  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accel);
  setMicrostep(microstepMode);

  Serial.println("=== Stepper Control Ready (Always Enabled) ===");
  msTick = millis();  
}

// =====================
// Main Loop
// =====================
void loop() 
{
  Loopcounter++;
  EverySecondCheck();                                                // Let the second led tick and run the clock program
  CheckDevices();
}

//--------------------------------------------                       //
// COMMON Check connected input devices
//--------------------------------------------
void CheckDevices(void)
{
 handleEncoder();
 handleButton();
 handleMotion();
 handleLED();
 stepper.runSpeed();
}

//--------------------------------------------                       //
// COMMON Update routine 
// Performs tasks every second
//--------------------------------------------
void EverySecondCheck(void)
{
 uint32_t msLeap = millis() - msTick;                                // 
 if (msLeap >999)                                                    // Every second enter the loop
  {
   msTick = millis();
   scheduleDebug();  
   Loopcounter=0;
  }  
}

//--------------------------------------------                       //
// Functional Block Set micro step
//--------------------------------------------
void setMicrostep(int mode) {
  digitalWrite(M0_PIN, microstepModes[mode][0]);
  digitalWrite(M1_PIN, microstepModes[mode][1]);
  digitalWrite(M2_PIN, microstepModes[mode][2]);
}
//--------------------------------------------                       //
// Functional Block Encoder LEFT and Right
//--------------------------------------------
void handleEncoder() 
{
  if (millis() - lastEncoderRead >= encoderDebounce) 
  {
    long newPosition = knob.read();
    if (newPosition != oldPosition) {
      long delta = newPosition - oldPosition;
      targetSpeed += delta * speedIncrement;
      targetSpeed = constrain(targetSpeed, -maxSpeed, maxSpeed);
      oldPosition = newPosition;
      lastEncoderRead = millis();
    }
  }
}
//--------------------------------------------                       //
// Functional Blocks Encoder PRESS shaft
//--------------------------------------------
void handleButton() 
{
  bool buttonState = digitalRead(ENC_SW);
  if (lastButtonState == HIGH && buttonState == LOW) 
  {
    buttonPressTime = millis();
  } 
  else if (lastButtonState == LOW && buttonState == HIGH) 
  {
    unsigned long pressDuration = millis() - buttonPressTime;
  if (pressDuration >= longPressTime) 
    {
      targetSpeed = 0;
      stepper.stop();
      Serial.println(">> Long press: Speed reset");
    } 
    else 
    {
      microstepMode = (microstepMode + 1) % 6;
      setMicrostep(microstepMode);
      Serial.print(">> Microstep mode: ");
      Serial.println(microstepMode);
    }
  }
  lastButtonState = buttonState;
}
//--------------------------------------------                       //
// Functional Block Set shaft speeds
//--------------------------------------------
void handleMotion() 
{
  if (targetSpeed == 0) 
  {
    digitalWrite(EN_PIN, HIGH);                                      // disable driver
    digitalWrite(LED_PIN, LOW);                                      // LED off
    stepper.setSpeed(0);
  } 
  else 
  {
    digitalWrite(EN_PIN, LOW);                                       // enable driver
    stepper.setSpeed(targetSpeed);
    stepper.runSpeed();
  }
}
//--------------------------------------------                       //
// Functional Block Pulse the LED
//--------------------------------------------
void handleLED() 
{
  if (targetSpeed != 0) 
  {
    float absSpeed = abs(targetSpeed);
    unsigned long flickerPeriod = map(absSpeed, 0, maxSpeed, 1000, 25);
    if (millis() - lastLED >= flickerPeriod) 
    {
      lastLED = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  } 
  else {digitalWrite(LED_PIN, LOW);  }
}
//--------------------------------------------                       //
// Functional Block Print Debug info
//--------------------------------------------
void scheduleDebug() 
{
// char text[120];  
// sprintf(text,"[Enc Pos] %ld | [u-step Mode] %d | %d l/s",
//                 oldPosition,   microstepMode , Loopcounter);
// Serial.println(text);

//    Serial.print("[Enc Pos] "); Serial.print(oldPosition);
 //   Serial.print(" | [Target Speed] "); Serial.print(targetSpeed);
    Serial.print(" | [Actual Speed] "); Serial.print(stepper.speed());
//    Serial.print(" | [Dir] "); Serial.print((stepper.speed() >= 0) ? "FWD" : "REV");
    Serial.print(" | [Microstep Mode] "); Serial.print(microstepMode);Serial.print(" | ");
    Serial.print(Loopcounter); Serial.println(" l/s");

}