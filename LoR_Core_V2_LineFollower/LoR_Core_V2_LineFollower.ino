/* Minimal Line Follower (ESP32) — T => 3s turnaround
 * - 3s startup hold
 * - 20 Hz non-blocking control
 * - Sensors active-high (true on black)
 * - ledcAttachChannel + ledcWrite(pin,duty)
 */

#include <Adafruit_NeoPixel.h>

// -------------------- Pins --------------------
#define LED_DataPin 12
#define LED_COUNT   8
#define LineSensor_Left   16
#define LineSensor_Center 17
#define LineSensor_Right  21

// Motors (tank: left=M1/M2, right=M5/M6)
#define motorPin_M1_A 5
#define motorPin_M1_B 14
#define motorPin_M2_A 18
#define motorPin_M2_B 26
#define motorPin_M5_A 27
#define motorPin_M5_B 25
#define motorPin_M6_A 32
#define motorPin_M6_B 4
#define M1 0
#define M2 1
#define M5 4
#define M6 5
#define MotorEnablePin 13

const int motorPins_A[] = { motorPin_M1_A, motorPin_M2_A, /*M3*/0, /*M4*/0, motorPin_M5_A, motorPin_M6_A };
const int motorPins_B[] = { motorPin_M1_B, motorPin_M2_B, /*M3*/0, /*M4*/0, motorPin_M5_B, motorPin_M6_B };

// -------------------- PWM --------------------
const int PWM_FREQUENCY = 20000;
const int PWM_RESOLUTION = 8;
const int MOTOR_PWM_Channel_A[] = { 0, 2, 4, 6, 8, 10 };
const int MOTOR_PWM_Channel_B[] = { 1, 3, 5, 7, 9, 11 };

// -------------------- Control constants --------------------
constexpr uint32_t START_DELAY_MS = 3000;
constexpr uint32_t CTRL_PERIOD_MS = 50;     // 20 Hz
constexpr int LINE_FWD_PCT  = 35;
constexpr int LINE_TURN_PCT = 50;
#define LINE_ACTIVE_LOW 0                   // sensors are active-high

// -------------------- State --------------------
uint32_t boot_ms = 0, lastCtrlMs = 0;
uint32_t turn_until_ms = 0;                 // 0 = not turning

// -------------------- NeoPixels --------------------
Adafruit_NeoPixel strip(LED_COUNT, LED_DataPin, NEO_GRB + NEO_KHZ800);
const uint32_t RED    = strip.Color(255,0,0,0);
const uint32_t GREEN  = strip.Color(0,255,0,0);
const uint32_t BLUE   = strip.Color(0,0,255,0);
const uint32_t PURPLE = strip.Color(255,0,255,0);
const uint32_t CYAN   = strip.Color(0,255,255,0);
const uint32_t YELLOW = strip.Color(255,255,0,0);
const uint32_t WHITE  = strip.Color(255,255,255,255);
const uint32_t OFF    = strip.Color(0,0,0,0);

uint32_t lastBlinkMs = 0; bool blinkOn=false;
inline void LEDs_All(uint32_t c){ for(int i=0;i<LED_COUNT;i++) strip.setPixelColor(i,c); strip.show(); }
inline void LEDs_LeftHalf(uint32_t c){ for(int i=0;i<LED_COUNT/2;i++) strip.setPixelColor(i,c); for(int i=LED_COUNT/2;i<LED_COUNT;i++) strip.setPixelColor(i,OFF); strip.show(); }
inline void LEDs_RightHalf(uint32_t c){ for(int i=0;i<LED_COUNT/2;i++) strip.setPixelColor(i,OFF); for(int i=LED_COUNT/2;i<LED_COUNT;i++) strip.setPixelColor(i,c); strip.show(); }
inline void LEDs_Blink(uint32_t c, uint16_t period_ms){ uint32_t now=millis(); if(now-lastBlinkMs>=period_ms){ lastBlinkMs=now; blinkOn=!blinkOn; } LEDs_All(blinkOn?c:OFF); }

// -------------------- Sensors --------------------
inline void INIT_LineSensors(){ pinMode(LineSensor_Left,INPUT_PULLUP); pinMode(LineSensor_Center,INPUT_PULLUP); pinMode(LineSensor_Right,INPUT_PULLUP); }
inline bool SenseOnLine(uint8_t pin){ int v=digitalRead(pin); return LINE_ACTIVE_LOW ? (v==LOW) : (v==HIGH); }

// -------------------- Motors --------------------
const int MIN_STARTING_SPEED=150, MAX_SPEED=254, STOP=0;
void Set_Motor_Output(int Output, int idx){
  int Mapped=map(abs(Output),0,100,MIN_STARTING_SPEED,MAX_SPEED);
  int A=0,B=0;
  if      (Output<0){ A=0;      B=Mapped; }
  else if (Output>0){ A=Mapped; B=0;      }
  else              { A=STOP;   B=STOP;   }
  if(motorPins_A[idx]) ledcWrite(motorPins_A[idx], A);  // write by PIN (new API)
  if(motorPins_B[idx]) ledcWrite(motorPins_B[idx], B);
}
inline void Motor_Control(int L,int R){ Set_Motor_Output(L,M1); Set_Motor_Output(L,M2); Set_Motor_Output(R,M5); Set_Motor_Output(R,M6); }
inline void Motor_STOP(){ Motor_Control(0,0); }

// -------------------- Setup --------------------
void setup(){
  for(int i=0;i<6;i++){
    if(motorPins_A[i]){ pinMode(motorPins_A[i],OUTPUT); digitalWrite(motorPins_A[i],LOW); }
    if(motorPins_B[i]){ pinMode(motorPins_B[i],OUTPUT); digitalWrite(motorPins_B[i],LOW); }
  }
  pinMode(LED_DataPin,OUTPUT);
  strip.begin(); strip.setBrightness(50); LEDs_All(CYAN);

  for(int i=0;i<6;i++){
    if(motorPins_A[i]) ledcAttachChannel(motorPins_A[i], PWM_FREQUENCY, PWM_RESOLUTION, MOTOR_PWM_Channel_A[i]);
    if(motorPins_B[i]) ledcAttachChannel(motorPins_B[i], PWM_FREQUENCY, PWM_RESOLUTION, MOTOR_PWM_Channel_B[i]);
  }
  pinMode(MotorEnablePin,OUTPUT); digitalWrite(MotorEnablePin,HIGH);

  INIT_LineSensors();
  boot_ms = millis(); lastCtrlMs = boot_ms;
}

// -------------------- Main loop --------------------
void loop(){
  // Startup hold
  if(millis()-boot_ms < START_DELAY_MS){ Motor_STOP(); LEDs_Blink(CYAN,300); return; }

  // 20 Hz tick
  uint32_t now=millis();
  if((now-lastCtrlMs)<CTRL_PERIOD_MS) return;
  lastCtrlMs=now;

  // If currently turning (non-blocking 3 s)
  if(now < turn_until_ms){
    Motor_Control(LINE_TURN_PCT, LINE_TURN_PCT);  // spot turn; flip signs to reverse
    LEDs_All(WHITE);
    return;
  }

  // Read sensors (active-high => true on black)
  const bool L=SenseOnLine(LineSensor_Left);
  const bool C=SenseOnLine(LineSensor_Center);
  const bool R=SenseOnLine(LineSensor_Right);

  // --- T triggers 3 s turnaround: all sensors FALSE ---
  if(L && !C && R){turn_until_ms = now + 1800; LEDs_All(WHITE); return; }

 // Forward whenever center sees line
  if(!L && C && !R){ Motor_Control(-LINE_FWD_PCT, LINE_FWD_PCT); LEDs_All(GREEN); return; }
  if(!L && !C && !R){Motor_STOP(); LEDs_Blink(RED,100); return;}

  // Corrections (priority over forward)
  if( L && !R ){ Motor_Control(-LINE_TURN_PCT, -LINE_TURN_PCT); LEDs_LeftHalf(BLUE); return; }
  if( R && !L ){ Motor_Control( LINE_TURN_PCT,  LINE_TURN_PCT); LEDs_RightHalf(YELLOW); return; }
 
  // Fallback
  Motor_STOP(); LEDs_Blink(PURPLE,300);
}
