#include <Arduino.h>

/////////////////////////////////////////////////////////////
// GP2YOA02 distance sensor
/////////////////////////////////////////////////////////////
#define DISTANCE_PIN      A3

unsigned int readDistance(int nb = 1) {
  unsigned int val = 0;
  for (int i=0 ; i<nb ;i++) {
    val += analogRead(DISTANCE_PIN);
    delayMicroseconds(10);
  }
  return val / nb; // / nb;
}

/////////////////////////////////////////////////////////////
// Stepper  
/////////////////////////////////////////////////////////////
#include <AccelStepper.h>

// CNC Shield
// pinout diagram
#define MOTOR_ENABLE    13          // PB5  13 ou  5
#define X_AXIS_STEP     10          //
#define X_AXIS_DIR      9           // PB1

#define MS1             12          // PB4
#define MS2             11          // PB4

#define FULL_STEPS_PER_ROTATION     200.0
#define MICRO_STEPPING              16
#define STEPS_PER_ROTATION          FULL_STEPS_PER_ROTATION * MICRO_STEPPING  /// 16 * 200 = 3200 ---- 32 x 200 = 6400
//int delay_steps = 100;
bool clockwise = true;

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, X_AXIS_STEP, X_AXIS_DIR);

/////////////////////////////////////////////////////////////
// PID
/////////////////////////////////////////////////////////////
const float Kp = 0.20;     //0.10
const float Kd = 1.50;     //1.00; 
const float Ki = 0.50;     //0.01;

int target = 300;

unsigned long last_T, last_millis = millis();
volatile float sum_error, delta_error, delta_T, output = 0;
volatile float P,I,D = 0;
volatile int input, last_input = target;
volatile int error, last_error = 0;

float calcul_pid() {
  delta_T = (millis() - last_T) / 1000.0F ;
  last_T = millis();
  error = target - input;
  sum_error += (error * delta_T);
  delta_error = (error - last_error) / delta_T ;
  P = (Kp * error);
  I = (Ki * sum_error);
  D = (Kd * delta_error);
  float out = P + I + D ; 
  last_error = error;
  return out;
}

/////////////////////////////////////////////////////////////
// AVR Timer
/////////////////////////////////////////////////////////////

ISR(TIMER1_COMPA_vect) {
  input = readDistance(5);
  output = calcul_pid();
  stepper.setSpeed(output);
  TCNT1 = 0;
}

void initTimer1() {
  cli();
  TCCR1A = 0;                       // Timer/Counter1 Control Register A (COM1A1 COM1A0 COM1B1 COM1B0 – – WGM11 WGM10)
  TCCR1B &= ~(1 << CS12);           // prescaler 8  = B010
  TCCR1B |=  (1 << CS11);           //           64 = B011
  TCCR1B |=  (1 << CS10);
  OCR1A  =  (F_CPU / 64) * 0.005;   // (16000000 / 64) = 250 000 (counts/s)   => compare on 0.020s =  5 000
                                    // (16000000 / 8 ) = 2 000 000 (counts/s) => compare on 0.005s = 10 000
  TIMSK1 |= (1 << OCIE1A);          // Output Compare A
  TCNT1 = 0;                        // set start counter to 0
  sei();
}


/////////////////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  stepper.setPinsInverted(true,false,true);    // TMC2208
  // stepper.setPinsInverted(false,true,true); // DRV8825
  stepper.setEnablePin(MOTOR_ENABLE);
  stepper.enableOutputs();
  stepper.setMaxSpeed(5000.0);
  stepper.setAcceleration(5000.0);

  // set microstepping to 16
  pinMode (MS1, OUTPUT);
  digitalWrite(MS1, HIGH);
  pinMode (MS2, OUTPUT);
  digitalWrite(MS2, HIGH);

  Serial.println("error P I D");

  // target = readDistance(100);

  initTimer1();

}

/////////////////////////////////////////////////////////////
// Loop
/////////////////////////////////////////////////////////////

void loop()
{

  // if (millis() - last_millis > 200) {
  //   Serial.print(error); Serial.print("\t");
  //   Serial.print(P); Serial.print("\t");
  //   Serial.print(I); Serial.print("\t");
  //   Serial.print(D / 100); Serial.print("\t");
  //   Serial.println();
  //   last_millis = millis();
  // }
  
  stepper.runSpeed();
  
}
