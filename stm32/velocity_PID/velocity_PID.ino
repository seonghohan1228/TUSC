#include <Wire.h>
#include <Servo.h>
#include <queue>

// AS5600 I2C address
#define AS5600_ADDR 0x36
#define ANGLE_ADDR 0x0E
#define ZPOSH_ADDR 0x01
#define ZPOSL_ADDR 0x02
#define STATUS_ADDR 0x0B

// AS5600 COM RESULTS
#define COM_FAIL -1
#define COM_SUCCESS 1

/*######################################################################
    Functions
 #####################################################################*/
float readAngle(TwoWire &wire);
float readRPM(TwoWire &wire);
bool setZeroPosition(TwoWire &wire, unsigned int zeroPosition);
byte readStatus(TwoWire &wire);
/*--------------------------------------------------------------------*/

/*######################################################################
    Global Variable
 #####################################################################*/
float KP = 0.03;   // 0.014;
float KI = 0.60;   // 0.25;
float KD = 0.0001; // 0.0005;

int v;
float pwmValue1 = 0;

float targetSpeed1 = 3000.0;
float currentSpeed1 = 0;

float stop_threshold = 30;
/*####################################################################*/

/*######################################################################
    Class
#####################################################################*/

//----------------------------------------------------------------------
/*
@brief moving average filter
@param size int : queue size
@example asd(a,f,e)
*/
class MovingAverageFilter
{
private:
  std::queue<float> window;
  int windowSize;
  float sum;

public:
  MovingAverageFilter(int size) : windowSize(size), sum(0) {}

  /*
  @brief moving average filter queue에 숫자를 추가, 필터링값 반환
  @param newValue int : queue에 추가할 수
  @return float : filtered output
  */
  float add(float newValue)
  {
    window.push(newValue);
    sum += newValue;
    if (window.size() > windowSize)
    {
      sum -= window.front();
      window.pop();
    }
    return sum / window.size();
  }
};
//----------------------------------------------------------------------

//----------------------------------------------------------------------
/*
@brief Calculate PID results
@param kp float : P Gain
@param ki float : I Gain
@param kd float : D Gain
@param min_Val float 입력값의 최댓값
@param max_Val float 입력값의 최솟값
@param stop_Val float 정지값
*/
class PID
{
private:
  float kp;
  float ki;
  float kd;

  float previousError;
  float integral;
  float goal; // 목표속도

  float max_Val;  // 입력값의 최댓값
  float min_Val;  // 입력값의 최솟값
  float stop_Val; // 정지값

  float antiwind_Threshold = 400; // Integral anti-windup 기준값
  unsigned long previousTime;

  int filter_Size = 10;
  MovingAverageFilter pidFilter;

public:
  PID(float p, float i, float d, float min, float max, float stop) : kp(p), ki(i), kd(d), max_Val(max), min_Val(min), stop_Val(stop), previousError(0), integral(0), goal(0), previousTime(0), pidFilter(filter_Size) {}

  /*
  @brief set goal velocity
  @param sp float : goal velocity
  @return none
  */
  void goalVelocity(float sp)
  {
    goal = sp;
  }

  /*
   @brief set Anti-Windup threshold
   @param value float : Anti-Windup threshold
   @return none
   */
  void setAntiWindup(float value)
  {
    antiwind_Threshold = value;
  }

  /*
     @brief compute ESC control input
     @param input float : current speed
     @return float : calculated PWM pulse width
     */
  float computePulseWidth(float input)
  {
    unsigned long currentTime = micros(); // us
    float dt = (currentTime - previousTime) / 1000000.0;

    if (dt == 0)
      return 0;

    float error = goal - input;                      // P
    integral += error * dt;                          // I
    float derivative = (error - previousError) / dt; // D

    if (ki * integral > antiwind_Threshold) // anti-windup
      integral = antiwind_Threshold / ki;
    else if (ki * integral < -antiwind_Threshold)
      integral = -antiwind_Threshold / ki;

    float output = kp * error + ki * integral + kd * derivative; // Control output

    previousError = error;
    previousTime = currentTime;

    float value = pidFilter.add(output); // Filtered control output

    value = value + stop_Val; // convert to PWM pulse width

    if (value > max_Val) // Saturation function
      value = max_Val;
    if (value < min_Val)
      value = min_Val;

    return value;
  }
};

/*####################################################################*/

/*######################################################################
    Class Instance
/*####################################################################*/
MovingAverageFilter rpmFilter(30);
TwoWire sen1(PB7, PB6); // I2C instance (SDA, SCL)
Servo test_motor;
PID pid1(KP, KI, KD, 1000, 2000, 1480);
/*####################################################################*/

/*######################################################################
    AS5600 Encoder Functions
 #####################################################################*/

//----------------------------------------------------------------------
/*
@brief read angle in degree
@param &wire TwoWire : sensor
@return float :  degrees
*/
float readAngle(TwoWire &wire)
{
  wire.beginTransmission(AS5600_ADDR);
  wire.write(ANGLE_ADDR);      // AS5600 register 0x0E for angle
  wire.endTransmission(false); // Don't end transmission for continuous sending

  wire.requestFrom(AS5600_ADDR, 2); // Request 2 bytes for angle data
  if (wire.available() >= 2)
  {
    byte highByte = wire.read(); // Read high byte
    byte lowByte = wire.read();  // Read low byte

    return ((highByte << 8) | lowByte) * 0.0879; // Convert to angle in degrees
  }
  return COM_FAIL; // Return FAIL if reading fails
}
//----------------------------------------------------------------------

//----------------------------------------------------------------------
/*
@brief set zero position
@param &wire  (TwoWire) sensor
@param zeroPosition  (unsigned int)  the zero position value (12 bits)
@return (bool) success
*/
bool setZeroPosition(TwoWire &wire, unsigned int zeroPosition)
{
  if (zeroPosition > 4095) // Check if the zeroPosition is within 12-bit range
    return COM_FAIL;       // Return false if zeroPosition is out of range

  byte highByte = zeroPosition >> 8;  // Extract high byte (higher 4 bits)
  byte lowByte = zeroPosition & 0xFF; // Extract low byte (lower 8 bits)

  wire.beginTransmission(AS5600_ADDR);
  wire.write(ZPOSH_ADDR);          // AS5600 register 0x01 for high byte of ZPOS
  wire.write(highByte);            // Write high byte of zero position
  wire.write(ZPOSL_ADDR);          // AS5600 register 0x02 for low byte of ZPOS
  wire.write(lowByte);             // Write low byte of zero position
  if (wire.endTransmission() == 0) // Check if transmission was successful
    return COM_SUCCESS;            // Return true if transmission succeeds
  else
    return COM_FAIL; // Return false if transmission fails
}
//----------------------------------------------------------------------

//----------------------------------------------------------------------
/*
@brief Read status register and return a 5-bit array
@details 5-bit array

@param &wire  (TwoWire) sensor
@returns (byte)5bit_status
         [ bit 0 - MD (Magnet Detected): magnet is detected,
         bit 1 - ML (Magnet Too Low): magnet too low,
         bit 2 - MH (Magnet Too High): magnet too high,
         bit 3 - CO (Cordic Overflow):Cordic overflow error occurred,
         bit 4 - No special error ]
*/
byte readStatus(TwoWire &wire)
{
  byte status;
  byte result = 0; // Initialize result

  wire.beginTransmission(AS5600_ADDR);
  wire.write(0x0B); // STATUS register address
  wire.endTransmission();
  wire.requestFrom(AS5600_ADDR, 1); // Request 1 byte

  if (wire.available())
    status = wire.read(); // Read status register
  else
    return result; // If no data is available, return 0

  // Extract bits from the status byte
  bool md = status & 0x01; // Magnet detected
  bool ml = status & 0x02; // Magnet too low
  bool mh = status & 0x04; // Magnet too high
  bool co = status & 0x08; // Cordic overflow

  // Assign bits to the result
  result |= (md << 0); // Set bit 0 for MD
  result |= (ml << 1); // Set bit 1 for ML
  result |= (mh << 2); // Set bit 2 for MH
  result |= (co << 3); // Set bit 3 for CO

  // Set the 5th bit if no special error (ML, MH, CO) is detected
  if (!ml && !mh && !co)
    result |= (1 << 4); // Set bit 4 if no errors

  return result; // Return the 5-bit status representation
}
//----------------------------------------------------------------------

//----------------------------------------------------------------------
/*
@brief Read angular velocity using AS5600 sensor
@param &wire TwoWire : reference to I2C bus instance
@note 이 함수는 주기적으로 호출되어야함
@return float : angular velocity in degrees per second, or COM_FAIL if fail
*/
float readRPM(TwoWire &wire)
{
  static float previousAngle = 0.0;
  static unsigned long previousMicros = 0;

  wire.beginTransmission(AS5600_ADDR);
  wire.write(ANGLE_ADDR);
  wire.endTransmission(false);
  wire.requestFrom(AS5600_ADDR, 2);

  if (wire.available() < 2)
    return COM_FAIL;

  byte highByte = wire.read();
  byte lowByte = wire.read();
  float currentRawAngle = (highByte << 8) | lowByte;       // 12-bit raw angle
  float currentAngle = (currentRawAngle / 4095.0) * 360.0; // Convert raw angle to degrees

  unsigned long currentMicros = micros();

  // Initialize time and angle
  if (previousMicros == 0)
  {
    previousMicros = currentMicros;
    previousAngle = currentAngle;
    return 0.0;
  }

  float dt = (currentMicros - previousMicros) / 1000000.0;
  if (dt == 0)
    return 0.0;

  float deltaAngle = currentAngle - previousAngle;

  // Adjust for wrapping
  if (deltaAngle > 180)
    deltaAngle -= 360;
  else if (deltaAngle < -180)
    deltaAngle += 360;

  // Calculate angular velocity in degrees per second
  float angularVelocity = deltaAngle / dt;

  // Convert angular velocity from degrees per second to RPM
  float rpm = angularVelocity / 360.0 * 60.0;

  // Update previous values
  previousAngle = currentAngle;
  previousMicros = currentMicros;

  return rpmFilter.add(rpm);
}

/*####################################################################*/

void setup()
{
  Serial.begin(115200, SERIAL_8E1);
  sen1.begin();

  test_motor.attach(PA0);
  test_motor.writeMicroseconds(1480);
  delay(7000);

  pid1.goalVelocity(targetSpeed1);
}

void loop()
{
  currentSpeed1 = readRPM(sen1); // 모터 1의 현재 속도 측정
  pwmValue1 = pid1.computePulseWidth(currentSpeed1);

  if (abs(targetSpeed1) < stop_threshold) // 정지속도 안정화
    test_motor.writeMicroseconds(1480);

  v = v + 1;
  test_motor.writeMicroseconds(pwmValue1);
  if (v > 100)
  {
    targetSpeed1 -= 50;
    Serial.print(currentSpeed1);
    Serial.print(" ");
    Serial.print(targetSpeed1);
    Serial.print(" ");
    Serial.println(pwmValue1);
    v = 0;
  }

  delay(2);
}