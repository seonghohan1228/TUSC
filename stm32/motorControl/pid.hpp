// pid.hpp

#ifndef PACKET_HPP
#define PACKET_HPP
#include <queue>

static const int PID_FILTER_SIZE = 10;
static const int RPM_FILTER_SIZE = 30;

// PID
float KP1 = 0.1;
float KI1 = 0;
float KD1 = 0.00;

float KP2 = 0.1;
float KI2 = 0;
float KD2 = 0.00;

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
  void pop()
  {
    for (int i = 0; window.size() > i; i++)
    {
      window.pop();
    }
  }
};

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
  float filtered_velocity;

  float max_Val;  // 입력값의 최댓값
  float min_Val;  // 입력값의 최솟값
  float stop_Val; // 정지값

  float previousAngle;
  unsigned long previousMicros;

  float antiwind_Threshold; // Integral anti-windup 기준값
  unsigned long previousTime;

  MovingAverageFilter pidFilter;
  MovingAverageFilter rpmFilter;

public:
  PID(float p, float i, float d, float min, float max, float stop) : kp(p), ki(i), kd(d), max_Val(max), min_Val(min), stop_Val(stop), pidFilter(PID_FILTER_SIZE), rpmFilter(RPM_FILTER_SIZE)
  {
    integral = 0;
    goal = 0;
    filtered_velocity = 0;

    previousTime = 0;
    previousAngle = 0.0;
    previousMicros = 0;
    previousError = 0;

    antiwind_Threshold = 400;
  }

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
    @brief 필터링된 현재 rpm을 반환
    @return float : 필터링된 현재 rpm
    */
  float get_Velocity()
  {
    return filtered_velocity;
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

  float readRPM(TwoWire &wire)
  {
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

    return rpm;
  }

  /*
     @brief compute ESC control input
     @param input float : current speed
     @return float : calculated PWM pulse width
     */
  float computePulseWidth(float input)
  {
    unsigned long currentTime = micros();     // us
    filtered_velocity = rpmFilter.add(input); // filtered velocity
    float dt = (currentTime - previousTime) / 1000000.0;

    if (dt == 0)
      return 0;

    float error = goal - filtered_velocity;          // P
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

#endif
