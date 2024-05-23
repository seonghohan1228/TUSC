// pid.hpp

#ifndef PACKET_HPP
#define PACKET_HPP
#include <queue>

// PID
float KP1 = 0.03;
float KI1 = 0.60;
float KD1 = 0.0001;

float KP2 = 0.03;
float KI2 = 0.60;
float KD2 = 0.0001;

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

#endif
