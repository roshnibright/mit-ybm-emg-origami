/*
 * SG90 Servo Motor Back and Forth Sweep
 * This program makes a servo motor sweep back and forth between two angles.
 * 
 * SG90 서보 모터 앞뒤 스윕
 * 이 프로그램은 서보 모터를 두 각도 사이에서 앞뒤로 스윕하도록 만듭니다.
 */

#include <Servo.h>

// Create a servo object to control the motor
// 서보 모터를 제어하기 위한 서보 객체 생성
Servo myServo;

// Angle limits for the sweep motion
// 스윕 동작을 위한 각도 제한값
// choose your limits / 원하는 제한값 선택
int minAngle = 30;   // Minimum angle (degrees) / 최소 각도 (도)
int maxAngle = 120;  // Maximum angle (degrees) / 최대 각도 (도)

// Current position of the servo
// 서보의 현재 위치
int currentAngle = minAngle;

// How many degrees to move each time through the loop
// 루프를 돌 때마다 몇 도씩 움직일지
int stepSize = 1;    // how many degrees to move each loop / 루프마다 움직일 각도

// Direction flag: true = moving forward (increasing angle), false = moving backward (decreasing angle)
// 방향 플래그: true = 앞으로 이동 (각도 증가), false = 뒤로 이동 (각도 감소)
bool goingForward = true;

void setup() {
  // Attach the servo to pin 7 (D7 on Arduino)
  // 서보를 핀 7번 (Arduino의 D7)에 연결
  myServo.attach(7);   // D7 on Arduino / Arduino의 D7 핀
}

void loop() {
  // Move the servo to the current angle
  // 서보를 현재 각도로 이동
  myServo.write(currentAngle);
  
  // Wait 5 milliseconds before next movement (controls speed; increase for slower motion)
  // 다음 움직임 전에 5밀리초 대기 (속도 제어; 더 느리게 하려면 값 증가)
  delay(5);   // controls speed; increase for slower motion / 속도 제어; 더 느리게 하려면 값 증가

  // Update the sweep direction and angle
  // 스윕 방향과 각도 업데이트
  // update sweep / 스윕 업데이트
  if (goingForward) {
    // Moving forward: increase the angle
    // 앞으로 이동: 각도 증가
    currentAngle += stepSize;
    
    // If we reached the maximum angle, reverse direction
    // 최대 각도에 도달하면 방향 반전
    if (currentAngle >= maxAngle) {
      currentAngle = maxAngle;
      goingForward = false;  // Start moving backward / 뒤로 이동 시작
    }
  } else {
    // Moving backward: decrease the angle
    // 뒤로 이동: 각도 감소
    currentAngle -= stepSize;
    
    // If we reached the minimum angle, reverse direction
    // 최소 각도에 도달하면 방향 반전
    if (currentAngle <= minAngle) {
      currentAngle = minAngle;
      goingForward = true;  // Start moving forward / 앞으로 이동 시작
    }
  }
}
