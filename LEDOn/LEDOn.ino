/*
 * Simple LED On
 * Turns on an LED connected to pin D5 and keeps it on.
 * 
 * 간단한 LED 켜기
 * 핀 D5에 연결된 LED를 켜고 계속 켜둡니다.
 */

#include <Arduino.h>

#define LED_PIN 5

void setup() {
  // Set LED_PIN as output / LED_PIN을 출력으로 설정
  pinMode(LED_PIN, OUTPUT);
  
  // Turn on the LED / LED 켜기
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  // Nothing to do - LED stays on / 할 일 없음 - LED는 계속 켜져 있음
  // The LED will remain on because we set it HIGH in setup() / setup()에서 HIGH로 설정했기 때문에 LED는 계속 켜져 있습니다
}

