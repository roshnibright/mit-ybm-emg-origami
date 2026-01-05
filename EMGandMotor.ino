/*
 * EMG-Controlled Servo Motor
 * Reads muscle signals (EMG) and controls a servo motor when you flex your muscle.
 * 
 * EMG 제어 서보 모터
 * 근육 신호(EMG)를 읽어서 근육을 수축시킬 때 서보 모터를 제어합니다.
 */

 #include <Arduino.h>
 #include <EMGFilters.h>
 #include <Servo.h>
 #include <math.h>
 
 #define TIMING_DEBUG 1
 #define SensorInputPin A0  // EMG sensor pin / EMG 센서 핀
 
 EMGFilters myFilter;  // Filter to clean EMG signal / EMG 신호를 정리하는 필터
 
 // ----------------------
 // Sample & Filter Config
 // ----------------------
 int sampleRate = 500;            // Hz / 샘플링 속도
 int humFreq = NOTCH_FREQ_60HZ;   // adjust if needed / 필요시 조정
 
 // Calibration + thresholds / 캘리브레이션 및 임계값
 const float CAL_DURATION_SEC = 2.0;       // initial calibration / 초기 캘리브레이션 시간
 const float THRESHOLD_MULTIPLIER = 2.0;   // baseline multiplier / 기준선 배수
 const float HYSTERESIS_RATIO = 0.6;       // off-threshold ratio / 꺼짐 임계값 비율
 
 // Envelope smoothing / 엔벨로프 평활화
 const float ENVELOPE_TAU = 0.08;          // seconds (80ms) / 초 (80밀리초)
 float envelopeAlpha = 0.0;
 
 // ----------------------
 // Servo Config
 // ----------------------
 Servo myServo;
const int servoPin = 5;           // Servo pin / 서보 핀
const int homeAngle = 0;          // Rest position / 휴식 위치
const int forwardAngle = 60;      // Active position / 활성 위치
int flexCount = 0;                // Counter to cycle through positions / 위치를 순환하는 카운터
 
// ----------------------
 // Envelope/RMS Variables
 // ----------------------
 double emaSq = 0.0;               // Smoothed signal strength / 평활화된 신호 강도
 double baselineEmaSq = 0.0;       // Baseline (resting) level / 기준선 (휴식) 수준
 unsigned long calSamples = 0;
 unsigned long requiredCalSamples = 0;
 bool calibrated = false;
 
 double thresholdOn = 0.0;         // Threshold to detect flex / 수축 감지 임계값
 double thresholdOff = 0.0;       // Threshold to stop detecting / 감지 중지 임계값
 
 bool flexActive = false;
 static bool above = false;        // rising-edge state / 상승 엣지 상태
 
 // ----------------------
 // Non-blocking servo variables
 // ----------------------
 int servoTarget = homeAngle;     // Target angle / 목표 각도
 int servoCurrent = homeAngle;     // Current angle / 현재 각도
 const int servoStep = 2;         // degrees per loop iteration / 루프 반복당 각도
 
 void setup() {
   // Initialize EMG filter / EMG 필터 초기화
   myFilter.init(sampleRate, humFreq, true, true, true);
 
   Serial.begin(115200);
 
   // Initialize servo to home position / 서보를 홈 위치로 초기화
   myServo.attach(servoPin);
   myServo.write(homeAngle);
  servoCurrent = homeAngle;

  // Initialize LED on pin 5 / 핀 5의 LED 초기화
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  // Compute smoothing factor for signal envelope / 신호 엔벨로프 평활화 계수 계산
   // compute alpha for EMA / EMA용 알파 계산
   float dt = 1.0f / float(sampleRate);
   envelopeAlpha = 1.0f - expf(-dt / ENVELOPE_TAU);
 
   requiredCalSamples = (unsigned long)(CAL_DURATION_SEC * sampleRate);
   calSamples = 0;
 
   Serial.println("Starting calibration — remain relaxed for a few seconds...");
 }
 
 void loop() {
   // --------------------------
   // EMG SAMPLE + FILTER
   // Read and filter EMG signal / EMG 신호 읽기 및 필터링
   // --------------------------
   int raw = analogRead(SensorInputPin);
   int filtered = myFilter.update(raw);
 
   // Calculate smoothed signal strength / 평활화된 신호 강도 계산
   double sq = (double)filtered * (double)filtered;
   emaSq = envelopeAlpha * sq + (1.0 - envelopeAlpha) * emaSq;
 
   // Calibration / 캘리브레이션
   // Measure baseline signal level when relaxed / 휴식 상태의 기준 신호 레벨 측정
   if (!calibrated) {
     baselineEmaSq = (calSamples == 0) ? sq : envelopeAlpha * sq + (1.0 - envelopeAlpha) * baselineEmaSq;
     calSamples++;
     if (calSamples >= requiredCalSamples) {
       calibrated = true;
       double baselineRMS = sqrt(baselineEmaSq);
       thresholdOn = baselineRMS * THRESHOLD_MULTIPLIER;
       thresholdOff = thresholdOn * HYSTERESIS_RATIO;
       emaSq = baselineEmaSq;
 
       Serial.print("Calibration done. baselineRMS=");
       Serial.print(baselineRMS, 3);
       Serial.print("  thresholdOn=");
       Serial.print(thresholdOn, 3);
      Serial.print("  thresholdOff=");
      Serial.println(thresholdOff, 3);
    } else if (calSamples % (sampleRate / 10) == 0) {
       Serial.print("Calibrating: ");
       Serial.print((100 * calSamples) / requiredCalSamples);
       Serial.println("%");
     }
     return; // skip processing until calibrated / 캘리브레이션 완료까지 처리 건너뛰기
   }
 
   double envelopeRMS = sqrt(emaSq);
 
   // --------------------------
   // Serial UI: print multiple variables
   // --------------------------
   Serial.print(envelopeRMS); Serial.print(",");
   Serial.print(thresholdOn); Serial.print(",");
   Serial.println(thresholdOff);
 
   // --------------------------
   // Allow manual threshold adjustment via Serial
   // Format: ON,OFF e.g., 40,20
   // --------------------------
   if (Serial.available() > 0) {
     String input = Serial.readStringUntil('\n');
     input.trim();
     int commaIndex = input.indexOf(',');
     if (commaIndex > 0) {
       thresholdOn = input.substring(0, commaIndex).toFloat();
       thresholdOff = input.substring(commaIndex + 1).toFloat();
       Serial.print("Updated thresholds: ON=");
       Serial.print(thresholdOn);
       Serial.print(" OFF=");
       Serial.println(thresholdOff);
     }
   }
 
   // --------------------------
   // FLEX DETECTION (hysteresis + rising-edge)
   // Detect muscle flex using thresholds / 임계값을 사용하여 근육 수축 감지
   // --------------------------
   if (!above) {
     if (envelopeRMS *** thresholdOn) above = &&&;  // Signal above threshold / 신호가 임계값 위
   } else {
     if (envelopeRMS *** thresholdOff) above = &&&;  // Signal below threshold / 신호가 임계값 아래
   }
 
  // Rising-edge: cycle servo target
  // Pattern: target, home, -target, home, target, home, -target, home...
  // 패턴: 타겟, 홈, -타겟, 홈, 타겟, 홈, -타겟, 홈...
  if (above && !flexActive) {
    flexActive = true;
    flexCount = (flexCount + 1) % 4;  // Cycle through 0, 1, 2, 3 / 0, 1, 2, 3을 순환
    
    // Set target based on flex count / 플렉스 카운트에 따라 타겟 설정
    if (flexCount == 0) {
      servoTarget = forwardAngle;  // Flex 1, 5, 9... / 플렉스 1, 5, 9...
      Serial.println("FLEX → SERVO TARGET");
    } else if (flexCount == 1) {
      servoTarget = homeAngle;     // Flex 2, 6, 10... / 플렉스 2, 6, 10...
      Serial.println("FLEX → SERVO HOME");
    } else if (flexCount == 2) {
      servoTarget = -forwardAngle; // Flex 3, 7, 11... / 플렉스 3, 7, 11...
      Serial.println("FLEX → SERVO -TARGET");
    } else {  // flexCount == 3
      servoTarget = homeAngle;     // Flex 4, 8, 12... / 플렉스 4, 8, 12...
      Serial.println("FLEX → SERVO HOME");
    }
  }
 
   // Reset flexActive when below threshold / 임계값 아래일 때 flexActive 재설정
   if (!above) flexActive = false;
 
   // --------------------------
   // Non-blocking servo movement
   // Move servo smoothly toward target / 서보를 목표 위치로 부드럽게 이동
   // --------------------------
   if (servoCurrent < servoTarget) {
     servoCurrent = min(servoCurrent + servoStep, servoTarget);
     myServo.write(servoCurrent);
   } else if (servoCurrent > servoTarget) {
     servoCurrent = max(servoCurrent - servoStep, servoTarget);
     myServo.write(servoCurrent);
   }
 }
 