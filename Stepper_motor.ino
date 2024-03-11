#include <FreeRTOS_ARM.h>

int16_t sharedData = 0;

TaskHandle_t task1Handle;
TaskHandle_t task2Handle;
TaskHandle_t task3Handle;

// Define pin connections for the TB6600 driver
#define STEP_PIN 5    // STEP 핀
#define DIR_PIN 6     // DIR 핀
#define ENABLE_PIN 7  // ENABLE 핀

// Stepper motor control variables
const float STEPS_PER_REVOLUTION = 6400;                        // 스텝모터의 한 회전 당 스텝 수
const float LEAD_LENGTH = 10.0;                                 // Linear stage의 lead 길이 (mm)
const float STEPS_PER_MM = STEPS_PER_REVOLUTION / LEAD_LENGTH;  // 1mm당 필요한 스텝 수

const unsigned long PULSE_LOOP_TIME = 100000;  // 전체 펄스 루프 시간 (마이크로초) 현재 : 10Hz
float stepInterval = 0;

int currentSteps = 0;     // 현재 위치
int targetPosition = 0;   // 목표 위치
int stepThreshold = 10;   // 작을수록 sensitive

void Task1(void *pvParameters) {
  (void)pvParameters;
  while (1) {
    if (Serial.available() >= 2) {
      uint8_t receivedBytes[2];
      Serial.readBytes(receivedBytes, 2);

      int16_t data1 = static_cast<int16_t>((receivedBytes[1] << 8) | receivedBytes[0]);
      sharedData = data1;
    }
    vTaskDelay(pdMS_TO_TICKS(1));  // 5ms 대기
  }
}

void Task2(void *pvParameters) {
  (void)pvParameters;

  while (1) {
    int16_t data = sharedData;
    if (data >= 1)
      digitalWrite(11, HIGH);
    else
      digitalWrite(11, LOW);
    vTaskDelay(pdMS_TO_TICKS(1));  // 5ms 대기
  }
}

void Task3(void *pvParameters) {
  (void)pvParameters;

  while (1) {
    int16_t data = sharedData;
    
    targetPosition = map(data, -4000, 4000, 4000, 0); //8000
    int length = 50;  //mm
    int targetSteps = targetPosition * STEPS_PER_MM / (4000 / length); //8000

    int direction = (targetSteps >= currentSteps) ? HIGH : LOW;
    int moveSteps = abs(targetSteps - currentSteps);

    digitalWrite(DIR_PIN, direction);

    if (targetSteps != currentSteps)
      stepInterval = (float)PULSE_LOOP_TIME / moveSteps;  // 스텝 간격 계산
    else
      stepInterval = 0;

    if ((targetSteps > currentSteps+stepThreshold) || (targetSteps < currentSteps-stepThreshold)) {
      for (int i = 0; i < moveSteps; i++) {
        pulse(stepInterval / 2);
      }
    }
    currentSteps = targetSteps;
  }
}

void setup() {
  Serial.begin(250000);
  pinMode(11, OUTPUT);

  // Set the pin modes for the TB6600 driver pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);  // 모터 드라이버 활성화

  while (!Serial)
    ;

  xTaskCreate(Task1, "Task1", 128, NULL, 1, &task1Handle);
  xTaskCreate(Task2, "Task2", 128, NULL, 1, &task2Handle);
  xTaskCreate(Task3, "Task3", 128, NULL, 1, &task3Handle);

  vTaskStartScheduler();
}

void loop() {}

void pulse(int delay) {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(delay);  // 스텝 펄스의 길이 (원하는 속도에 따라 조정)
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delay);  // 스텝 펄스의 길이 (원하는 속도에 따라 조정)
}
