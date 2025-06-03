#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <MPU6050.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Конфигурация NRF24L01
#define CE_PIN 9
#define CSN_PIN 10
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "ROBOT";

// Структура для данных с пульта
struct RemoteData {
  int leftX;
  int leftY;
  int rightX;
  int rightY;
};

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
MPU6050 mpu;

// Конфигурация сервоприводов
const int NUM_LEGS = 4;
const int NUM_JOINTS_PER_LEG = 3; // Плечо, бедро, колено
const int FL = 0, FR = 1, RL = 2, RR = 3; // Индексы ног
const int SHOULDER = 0, HIP = 1, KNEE = 2; // Индексы суставов

// Порты сервоприводов [плечо, бедро, колено]
const int SERVO_PINS[NUM_LEGS][NUM_JOINTS_PER_LEG] = {
  {0, 1, 2},   // Передняя левая (FL)
  {3, 4, 5},   // Передняя правая (FR)
  {6, 7, 8},   // Задняя левая (RL)
  {9,10,11}    // Задняя правая (RR)
};

// Ограничения сервоприводов [мин, макс]
const int JOINT_LIMITS[NUM_LEGS][NUM_JOINTS_PER_LEG][2] = {
  {{45,135}, {30,150}, {40,160}},   // FL
  {{45,135}, {20,160}, {30,150}},   // FR
  {{45,135}, {25,155}, {35,165}},   // RL
  {{45,135}, {35,145}, {25,155}}    // RR
};

// Параметры ног
const float FEMUR_LENGTH = 80.0;   // Длина бедра (мм)
const float TIBIA_LENGTH = 100.0;  // Длина голени (мм)
float BASE_HEIGHT = 120.0;         // Базовая высота (мм)
float currentHeight = BASE_HEIGHT; // Текущая высота корпуса

// Параметры походки
float STEP_LENGTH = 60.0;          // Длина шага (мм)
float STEP_HEIGHT = 40.0;          // Высота подъема (мм)
float GAIT_PERIOD = 4.0;           // Полный период цикла (секунды)
const float FRONT_LEAD_PERCENT = 0.01; // Опережение передних ног (1%)

// Параметры поворота
float TURN_AMPLITUDE = 10.0;       // Макс. смещение при повороте (мм)
float turnAmount = 0.0;            // Текущее смещение поворота (-1.0 до 1.0)
float turnTarget = 0.0;            // Целевое смещение поворота
float turnSpeed = 0.0;             // Текущая скорость поворота
const float MAX_TURN_SPEED = 0.05; // Макс. скорость поворота
const float TURN_ACCEL = 0.001;    // Ускорение поворота

// Текущие координаты ног [x, y, z]
float legPositions[NUM_LEGS][3] = {
  {0, 0, BASE_HEIGHT},  // FL
  {0, 0, BASE_HEIGHT},  // FR
  {0, 0, BASE_HEIGHT},  // RL
  {0, 0, BASE_HEIGHT}   // RR
};

// Состояние движения
enum MovementState {
  STOPPED,
  MOVING_FORWARD,
  MOVING_BACKWARD,
  TURNING_LEFT,
  TURNING_RIGHT
};
MovementState currentState = STOPPED;

// Управление скоростью
unsigned long lastGaitTime = 0;
float gaitProgress = 0.0;
float targetSpeed = GAIT_PERIOD;
float currentSpeed = GAIT_PERIOD;

// Настройки датчиков
#define TRIG_LEFT 12
#define ECHO_LEFT 13
#define TRIG_RIGHT 14
#define ECHO_RIGHT 15

// Системные настройки
const int SMOOTHNESS_STEPS = 15; // Шагов для плавных движений

// Управление высотой
float frontHeightAdjustment = 0;
float rearHeightAdjustment = 0;
float globalHeightAdjustment = 0;
const float HEIGHT_ADJUSTMENT_SPEED = 0.5;

void setup() {
  Serial.begin(115200);
  Serial.println("Инициализация квадропеда...");
  
  // Инициализация PCA9685
  pca.begin();
  pca.setPWMFreq(50);
  
  // Инициализация GY-521
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Ошибка подключения GY-521!");
  }
  
  // Инициализация HC-SR04
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Инициализация NRF24L01
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("NRF24L01 инициализирован");
  
  // Мягкий старт - плавный переход в начальное положение
  resetToNeutral();
  
  Serial.println("Система готова к работе");
  Serial.println("Управление:");
  Serial.println("f - вперед, b - назад, s - стоп");
  Serial.println("l - поворот влево, r - поворот вправо");
  Serial.println("u - поднять корпус, d - опустить корпус");
  Serial.println("+ - ускорить, - - замедлить");
}

void loop() {
  // Чтение данных с пульта
  readRemoteData();
  
  // Чтение и вывод данных с датчиков
  readAndLogSensors();
  
  // Обработка команд пользователя
  handleUserCommands();
  
  // Плавное изменение скорости
  updateSpeed();
  
  // Обновление поворота
  updateTurn();
  
  // Управление движением
  switch(currentState) {
    case MOVING_FORWARD:
      updateMovement(true); // Движение вперед
      break;
      
    case MOVING_BACKWARD:
      updateMovement(false); // Движение назад
      break;
      
    case TURNING_LEFT:
    case TURNING_RIGHT:
      updateTurnMovement(); // Поворот на месте
      break;
      
    case STOPPED:
      // Ничего не делаем, ждем команд
      delay(50);
      break;
  }
}

// Чтение данных с пульта
void readRemoteData() {
  if (radio.available()) {
    RemoteData data;
    radio.read(&data, sizeof(data));
    processRemoteData(data);
  }
}

// Обработка данных с пульта
void processRemoteData(RemoteData data) {
  // Движение вперед/назад/повороты
  if (data.leftY < 400) {
    startMovingForward();
  } else if (data.leftY > 600) {
    startMovingBackward();
  } else if (data.leftX < 400) {
    startTurningLeft();
  } else if (data.leftX > 600) {
    startTurningRight();
  } else if (abs(data.leftX - 512) < 100 && abs(data.leftY - 512) < 100) {
    stopMoving();
  }

  // Управление высотой
  if (data.rightY < 400) { // Джойстик вверх
    if (data.rightX < 400) { // Передние ноги
      frontHeightAdjustment += HEIGHT_ADJUSTMENT_SPEED;
    } else if (data.rightX > 600) { // Задние ноги
      rearHeightAdjustment += HEIGHT_ADJUSTMENT_SPEED;
    } else { // Все ноги
      globalHeightAdjustment += HEIGHT_ADJUSTMENT_SPEED;
    }
  } else if (data.rightY > 600) { // Джойстик вниз
    if (data.rightX < 400) { // Передние ноги
      frontHeightAdjustment -= HEIGHT_ADJUSTMENT_SPEED;
    } else if (data.rightX > 600) { // Задние ноги
      rearHeightAdjustment -= HEIGHT_ADJUSTMENT_SPEED;
    } else { // Все ноги
      globalHeightAdjustment -= HEIGHT_ADJUSTMENT_SPEED;
    }
  }
  
  // Ограничения регулировок
  frontHeightAdjustment = constrain(frontHeightAdjustment, -20, 20);
  rearHeightAdjustment = constrain(rearHeightAdjustment, -20, 20);
  globalHeightAdjustment = constrain(globalHeightAdjustment, -30, 30);
  
  // Обновление текущей высоты
  currentHeight = BASE_HEIGHT + globalHeightAdjustment;
}

// Чтение и вывод данных с датчиков
void readAndLogSensors() {
  static unsigned long lastSensorTime = 0;
  if (millis() - lastSensorTime > 200) { // Каждые 200 мс
    lastSensorTime = millis();
    
    // Чтение гироскопа
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    Serial.print("Гироскоп: ");
    Serial.print("X="); Serial.print(gx);
    Serial.print(" Y="); Serial.print(gy);
    Serial.print(" Z="); Serial.println(gz);
    
    // Чтение дальномеров
    float distLeft = readDistance(TRIG_LEFT, ECHO_LEFT);
    float distRight = readDistance(TRIG_RIGHT, ECHO_RIGHT);
    Serial.print("Дальномеры: ");
    Serial.print("Левый="); Serial.print(distLeft);
    Serial.print("см, Правый="); Serial.print(distRight);
    Serial.println("см");
  }
}

// Чтение расстояния с HC-SR04
float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

// Обработка команд пользователя
void handleUserCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case 'f': startMovingForward(); break;
      case 'b': startMovingBackward(); break;
      case 's': stopMoving(); break;
      case 'l': startTurningLeft(); break;
      case 'r': startTurningRight(); break;
      case 'u': globalHeightAdjustment += 5; break; // Поднять
      case 'd': globalHeightAdjustment -= 5; break; // Опустить
      case '+': targetSpeed = constrain(targetSpeed * 0.9, 0.5, 10.0); break;
      case '-': targetSpeed = constrain(targetSpeed * 1.1, 0.5, 10.0); break;
    }
    
    // Ограничение регулировок высоты
    globalHeightAdjustment = constrain(globalHeightAdjustment, -30, 30);
    currentHeight = BASE_HEIGHT + globalHeightAdjustment;
  }
}

// Плавное изменение скорости
void updateSpeed() {
  if (fabs(targetSpeed - currentSpeed) > 0.01) {
    currentSpeed += (targetSpeed - currentSpeed) * 0.05;
    Serial.print("Скорость: ");
    Serial.print(currentSpeed);
    Serial.println(" сек/цикл");
  }
}

// Начать движение вперед
void startMovingForward() {
  if (currentState != MOVING_FORWARD) {
    Serial.println("Начало движения вперед");
    currentState = MOVING_FORWARD;
    gaitProgress = 0.0;
    lastGaitTime = millis();
  }
}

// Начать движение назад
void startMovingBackward() {
  if (currentState != MOVING_BACKWARD) {
    Serial.println("Начало движения назад");
    currentState = MOVING_BACKWARD;
    gaitProgress = 0.0;
    lastGaitTime = millis();
  }
}

// Начать поворот влево
void startTurningLeft() {
  if (currentState != TURNING_LEFT) {
    Serial.println("Начало поворота влево");
    currentState = TURNING_LEFT;
    turnTarget = -1.0;
    gaitProgress = 0.0;
    lastGaitTime = millis();
  }
}

// Начать поворот вправо
void startTurningRight() {
  if (currentState != TURNING_RIGHT) {
    Serial.println("Начало поворота вправо");
    currentState = TURNING_RIGHT;
    turnTarget = 1.0;
    gaitProgress = 0.0;
    lastGaitTime = millis();
  }
}

// Остановить движение
void stopMoving() {
  if (currentState != STOPPED) {
    Serial.println("Остановка...");
    
    // Плавный переход к нейтральному положению
    for (int i = 0; i < SMOOTHNESS_STEPS; i++) {
      float progress = i / (float)SMOOTHNESS_STEPS;
      for (int leg = 0; leg < NUM_LEGS; leg++) {
        float targetX = 0;
        float targetY = 0;
        float targetZ = currentHeight;
        float x = legPositions[leg][0] + (targetX - legPositions[leg][0]) * progress;
        float y = legPositions[leg][1] + (targetY - legPositions[leg][1]) * progress;
        float z = legPositions[leg][2] + (targetZ - legPositions[leg][2]) * progress;
        moveLegTo(leg, x, y, z);
      }
      delay(30);
    }
    
    currentState = STOPPED;
    resetToNeutral();
    turnAmount = 0.0;
    turnSpeed = 0.0;
    Serial.println("Остановлено");
  }
}

// Обновление параметров поворота
void updateTurn() {
  // Плавное изменение скорости поворота
  float targetSpeed = (turnTarget - turnAmount) * MAX_TURN_SPEED;
  if (fabs(targetSpeed - turnSpeed) > 0.001) {
    turnSpeed += (targetSpeed - turnSpeed) * 0.1;
  }
  
  // Обновление текущего поворота
  turnAmount += turnSpeed;
  turnAmount = constrain(turnAmount, -1.0, 1.0);
  
  // Автоматическая остановка при достижении цели
  if (fabs(turnAmount - turnTarget) < 0.05) {
    turnSpeed *= 0.9; // Замедление
    if (fabs(turnSpeed) < 0.005) {
      turnTarget = 0.0;
    }
  }
  
  // Если достигли нуля - остановка
  if (fabs(turnAmount) < 0.01 && fabs(turnSpeed) < 0.005) {
    turnAmount = 0.0;
    turnSpeed = 0.0;
  }
}

// Обновление движения для ходьбы вперед/назад
void updateMovement(bool forward) {
  // Рассчет времени
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastGaitTime) / 1000.0;
  lastGaitTime = currentTime;
  
  // Обновление прогресса походки
  gaitProgress += deltaTime / currentSpeed;
  if (gaitProgress >= 1.0) gaitProgress -= 1.0;
  
  // Вычисление позиций для каждой ноги
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    float phase = calculateLegPhase(leg, gaitProgress);
    updateLegPosition(leg, phase, forward);
    moveLegTo(leg, legPositions[leg][0], legPositions[leg][1], legPositions[leg][2]);
  }
  
  // Задержка для стабильности
  delay(10);
}

// Обновление движения для поворота
void updateTurnMovement() {
  // Рассчет времени
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastGaitTime) / 1000.0;
  lastGaitTime = currentTime;
  
  // Обновление прогресса походки
  gaitProgress += deltaTime / currentSpeed;
  if (gaitProgress >= 1.0) gaitProgress -= 1.0;
  
  // Вычисление позиций для каждой ноги
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    float phase = calculateLegPhase(leg, gaitProgress);
    updateTurnPosition(leg, phase);
    moveLegTo(leg, legPositions[leg][0], legPositions[leg][1], legPositions[leg][2]);
  }
  
  // Задержка для стабильности
  delay(10);
}

// Расчет фазы для ноги
float calculateLegPhase(int leg, float progress) {
  // Базовые фазы:
  // FL: 0.0, FR: 0.5, RL: 0.5, RR: 0.0
  float basePhase = (leg == FL || leg == RR) ? 0.0 : 0.5;
  
  // Добавляем опережение для передних ног
  if (leg == FL || leg == FR) {
    basePhase += FRONT_LEAD_PERCENT;
  }
  
  return fmod(progress + basePhase, 1.0);
}

// Обновление позиции ноги для ходьбы
void updateLegPosition(int leg, float phase, bool forward) {
  // Инвертируем направление для движения назад
  float direction = forward ? 1.0 : -1.0;
  
  // Разделение фазы на подъем и возврат
  float liftPhase = 0.5; // 50% цикла на подъем
  
  if (phase < liftPhase) {
    // Фаза подъема и шага
    float progress = phase / liftPhase;
    legPositions[leg][0] = STEP_LENGTH * progress * direction;
    legPositions[leg][1] = 0; // Боковое смещение не используется
    legPositions[leg][2] = currentHeight + STEP_HEIGHT * sin(M_PI * progress);
  } else {
    // Фаза возврата
    float progress = (phase - liftPhase) / (1.0 - liftPhase);
    legPositions[leg][0] = STEP_LENGTH * (1 - progress) * direction;
    legPositions[leg][1] = 0;
    legPositions[leg][2] = currentHeight;
  }
}

// Обновление позиции ноги для поворота
void updateTurnPosition(int leg, float phase) {
  // Разделение фазы на подъем и возврат
  float liftPhase = 0.5; // 50% цикла на подъем
  
  // Определение направления поворота
  float turnDir = (currentState == TURNING_LEFT) ? -1.0 : 1.0;
  
  if (phase < liftPhase) {
    // Фаза подъема и шага
    float progress = phase / liftPhase;
    
    // Вертикальное движение
    legPositions[leg][2] = currentHeight + STEP_HEIGHT * sin(M_PI * progress);
    
    // Боковое смещение для поворота
    if (leg == FL || leg == RR) {
      legPositions[leg][1] = TURN_AMPLITUDE * turnAmount * turnDir * progress;
    } else {
      legPositions[leg][1] = -TURN_AMPLITUDE * turnAmount * turnDir * progress;
    }
    
    // Небольшое движение вперед для стабильности
    legPositions[leg][0] = STEP_LENGTH * progress * 0.3;
  } else {
    // Фаза возврата
    float progress = (phase - liftPhase) / (1.0 - liftPhase);
    
    // Возврат в исходное положение
    legPositions[leg][0] = STEP_LENGTH * (1 - progress) * 0.3;
    legPositions[leg][1] = 0;
    legPositions[leg][2] = currentHeight;
  }
}

// Перемещение ноги в заданную точку
void moveLegTo(int leg, float x, float y, float z) {
  // Применяем регулировки высоты
  float adjustedZ = z;
  if (leg == FL || leg == FR) {
    adjustedZ += frontHeightAdjustment;
  } else {
    adjustedZ += rearHeightAdjustment;
  }
  
  // Сохраняем позицию
  legPositions[leg][0] = x;
  legPositions[leg][1] = y;
  legPositions[leg][2] = adjustedZ;
  
  // Рассчитываем углы для суставов
  float angles[3];
  inverseKinematics(x, y, adjustedZ, angles);
  
  // Устанавливаем углы с ограничениями
  for (int joint = 0; joint < NUM_JOINTS_PER_LEG; joint++) {
    setJointAngle(leg, joint, angles[joint]);
  }
}

// Обратная кинематика для 3 суставов
void inverseKinematics(float x, float y, float z, float* angles) {
  // Угол плеча (горизонтальное вращение)
  angles[SHOULDER] = atan2(y, x);
  
  // Рассчет для вертикальной плоскости
  float planarDistance = sqrt(x*x + y*y);
  float distance = sqrt(planarDistance*planarDistance + z*z);
  
  // Расчет угла колена
  float cos_knee = (FEMUR_LENGTH*FEMUR_LENGTH + TIBIA_LENGTH*TIBIA_LENGTH - distance*distance) 
                  / (2 * FEMUR_LENGTH * TIBIA_LENGTH);
  cos_knee = constrain(cos_knee, -1, 1);
  float knee_angle = acos(cos_knee);
  
  // Расчет углов бедра
  float hip_angle1 = atan2(z, planarDistance);
  float cos_hip2 = (FEMUR_LENGTH*FEMUR_LENGTH + distance*distance - TIBIA_LENGTH*TIBIA_LENGTH) 
                  / (2 * FEMUR_LENGTH * distance);
  cos_hip2 = constrain(cos_hip2, -1, 1);
  float hip_angle2 = acos(cos_hip2);
  
  // Конвертация в градусы
  angles[SHOULDER] = degrees(angles[SHOULDER]);
  angles[HIP] = degrees(hip_angle1 + hip_angle2);
  angles[KNEE] = degrees(knee_angle);
}

// Установка угла для сустава с ограничениями
void setJointAngle(int leg, int joint, float angle) {
  // Применяем ограничения
  float min_angle = JOINT_LIMITS[leg][joint][0];
  float max_angle = JOINT_LIMITS[leg][joint][1];
  angle = constrain(angle, min_angle, max_angle);
  
  // Преобразуем угол в импульс
  int pulse = map(angle * 100, min_angle * 100, max_angle * 100, 150, 600);
  
  // Управляем сервоприводом
  int servo_pin = SERVO_PINS[leg][joint];
  pca.setPWM(servo_pin, 0, pulse);
}

// Сброс в нейтральное положение
void resetToNeutral() {
  Serial.println("Переход в нейтральное положение");
  
  for (int i = 0; i <= SMOOTHNESS_STEPS; i++) {
    float progress = i / (float)SMOOTHNESS_STEPS;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
      float x = 0;
      float y = 0;
      float z = BASE_HEIGHT;
      moveLegTo(leg, x, y, z);
    }
    delay(30);
  }
  
  // Сброс состояний
  currentHeight = BASE_HEIGHT;
  gaitProgress = 0.0;
  frontHeightAdjustment = 0;
  rearHeightAdjustment = 0;
  globalHeightAdjustment = 0;
}