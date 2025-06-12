#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EncButton.h>

// Конфигурация NRF24L01
#define CE_PIN 7
#define CSN_PIN 8
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "ROBOT";

// Пины джойстиков
const int LEFT_JOYSTICK_X = A0;
const int LEFT_JOYSTICK_Y = A1;
const int RIGHT_JOYSTICK_X = A2;
const int RIGHT_JOYSTICK_Y = A3;

struct RemoteData {
  int leftX;
  int leftY;
  int rightX;
  int rightY;
};

// Параметры калибровки
const int NEUTRAL_ZONE = 50;  // Зона нечувствительности
const int MIN_THRESHOLD = 400; // Нижний порог активации
const int MAX_THRESHOLD = 600; // Верхний порог активации

Button btn1(3);
Button btn2(2);
Button btn3(6);
Button btn4(5);

// Светодиод статуса
//const int STATUS_LED = 4;

// Таймер для отправки данных
unsigned long lastSendTime = 0;
const int SEND_INTERVAL = 200; // Интервал отправки (мс)

void setup() {
  Serial.begin(115200);
  Serial.println("Инициализация пульта управления...");
  
  // Настройка пинов джойстиков
  pinMode(LEFT_JOYSTICK_X, INPUT);
  pinMode(LEFT_JOYSTICK_Y, INPUT);
  pinMode(RIGHT_JOYSTICK_X, INPUT);
  pinMode(RIGHT_JOYSTICK_Y, INPUT);

  radio.begin();
  radio.setChannel(108); // Канал 108 (2.508 GHz)
  radio.setDataRate(RF24_250KBPS); // Низкая скорость = стабильнее
  radio.setPALevel(RF24_PA_MAX); // Макс. мощность
  radio.setRetries(3, 5); // 3*250мкс задержка, 5 попыток
  radio.openWritingPipe(address);
  radio.stopListening();
  
  while (!radio.isChipConnected()) {
    Serial.println("Ошибка: nRF24L01 не подключен!");
    delay(1000);
  }

  Serial.println("Пульт готов к работе");
}

void loop() {
  // Чтение значений с джойстиков
  RemoteData data;

  btn1.tick();
  btn2.tick();
  btn3.tick();
  btn4.tick();

  //if (btn1.click()) Serial.println("btn 1 click!");
  if (btn1.hold()) Serial.println("btn 1!");
  if (btn2.hold()) Serial.println("btn 2!");
  if (btn3.hold()) Serial.println("btn 3!");
  if (btn4.hold()) Serial.println("btn 4!");
  /*SensorData data = {
    analogRead(A0),
    analogRead(A1),
    analogRead(A2),
    analogRead(A3)
  };*/
  
  // Отправка данных с заданным интервалом
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    data.leftX = analogRead(LEFT_JOYSTICK_X);
    data.leftY = analogRead(LEFT_JOYSTICK_Y);
    data.rightX = analogRead(RIGHT_JOYSTICK_X);
    data.rightY = analogRead(RIGHT_JOYSTICK_Y);

    applyCalibration(data);
    sendData(data);
    lastSendTime = millis();
  }
  
  // Вывод значений в Serial для отладки
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 500) {
    printJoystickData(data);
    lastPrintTime = millis();
  }
  delay(100);
}

// Применение калибровки и зоны нечувствительности
void applyCalibration(RemoteData &data) {
  // Левый джойстик (движение)
  data.leftX = mapAndFilter(data.leftX);
  data.leftY = mapAndFilter(data.leftY);
  
  // Правый джойстик (высота)
  data.rightX = mapAndFilter(data.rightX);
  data.rightY = mapAndFilter(data.rightY);
}

// Фильтрация значений и преобразование в диапазон 0-1023
int mapAndFilter(int value) {
  // Применение зоны нечувствительности
  if (abs(value - 512) < NEUTRAL_ZONE) {
    return 512; // Нейтральное положение
  }
  return value;
}

// Отправка данных роботу
void sendData(RemoteData data) {
  bool result = radio.write(&data, sizeof(data));
  //Serial.print(result ? "[+]" : "[-]");
}

// Вывод данных джойстиков в Serial
void printJoystickData(RemoteData data) {
  Serial.print("Левый X: ");
  Serial.print(data.leftX);
  Serial.print(" Y: ");
  Serial.print(data.leftY);
  
  Serial.print(" | Правый X: ");
  Serial.print(data.rightX);
  Serial.print(" Y: ");
  Serial.print(data.rightY);
  
  // Расшифровка команд
  Serial.print(" | Команда: ");
  
  // Движение
  if (data.leftY < MIN_THRESHOLD) {
    Serial.print("ВПЕРЕД ");
  } else if (data.leftY > MAX_THRESHOLD) {
    Serial.print("НАЗАД ");
  }
  
  if (data.leftX < MIN_THRESHOLD) {
    Serial.print("ВЛЕВО ");
  } else if (data.leftX > MAX_THRESHOLD) {
    Serial.print("ВПРАВО ");
  }
  
  // Управление высотой
  if (data.rightY < MIN_THRESHOLD) {
    if (data.rightX < MIN_THRESHOLD) {
      Serial.print("ПОДНЯТЬ ПЕРЕД ");
    } else if (data.rightX > MAX_THRESHOLD) {
      Serial.print("ПОДНЯТЬ ЗАД ");
    } else {
      Serial.print("ПОДНЯТЬ ВСЕ ");
    }
  } else if (data.rightY > MAX_THRESHOLD) {
    if (data.rightX < MIN_THRESHOLD) {
      Serial.print("ОПУСТИТЬ ПЕРЕД ");
    } else if (data.rightX > MAX_THRESHOLD) {
      Serial.print("ОПУСТИТЬ ЗАД ");
    } else {
      Serial.print("ОПУСТИТЬ ВСЕ ");
    }
  }
  
  if (data.leftX >= MIN_THRESHOLD && data.leftX <= MAX_THRESHOLD && 
      data.leftY >= MIN_THRESHOLD && data.leftY <= MAX_THRESHOLD &&
      data.rightY >= MIN_THRESHOLD && data.rightY <= MAX_THRESHOLD) {
    Serial.print("СТОП");
  }
  
  Serial.println();
}