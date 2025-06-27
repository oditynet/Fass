#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Конфигурация NRF24L01
#define CE_PIN   7
#define CSN_PIN 8
const byte slaveAddress[4] = {'F','A','S','S'};
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

// Пины джойстиков
const int LEFT_JOYSTICK_X = A2;
const int LEFT_JOYSTICK_Y = A3;
const int RIGHT_JOYSTICK_X = A0;
const int RIGHT_JOYSTICK_Y = A1;

struct RemoteData {
  int leftX;
  int leftY;
  int rightX;
  int rightY;
  int btn1;
  int btn2;
  int btn3;
  int btn4;
  int checksum; // Добавляем поле для контрольной суммы
};

// Параметры калибровки
const int NEUTRAL_ZONE = 50;  // Зона нечувствительности
const int MIN_THRESHOLD = 400; // Нижний порог активации
const int MAX_THRESHOLD = 600; // Верхний порог активации

const int button1 = 3;
const int button2 = 2;
const int button3 = 6;
const int button4 = 5;

// Таймер для отправки данных
unsigned long lastSendTime = 0;
const int SEND_INTERVAL = 500; // Интервал отправки (мс)

void setup() {
  Serial.begin(115200);
  Serial.println("Инициализация пульта управления...");
  
  // Настройка пинов джойстиков
  pinMode(LEFT_JOYSTICK_X, INPUT);
  pinMode(LEFT_JOYSTICK_Y, INPUT);
  pinMode(RIGHT_JOYSTICK_X, INPUT);
  pinMode(RIGHT_JOYSTICK_Y, INPUT);

    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    //radio.setRetries(1,1); // delay, count
    radio.openWritingPipe(slaveAddress);
    radio.setChannel(98);
     radio.setAutoAck(true);
  
  while (!radio.isChipConnected()) {
    Serial.println("Ошибка: nRF24L01 не подключен!");
    delay(1000);
  }
  
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);

  Serial.println("Пульт готов к работе");
}

void loop() {
  // Чтение значений с джойстиков
  RemoteData data;
  
  // Отправка данных с заданным интервалом
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    data.leftX = analogRead(LEFT_JOYSTICK_X);
    data.leftY = analogRead(LEFT_JOYSTICK_Y);
    data.rightX = analogRead(RIGHT_JOYSTICK_X);
    data.rightY = analogRead(RIGHT_JOYSTICK_Y);
    

    data.btn1 = !digitalRead(button1); // Инвертируем, так как INPUT_PULLUP
    data.btn2 = !digitalRead(button2);
    data.btn3 = !digitalRead(button3);
    data.btn4 = !digitalRead(button4);

    applyCalibration(data);

    // Вычисление контрольной суммы
    data.checksum = data.leftX +data.leftY +data.rightX +data.rightY +data.btn1+data.btn2+data.btn3+data.btn4;

    bool result = radio.write(&data, sizeof(data));
    if (result) {
        Serial.println("[+]");
    }
    else {
        Serial.print("-");
    }


    lastSendTime = millis();
  }
  
  // Вывод значений в Serial для отладки
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 500) {
    printJoystickData(data);
    lastPrintTime = millis();
  }
  delay(300);
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