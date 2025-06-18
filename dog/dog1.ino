#include <Thread.h>  
#include <Wire.h>
#include <math.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();

// Конфигурация NRF24L01
#define CE_PIN   7
#define CSN_PIN 8
const byte thisSlaveAddress[4] = {'F','A','S','S'};
RF24 radio(CE_PIN, CSN_PIN);

// Структура для данных с пульта
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


int spd = 50;

int L1=108;
int L2=130;

//double homeX, homeZ;

// Конфигурация сервоприводов
const int NUM_LEGS = 4;
const int NUM_JOINTS_PER_LEG = 3; // Плечо, бедро, колено
//const int FL = 0, FR = 1, RL = 2, RR = 3; // Индексы ног
//const int SHOULDER = 0, HIP = 1, KNEE = 2; // Индексы суставов

const int STEP_LENGTH = 80; //длина шага
const int STEP_HEIGHT = 40; //высота шага

//unsigned long previousMillis = 0;  // время хранения loop
//const long loop_interval = 500; // время проверки данных с джостика

const int legs[4][3][4] = {
  {{0, 130 ,600, 350}, {1, 130 ,600, 330}, {2,130, 600, 375}},   // FL плечо,бедро,локоть    
  {{4, 130 ,600, 340}, {5, 130 ,600, 347}, {6,130, 600, 395}},   // FR
  {{8, 130 ,600, 380}, {9, 130 ,600, 350}, {10,130, 600, 375}},   // RL
  {{12, 130 ,600, 365}, {13, 600 ,130, 385}, {14,600, 130, 350}}    // RR
};

float legspos[4][3] = { //X;Y , Height
  {0, 0, 220},  // FL   200 - примерно. надо вычислить home высоту TODO 
  {0, 0, 220},  // FR
  {0, 0, 220},  // RL
  {0, 0, 220}   // RR
};

Thread gefromjosticdata = Thread();

void processRemoteData(RemoteData data) {
  // Движение вперед/назад/повороты
  if (data.leftY < 400) {
      stepalllegs(false,2000); 
  } else if (data.leftY > 600) {
    stepalllegs(true,2000);
  } else if (data.leftX < 400) {
    heightchange4leg(30); //LEft
  } else if (data.leftX > 600) {
    heightchange4leg(-30);//right
  } else if (abs(data.leftX - 512) < 100 && abs(data.leftY - 512) < 100) {
    gohome();
  }
}


void readRemoteData() {
  if (radio.available()) {
    RemoteData data;

    radio.read(&data, sizeof(data));

    int calculatedChecksum = data.checksum = data.leftX +data.leftY +data.rightX +data.rightY +data.btn1+data.btn2+data.btn3+data.btn4;
    if (data.checksum == calculatedChecksum && calculatedChecksum != 0) {
      
      Serial.println("Received raw data:");
      Serial.print("LX: "); Serial.print(data.leftX);
      Serial.print(" LY: "); Serial.print(data.leftY);
      Serial.print(" RX: "); Serial.print(data.rightX);
      Serial.print(" RY: "); Serial.print(data.rightY);
      Serial.print(" BTN: "); Serial.print(data.btn1);
      Serial.print(","); Serial.print(data.btn2);
      Serial.print(","); Serial.print(data.btn3);
      Serial.print(","); Serial.print(data.btn4);
      Serial.print(" Checksum: "); Serial.println(data.checksum);
      
      processRemoteData(data);
    }
  }
}

double realangleToPulse(double angle, int minPulse, int maxPulse, double minAngle = 0, double maxAngle = 180) {
    if (minPulse > maxPulse) {
        return maxPulse + (minPulse - maxPulse) * (1 - angle/180.0);
    }
    return minPulse + (maxPulse - minPulse) * (angle/180.0);
}

// Преобразование импульса в угол с учетом реального диапазона
double realpulseToAngle(int pulse, int minPulse, int maxPulse, double minAngle = 0, double maxAngle = 180) { 
    if (minPulse > maxPulse) {
        return 180.0 - 180.0 * (pulse - maxPulse) / (minPulse - maxPulse);
    }
    return 180.0 * (pulse - minPulse) / (maxPulse - minPulse);
}

void forwardKinematics(int leg,int hipPulse, int kneePulse, double &x, double &z) {
  double b = realpulseToAngle(kneePulse, legs[leg][2][1], legs[leg][2][2]);
  double a = realpulseToAngle(hipPulse, legs[leg][1][1], legs[leg][1][2]);

  //Serial.print("a=");Serial.print(a);Serial.print(" b=");Serial.println(b);
  //double P1 = (600-130)/180; //1 пульс = 2,61 градусам

  double b_Rad = radians(b);
  double a_Rad = radians(a);
  double x1,z1;
  
  a_Rad = radians(45.5744 - a);
  x1 = L1 * cos(a_Rad);
  z1 = L1 * sin(a_Rad);
  
  x = x1 + L2 * cos((a_Rad - b_Rad));
  z = z1 + L2 * sin((a_Rad - b_Rad));
  /*Serial.print("leg ");Serial.print(leg);
  Serial.print(" x1="); Serial.print(x1);
  Serial.print(" z1="); Serial.print(z1);
  Serial.print(" x=");  Serial.print(x); Serial.print(" z="); Serial.println(z);*/
}

bool inverseKinematics(int leg, double X, double Z, int &hipPulse, int &kneePulse) {
  double D = sqrt(X*X+Z*Z);
 // Serial.print("D=");Serial.println(D);
  if (D > L1 + L2 || D < abs(L1 - L2)) {
        Serial.println("Точка недостижима");
        return false;
  }
  double a,b,g,t,k;
  double _const =  45.5744;//(180/(600-130)) * (249-130);
  b = M_PI - acos((L2*L2+L1*L1-D*D)/(2*L2*L1));

  if (X >= 0 && Z <= 0){ // 1 случай
    //Serial.println("[2]");
    t=acos((L1*L1-L2*L2+D*D)/(2*L1*D));
    k=acos(abs(X)/D);
    a = k-t+45.5744/57.29577; //+
  }
  else if (X >= 0 && Z >= 0){
    //Serial.println("[1]");
    k = acos(X/D);
    t = acos((L1*L1+D*D-L2*L2)/(2*L1*D));
    a = abs(45.5744/57.29577 - k - t);  //+
  }
  else if (X <= 0 && Z >= 0){
   // Serial.println("[4]");
    a = PI - acos((D*D + L1*L1 - L2*L2)/(2*L1*D));
  }
  else if (X <= 0 && Z <= 0){
    //Serial.println("[3]");
    //double tmp = _const / 57.29577;
    a = PI - (acos(abs(X)/D) - 45.5744/57.29577) - acos((D*D + L1*L1 - L2*L2)/(2*L1*D));
  }
  else{
    Serial.println("[!] Not calculate");
    return false;
  }
  //Serial.println("угол: ");
  //Serial.println(a*57.29577);
  //Serial.println(b*57.29577);
  
  double rad_a = degrees(a);//-0.795423467); // это 45.5744 градусов
  double rad_b = degrees(b);

  kneePulse = realangleToPulse(rad_b, legs[leg][2][1], legs[leg][2][2]);
  hipPulse = realangleToPulse(rad_a, legs[leg][1][1], legs[leg][1][2]);
  return true;
}

void move1Leg(bool forward,int STEP_DURATION,double STEP_LENGTH,double STEP_HEIGHT , int servnum1, int servnum2 , int homeX, int homeZ,int leg) {

  const int NUM_STEPS = 30;        // Количество промежуточных точек
  double x,z;
  for (int i = 0; i <= NUM_STEPS; i++) {
    double t = (double)i / NUM_STEPS;
    
    // Фаза переноса (ПОДЪЕМ ноги)
    if (forward) {
      x = homeX + STEP_LENGTH * (1 - cos(PI * t)) / 2;
    } else {
      x = homeX - STEP_LENGTH * (1 - cos(PI * t)) / 2;
    }
    z = homeZ + STEP_HEIGHT * sin(PI * t)/2; 
    
    // Отладочный вывод
    //Serial.print("Target: X="); Serial.print(x);
    //Serial.print(" Z="); Serial.println(z);
    moveToPoint(leg,x,z,servnum1,servnum2);

    delay(STEP_DURATION / (2 * NUM_STEPS));
  }
  double tmp;
  if (x > homeX) {
    tmp =  x - homeX;
  } else {
    tmp = homeX - x;
  }
  //2 фаза: возврат назад
  for (int i = 0; i <= NUM_STEPS; i++) {
   
    if (forward) {
      x = x - tmp / NUM_STEPS;
    } else {
      x = x + tmp / NUM_STEPS; 
    }
    moveToPoint(leg,x,z,servnum1,servnum2);
    delay(STEP_DURATION / (2 * NUM_STEPS));
  }
}

bool moveToPoint(int leg,double targetX, double targetZ, int servnum1, int servnum2 ) {
  int hipPulse, kneePulse;
  
  if (inverseKinematics(leg,targetX, targetZ, hipPulse, kneePulse)) {
    // Управление сервоприводами
    pwm1.setPWM(servnum1, 0, hipPulse);
    pwm1.setPWM(servnum2, 0, kneePulse);
    return true;
  } else {
    Serial.println("Ошибка перемещения");
  }
  return false;
}

void stepalllegs(bool forward,int speed){
  step2legs(forward,0,3, legspos[0][0], legspos[0][1], legspos[3][0], legspos[3][1], speed); 
  step2legs(forward,1,2, legspos[1][0], legspos[1][1], legspos[2][0], legspos[2][1], speed);

}
/*
void step2legs(bool forward, int legF, int legR, int homeX, int homeZ, int homeX1, int homeZ1,int STEP_DURATION){
  const int NUM_STEPS = 30;        // Количество промежуточных точек
  const int delta = 2;        // Разница можду шагаме переда и зада
  double x,z,x1,z1, t, t1;
  for (int i = 0; i <= NUM_STEPS+delta; i++) {
    if (i<= NUM_STEPS){
      t = (double)i / NUM_STEPS;

      if (forward) {
        x = homeX + STEP_LENGTH * (1 - cos(PI * t)) / 2;
      } else {
        x = homeX - STEP_LENGTH * (1 - cos(PI * t)) / 2;
      }
      z = homeZ + STEP_HEIGHT * sin(PI * t)/2; 
      moveToPoint(legF, x, z , legs[legF][1][0] , legs[legF][2][0]);

      legspos[legF][0] = x;
      legspos[legF][1] = z;
    }
    if (i >= delta){
      t1 = (double)(i-delta) / NUM_STEPS;

      if (forward) {
        x1 = homeX1 + STEP_LENGTH * (1 - cos(PI * t1)) / 2;
      } else {
        x1 = homeX1 - STEP_LENGTH * (1 - cos(PI * t1)) / 2;
      }
      z1 = homeZ1 + STEP_HEIGHT * sin(PI * t1)/2; 

      moveToPoint(legR, x1, z1 , legs[legR][1][0] , legs[legR][2][0]);
      
      legspos[legR][0] = x1;
      legspos[legR][1] = z1;
    }
    delay(STEP_DURATION / (2 * NUM_STEPS));
  }
  double tmp;
  if (x > homeX) {
    tmp =  x - homeX;
  } else {
    tmp = homeX - x;
  }
  //2 фаза: возврат назад
  for (int i = 0; i <= NUM_STEPS; i++) {
   
    if (forward) {
      x = x - tmp / NUM_STEPS;
    } else {
      x = x + tmp / NUM_STEPS; 
    }
    moveToPoint(x,z);
    delay(STEP_DURATION / (2 * NUM_STEPS));
  }
}
*/

void step2legs(bool forward, int legF, int legR, int homeX, int homeZ, int homeX1, int homeZ1, int STEP_DURATION) {
  const int NUM_STEPS = 30;
  const int delta = 2;
  const int NUM_LEGS = 4; // Общее количество ног (квадропод)

  // Определение индексов двух других ног (не legF и legR)
  int otherLegs[2];
  int idx = 0;
  for (int j = 0; j < NUM_LEGS; j++) {
    if (j != legF && j != legR) {
      otherLegs[idx++] = j;
    }
  }

  // Получение начальных позиций для дополнительных ног
  double homeX_other1 = legspos[otherLegs[0]][0];
  double homeZ_other1 = legspos[otherLegs[0]][1];
  double homeX_other2 = legspos[otherLegs[1]][0];
  double homeZ_other2 = legspos[otherLegs[1]][1];

  double x, z, x1, z1, t, t1;
  for (int i = 0; i <= NUM_STEPS + delta; i++) {
    // Прогресс для линейного перемещения (0.0 до 1.0)
    double progress = (double)i / (NUM_STEPS + delta);
    
    // Вычисление линейного смещения для дополнительных ног
    double dx_other = (forward ? -STEP_LENGTH : STEP_LENGTH) * progress;
    
    // Позиции для дополнительных ног (линейное движение)
    double x_other1 = homeX_other1 + dx_other;
    double z_other1 = homeZ_other1;
    double x_other2 = homeX_other2 + dx_other;
    double z_other2 = homeZ_other2;

    // Перемещение дополнительных ног
    moveToPoint(otherLegs[0], x_other1, z_other1, legs[otherLegs[0]][1][0], legs[otherLegs[0]][2][0]);
    moveToPoint(otherLegs[1], x_other2, z_other2, legs[otherLegs[1]][1][0], legs[otherLegs[1]][2][0]);
    
    // Обновление позиций дополнительных ног
    legspos[otherLegs[0]][0] = x_other1;
    legspos[otherLegs[0]][1] = z_other1;
    legspos[otherLegs[1]][0] = x_other2;
    legspos[otherLegs[1]][1] = z_other2;

    // Оригинальная логика для ног legF и legR (криволинейное движение)
    if (i <= NUM_STEPS) {
      t = (double)i / NUM_STEPS;
      
      if (forward) {
        x = homeX + STEP_LENGTH * (1 - cos(PI * t)) / 2;
      } else {
        x = homeX - STEP_LENGTH * (1 - cos(PI * t)) / 2;
      }
      z = homeZ + STEP_HEIGHT * sin(PI * t) / 2; 
      moveToPoint(legF, x, z, legs[legF][1][0], legs[legF][2][0]);
      legspos[legF][0] = x;
      legspos[legF][1] = z;
    }
    
    if (i >= delta) {
      t1 = (double)(i - delta) / NUM_STEPS;

      if (forward) {
        x1 = homeX1 + STEP_LENGTH * (1 - cos(PI * t1)) / 2;
      } else {
        x1 = homeX1 - STEP_LENGTH * (1 - cos(PI * t1)) / 2;
      }
      z1 = homeZ1 + STEP_HEIGHT * sin(PI * t1) / 2; 
      moveToPoint(legR, x1, z1, legs[legR][1][0], legs[legR][2][0]);
      legspos[legR][0] = x1;
      legspos[legR][1] = z1;
    }
    
    delay(STEP_DURATION / (2 * NUM_STEPS));
  }
}

void heightchange1legonstep(int leg,int step){
  int curr_height = legspos[leg][2];
  int x = legspos[leg][0];
  int z = legspos[leg][1];
  if (step >= 0){ //ВВЕРХ
      moveToPoint(leg, x , z - step, legs[leg][1][0], legs[leg][2][0] );
      legspos[leg][1] = z - step;
  }
  else {//ВНИЗ
    moveToPoint(leg, x , z + step, legs[leg][1][0], legs[leg][2][0] );
    legspos[leg][1] = z + step;
  }
}

void heightchange4leg(int hight){
  for (int i = 0 ; i <  hight; i++){
    int x,z;
    if (hight >= 0){ //ВВЕРХ
      for (int leg = 0 ; leg <  NUM_LEGS; leg++){
        //x = legspos[leg][0];
        z = legspos[leg][1];
        heightchange1legonstep(leg,z + i);
        legspos[leg][1] = z + i;
        legspos[leg][2] = legspos[leg][2] + i;
      }

    }else{
      for (int leg = 0 ; leg <  NUM_LEGS; leg++){
        //x = legspos[leg][0];
        z = legspos[leg][1];
        heightchange1legonstep(leg,z - i);
        legspos[leg][1] = z - i;
        legspos[leg][2] = legspos[leg][2] - i;
      }
    }
  }
}

void heightchangeleftleg(int hight){
  for (int i = 0 ; i <  hight; i++){
    if (hight >= 0){ //ВВЕРХ
      heightchange1legonstep(0,legspos[0][1] + i);
      legspos[0][1] = legspos[0][1] + i;
      legspos[0][2] = legspos[0][2] + i;

      heightchange1legonstep(3,legspos[3][1]+ i);
      legspos[3][1] = legspos[3][1] + i;
      legspos[3][2] = legspos[3][2] + i;
    }else{
      heightchange1legonstep(0,legspos[0][1] - i);
      legspos[0][1] = legspos[0][1] - i;
      legspos[0][2] = legspos[0][2] - i;

      heightchange1legonstep(3,legspos[3][1]- i);
      legspos[3][1] = legspos[3][1] - i;
      legspos[3][2] = legspos[3][2] - i;
    }
  }
}

void heightchangerightleg(int hight){
  for (int i = 0 ; i <  hight; i++){
    if (hight >= 0){ //ВВЕРХ
      heightchange1legonstep(1,legspos[1][1] + i);
      legspos[1][1] = legspos[1][1] + i;
      legspos[1][2] = legspos[1][2] + i;

      heightchange1legonstep(2,legspos[2][1]+ i);
      legspos[2][1] = legspos[2][1] + i;
      legspos[2][2] = legspos[2][2] + i;
    }else{
      heightchange1legonstep(1,legspos[1][1] - i);
      legspos[1][1] = legspos[1][1] - i;
      legspos[1][2] = legspos[1][2] - i;

      heightchange1legonstep(2,legspos[2][1]- i);
      legspos[2][1] = legspos[2][1] - i;
      legspos[2][2] = legspos[2][2] - i;
    }
  }
}

void heightchangeforwardleg(int hight){
  for (int i = 0 ; i <  hight; i++){
    if (hight >= 0){ //ВВЕРХ
      heightchange1legonstep(0,legspos[0][1] + i);
      legspos[0][1] = legspos[0][1] + i;
      legspos[0][2] = legspos[0][2] + i;

      heightchange1legonstep(1,legspos[1][1]+ i);
      legspos[1][1] = legspos[1][1] + i;
      legspos[1][2] = legspos[1][2] + i;
    }else{
      heightchange1legonstep(0,legspos[0][1] - i);
      legspos[0][1] = legspos[0][1] - i;
      legspos[0][2] = legspos[0][2] - i;

      heightchange1legonstep(1,legspos[1][1]- i);
      legspos[1][1] = legspos[1][1] - i;
      legspos[1][2] = legspos[1][2] - i;
    }
  }
}

void heightchangebackleg(int hight){
  for (int i = 0 ; i <  hight; i++){
    if (hight >= 0){ //ВВЕРХ
      heightchange1legonstep(2,legspos[2][1] + i);
      legspos[2][1] = legspos[2][1] + i;
      legspos[2][2] = legspos[2][2] + i;

      heightchange1legonstep(3,legspos[3][1]+ i);
      legspos[3][1] = legspos[3][1] + i;
      legspos[3][2] = legspos[3][2] + i;
    }else{
      heightchange1legonstep(2,legspos[2][1] - i);
      legspos[2][1] = legspos[2][1] - i;
      legspos[2][2] = legspos[2][2] - i;

      heightchange1legonstep(3,legspos[3][1]- i);
      legspos[3][1] = legspos[3][1] - i;
      legspos[3][2] = legspos[3][2] - i;
    }
  }
}

void gohome(){  // от любого положения до домашних позиций. TODO - проверять знаки!!!
  int count = 20; 
  double x,z;
  int delta[1][2];
  for (int i=0; i< NUM_LEGS; i++){
    forwardKinematics(i,legs[i][1][4], legs[i][2][4], x, z); // HOME x,y
    delta[i][0] = x - legspos[i][0];
    delta[i][1] = z - legspos[i][1];
  }
  for (int i=0; i< NUM_LEGS; i++){
    float stepx =  delta[i][0] / count;
    float stepz =  delta[i][0] / count;
    moveToPoint(i,legspos[i][0]+stepx, legspos[i][1]+stepz,legs[i][1][0], legs[i][2][0] ) ;
    legspos[i][0] += stepx;
    legspos[i][1] += stepz;
    legspos[i][2] += stepz;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start...");

  pwm1.begin();
  pwm1.setOscillatorFrequency(25000000);
  pwm1.setPWMFreq(60);

  // Инициализация NRF24L01
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.setChannel(100);
  radio.setAutoAck(true);
  radio.startListening();
  Serial.println("NRF24L01 инициализирован");

  gefromjosticdata.onRun(readRemoteData);  // назначаем потоку задачу
  gefromjosticdata.setInterval(1000);
 
  //Go HOME
  for (int j = 0; j < NUM_JOINTS_PER_LEG; j++) {
     for (int leg = 0; leg < NUM_LEGS ; leg++) {
      pwm1.setPWM(legs[leg][j][0], 0, legs[leg][j][3]);
      delay(30);
    }
  }
  
  // Get current X;Y
  double x, z;
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    forwardKinematics(leg,legs[leg][1][3], legs[leg][2][3], x, z);
    legspos[leg][0] = x;
    legspos[leg][1] = z;
  }
  //pwm1.setPWM(13, 0, 495);
  //pwm1.setPWM(14, 0, 250);
/*
  int h,k;
  double x, z;
  //moveToPoint(0, x, z , legs[legF][1][0] , legs[legF][2][0]);
  forwardKinematics(3,legs[0][1][3], legs[0][2][3], x, z);
  Serial.print("X ");Serial.print(x);Serial.print(" Z ");Serial.println(z);
  inverseKinematics(3, x, z, h, k);
  Serial.print("hippulse ");Serial.print(h);Serial.print(" kneepulse ");Serial.println(k);


*/

  //move1Leg(false,30,80,40 ,5,6 , legspos[3][0], legspos[3][1],3);
//stepalllegs(false,5000);
 
  //while(true){}
}

void loop() {
  delay(50);
  stepalllegs(false,1000);
//move1Leg(false,30,80,40 ,5,6 , legspos[3][0], legspos[3][1],3);
  //if (gefromjosticdata.shouldRun())
   //     gefromjosticdata.run();
  /*  АНАЛОГ
  unsigned long currentMillis = millis(); 
  if (currentMillis - previousMillis >= loop_interval){
     previousMillis = currentMillis;
     // чтение данных с джостика
  }*/
}