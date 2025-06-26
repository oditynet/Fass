
#include <Thread.h>  
#include <Wire.h>
#include <math.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();

#define TRIG1 2
#define ECHO1 3
#define TRIG2 5
#define ECHO2 4

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


//int spd = 50;

int L0 = 38;
int L1 = 108;
int L2 = 130;

int Lbody = 280;
//double homeX, homeZ;

// Конфигурация сервоприводов
const int NUM_LEGS = 4;
const int NUM_JOINTS_PER_LEG = 3; // Плечо, бедро, колено
//const int FL = 0, FR = 1, RL = 2, RR = 3; // Индексы ног
//const int SHOULDER = 0, HIP = 1, KNEE = 2; // Индексы суставов

const int STEP_LENGTH = 40; //длина шага
const int STEP_HEIGHT = 30; //высота шага

//unsigned long previousMillis = 0;  // время хранения loop
//const long loop_interval = 500; // время проверки данных с джостика

const int legs[4][3][4] = {
  {{0, 130 ,600, 353}, {1, 130 ,600, 400}, {2,130, 600, 345}},   // FL плечо,бедро,локоть    
  {{4, 130 ,600, 335}, {5, 600 ,130, 347}, {6,600, 130, 379}},   // FR
  {{8, 130 ,600, 370}, {9, 130 ,600, 350}, {10,130, 600, 360}},   // RL
  {{12, 130 ,600, 372}, {13, 600 ,130, 385}, {14,600, 130, 360}}    // RR
};

float legspos[4][4] = { //X, Z, Height, Y
  {0, 0, 220, 0},  // FL   200 - примерно. надо вычислить home высоту TODO 
  {0, 0, 220, 0},  // FR
  {0, 0, 220, 0},  // RL
  {0, 0, 220, 0}   // RR
};

//Thread gefromjosticdata = Thread();

float getDistance(int trigPin, int echoPin) {
  // Генерируем импульс 10 мкс
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Измеряем длительность импульса
  long duration = pulseIn(echoPin, HIGH);
  
  // Рассчитываем расстояние (см)
  //return duration * 0.034 / 2;
   return duration / 58;
}

void processRemoteData(RemoteData data) {
  // Движение вперед/назад/повороты
  if (data.leftY < 400) {
      stepalllegs(false,2000,0); 
  } else if (data.leftY > 600) {
    stepalllegs(true,2000,0);
  } else if (data.leftX < 400) {
    heightchange4leg(30); //LEft
  } else if (data.leftX > 600) {
    heightchange4leg(-30);//right
  } else if (abs(data.leftX - 512) < 100 && abs(data.leftY - 512) < 100) {
    gohome();
  }
  /*if (data.rightX < 400) {
    step4legstoside(true, 30, 2000); // Влево
  } else if (data.rightX > 600) {
    step4legstoside(false, 30, 2000); // Вправо
  }*/
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
  double MM_PI = 3.1415926;
  b = MM_PI - acos((L2*L2+L1*L1-D*D)/(2*L2*L1));

  if (X >= 0 && Z <= 0){ // 1 случай
   // Serial.println("[2]");
    t=acos((L1*L1-L2*L2+D*D)/(2*L1*D));
    k=acos(abs(X)/D);
    a = k-t+_const/57.29577; //+
  }
  else if (X >= 0 && Z >= 0){
    //Serial.println("[1]");
    k = acos(X/D);
    t = acos((L1*L1+D*D-L2*L2)/(2*L1*D));
    a = abs(_const/57.29577 - k - t);  //+
  }
  else if (X <= 0 && Z >= 0){
    //Serial.println("[4]");
    //a = MM_PI - acos((D*D + L1*L1 - L2*L2)/(2*L1*D)); //TODO кажется неверно ищится,т.к. весь треугольник не надо учитывать,раз у нас есть линия по 600 имп.
    a = /*90 гр перевел в рад*/1.57079632679 + (acos(abs(X)/D)) -  acos((D*D + L1*L1 - L2*L2)/(2*L1*D)); // перепроверить надо!!!!!
  }
  else if (X <= 0 && Z <= 0){
    //Serial.println("[3]");//Serial.println(MM_PI);Serial.println(PI);
    double k = acos(abs(X)/D);
    if (k * 57.29577 >= _const ) {// гр > гр
      //Serial.println("[3_1]");
     // Serial.println(k * 57.29577);
      a = MM_PI - (acos(abs(X)/D) - _const/57.29577) - acos((D*D + L1*L1 - L2*L2)/(2*L1*D));
    }else{
      //Serial.println("[3_2]");
      double kk = asin(abs(Z)/D);
      //a = kk-0.77537299217 ;
      a = MM_PI - (acos((D*D + L1*L1 - L2*L2)/(2*L1*D)) - kk + 0.77537299217); /*(90-45гр и в радианы перевел)*/
    }
    //double tmp = _const / 57.29577;
    //a = MM_PI - (acos(abs(X)/D) - (90 - _const)/57.29577) - acos((D*D + L1*L1 - L2*L2)/(2*L1*D));
    /*double theta = atan2(abs(X), abs(Z)); // Угол от отрицательной оси Z
    double cosGamma = (D*D + L1*L1 - L2*L2) / (2*L1*D);
    double gamma = acos(cosGamma);
    a = theta - gamma + _const/57.29577;*/
    //a = /*90 гр перевел в рад*/1.57079632679 + (acos(abs(X)/D)) -  acos((D*D + L1*L1 - L2*L2)/(2*L1*D));
  }
  else{
    Serial.println("[!] Not calculate");
    return false;
  }
  /*Serial.println("угол: ");
  Serial.println(a*57.29577);
  Serial.println(b*57.29577);
  */
  double rad_a = degrees(a);//-0.795423467); // это 45.5744 градусов
  double rad_b = degrees(b);

  kneePulse = realangleToPulse(rad_b, legs[leg][2][1], legs[leg][2][2]);
  hipPulse = realangleToPulse(rad_a, legs[leg][1][1], legs[leg][1][2]);
  return true;
}

void move1Leg(bool forward,int STEP_DURATION,double STEP_LENGTH,double STEP_HEIGHT , int servnum1, int servnum2 , int homeX, int homeZ,int leg) {

  const int NUM_STEPS = 25;        // Количество промежуточных точек
  double x,z;
  for (int i = 0; i <= NUM_STEPS; i++) {
    double t = (double)i / NUM_STEPS;
    
    // Фаза переноса (ПОДЪЕМ ноги)
    if (forward) {
      x = homeX + STEP_LENGTH * (1 - cos(PI * t)) ;
    } else {
      x = homeX - STEP_LENGTH * (1 - cos(PI * t)) ;
    }
    z = homeZ + STEP_HEIGHT * sin(PI * t); 
    
    // Отладочный вывод
    /*Serial.print("Target: X="); Serial.print(x);
    Serial.print(" Z="); Serial.println(z);
    moveToPoint(leg,x,z,servnum1,servnum2);*/

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

   /* Serial.print("Target: X="); Serial.print(x);
    Serial.print(" Z="); Serial.println(z);
*/
    moveToPoint(leg,x,z,servnum1,servnum2);
    delay(STEP_DURATION / (2 * NUM_STEPS));
  }
}

bool moveToPoint(int leg,double targetX, double targetZ, int servnum1, int servnum2 ) {
  int hipPulse, kneePulse;

 // Serial.println(targetZ);
  
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


void stepalllegs(bool forward,int speed, int left){ // left ^ 0- forward, 1- left, 2- right
 /* if (left == 1 || left ==0 ){
    step2legs(forward,0,3, legspos[0][0], legspos[0][1], legspos[3][0], legspos[3][1], speed); //
    step2legs(forward,1,2, legspos[1][0], legspos[1][1], legspos[2][0], legspos[2][1], speed); // поворот налево только
  }
  if (left == 1 || left == 0){
    step2legs(forward,1,2, legspos[1][0], legspos[1][1], legspos[2][0], legspos[2][1], speed);
    step2legs(forward,0,3, legspos[0][0], legspos[0][1], legspos[3][0], legspos[3][1], speed); 
  }*/
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
int changeleg(int leg) {
    const int nextLeg[] = {2, 3, 1, 0}; // 0->2, 1->3, 2->1, 3->0
    return nextLeg[leg];
}
int changeleg0312(int leg) {
    const int nextLeg[] = {3, 2, 0, 1}; // 0->2, 1->3, 2->1, 3->0
    return nextLeg[leg];
}

void step4legs(bool forward, int STEP_DURATION, int len4step = 150) {
    const int NUM_SECTIONS = 4;
    const int STEPS_PER_SECTION = 10; // Фиксированное количество шагов на секцию
    const float SECTION_DURATION = STEP_DURATION / (float)NUM_SECTIONS;
    const float TIME_PER_STEP = SECTION_DURATION / STEPS_PER_SECTION;
    const float SECTION_LENGTH = len4step / (float)NUM_SECTIONS;

    int active_leg = 0; // Начинаем с ноги 0
    float start_positions[4][2]; // Стартовые позиции для всех ног

    for (int section = 0; section < NUM_SECTIONS; section++) {
        // Сохраняем стартовые позиции для всех ног
        for (int l = 0; l < 4; l++) {
            start_positions[l][0] = legspos[l][0];
            start_positions[l][1] = legspos[l][1];
        }

        for (int step = 0; step <= STEPS_PER_SECTION; step++) {
            float t = (float)step / STEPS_PER_SECTION; // Прогресс [0, 1]

            // Обработка ВСЕХ ног
            for (int l = 0; l < 4; l++) {
                if (l == active_leg) {
                    // Траектория активной ноги (полуэллипс)
                    float x, z;
                    if (forward) {
                        x = start_positions[l][0] + SECTION_LENGTH * t;
                    } else {
                        x = start_positions[l][0] - SECTION_LENGTH * t;
                    }
                    z = start_positions[l][1] + STEP_HEIGHT * sin(PI * t);
                    moveToPoint(l, x, z, legs[l][1][0], legs[l][2][0]);
                    legspos[l][0] = x;
                    legspos[l][1] = z;
                    delay(TIME_PER_STEP/(4*10));//быстрее
                } else {
                    // Траектория опорных ног (прямая с компенсацией)
                    float compensation = (forward ? -1 : 1) * SECTION_LENGTH * t / 3.0f;
                    float x = start_positions[l][0] + compensation;
                    moveToPoint(l, x, start_positions[l][1], legs[l][1][0], legs[l][2][0]);
                    // Не обновляем legspos здесь - только в конце секции
                    delay(TIME_PER_STEP/4);
                }
            }
            //delay(TIME_PER_STEP);
        }

        // Финализируем позиции после секции
        for (int l = 0; l < 4; l++) {
            if (l != active_leg) {
                float compensation = (forward ? -1 : 1) * SECTION_LENGTH / 3.0f;
                legspos[l][0] = start_positions[l][0] + compensation;
            }
        }

        // Логирование после каждой секции
        Serial.print("End section ");
        Serial.print(section);
        Serial.print(" - ");
        for (int l = 0; l < 4; l++) {
            Serial.print("Leg");
            Serial.print(l);
            Serial.print(": ");
            Serial.print(legspos[l][0]);
            Serial.print(" ");
        }
        Serial.println();

        // Переключаем активную ногу
        active_leg = changeleg(active_leg);
    }
}

void step1legtoside(bool left, int leg,int len){  // work!!
  double ab = (left ? 1 : -1)*(atan(len/abs(legspos[leg][1])) + atan((L1+L2)/L0));
  Serial.println(ab);
  //double rad_ab = degrees(ab);
  double shoulder = realangleToPulse(degrees(ab), legs[leg][0][1], legs[leg][0][2]);// +(left ? 1 : -1)*legs[leg][0][3];
  Serial.println(shoulder);
  pwm1.setPWM(legs[leg][0][0], 0, shoulder);
}

void step4legstoside(bool left, int STEP_DURATION, int Rcentr = 200) {
    int max_step = 30;//50 мм может шагнуть максимально в сторону

    const int NUM_SECTIONS = 4;
    const int STEPS_PER_SECTION = 10;
    const float SECTION_DURATION = STEP_DURATION / (float)NUM_SECTIONS;
    const float TIME_PER_STEP = SECTION_DURATION / STEPS_PER_SECTION;
    float SECTION_LENGTH = max_step / (float)NUM_SECTIONS;

    int active_leg = 0;
    float angle_curr[4][1]; // [0]-Y, [1]-Z

    double Rbody = Rcentr + Lbody;
    
    // Ограничение угла поворота по максимально возможному смещению
    double maxAngle = tan( max_step / Rbody) ;// *57.29577;
    // Смещение для передних лап (меньший радиус)
    double FShift = Rcentr * tan(maxAngle);
    // Смещение для задних лап (больший радиус)
    double RShift = max_step; 

   // double dhF=0,dhR=0;
    Serial.println("[FR] ");
    Serial.println(FShift);
    Serial.println(RShift);
    // Сохраняем стартовые позиции угла
    for (int l = 0; l < 4; l++) {   
        angle_curr[l][0] = realpulseToAngle(legs[l][0][3], legs[l][0][1],legs[l][0][2])/57.29577;   //1

        float dy;
        if (l == 0 || l == 1){
          dy = FShift ;
        }else{
          dy = RShift;
        }
        float H = legspos[l][1];
        float alfa = acos((H*H+H*H-dy*dy)/(2*H*H));
        float beta = (180-alfa)/2;
        float dz = sin(90 - beta) * dy; // высота по Z

        angle_curr[l][1] = dz;                                              //2
    }

    for (int section = 0; section < NUM_SECTIONS; section++) {
      for (int step = 0; step <= STEPS_PER_SECTION; step++) {
        if (active_leg == 0 || active_leg == 1){
          SECTION_LENGTH = FShift / STEPS_PER_SECTION;
        }else{
          SECTION_LENGTH = RShift / STEPS_PER_SECTION;
        }
        for (int l = 0; l < 4; l++) {
            if (l == active_leg) {

              float newY, newZ;
              newY =legspos[l][3]  + (left ? 1 : -1) * SECTION_LENGTH ;
              float dy = abs(newY -legspos[l][3]); 

              float H = legspos[l][1];

              float alfa = acos((H*H+H*H-dy*dy)/(2*H*H));
              float beta = (180-alfa)/2;
              float dz = sin(90 - beta) * dy; // высота по Z

              angle_curr[active_leg][0] = (left ? -1 : 1)*alfa + angle_curr[active_leg][0];
              float angle_deg = degrees(angle_curr[active_leg][0]) ; // отклонение влево/вправо

              int shoulderPulse = realangleToPulse(angle_deg, legs[l][0][1], legs[l][0][2]) ;                    
              pwm1.setPWM(legs[l][0][0], 0, shoulderPulse);

              if (step >= STEPS_PER_SECTION/2){ // вторая часть пути должна идти вниз,чтоб получился полуэллипс
                legspos[l][1] = legspos[l][1] - (angle_curr[l][1]/2)*(step - STEPS_PER_SECTION/2)/(STEPS_PER_SECTION/2); // BUG fix!!!!!!!!!! Вторая часть пути разбивается на равные доли,чтоб под конец имень нужную высоту  DZ
                moveToPoint(l ,legspos[l][0],legspos[l][1] ,legs[l][1][0],legs[l][2][0]);
              }
              // Сохраняем временные позиции
              legspos[l][3] = newY;
                                  
              delay(TIME_PER_STEP/(4*10));
            } else {
              float newY =legspos[l][3] + (left ? -1 : 1) * SECTION_LENGTH/3 ;
              float H = legspos[l][1];
              
              // Рассчитываем угол плеча
              float dy = abs(newY -legspos[l][3]);

              float alfa = acos((H*H+H*H-dy*dy)/(2*H*H));
              float beta = (180-alfa)/2;
              float dz = sin(90 - beta) * dy; // высота по Z

              angle_curr[l][0] = (left ? 1 : -1)*alfa + angle_curr[l][0];
              float angle_deg = degrees(angle_curr[l][0]) ; // отклонение влево/вправо       
              int shoulderPulse = realangleToPulse( angle_deg, legs[l][0][1], legs[l][0][2]);
              if (active_leg == 0 || active_leg == 1){  //компенсация высоты набранная за 1\4 и должна вернуться за 3\4
                legspos[l][1] = legspos[l][1] + angle_curr[1][1]/(STEPS_PER_SECTION * 3);
              }else{
                legspos[l][1] = legspos[l][1] + angle_curr[1][1]/(STEPS_PER_SECTION * 3);
              }
              moveToPoint(l ,legspos[l][0],legspos[l][1] ,legs[l][1][0],legs[l][2][0]);
              legspos[l][3] = newY;
              pwm1.setPWM(legs[l][0][0], 0, shoulderPulse);
              delay(TIME_PER_STEP/4);
            }
        }
      }
      active_leg = changeleg0312(active_leg);
    }
    for (int l = 0; l < 4; l++) {
     // legspos[l][1] = angle_curr[l][1];
      moveToPoint(l ,legspos[l][0],legspos[l][1] ,legs[l][1][0],legs[l][2][0]);
    }
    // Возвращаем Y в исходное положение
    for (int step = 0; step <= STEPS_PER_SECTION; step++) {
      for (int l = 0; l < 4; l++) {
        float dy = (realangleToPulse(angle_curr[l][0]*57.29577, legs[l][0][1], legs[l][0][2])  -  legs[l][0][3]) / (STEPS_PER_SECTION);
        dy = realangleToPulse(angle_curr[l][0]*57.29577, legs[l][0][1], legs[l][0][2]) - step*dy;                
        pwm1.setPWM(legs[l][0][0], 0, dy);
      }
      delay(20);
    }
}

void step2legs(bool forward, int legF, int legR, int homeX, int homeZ, int homeX1, int homeZ1, int STEP_DURATION) {
  const int NUM_STEPS = 25;
  const int delta = 9; // разница в тиках на сколько раньше начнет двигаться первая лапа
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
  float homeX_other1 = legspos[otherLegs[0]][0];
  float homeZ_other1 = legspos[otherLegs[0]][1];
  float homeX_other2 = legspos[otherLegs[1]][0];
  float homeZ_other2 = legspos[otherLegs[1]][1];

 
  float x, z, x1, z1, t, t1;
  for (int i = 0; i <= NUM_STEPS + delta; i++) {
    // Прогресс для линейного перемещения (0.0 до 1.0)
    float progress = (float)i / (NUM_STEPS + delta);
    
    // Оригинальная логика для ног legF и legR (криволинейное движение)
    if (i <= NUM_STEPS) {
      t = (float)i / NUM_STEPS;
      
      if (forward) {
        x = homeX + STEP_LENGTH * (1 - cos(PI * t)) / 2;
      } else {
        x = homeX - STEP_LENGTH * (1 - cos(PI * t)) / 2;
      }
      z = homeZ + STEP_HEIGHT * sin(PI * t) / 2; 
      moveToPoint(legF, x, z, legs[legF][1][0], legs[legF][2][0]);
      legspos[legF][0] = x;
      legspos[legF][1] = z;

      //Serial.println("main после step1: ");
   // Serial.print("X1 ");Serial.print(x);Serial.print(" Z1 ");Serial.println(z);

    }
    
    if (i >= delta) {
      t1 = (float)(i - delta) / NUM_STEPS;

      if (forward) {
        x1 = homeX1 + STEP_LENGTH * (1 - cos(PI * t1)) / 2;
      } else {
        x1 = homeX1 - STEP_LENGTH * (1 - cos(PI * t1)) / 2;
      }
      z1 = homeZ1 + STEP_HEIGHT * sin(PI * t1) / 2; 
      moveToPoint(legR, x1, z1, legs[legR][1][0], legs[legR][2][0]);
      legspos[legR][0] = x1;
      legspos[legR][1] = z1;


    // остальные НОГИ НАЗАД пошли с задней которая вперед и вверх
    // Вычисление линейного смещения для дополнительных ног
    float dx_other = (forward ? -STEP_LENGTH : STEP_LENGTH) * progress;
    
    // Позиции для дополнительных ног (линейное движение)
    float x_other1 = homeX_other1 + dx_other;
    float z_other1 = homeZ_other1;
    float x_other2 = homeX_other2 + dx_other;
    float z_other2 = homeZ_other2;

    // Перемещение дополнительных ног
    moveToPoint(otherLegs[0], x_other1, z_other1, legs[otherLegs[0]][1][0], legs[otherLegs[0]][2][0]);
    moveToPoint(otherLegs[1], x_other2, z_other2, legs[otherLegs[1]][1][0], legs[otherLegs[1]][2][0]);
    
    // Обновление позиций дополнительных ног
    legspos[otherLegs[0]][0] = x_other1;
    legspos[otherLegs[0]][1] = z_other1;
    legspos[otherLegs[1]][0] = x_other2;
    legspos[otherLegs[1]][1] = z_other2;
    }
    delay(STEP_DURATION / (2 * NUM_STEPS));
  }
}

void heightchange1legonstep(int leg,int step){
  //int curr_height = legspos[leg][2];
  int x = legspos[leg][0];
  int z = legspos[leg][1];
  if (step >= 0){ //ВВЕРХ
      moveToPoint(leg, x , z - step, legs[leg][1][0], legs[leg][2][0] );
      legspos[leg][1] = z - step;
  }
  else {//ВНИЗ
    moveToPoint(leg, x , z - step, legs[leg][1][0], legs[leg][2][0] );
    legspos[leg][1] = z - step;
  }
}

void heightchange4leg(int hight){
  for (int i = 0 ; i <  abs(hight); i++){
    int x,z;
    if (hight >= 0){ //ВВЕРХ
      for (int leg = 0 ; leg <  NUM_LEGS; leg++){
        z = legspos[leg][1];
        heightchange1legonstep(leg,1);
        legspos[leg][1] = z - 1;
        legspos[leg][2] = legspos[leg][2] - 1;
      }

    }else{
      for (int leg = 0 ; leg <  NUM_LEGS; leg++){
        z = legspos[leg][1];
        heightchange1legonstep(leg,-1);
        legspos[leg][1] = z + 1;
        legspos[leg][2] = legspos[leg][2] + 1;
      }
    }
  }
}

void heightchangeleftleg(int hight){
  for (int i = 0 ; i <  abs(hight); i++){
    if (hight >= 0){ //ВВЕРХ
      heightchange1legonstep(0,1);
      legspos[0][1] = legspos[0][1] - 1;
      legspos[0][2] = legspos[0][2] - 1;

      heightchange1legonstep(2,1);
      legspos[3][1] = legspos[3][1] -1;
      legspos[3][2] = legspos[3][2] - 1;
    }else{
      heightchange1legonstep(0,-1);
      legspos[0][1] = legspos[0][1] + 1;
      legspos[0][2] = legspos[0][2] + 1;

      heightchange1legonstep(2,-1);
      legspos[3][1] = legspos[3][1] + 1;
      legspos[3][2] = legspos[3][2] + 1;
    }
  }
}

void heightchangerightleg(int hight){
  for (int i = 0 ; i <  abs(hight); i++){
    if (hight >= 0){ //ВВЕРХ
      heightchange1legonstep(1,1);
      legspos[1][1] = legspos[1][1] - 1;
      legspos[1][2] = legspos[1][2] - 1;

      heightchange1legonstep(3,1);
      legspos[2][1] = legspos[2][1] - 1;
      legspos[2][2] = legspos[2][2] - 1;
    }else{
      heightchange1legonstep(1,-1);
      legspos[1][1] = legspos[1][1] + 1 ;
      legspos[1][2] = legspos[1][2] + 1;

      heightchange1legonstep(3,-1);
      legspos[2][1] = legspos[2][1] + 1;
      legspos[2][2] = legspos[2][2] + 1;
    }
  }
}

void heightchangeforwardleg(int hight){
  for (int i = 0 ; i <  abs(hight); i++){
    if (hight >= 0){ //ВВЕРХ
      heightchange1legonstep(0,1);
      legspos[0][1] = legspos[0][1] - 1;
      legspos[0][2] = legspos[0][2] - 1;

      heightchange1legonstep(1,1);
      legspos[1][1] = legspos[1][1] - 1;
      legspos[1][2] = legspos[1][2] - 1;
    }else{
      heightchange1legonstep(0,-1);
      legspos[0][1] = legspos[0][1] + 1;
      legspos[0][2] = legspos[0][2] + 1;

      heightchange1legonstep(1,-1);
      legspos[1][1] = legspos[1][1] + 1;
      legspos[1][2] = legspos[1][2] + 1;
    }
  }
}

void heightchangebackleg(int hight){
  for (int i = 0 ; i <  abs(hight); i++){
    if (hight >= 0){ //ВВЕРХ
      heightchange1legonstep(2,1);
      legspos[2][1] = legspos[2][1] - 1;
      legspos[2][2] = legspos[2][2] - 1 ;

      heightchange1legonstep(3,1);
      legspos[3][1] = legspos[3][1] - 1;
      legspos[3][2] = legspos[3][2] - 1;
    }else{
      heightchange1legonstep(2,-1);
      legspos[2][1] = legspos[2][1] + 1;
      legspos[2][2] = legspos[2][2] + 1;

      heightchange1legonstep(3,-1);
      legspos[3][1] = legspos[3][1] + 1;
      legspos[3][2] = legspos[3][2] + 1;
    }
  }
}

void gohome(){  // от любого положения до домашних позиций. TODO - проверять знаки!!!
  int count = 5; 
  double x,z;
  int delta[1][2];
  for (int i=0; i< NUM_LEGS; i++){
    forwardKinematics(i,legs[i][1][3], legs[i][2][3], x, z); // HOME x,y
    delta[i][0] = x - legspos[i][0];
    delta[i][1] = z - legspos[i][1];
    
    Serial.print("deltaZ ");
    Serial.println( delta[i][1]);
  }
  for (int c=0; c< count; c++){
    for (int i=0; i< NUM_LEGS; i++){
      float stepx =  delta[i][0] / count;
      float stepz =  delta[i][1] / count;
      moveToPoint(i,legspos[i][0]+stepx, legspos[i][1]+stepz,legs[i][1][0], legs[i][2][0] ) ;
      legspos[i][0] += stepx;
      legspos[i][1] += stepz;
      //legspos[i][2] += stepz; ??????????? а надо ли
    }
    delay(20);
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

   //Лидар 2 шт
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  //gefromjosticdata.onRun(readRemoteData);  // назначаем потоку задачу
  //gefromjosticdata.setInterval(1000);
  
 // pwm1.setPWM(0, 0, 350);

  //while(true){}

  //Go HOME
  for (int j = 0; j < NUM_JOINTS_PER_LEG; j++) {
     for (int leg = 0; leg < NUM_LEGS ; leg++) {
      pwm1.setPWM(legs[leg][j][0], 0, legs[leg][j][3]);
      delay(15);
    }
  }
  
  // Get current X;Y
  double x, z;
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    forwardKinematics(leg,legs[leg][1][3], legs[leg][2][3], x, z);
    legspos[leg][0] = x;
    legspos[leg][1] = z;
    legspos[leg][3] = 0;
    //Serial.print("X ");Serial.print(x);Serial.print(" Y ");Serial.println(legspos[leg][3]);Serial.print(" Z ");Serial.println(z);
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

/*pwm1.setPWM(1, 0, 130);
pwm1.setPWM(9, 0, 130);
pwm1.setPWM(13, 0, 600);
pwm1.setPWM(5, 0, 600);

pwm1.setPWM(2, 0, 130);
pwm1.setPWM(10, 0, 130);
pwm1.setPWM(14, 0, 600);
pwm1.setPWM(6, 0, 600);
*/
/*pwm1.setPWM(0, 0, 130);
pwm1.setPWM(8, 0, 130);
pwm1.setPWM(12, 0, 600);
pwm1.setPWM(4, 0, 600);*/
//delay(2000);
//pwm1.setPWM(5, 0, 130);
//delay(2000);
//forwardKinematics(1,legs[1][1][3], legs[1][2][3], x, z);
//Serial.print("X ");Serial.print(x);Serial.print(" Z ");Serial.println(z);

  //move1Leg(false,5000,60,40 ,5,6 , legspos[1][0], legspos[1][1],1);
  //delay(1500);
 // moveToPoint(1,-120, -170, 5,6);
 
 
  delay(1000);
  //step1legtoside(true,0,160);



  //moveToPoint(1,-20, -170, 5,6);
  //move1Leg(false,5000,50,40 ,5,6 , legspos[1][0], legspos[1][1],1);
  //move1Leg(false,5000,50,40 ,5,6 , legspos[1][0], legspos[1][1],1);
  //delay(1500);
  //move1Leg(false,5000,80,40 ,9,10 , legspos[2][0], legspos[2][1],2);
  //delay(1500);
  //move1Leg(false,5000,50,30 ,13,14 , legspos[3][0], legspos[3][1],3);
//stepalllegs(false,5000);
 // heightchangebackleg(20);
 // heightchange1legonstep(1,-40);
 // delay(1000);
  //heightchangebackleg(-40);
 // heightchange1legonstep(1,40);
 // while(true){}
}

void loop() {

  //for (int i=0;i< 10;i++){
  step4legstoside(true, 1000); 
  //}
  gohome();
  delay(20);

  /*float dist1 = getDistance(TRIG1, ECHO1);
  float dist2 = getDistance(TRIG2, ECHO2);
  Serial.print("Left: ");
  Serial.print(dist1);
  Serial.print(" cm | Right: ");
  Serial.print(dist2);
  Serial.println(" cm");
  delay(200);
*/
  /*for (int i=0;i< 10;i++){
  delay(7);
  step4legs(false,700); // технология 1х3
  }
  for (int i=0;i< 10;i++){
  delay(7);
  step4legs(false,400); // технология 1х3
  }
  for (int i=0;i< 10;i++){
  delay(7);
  step4legs(false,200); // технология 1х3
  }*/
  //stepalllegs(false,1000,0); // технология 2х2
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
