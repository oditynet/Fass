
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();

int spd = 500;

int cal_L1 = 0; //1 - calibre
int cal_L2 = 0; //1 - calibre

int L1=108;
int L2=130;

double homeX, homeZ;

//RF
uint8_t servonum1 = 0; //колено
#define SERVOMIN1  130
#define SERVOMAX1  600
#define SERVOHOME1  365//350//125//250  365-90

uint8_t servonum2 = 1; //бедро
#define SERVOMIN2  130
#define SERVOMAX2  600
#define SERVOHOME2  330//249//330//499//370  249-0 гр

double _acos(double x) {
  // Проверка границ входа
  if (x <= -1.0f) return PI;
  if (x >= 1.0f) return 0.0f;
  
  // Аппроксимация рядом Тейлора (оптимизированная)
  double negate = (x < 0) ? 1.0f : 0.0f;
  x = abs(x);
  
  double ret = -0.0187293f;
  ret = ret * x;
  ret = ret + 0.0742610f;
  ret = ret * x;
  ret = ret - 0.2121144f;
  ret = ret * x;
  ret = ret + 1.5707288f;
  ret = ret * sqrt(1.0f - x);
  ret = ret - 2 * negate * ret;
  return negate * PI + ret;
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

void forwardKinematics(int hipPulse, int kneePulse, double &x, double &z) {
  double b = realpulseToAngle(kneePulse, SERVOMIN1, SERVOMAX1);
  double a = realpulseToAngle(hipPulse, SERVOMIN2, SERVOMAX2);

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

  /*Serial.print("x1="); Serial.print(x1);
  Serial.print(" z1="); Serial.print(z1);
  Serial.print(" x=");  Serial.print(x); Serial.print(" z="); Serial.println(z);*/
}

bool inverseKinematics(double X, double Z, int &hipPulse, int &kneePulse) {
  double D = sqrt(X*X+Z*Z);
  //Serial.print("D=");Serial.println(D);
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

  kneePulse = realangleToPulse(rad_b, SERVOMIN1, SERVOMAX1);
  hipPulse = realangleToPulse(rad_a, SERVOMIN2, SERVOMAX2);
  return true;
}

void moveLeg(bool forward,int STEP_DURATION,double STEP_LENGTH,double STEP_HEIGHT) {
  
  /*const double STEP_LENGTH = 100.0;  // Длина шага (мм)
  const double STEP_HEIGHT = 60.0;  // Высота подъема (мм)
  const int STEP_DURATION = 1000;  // Длительность шага (мс)*/
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
    moveToPoint(x,z);

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

bool moveToPoint(double targetX, double targetZ) {
  int hipPulse, kneePulse;
  
  if (inverseKinematics(targetX, targetZ, hipPulse, kneePulse)) {
    // Управление сервоприводами
    //Serial.print("PulseX=");Serial.print(hipPulse);Serial.print(" PulseZ=");Serial.println(kneePulse);
    pwm1.setPWM(servonum2, 0, hipPulse);
    pwm1.setPWM(servonum1, 0, kneePulse);
    return true;
  } else {
    Serial.println("Ошибка перемещения");
  }
  return false;
}


void setup() {
  Serial.begin(115200);
  Serial.println("Start...");

  pwm1.begin();
  pwm1.setOscillatorFrequency(25000000);
  pwm1.setPWMFreq(60);

  if (cal_L1) {
    //pwm1.setPWM(servonum1, 0, SERVOHOME1);
    pwm1.setPWM(servonum1, 0, 130);
    delay(3000);
    pwm1.setPWM(servonum1, 0, 600);
    delay(3000);
  }else{
    pwm1.setPWM(servonum1, 0, SERVOHOME1);
  }
  
  if (cal_L2) {
    //pwm1.setPWM(servonum2, 0, SERVOHOME2);
    pwm1.setPWM(servonum2, 0, 130);
    delay(3000);
    pwm1.setPWM(servonum2, 0, 600);
    delay(3000);
  }else{
    pwm1.setPWM(servonum2, 0, SERVOHOME2);
  }

  
  double x, z;
  forwardKinematics(SERVOHOME2, SERVOHOME1, x, z);
  delay(500);
  //moveToPoint(x,z);
  homeX=x;homeZ=z;
  //while(true){}
}


void loop() {
  moveLeg(false,1000,100,80); // false - вперед
}