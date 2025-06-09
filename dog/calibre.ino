
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();

int spd = 500;

int cal_L1 = 1; //1 = home  2 = run 
int cal_L2 = 1; //1 = home  2 = run 

int L1=108;
int L2=130;
int C,H;

float homeX, homeZ;

//RF
uint8_t servonum1 = 0; //колено
#define SERVOMIN1  130
#define SERVOMAX1  600
#define SERVOHOME1  365//350//125//250  365-90

uint8_t servonum2 = 1; //бедро
#define SERVOMIN2  130
#define SERVOMAX2  600
#define SERVOHOME2  330//249//330//499//370  249-0 гр




float realangleToPulse(float angle, int minPulse, int maxPulse, float minAngle = 0, float maxAngle = 180) {
    if (minPulse > maxPulse) {
        return maxPulse + (minPulse - maxPulse) * (1 - angle/180.0);
    }
    return minPulse + (maxPulse - minPulse) * (angle/180.0);
}

// Преобразование импульса в угол с учетом реального диапазона
float realpulseToAngle(int pulse, int minPulse, int maxPulse, float minAngle = 0, float maxAngle = 180) { 
    if (minPulse > maxPulse) {
        return 180.0 - 180.0 * (pulse - maxPulse) / (minPulse - maxPulse);
    }
    return 180.0 * (pulse - minPulse) / (maxPulse - minPulse);
}

void forwardKinematics(int hipPulse, int kneePulse, float &x, float &z) {
  float kneeAngle = realpulseToAngle(kneePulse, SERVOMIN1, SERVOMAX1);
  float hipAngle = realpulseToAngle(hipPulse, SERVOMIN2, SERVOMAX2); 
  if (hipAngle > 45.57){
    hipAngle = hipAngle -45.57; // чтоб локоть не поднимался выше Z>0 45.57 это импульсы от 130 до 249 !!! в inverseKinematics +119=249-130 еще сделать
    Serial.println("-45.57");
    if (hipAngle > 0){ // жуе работает дл 330
      hipAngle=-1*hipAngle;
    }
  }
  
  Serial.print("kneePulse=");Serial.print(kneePulse);Serial.print(" hipPulse=");Serial.println(hipPulse);

  Serial.print("Угол колена: "); Serial.print(kneeAngle);  Serial.print("° | Угол бедра: "); Serial.print(hipAngle); Serial.print("° ");

  // Переводим углы в радианы для тригонометрии
  float kneeRad = radians(kneeAngle);
  float hipRad = radians(hipAngle);

  //Serial.print("Ra1: "); Serial.print(kneeRad);  Serial.print(" Ra2: "); Serial.println(hipRad);
  
  // Корректный расчет координат
  float x1 = L1 * cos(hipRad);
  float z1 = L1 * sin(hipRad);
  
  x = x1 + L2 * cos( (hipRad - kneeRad));
  z = z1 + L2 * sin( (hipRad - kneeRad));
  
  Serial.print("x1="); Serial.print(x1);
  Serial.print(" z1="); Serial.print(z1);
  Serial.print(" x=");  Serial.print(x); Serial.print(" z="); Serial.println(z);
}

bool inverseKinematics(float targetX, float targetZ, int &hipPulse, int &kneePulse) {
    // Рассчитываем расстояние до цели
    float distance = sqrt(targetX*targetX + targetZ*targetZ);
    
    // Проверка достижимости точки
    if (distance > L1 + L2 || distance < abs(L1 - L2)) {
        Serial.println("Точка недостижима");
        return false;
    }
    float Q11 = acos( targetX/distance ) ;
    float Q12 = acos( (L1*L1 - L2*L2 + distance*distance) / (2*distance*L1) );
    float Q1 = Q11-Q12;
    float Q2 = acos( (L1*L1 + L2*L2 - distance*distance) / (2*L1*L2)  );
    
    
    
    Serial.println("___");
  
    //Serial.println(degrees(Q11));
    //Serial.println(degrees(Q12));
    /*Serial.println(degrees(Q1));
    Serial.println(degrees(Q2));
    Serial.println(realangleToPulse(degrees(Q1),SERVOMIN1, SERVOMAX1)+119); // 130-249 Это чтоб Z>0  не поднимал локоть!!!!!!!
    Serial.println(realangleToPulse(degrees(Q2),SERVOMIN2, SERVOMAX2));*/
    float kneeAngle = degrees(Q2);
    float hipAngle = degrees(Q1);
    // Преобразование углов в импульсы
    kneePulse = realangleToPulse(kneeAngle, SERVOMIN1, SERVOMAX1);
    hipPulse = realangleToPulse(hipAngle, SERVOMIN2, SERVOMAX2)+119;// 130-249 Это чтоб Z>0  не поднимал локоть!!!!!!!
    
    return true;
}

void moveLeg(bool forward) {
  
  const float STEP_LENGTH = 100.0;  // Длина шага (мм)
  const float STEP_HEIGHT = 80.0;  // Высота подъема (мм)
  const int STEP_DURATION = 2000;  // Длительность шага (мс)
  const int NUM_STEPS = 30;        // Количество промежуточных точек
  float x,z;
  for (int i = 0; i <= NUM_STEPS; i++) {
    float t = (float)i / NUM_STEPS;
    
    // Фаза переноса (ПОДЪЕМ ноги)
    if (forward) {
      x = homeX + STEP_LENGTH * (1 - cos(PI * t)) / 2;
    } else {
      x = homeX - STEP_LENGTH * (1 - cos(PI * t)) / 2;
    }
    z = homeZ + STEP_HEIGHT * sin(PI * t)/2; // Уменьшаем Z для подъема!
    
    // Отладочный вывод
    Serial.print("Target: X="); Serial.print(x);
    Serial.print(" Z="); Serial.println(z);
    moveToPoint(x,z);

    delay(STEP_DURATION / (2 * NUM_STEPS));
  }
}

bool moveToPoint(float targetX, float targetZ) {
  int hipPulse, kneePulse;
  
  if (inverseKinematics(targetX, targetZ, hipPulse, kneePulse)) {
    // Управление сервоприводами
    Serial.print("PulseX=");Serial.print(hipPulse);Serial.print(" PulseZ=");Serial.println(kneePulse);
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
    pwm1.setPWM(servonum1, 0, SERVOHOME1);
  }
  if (cal_L2) {
    pwm1.setPWM(servonum2, 0, SERVOHOME2);
  }
  delay(1000);
  float x, z;
  forwardKinematics(SERVOHOME2, SERVOHOME1, x, z);
  //moveToPoint(x,z);
  //moveToPoint(25,-102.69);
  //forwardKinematics(323, 417, x, z);
  //while(true){}
  
  //forwardKinematics(SERVOHOME2, SERVOHOME1, x, z);
 // float distance = sqrt(x*x + z*z);

  homeX=x;homeZ=z;
  //moveLeg(true);
  
  /*moveToPoint(-25,-167);
  delay(500);
  moveToPoint(25,-167);
*/
  //moveToPoint(-104,-102.69);
  //delay(500);
  //forwardKinematics(SERVOHOME2, SERVOHOME1, x, z);
  //delay(1000);
   //x=x+20;
  //for (int i=0;i<100; i+=5) {
    //forwardKinematics(SERVOHOME2, SERVOHOME1, x, z);
    //Serial.print(x+i);Serial.print(":");Serial.println(z);
   // delay(500);
  //}
   // delay(600);

  //while(true){}
}


void loop() {
  moveLeg(false);
}