extern "C" void esp_bootloader_disable_logs(void);
#include <AccelStepper.h>
#include "esp_log.h"

// Пины для CNC Shield (X и Y оси)
#define STEP_PIN_A 18  // STEP для двигателя A (X на CNC Shield)
#define DIR_PIN_A 19   // DIR для двигателя A (X на CNC Shield)
#define STEP_PIN_Z 33  // STEP для двигателя B (Y на CNC Shield)
#define DIR_PIN_Z 27   // DIR для двигателя B (Y на CNC Shield)
#define STEP_PIN_B 17  // STEP для двигателя Z (Z на CNC Shield)
#define DIR_PIN_B 14   // DIR для двигателя Z (Z на CNC Shield)
//#define ENABLE_PIN 8  // Пин ENABLE для драйверов
#define VACUUM_PUMP_PIN 25  //  Пин вкл/выкл для насоса вакуумного захвата
#define VACUUM_VALVE_PIN 21 //  Пин открыто/закрыто для вакуумного клапана
// Пины для концевиков
#define X_LIMIT_PIN 34  // Пин для концевика X-min
#define Y_LIMIT_PIN 35 // Пин для концевика Y-min
#define Z_LIMIT_PIN 4 // Пин для концевика Z-min
#define frequency 5   // Задержка между шагами двигателя вертикальной оси Z

// Параметры механики
const float STEPS_PER_MM = 5.0;  // 200 шагов / 40 мм = 5 шагов/мм (для шкива с 20 зубьями)
const float MAX_SPEED = 300.0;  // Максимальная скорость (шагов/сек)
const float ACCELERATION = 120.0;  // Ускорение (шагов/сек^2)
const float HOMING_SPEED = 70.0;  // Скорость для хоуминга (шагов/сек, медленнее для точности)

// Положение вакуумного захвата закрыто/открыто
bool CATCH_FLAG = 0;  //  0 - открыто, 1 - закрыто

//  Высота подъема груза в шагах мотора
int Z_HEIGHT = 400;  //830

// Инициализация двигателей
AccelStepper motorA(1, STEP_PIN_A, DIR_PIN_A); // Драйвер для двигателя A
AccelStepper motorB(1, STEP_PIN_B, DIR_PIN_B); // Драйвер для двигателя B

// Текущая позиция (в мм)
float currentX = 0.0;
float currentY = 0.0;

// Перемещение в точку X, Y
void moveToXY(float targetX, float targetY) {
  targetX = constrain(targetX, 0, 550);
  targetY = constrain(targetY, 0, 500);

  // Правильная CoreXY кинематика (ты был прав!)
  long targetA = round(( targetX + targetY) * STEPS_PER_MM);
  long targetB = round(( targetX - targetY) * STEPS_PER_MM);

  motorA.moveTo(targetA);
  motorB.moveTo(targetB);

  // Ждём полного завершения
  while (motorA.isRunning() || motorB.isRunning()) {
    motorA.run();
    motorB.run();
  }

  currentX = targetX;
  currentY = targetY;

  Serial.print("Position: X="); Serial.print(currentX);
  Serial.print(" Y="); Serial.println(currentY);
}

// Функция home
void homeXY() {
  // Устанавливаем скорость для хоуминга
  motorA.setMaxSpeed(HOMING_SPEED);
  motorB.setMaxSpeed(HOMING_SPEED);

  // 1. Хоуминг по оси X (двигаем каретку влево, пока не сработает X-min)
  // Для CoreXY: движение влево (X уменьшается) → ΔX < 0 → A и B двигаются в противоположных направлениях
  while (digitalRead(X_LIMIT_PIN) == LOW) { //  NC-концевик (HIGH при нажатии)
    motorA.move(-1000); // Двигаем A в отрицательном направлении
    motorB.move(1000);  // Двигаем B в положительном направлении
    motorA.run();
    motorB.run();
  }

  // Останавливаем двигатели
  motorA.stop();
  motorB.stop();
  while (motorA.distanceToGo() != 0 || motorB.distanceToGo() != 0) {
    motorA.run();
    motorB.run();
  }

  // 2. Хоуминг по оси Y (двигаем каретку к Y=0, пока не сработает Y-min)
  // Для CoreXY: движение к Y=0 (Y уменьшается) → ΔY < 0 → A и B двигаются в одном направлении, но в отрицательном
  while (digitalRead(Y_LIMIT_PIN) == LOW) {
    motorA.move(-1000); // Оба двигателя в отрицательном направлении
    motorB.move(-1000);
    motorA.run();
    motorB.run();
  }

  motorA.stop();
  motorB.stop();
  while (motorA.distanceToGo() != 0 || motorB.distanceToGo() != 0) {
    motorA.run();
    motorB.run();
  }

  // Сбрасываем позицию
  currentX = 0.0;
  currentY = 0.0;
  motorA.setCurrentPosition(0);
  motorB.setCurrentPosition(0);

  // Возвращаем нормальную скорость
  motorA.setMaxSpeed(MAX_SPEED);
  motorB.setMaxSpeed(MAX_SPEED);


  while(digitalRead(Z_LIMIT_PIN) == LOW){
      digitalWrite(DIR_PIN_Z, HIGH);
      digitalWrite(STEP_PIN_Z, HIGH);
      delay(frequency + 5);
      digitalWrite(STEP_PIN_Z, LOW);
      
    }

  Serial.println("Homing complete");
}

// Обработка G-кода
void processGCode(String command) {
  if (command.startsWith("G0") || command.startsWith("G1")) {
    float targetX = currentX;                 
    float targetY = currentY;

    int xIndex = command.indexOf('X');
    int yIndex = command.indexOf('Y');

    if (xIndex != -1) {
      targetX = command.substring(xIndex + 1, command.indexOf(' ', xIndex)).toFloat();
    }
    if (yIndex != -1) {
      targetY = command.substring(yIndex + 1).toFloat();
    }

    moveToXY(targetX, targetY);
    object_catch();
    delay(500);
    Serial.println("OK");
  } else if (command.startsWith("G28") ){
    homeXY();  //  Хоуминг
    moveToXY(7,7);  //  Откат на 7 мм для исключения давления на концевики
    Serial.println("OK");
    }
}


// Захват объекта
void object_catch(){
      // Опускание захвата
      digitalWrite(DIR_PIN_Z, LOW);
      for (int i = 0; i < Z_HEIGHT; i++){
      digitalWrite(STEP_PIN_Z, HIGH);
      delay(frequency);
      digitalWrite(STEP_PIN_Z, LOW);
      }
      delay(1000);

     // Захват объекта
      if (CATCH_FLAG == 0){
        digitalWrite(VACUUM_VALVE_PIN, HIGH);
        digitalWrite(VACUUM_PUMP_PIN, HIGH);
        delay(2000);
        CATCH_FLAG = 1;
        digitalWrite(VACUUM_PUMP_PIN, LOW);
      }
      else{
        digitalWrite(VACUUM_VALVE_PIN, LOW);
        digitalWrite(VACUUM_PUMP_PIN, LOW);
        delay(1000);
        CATCH_FLAG = 0;
        }

      // Подъем объекта
      digitalWrite(DIR_PIN_Z, HIGH);
      delay(2000);
      for (int i = 0; i < Z_HEIGHT; i++){
      digitalWrite(STEP_PIN_Z, HIGH);
      delay(frequency);
      digitalWrite(STEP_PIN_Z, LOW);
      }
  }

void setup() {
  Serial.begin(74880);
  delay(50);
  Serial.flush();
  // Настройка пина ENABLE
//  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DIR_PIN_Z, OUTPUT);
  pinMode(STEP_PIN_Z, OUTPUT);
  pinMode(VACUUM_VALVE_PIN, OUTPUT);
  pinMode(VACUUM_PUMP_PIN, OUTPUT);
  pinMode(X_LIMIT_PIN, INPUT_PULLDOWN);
  pinMode(Y_LIMIT_PIN, INPUT_PULLDOWN);  
//  pinMode(Z_LIMIT_PIN, INPUT_PULLDOWN);  
//  digitalWrite(ENABLE_PIN, LOW); // Включаем драйверы (LOW = включено)
  digitalWrite(VACUUM_VALVE_PIN, LOW);
  digitalWrite(VACUUM_PUMP_PIN, LOW);
  // Инициализация Serial
  Serial.begin(115200);
//  while (!Serial) {
//    ;
//  }

  delay(15000);
  esp_log_level_set("*", ESP_LOG_NONE);  // Отключает все app-логи (ESP_LOGx)
  delay(50);
  // Настройка двигателей
  motorA.setMaxSpeed(MAX_SPEED);
  motorA.setAcceleration(ACCELERATION);
  motorB.setMaxSpeed(MAX_SPEED);
  motorB.setAcceleration(ACCELERATION);
  Serial.println();
  Serial.println("CoreXY System Ready");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      processGCode(command);
    }
  }
}
