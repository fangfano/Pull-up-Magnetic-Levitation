/*
  磁悬浮 PID 控制程序 - 带串口通信功能、动态 dt 计算和电流监测
  The basic version of the pull-up magnet levitation with Serial Communication, dynamic dt, and current monitoring.
*/

// 引脚定义
const int readIPin = A0;      // 电流传感器连接引脚 A0
const int readPin = A1;
const int IN1Pin = 4;
const int IN2Pin = 5;
const int EA1Pin = 6;
bool flag = true;

// 传感器和状态变量
int readValue = 0;
float filtValue = 0.0;
// 【修改】声明为volatile，因为它在中断中被修改，在主循环中被读取
volatile float currentValue = 0.0; 

// PID 控制核心变量
bool pid_running = false;
char printMode = 'D';         // 打印模式控制变量, 'D'为默认PID数据, 'P'为电流

float targetValue = 250.0; //  需要调参
float error = 0.0;
float prevError = 0.0;
float integral = 0.0;
float derivative = 0.0;

// PID 参数     需要调参
float kp = 1.2;
float ki = 3.8;
float kd = 0.018;

float power = 0.0;
float pid_output = 0.0;

// 时间变量
float dt = 0.0;
unsigned long lastTime = 0;
const float integralLimit = 20.0;


// Timer1 中断服务程序 (ISR)
// 此函数在后台自动运行，用于定时测量电流
ISR(TIMER1_COMPA_vect) {
  int rawCurrentValue = analogRead(readIPin);
  // 将模拟读数(0-1023)转换为电压(0-5V)
  float currentVoltage = rawCurrentValue * (5.0 / 1023.0);
  // 根据100mV/A (即0.1V/A)的关系换算成电流
  // 这个计算结果会安全地更新到全局变量 currentValue
  currentValue = currentVoltage;
}


void setup() {
  Serial.begin(9600);
  pinMode(readIPin, INPUT);
  pinMode(readPin, INPUT);
  pinMode(IN1Pin, OUTPUT);
  pinMode(IN2Pin, OUTPUT);
  
  // 配置Timer1以创建定时中断
  // 注意: 此配置会与使用Timer1的库（如Servo库）冲突
  cli(); // 关闭全局中断

  // 1. 设置Timer1控制寄存器
  TCCR1A = 0; // TCCR1A和TCCR1B全部置零
  TCCR1B = 0;
  
  // 2. 设置比较匹配寄存器 OCR1A (决定中断频率)
  // 目标频率 = 50 Hz (每20ms中断一次)
  // 计算公式: OCR1A = (时钟频率 / (预分频 * 目标频率)) - 1
  // (16,000,000 / (256 * 50)) - 1 = 1250 - 1 = 1249
  OCR1A = 1249;

  // 3. 开启CTC模式并设置预分频
  // WGM12 = 1: 开启CTC模式 (Clear Timer on Compare Match)
  // CS12 = 1: 设置预分频为256
  TCCR1B |= (1 << WGM12) | (1 << CS12); 
  
  // 4. 开启Timer1的A比较匹配中断
  TIMSK1 |= (1 << OCIE1A);

  sei(); // 开启全局中断

  Serial.println("Arduino is ready. Send 'start' to begin PID control.");
  Serial.println("Send 'D' to monitor PID values, 'P' to monitor current.");
}

void loop() {
  handleSerialCommand();
  if (pid_running) {
    // 动态计算 dt
    unsigned long currentTime = micros();
    dt = (currentTime - lastTime) / 1000000.0; 
    lastTime = currentTime; 

    // 位置传感器读数与滤波
    readValue = 0;
    for (int i = 0; i < 20; i++){
      readValue += analogRead(readPin)-500;
    }
    filtValue = readValue / 20.0; 

    if(filtValue <= 60){
      digitalWrite(EA1Pin, 0);
      return;
    }
    
    // PID计算
    error = filtValue - targetValue ;
    integral += error * dt;
    if (integral > integralLimit) {
      integral = integralLimit;
    } else if (integral < -integralLimit) {
      integral = -integralLimit;
    }

    if (dt > 0) {
      derivative = (error - prevError) / dt;
    } else {
      derivative = 0;
    }
    
    power = (-kp * error) + (-ki * integral) + (-kd * derivative);
    prevError = error;

    // 功率输出限制
    if (power > 255){
      power = 255;
    } else if (power < -255){
      power = -255;
    }
    
    if (power <= 0){
      flag = true;
      digitalWrite(IN1Pin, flag);  
      digitalWrite(IN2Pin, !flag);
      pid_output = -power;
      // analogWrite(EA1Pin, pid_output); 
    } else {
      flag = false;
      digitalWrite(IN1Pin, flag);  
      digitalWrite(IN2Pin, !flag);
      pid_output = power;
      analogWrite(EA1Pin, pid_output);
    }

    // 根据打印模式输出不同的信息
    if (printMode == 'D') {
        Serial.print(targetValue);
        Serial.print(",");
        Serial.print(filtValue);
        Serial.print(",");
        Serial.println(power);
    } else if (printMode == 'P') {
        // 直接打印由中断更新的currentValue
        Serial.print(targetValue);
        Serial.print(",");
        Serial.print(filtValue);
        Serial.print(",");
        Serial.println(currentValue); 
    }
  }
} 

void handleSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equals("start")) {
      pid_running = true;
      prevError = 0;
      integral = 0;
      lastTime = micros(); 
      digitalWrite(IN1Pin, flag);  
      digitalWrite(IN2Pin, !flag);
      digitalWrite(EA1Pin, 0);
      Serial.println("PID control started.");
      Serial.println(kp, 4);
      Serial.print(",");
      Serial.println(ki, 4);
      Serial.print(",");
      Serial.println(kd, 4);
      Serial.println("Format: target,filtValue,pid_output");
      
    } else if (command.equals("stop")) {
      pid_running = false;
      digitalWrite(IN1Pin, flag);  
      digitalWrite(IN2Pin, !flag);
      digitalWrite(EA1Pin, 0);
      Serial.println("PID control stopped.");
      Serial.println(kp, 4);
      Serial.print(",");
      Serial.println(ki, 4);
      Serial.print(",");
      Serial.println(kd, 4);

    } else if (command.startsWith("t")) {
      targetValue = command.substring(1).toFloat();
      Serial.print("Target value set to: ");
      Serial.println(targetValue);
    } else if (command.startsWith("p")) {
      kp = command.substring(1).toFloat();
      Serial.print("Kp set to: ");
      Serial.println(kp, 4);
      
    } else if (command.startsWith("i")) {
      ki = command.substring(1).toFloat();
      integral = 0;
      Serial.print("Ki set to: ");
      Serial.println(ki, 4);
      
    } else if (command.startsWith("d")) {
      kd = command.substring(1).toFloat();
      Serial.print("Kd set to: ");
      Serial.println(kd, 4);
    } else if (command.equalsIgnoreCase("D")) { 
      printMode = 'D';
      Serial.println("Printing mode set to: PID Data (target,filtValue,power)");
    } else if (command.equalsIgnoreCase("P")) {
      printMode = 'P';
      Serial.println("Printing mode set to: Current (A)");
    }
  }
}