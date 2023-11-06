#include <LWiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <Servo.h>

#define SERVO_DEFAULT_VALUE 1500
#define SERVO_DEFAULT_MAX_VALUE 2000
#define SERVO_DEFAULT_MIN_VALUE 1000
#define PACKET_LENGTH 20
#define isDebug true

#define LOCAL_PORT 6188 //控制馬達的Port號

#define HEARTBEAT_TIME 1000 //回傳Heartbeat間隔時間（1秒）

const char *ssid = "TinyPlan97_4Motor";
const char *password = "12345678";

WiFiServer server(LOCAL_PORT);
WiFiUDP Udp;

int status = WL_IDLE_STATUS;

char packetBuffer[255];

int count = 0;               //計數用

int bytes[PACKET_LENGTH];

int dcMotorPinA[] = {11, 12};     // DC motor A
int dcMotorPinB[] = {16, 17};     // DC motor B
int dcMotorPinC[] = {4, 7};       // DC motor C
int dcMotorPinD[] = {15, 3};       // DC motor D

int receiveServoValue[4];
double levelOfTurn = 1; //  左右面向的靈敏度

long startProcessTime = 0;
long endProcessTime = 0;

// 測試Servo相關數據
boolean ifTestMode = false;   // 是否進入測試模式
boolean ifAddValue = false;   // 是否增加資料
int servoMoveStepValue = 80;  // 測試用的速度

#define LOST_SIGNAL_MAX_TIME 500 // 最大失去信號時間;

int currentLostSignalTime = 0;

int accelPWMValue = SERVO_DEFAULT_VALUE;   // 需要控制油門的的PWM;
int accelPWMChannel = 1;      // Channel0, 1, 2, 3, 4, 5, 6....
bool isControlAccelerator = false;          // 是否需要限制油門;

unsigned long startTime;

String motorSetting = "SettingA";  //default(原始轉速) & SettingA(調整後轉速)

void setup() {

  // put your setup code here, to run once:

  pinMode(dcMotorPinA[0], OUTPUT);
  pinMode(dcMotorPinA[1], OUTPUT);
  pinMode(dcMotorPinB[0], OUTPUT);
  pinMode(dcMotorPinB[1], OUTPUT);
  pinMode(dcMotorPinC[0], OUTPUT);
  pinMode(dcMotorPinC[1], OUTPUT);
  pinMode(dcMotorPinD[0], OUTPUT);
  pinMode(dcMotorPinD[1], OUTPUT);


  processDCMotor(SERVO_DEFAULT_VALUE, dcMotorPinA);
  processDCMotor(SERVO_DEFAULT_VALUE, dcMotorPinB);
  processDCMotor(SERVO_DEFAULT_VALUE, dcMotorPinC);
  processDCMotor(SERVO_DEFAULT_VALUE, dcMotorPinD);

  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {

  }
  if (isDebug) {
    Serial.println(F("================================"));
    Serial.println(F("======= Serial Ready !!! ======="));
    Serial.println(F("================================"));
    Serial.println();
    Serial.println(F("AP Setting..."));
  }

  WiFi.softAP(ssid,password);
  IPAddress myIP = WiFi.softAPIP();

  if (isDebug) {
    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F("========= AP Ready !!! ========="));
    Serial.println(F("================================"));
    Serial.println();

    Serial.print("Connect to AP [ ");
    Serial.print(ssid);
    Serial.print(" ] and visit [ http://");
    Serial.print(myIP);
    Serial.println(F(" ] "));

    Serial.print("AP MAC = ");
    Serial.println(WiFi.softAPmacAddress());
    Serial.println();
  }

  server.begin();

  Udp.begin(LOCAL_PORT);

  startTime = millis(); //計時

}

int dataIndex = 0;
byte dataBytes[PACKET_LENGTH];
String thisPacket;

bool isDeviceCheck = false;

unsigned long loopCheck;

IPAddress replyPlace; //對方裝置的位置

void loop() {

  int packetSize = Udp.parsePacket();

  if (packetSize) { //LOCAL_PORT有收到資料

    if (isDebug) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      replyPlace = Udp.remoteIP();
      Serial.print(replyPlace);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());
    }

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }

    if (isDebug) {
      Serial.println("Contents:");
      Serial.println(packetBuffer);
    }

    String receiveCommand = String(packetBuffer);

    processRCString(receiveCommand);

    isDeviceCheck = true; //開始傳送Heartbeat
  }

  loopCheck = millis();

  if(loopCheck - startTime > HEARTBEAT_TIME){ //計數1秒
    
    startTime = loopCheck;
    
    if(isDeviceCheck){  //已收到連接裝置的資料

      //傳送UDP資料給該連接裝置
      Udp.beginPacket(replyPlace, LOCAL_PORT);
      Udp.write("OK");
      Udp.endPacket();
      
      if(isDebug){
        
        Serial.print(F("sending [OK] to : "));
        Serial.print(replyPlace);
        Serial.print(F(":"));
        Serial.println(LOCAL_PORT);
        
      }
      
    }else{
      
      if(isDebug){
        Serial.println(F("****** Nobody connected !!! ******"));
      }
      
    }

  }

}

void processRCString(String command) {

  int dcPwmValue[] = {1500, 1500, 1500, 1500};

  int commandLength = command.length();

  if (commandLength > 15) {

    //    Serial.println(command);

    if (command.charAt(commandLength - 1) != '#') {  // 表示結尾不是預設的結果;

      return;

    }

  } else {

    return;
  }

  if (command.indexOf("SRT") > -1 ) {  // 表示伺服馬達操作;

    // Serial.println("接收到伺服馬達命令;");

    int i = 3;
    int servoIndex = 0;

    while (i < commandLength - 1) {     // 解碼;

      if (i + 3 < commandLength) {

        if (servoIndex <= 3) {

          String singleCommand = command.substring(i, i + 4);
          receiveServoValue[servoIndex] = singleCommand.toInt();

          //          Serial.print(F("receiveServoValue["));
          //          Serial.print(servoIndex);
          //          Serial.print(F("] = "));
          //          Serial.println(singleCommand.toInt());

          servoIndex ++;

        }

      }

      i = i + 4;

    }

    int directYChangeValue = 0;
    int directXChangeValue = 0;
    int turnValue = 0;

    directYChangeValue = receiveServoValue[1] - 1500; // 前進, 後退
    directXChangeValue = receiveServoValue[0] - 1500; // 左移, 右移 
    turnValue = receiveServoValue[3] - 1500;  // 右轉，左轉

    dcPwmValue[0] -= directYChangeValue;
    dcPwmValue[1] -= directYChangeValue;
    dcPwmValue[2] -= directYChangeValue;
    dcPwmValue[3] -= directYChangeValue;

    dcPwmValue[0] -= directXChangeValue;
    dcPwmValue[1] += directXChangeValue;
    dcPwmValue[2] += directXChangeValue;
    dcPwmValue[3] -= directXChangeValue;

    dcPwmValue[0] += turnValue;
    dcPwmValue[1] -= turnValue;
    dcPwmValue[2] += turnValue;
    dcPwmValue[3] -= turnValue;

     /*speedSetting是控制馬達轉速的設定*/

    for(i = 0; i < 4; i ++) {

      if(dcPwmValue[i] > 2000) {

        dcPwmValue[i] = 2000;
      
      } else if(dcPwmValue[i] < 1000) {
      
        dcPwmValue[i] = 1000;
      
      }
      
    }

    processDCMotor(dcPwmValue[0], dcMotorPinA);
    processDCMotor(dcPwmValue[1], dcMotorPinB);
    processDCMotor(dcPwmValue[2], dcMotorPinC);
    processDCMotor(dcPwmValue[3], dcMotorPinD);
    

    if (isDebug) {
      
      Serial.println(F("=============================="));
      Serial.println();

      Serial.print(F("MotorA : "));
      Serial.print(dcPwmValue[0]);
      Serial.print(F(" ; MotorC : "));
      Serial.println(dcPwmValue[1]);
      Serial.print(F("MotorB : "));
      Serial.print(dcPwmValue[2]);
      Serial.print(F(" ; MotorD : "));
      Serial.println(dcPwmValue[3]);
      Serial.println();
      
    }


  }
}


void processDCMotor(int pwmValue, int dcMotor[]) {
  if (pwmValue == 1500) {

    digitalWrite(dcMotor[0], LOW);
    analogWrite(dcMotor[1], 0);

  } else if (pwmValue > 1500) {
    int power = map(pwmValue, 1500, 2000, 0 , 255);
    digitalWrite(dcMotor[0], LOW);
    analogWrite(dcMotor[1], power);
  } else {
    int power = map(pwmValue, 1500, 1000, 255 , 0);
    digitalWrite(dcMotor[0], HIGH);
    analogWrite(dcMotor[1], power);
  }
}


int StrToHex(char str[]) {
  return (int) strtol(str, 0, 16);
}


int convertHexStringToInt(char in[]) {
  int tens;
  int digits;

  if (!isxdigit(in[0]) || !isxdigit(in[1]))   // Valid hex digit character?
    return -1;

  in[0] = toupper(in[0]);   // Use upper case
  in[1] = toupper(in[1]);

  tens = in[0] >= 'A' ? (in[0] - 'A' + 10) : in[0] - '0';
  digits = in[1] >= 'A' ? (in[1] - 'A' + 10) : in[1] - '0';
  return tens * 16 + digits;
}


int speedSetting(int MotorValue) {

  int valueR = MotorValue;
  int stepN = 1500 - MotorValue;

  if (motorSetting == "default") {  //原始轉速

    valueR = MotorValue;

  } else if (motorSetting == "SettingA") {  //修正反向轉速使他接近正向轉速的運動曲線

    if (valueR < 1500) {
      
      if (stepN <= 100) {
        valueR = 1500 ; //馬達正向運動時在低轉速拉高時此處容易出現死區（不會轉）

      } else if (stepN > 100 && stepN <= 500) {

        if (stepN > 100 && stepN <= 110) {
          //＋1區段
          valueR = 1470 - (stepN - 100);
        } else if (stepN > 110 && stepN <= 120) {
          //＋2區段
          valueR = 1460 - ((stepN - 110) * 2);
        } else if (stepN > 120 && stepN <= 130) {
          //＋19區段
          valueR = 1440 - ((stepN - 120) * 19);
        } else if (stepN > 130 && stepN <= 150) {
          //＋1區段
          valueR = 1250 - (stepN - 130);
        } else if (stepN > 150 && stepN <= 170) {
          //＋1區段
          valueR = 1230 - (stepN - 150);
        } else if (stepN > 170 && stepN <= 190) {
          //＋1區段
          valueR = 1210 - (stepN - 170);
        } else if (stepN > 190 && stepN <= 210) {
          //＋1區段
          valueR = 1190 - (stepN - 190);
        } else if (stepN > 210 && stepN <= 230) {
          //＋1區段
          valueR = 1170 - (stepN - 210);
        } else if (stepN > 230 && stepN <= 250) {
          //＋1區段
          valueR = 1150 - (stepN - 230);
        } else if (stepN > 250 && stepN <= 270) {
          //＋1區段
          valueR = 1130 - (stepN - 250);
        } else if (stepN > 270 && stepN <= 290) {
          //＋1區段
          valueR = 1110 - (stepN - 270);
        } else if (stepN > 290 && stepN <= 310) {
          //趨近區段
          valueR = 1090 - ((stepN - 290) / 2);
        } else if (stepN > 310 && stepN <= 330) {
          //趨近區段
          valueR = 1080 - ((stepN - 310) / 2);
        } else if (stepN > 330 && stepN <= 350) {
          //趨近區段
          valueR = 1070 - ((stepN - 330) / 2);
        } else if (stepN > 350 && stepN <= 410) { //速度平衡點(當速度達到1060時與正轉轉速相差極小)
          //平衡區段
          valueR = 1060 - (stepN - 350);
        } else if (stepN > 410) { //馬達正轉時在此處時就已達最高速度
          valueR = 1000;
        }

      }

      if (isDebug) {
        
        Serial.print(F("valueR = "));
        Serial.println(valueR);

        Serial.print(F("stepN = "));
        Serial.println(stepN);
        
      }

    }

    if (MotorValue > 1993) {  //修正馬達反轉最高速時會略低於正轉的最高速
      valueR = 1993;
    }


  }

  return valueR;
}
