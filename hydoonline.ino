#include <WiFi.h>
#include <ThingSpeak.h>
#include <Firebase_ESP_Client.h>

//================ WIFI =================
const char* ssid = "rathiot";
const char* password = "12345678*";

WiFiClient client;

//================ WIFI RETRY =================
int wifiRetryCount = 0;
unsigned long lastWifiCheck = 0;
bool offlineMode = false;

//================ THINGSPEAK =================
unsigned long channelSensor = 3250622;
const char* writeKeySensor = "DIF6E7LDGO25BOED";

unsigned long channelPump = 3298639;
const char* writeKeyPump = "57H5NV7710JR6OC8";

//================ FIREBASE =================
#define API_KEY "AIzaSyAIzstt1xaSJrRnBlFh6GgXd8KUPguAkxI"
#define DATABASE_URL "smarthydroponic-f1a49-default-rtdb.asia-southeast1.firebasedatabase.app"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool signupOK=false;

String mode="auto";
String plantProfile="kana";

//================ PIN =================
#define RELAY_FERTA 14
#define RELAY_FERTB 27
#define RELAY_WATER 25
#define RELAY_ACID 12
#define RELAY_BASE 13
#define RELAY_MIX 26

#define FAN_PIN 33
#define LIGHT_PIN 32

#define PH_PIN 34
#define TDS_PIN 35

#define TRIG_PIN 5
#define ECHO_PIN 18

//================ RANGE =================
float PH_LOW  = 5.5;
float PH_HIGH = 7.5;

int TDS_LOW  = 850;
int TDS_HIGH = 950;

float LEVEL_MIN = 25;
float LEVEL_MAX = 40;

// DEAD BAND
int TDS_DEADBAND = 50;
float PH_DEADBAND = 0.2;

//================ PID SETPOINT =================
float PH_SETPOINT = 6.2;
float TDS_SETPOINT = 900;

//================ PID PH =================
float Kp_ph = 2.0;
float Ki_ph = 0.05;
float Kd_ph = 0.5;

float ph_error=0;
float ph_lastError=0;
float ph_integral=0;

//================ PID TDS =================
float Kp_tds = 0.8;
float Ki_tds = 0.02;
float Kd_tds = 0.2;

float tds_error=0;
float tds_lastError=0;
float tds_integral=0;

//================ TIMER =================
unsigned long pumpOn  = 3000;
unsigned long pumpOff = 15000;

unsigned long lastPHrun=0;
unsigned long lastTDSrun=0;
unsigned long lastSend=0;

//================ SENSOR =================
float phValue=0;
int tdsValue=0;
float level=0;

float temperature=29;
float humidity=60;

//================ SETUP =================
void setup(){

Serial.begin(115200);

pinMode(RELAY_FERTA,OUTPUT);
pinMode(RELAY_FERTB,OUTPUT);
pinMode(RELAY_WATER,OUTPUT);
pinMode(RELAY_ACID,OUTPUT);
pinMode(RELAY_BASE,OUTPUT);
pinMode(RELAY_MIX,OUTPUT);

pinMode(FAN_PIN,OUTPUT);
pinMode(LIGHT_PIN,OUTPUT);

pinMode(TRIG_PIN,OUTPUT);
pinMode(ECHO_PIN,INPUT);

digitalWrite(RELAY_MIX,HIGH);
digitalWrite(FAN_PIN,HIGH);
digitalWrite(LIGHT_PIN,HIGH);

WiFi.begin(ssid,password);

while(WiFi.status()!=WL_CONNECTED){
delay(500);
Serial.print(".");
}

Serial.println("WiFi Connected");

ThingSpeak.begin(client);

config.api_key=API_KEY;
config.database_url=DATABASE_URL;

if(Firebase.signUp(&config,&auth,"","")){
signupOK=true;
}

Firebase.begin(&config,&auth);
Firebase.reconnectWiFi(true);

}

//================ LOOP =================
void loop(){

handleWiFi();   // 🔥 เพิ่ม

readSensors();
readMode();
readProfile();

if(mode=="auto"){

controlPH();
controlTDS();
controlWater();

}
else{

readPump("/pump/water",RELAY_WATER);
readPump("/pump/fertA",RELAY_FERTA);
readPump("/pump/fertB",RELAY_FERTB);
readPump("/pump/acid",RELAY_ACID);
readPump("/pump/base",RELAY_BASE);
readPump("/pump/fan",FAN_PIN);
readPump("/pump/light",LIGHT_PIN);

}

printStatus();

if(millis()-lastSend>20000){

sendSensor();
delay(2000);
sendPump();

lastSend=millis();

}

delay(1000);

}

//================ WIFI HANDLER =================
void handleWiFi(){

  if(WiFi.status() == WL_CONNECTED){
    offlineMode = false;
    wifiRetryCount = 0;
    return;
  }

  if(offlineMode){
    if(millis() - lastWifiCheck > 300000){
      Serial.println("Retry WiFi after 5 minutes...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      lastWifiCheck = millis();
      wifiRetryCount = 0;
      offlineMode = false;
    }
    return;
  }

  if(wifiRetryCount < 2){
    Serial.println("WiFi reconnect attempt...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    wifiRetryCount++;
    delay(3000);
  } else {
    Serial.println("WiFi FAILED → OFFLINE MODE");
    offlineMode = true;
    lastWifiCheck = millis();
  }
}

//================ PROFILE =================
void readProfile(){

if(!offlineMode && Firebase.RTDB.getString(&fbdo,"/profile")){
plantProfile=fbdo.stringData();
}

if(plantProfile=="kana"){
PH_LOW=6.0; PH_HIGH=6.8;
TDS_LOW=1200; TDS_HIGH=1600;
PH_SETPOINT=6.3; TDS_SETPOINT=1400;
}
else if(plantProfile=="greencos"){
PH_LOW=5.5; PH_HIGH=6.5;
TDS_LOW=800; TDS_HIGH=1200;
PH_SETPOINT=6.0; TDS_SETPOINT=1000;
}

}

//================ SENSOR =================
void readSensors(){

long phSum=0;
for(int i=0;i<10;i++){
phSum+=analogRead(PH_PIN);
delay(10);
}

float phRaw=phSum/10.0;
float voltage=phRaw*(3.3/4095.0);
phValue=7+((2.5-voltage)/0.18);

long tdsSum=0;
for(int i=0;i<10;i++){
tdsSum+=analogRead(TDS_PIN);
delay(10);
}

int tdsRaw=tdsSum/10;
tdsValue=map(tdsRaw,0,4095,0,2000);

digitalWrite(TRIG_PIN,LOW);
delayMicroseconds(2);
digitalWrite(TRIG_PIN,HIGH);
delayMicroseconds(10);
digitalWrite(TRIG_PIN,LOW);

long duration=pulseIn(ECHO_PIN,HIGH,30000);
float distance=duration*0.034/2;

if(duration==0) level=-1;
else level=50-distance;

}

//================ MODE =================
void readMode(){

if(!offlineMode && Firebase.RTDB.getString(&fbdo,"/mode")){
mode=fbdo.stringData();
}

}

//================ MANUAL =================
void readPump(String path,int pin){

if(!offlineMode && Firebase.RTDB.getBool(&fbdo,path)){
bool state=fbdo.boolData();
digitalWrite(pin,state);
}

}

//================ PH CONTROL =================
void controlPH(){

if(millis()-lastPHrun < pumpOff) return;

float error = PH_SETPOINT - phValue;
ph_error = error;

if(abs(error) <= PH_DEADBAND){
ph_integral = 0;
return;
}

ph_integral += error;
float derivative = error - ph_lastError;

float output = (Kp_ph * error) +
               (Ki_ph * ph_integral) +
               (Kd_ph * derivative);

ph_lastError = error;

int pumpTime = constrain(abs(output)*1000, 500, 5000);

if(error < 0){
digitalWrite(RELAY_ACID, HIGH);
delay(pumpTime);
digitalWrite(RELAY_ACID, LOW);
}else{
digitalWrite(RELAY_BASE, HIGH);
delay(pumpTime);
digitalWrite(RELAY_BASE, LOW);
}

lastPHrun = millis();
}

//================ TDS CONTROL =================
void controlTDS(){

if(millis()-lastTDSrun < pumpOff) return;

int error = TDS_SETPOINT - tdsValue;
tds_error = error;

if(abs(error) <= TDS_DEADBAND){
tds_integral = 0;
return;
}

tds_integral += error;
float derivative = error - tds_lastError;

float output = (Kp_tds * error) +
               (Ki_tds * tds_integral) +
               (Kd_tds * derivative);

tds_lastError = error;

int pumpTime = constrain(abs(output)*10, 500, 5000);

if(error > 0){
digitalWrite(RELAY_FERTA, HIGH);
digitalWrite(RELAY_FERTB, HIGH);
delay(pumpTime);
digitalWrite(RELAY_FERTA, LOW);
digitalWrite(RELAY_FERTB, LOW);
}

lastTDSrun = millis();
}

//================ WATER =================
void controlWater(){

if(level<LEVEL_MIN && level!=-1){
digitalWrite(RELAY_WATER,HIGH);
}
else if(level>LEVEL_MAX){
digitalWrite(RELAY_WATER,LOW);
}

}

//================ THINGSPEAK =================
void sendSensor(){

ThingSpeak.setField(1,phValue);
ThingSpeak.setField(2,tdsValue);
ThingSpeak.setField(3,level);
ThingSpeak.setField(4,temperature);
ThingSpeak.setField(5,humidity);

if(!offlineMode){
ThingSpeak.writeFields(channelSensor,writeKeySensor);
}

}

void sendPump(){

ThingSpeak.setField(1,digitalRead(RELAY_WATER));
ThingSpeak.setField(2,digitalRead(RELAY_FERTA));
ThingSpeak.setField(3,digitalRead(RELAY_FERTB));
ThingSpeak.setField(4,digitalRead(RELAY_ACID));
ThingSpeak.setField(5,digitalRead(RELAY_BASE));
ThingSpeak.setField(6,digitalRead(RELAY_MIX));
ThingSpeak.setField(7,digitalRead(FAN_PIN));
ThingSpeak.setField(8,digitalRead(LIGHT_PIN));

if(!offlineMode){
ThingSpeak.writeFields(channelPump,writeKeyPump);
}

}

//================ SERIAL =================
void printStatus(){

Serial.println("================================================");
Serial.println("           HYDROPONIC SMART FARM SYSTEM");
Serial.println("================================================");

Serial.println("--------------- SYSTEM ----------------");

Serial.print("MODE            : ");
Serial.println(mode);

Serial.print("PLANT PROFILE   : ");
Serial.println(plantProfile);

Serial.println("--------------- SENSOR DATA ----------------");

Serial.print("PH VALUE        : ");
Serial.println(phValue,2);

Serial.print("TDS VALUE       : ");
Serial.println(tdsValue);

Serial.print("TEMPERATURE     : ");
Serial.print(temperature);
Serial.println(" C");

Serial.print("HUMIDITY        : ");
Serial.print(humidity);
Serial.println(" %");

Serial.print("WATER LEVEL     : ");
Serial.print(level);
Serial.println(" cm");

Serial.println("--------------- SETPOINT ----------------");

Serial.print("PH SETPOINT     : ");
Serial.println(PH_SETPOINT);

Serial.print("TDS SETPOINT    : ");
Serial.println(TDS_SETPOINT);

Serial.print("PH RANGE        : ");
Serial.print(PH_LOW);
Serial.print(" - ");
Serial.println(PH_HIGH);

Serial.print("TDS RANGE       : ");
Serial.print(TDS_LOW);
Serial.print(" - ");
Serial.println(TDS_HIGH);

Serial.println("--------------- PID DATA ----------------");

Serial.print("PH ERROR        : ");
Serial.println(ph_error);

Serial.print("PH INTEGRAL     : ");
Serial.println(ph_integral);

Serial.print("TDS ERROR       : ");
Serial.println(tds_error);

Serial.print("TDS INTEGRAL    : ");
Serial.println(tds_integral);

Serial.println("--------------- WATER CONTROL ----------------");

Serial.print("LEVEL MIN       : ");
Serial.println(LEVEL_MIN);

Serial.print("LEVEL MAX       : ");
Serial.println(LEVEL_MAX);

Serial.println("--------------- PUMP STATUS ----------------");

Serial.print("WATER PUMP      : ");
Serial.println(digitalRead(RELAY_WATER) ? "ON" : "OFF");

Serial.print("FERTILIZER A    : ");
Serial.println(digitalRead(RELAY_FERTA) ? "ON" : "OFF");

Serial.print("FERTILIZER B    : ");
Serial.println(digitalRead(RELAY_FERTB) ? "ON" : "OFF");

Serial.print("ACID PUMP       : ");
Serial.println(digitalRead(RELAY_ACID) ? "ON" : "OFF");

Serial.print("BASE PUMP       : ");
Serial.println(digitalRead(RELAY_BASE) ? "ON" : "OFF");

Serial.print("MIX MOTOR       : ");
Serial.println(digitalRead(RELAY_MIX) ? "ON" : "OFF");

Serial.println("--------------- OTHER DEVICE ----------------");

Serial.print("FAN             : ");
Serial.println(digitalRead(FAN_PIN) ? "ON" : "OFF");

Serial.print("LIGHT           : ");
Serial.println(digitalRead(LIGHT_PIN) ? "ON" : "OFF");

Serial.println("--------------- TIMER ----------------");

Serial.print("PUMP ON TIME    : ");
Serial.print(pumpOn/1000);
Serial.println(" sec");

Serial.print("PUMP WAIT TIME  : ");
Serial.print(pumpOff/1000);
Serial.println(" sec");

Serial.println("================================================");
Serial.println();

}