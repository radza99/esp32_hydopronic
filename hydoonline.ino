#include <WiFi.h>
#include <ThingSpeak.h>
#include <Firebase_ESP_Client.h>
#include <DHT.h>

//================ DHT =================
#define DHT_PIN 4
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

//================ WIFI =================
const char* ssid = "";
const char* password = "";


WiFiClient client;

//================ THINGSPEAK =================
unsigned long channelSensor = ;
const char* writeKeySensor = "";

unsigned long channelPump = ;
const char* writeKeyPump = "";

//================ FIREBASE =================
#define API_KEY ""
#define DATABASE_URL ""

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

#define FAN_PIN 32
#define LIGHT_PIN 33

#define PH_PIN 34
#define TDS_PIN 35

#define TRIG_PIN 18
#define ECHO_PIN 19

//================ RANGE =================
float PH_LOW  = 5.5;
float PH_HIGH = 7.5;

int TDS_LOW  = 850;
int TDS_HIGH = 950;

float LEVEL_MIN = 25;
float LEVEL_MAX = 40;

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

dht.begin();

}

//================ LOOP =================
void loop(){

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

//================ PROFILE =================
void readProfile(){

if(Firebase.RTDB.getString(&fbdo,"/profile")){
plantProfile=fbdo.stringData();
}

if(plantProfile=="kana"){

PH_LOW=6.0;
PH_HIGH=6.8;

TDS_LOW=1200;
TDS_HIGH=1600;

PH_SETPOINT=6.3;
TDS_SETPOINT=1400;

}

else if(plantProfile=="greencos"){

PH_LOW=5.5;
PH_HIGH=6.5;

TDS_LOW=800;
TDS_HIGH=1200;

PH_SETPOINT=6.0;
TDS_SETPOINT=1000;

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

if(duration==0)
level=-1;
else
level=50-distance;

float h = dht.readHumidity();
float t = dht.readTemperature();
if (!isnan(h) && !isnan(t)) {
  humidity    = h;
  temperature = t;
}

}

//================ MODE =================
void readMode(){

if(Firebase.RTDB.getString(&fbdo,"/mode")){
mode=fbdo.stringData();
}

}

//================ MANUAL =================
void readPump(String path,int pin){

if(Firebase.RTDB.getBool(&fbdo,path)){

bool state=fbdo.boolData();
digitalWrite(pin,state);

}

}

//================ PH CONTROL =================
void controlPH(){

if(millis()-lastPHrun<pumpOff) return;

if(phValue>=PH_LOW && phValue<=PH_HIGH) return;

if(phValue>PH_HIGH){

Serial.println("PH HIGH -> ADD ACID");

digitalWrite(RELAY_ACID,HIGH);
delay(pumpOn);
digitalWrite(RELAY_ACID,LOW);

}

else if(phValue<PH_LOW){

Serial.println("PH LOW -> ADD BASE");

digitalWrite(RELAY_BASE,HIGH);
delay(pumpOn);
digitalWrite(RELAY_BASE,LOW);

}

lastPHrun=millis();

}

//================ TDS CONTROL =================
void controlTDS(){

if(millis()-lastTDSrun<pumpOff) return;

if(tdsValue>=TDS_LOW && tdsValue<=TDS_HIGH) return;

if(tdsValue<TDS_LOW){

Serial.println("TDS LOW -> ADD FERT");

digitalWrite(RELAY_FERTA,HIGH);
digitalWrite(RELAY_FERTB,HIGH);

delay(pumpOn);

digitalWrite(RELAY_FERTA,LOW);
digitalWrite(RELAY_FERTB,LOW);

}

lastTDSrun=millis();

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

ThingSpeak.writeFields(channelSensor,writeKeySensor);

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

ThingSpeak.writeFields(channelPump,writeKeyPump);

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