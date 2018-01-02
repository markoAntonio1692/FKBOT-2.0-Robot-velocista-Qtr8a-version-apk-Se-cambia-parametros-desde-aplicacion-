#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <QTRSensors.h>
#define NUM_SENSORS             10  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  2  // average 4 analog samples per sensor reading
#define EMITTER_PIN             9   // emitter is controlled by digital pin 2
QTRSensorsAnalog qtra((unsigned char[]) {7,7, 6, 5, 4, 3, 2,1,0,0},NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
SoftwareSerial bluetooth(10, 11); // RX, TX
//entradas
  #define btn1      0
  #define btn2      1
//salidas
  #define led1      13
  #define led2      12
  #define led3      2
  #define led_on    9   //~
  #define mi1       4
  #define mi2       3
  #define pwmi      6   //~
  #define md1       8
  #define md2       7
  #define pwmd      5   //~
//comunicacion
  #define tx        11
  #define rx        10
//variables  eeprom//
  float e_kp=9.898;      //direccion de 0 a 19
  float e_kd=10.010;      //direccion de 20 a 39
  float e_ki=0.171;       //direccion de 40 a 59
  int   e_velocidad=120;  //direccion de 60 a 79
  int   e_freno_atras=0;      //direccion de 80 a 99
  int   e_freno_adelante=0;   //direccion de 100 a 119
  int   e_flanco_comparacion=0; //direccion de 120 a 139
  int   e_flanco_color=0;
  int   e_en_linea=0;
  int   e_ruido=0;
  bool var_tipo_pid=1;


String readString="";
char c=0;
int freno_atras=0;
int freno_adelante=0;
int flanco_comparacion=0;
int flanco_color=0;
int en_linea=0;
int ruido=0;
int proporcional=0;
int derivativo=0;
long integral=0;
int salida_pwm=0;
int proporcional_pasado=0;
int velocidad=80;
float KP=0.02, KD=0.5, KI=0.0001;
int position=0;

int var_bit1=0;
bool var_test=0;
int boton1=7;
int boton2=7;
void setup()
{
    Serial.begin(115200);
    bluetooth.begin(115200);
    delay(500);
    pinMode(led1,OUTPUT);
    pinMode(led2,OUTPUT);
    pinMode(led3,OUTPUT);
    pinMode(led_on,OUTPUT);
    pinMode(mi1,OUTPUT);
    pinMode(mi2,OUTPUT);
    pinMode(pwmi,OUTPUT);
    pinMode(md1,OUTPUT);
    pinMode(md2,OUTPUT);
    pinMode(pwmd,OUTPUT);
digitalWrite(led1,HIGH);
digitalWrite(led2,HIGH);
digitalWrite(led3,HIGH);
delay(300);
digitalWrite(led1,LOW);
digitalWrite(led2,LOW);
digitalWrite(led3,LOW);
delay(300);
leer_eeprom();   
velocidad=e_velocidad;
KP=e_kp;
KD=e_kd;
KI=e_ki;
freno_atras=e_freno_atras;
freno_adelante=e_freno_adelante;
flanco_comparacion=e_flanco_comparacion;
flanco_color=e_flanco_color;
en_linea=e_en_linea;
ruido=e_ruido; 
   for (int i = 0; i < 100; i++)  // make the calibration take about 10 seconds
   {
      qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call) 
      digitalWrite(led2, !digitalRead(led2));
   }
  digitalWrite(led2, LOW);     // turn off Arduino's LED to indicate we are through with calibration
 //Serial.begin(115200);
   //Serial.println();

   while(true)
   {
    recepcion_data();   
   botones();
      if(boton2==0 || var_bit1==1) 
      {
        delay(20);
        digitalWrite(led2,HIGH);
        digitalWrite(led3,HIGH);
        delay(100);
        digitalWrite(led2,LOW);
        digitalWrite(led3,LOW);
        delay(100);
        var_test=0;
        var_bit1=1;
        break;
      }
      if(boton1==0)//modo testeo 
      {
        delay(20);
        digitalWrite(led2,HIGH);
        digitalWrite(led3,HIGH);
        delay(100);
        digitalWrite(led2,LOW);
        digitalWrite(led3,LOW);
        delay(100);
        var_test=1;
        var_bit1=1;
        break;
      }
   }
    
}

void loop()
{
recepcion_data();
velocidad=e_velocidad;
KP=e_kp;
KD=e_kd;
KI=e_ki;
freno_atras=e_freno_atras;
freno_adelante=e_freno_adelante;
flanco_comparacion=e_flanco_comparacion;
flanco_color=e_flanco_color;
en_linea=e_en_linea;
ruido=e_ruido;
  pid(1,velocidad,KP,KI,KD, flanco_color,en_linea, ruido);
  frenos_contorno(freno_atras, freno_adelante,flanco_comparacion);
  funcion_pausa();
   
  //if(var_test==1){ test(flanco_color, en_linea,ruido);}

}
void funcion_pausa()
{
  if (var_bit1==0)
  {
    while(true)
    {
      recepcion_data();
      motores(0,0);
      if (var_bit1==1) break;
      
    }
   
  }  

}
void frenos_contorno(int freno_atras, int freno_adelante, int flanco_comparacion)
{
    if (position<=0) //si se salio por la parte derecha de la linea
    {
    
      while(true)
      {
        recepcion_data();
        digitalWrite(led2,HIGH);
        motores(freno_atras,freno_adelante);
        qtra.read(sensorValues); //lectura en bruto de sensor
        if ( sensorValues[0]<flanco_comparacion || sensorValues[1]<flanco_comparacion || sensorValues[2]<flanco_comparacion || sensorValues[3]<flanco_comparacion
        ||sensorValues[4]<flanco_comparacion||sensorValues[5]<flanco_comparacion||sensorValues[6]<flanco_comparacion||sensorValues[7]<flanco_comparacion    )
        {
          break;
        }
        
      }
    }

    if (position>=9000) //si se salio por la parte izquierda de la linea
    {
      while(true)
      {
        recepcion_data();
        digitalWrite(led2,HIGH);
        motores(freno_adelante,freno_atras); 
        qtra.read(sensorValues);
        if (sensorValues[7]<flanco_comparacion || sensorValues[6]<flanco_comparacion|| sensorValues[5]<flanco_comparacion|| sensorValues[4]<flanco_comparacion
        || sensorValues[3]<flanco_comparacion|| sensorValues[2]<flanco_comparacion|| sensorValues[1]<flanco_comparacion|| sensorValues[0]<flanco_comparacion)
        {
          break;
        }
      }
  }
  digitalWrite(led2,LOW);
}
  
void pid(int linea, int velocidad, float Kp, float Ki, float Kd, int flanco_color, int en_linea, int ruido)
{
 position = qtra.readLine(sensorValues,QTR_EMITTERS_ON, 0, flanco_color, en_linea, ruido);
 //negra, 1 para linea blanca
  proporcional = (position) - 4500; // set point es 3500, asi obtenemos el error
  integral=integral + proporcional_pasado; //obteniendo integral
 
  derivativo = (proporcional - proporcional_pasado); //obteniedo el derivativo
  if (integral>7000) integral=7000; //limitamos la integral para no causar problemas
  if (integral<-7000) integral=-7000;
  

  salida_pwm =(int)(( proporcional * Kp ) + ( derivativo * Kd )+(integral*Ki));
 
  if (  salida_pwm > velocidad )  salida_pwm = velocidad; //limitamos la salida de pwm
  if ( salida_pwm < -velocidad )  salida_pwm = -velocidad;


if(var_tipo_pid==1)
{
  if (salida_pwm < 0)
  {
      int der=velocidad-salida_pwm;
      if(der>255)der=255;
    motores(velocidad+salida_pwm, der);
  }
  if (salida_pwm >0)
  {
    int izq=velocidad+salida_pwm;
    if(izq>255) izq=255;
    motores(izq, velocidad-salida_pwm);
  }
}

if(var_tipo_pid==0)
{

  if (salida_pwm < 0)
  {
    motores(velocidad+salida_pwm, velocidad);
  }
  if (salida_pwm >0)
  {
    motores(velocidad, velocidad-salida_pwm);
  }
}

 proporcional_pasado = proporcional;  
}

void motores(int motor_izq, int motor_der)
{
  if ( motor_izq >= 0 )  
  {
    digitalWrite(mi1,LOW);
    digitalWrite(mi2,HIGH); 
    analogWrite(pwmi,motor_izq); 
  }
  else
  {
    digitalWrite(mi1,HIGH); 
    digitalWrite(mi2,LOW);
    motor_izq = motor_izq*(-1); 
    analogWrite(pwmi,motor_izq);
  }

  if ( motor_der >= 0 ) //motor derecho
  {
    digitalWrite(md1,LOW);
    digitalWrite(md2,HIGH);
    analogWrite(pwmd,motor_der);
  }
  else
  {
    digitalWrite(md1,HIGH);
    digitalWrite(md2,LOW);
    motor_der= motor_der*(-1);
    analogWrite(pwmd,motor_der);
  }
}

void botones()
{
boton1=digitalRead(btn2);boton2=digitalRead(btn1); ///boton izquierdo, derecho
}
void recepcion_data()
{
  readString="";
  while( bluetooth.available())
  {
     c=bluetooth.read();
     
     if(c=='#') 
     {
       var_bit1=readString.toInt();
     }
     if(c=='p') //guardando kp
     { 
      for(int k=0; k<=19;k++){  EEPROM.write(k,0); } // borrando    
       e_kp=readString.toFloat(); 
       EEPROM.put(0, e_kp); //almacenamiento kp   
       Serial.print("dato kp: ");   
       Serial.println(e_kp,5);
       }  if(c=='P')
          {  
            e_kp=0;
            EEPROM.get(0,e_kp);   
             Serial.print("R");
             Serial.print(e_kp,5); 
             bluetooth.println(e_kp,5); 
          }
          
     if(c=='d')
    {
      for(int k=20; k<=39;k++){  EEPROM.write(k,0); } 
      e_kd=readString.toFloat();
      EEPROM.put(20, e_kd); //almacenamiento kd   
      Serial.print("dato kd; ");
      Serial.println(e_kd,5);
    }     if(c=='D')
          {
            EEPROM.get(20,e_kd);
            Serial.print("R");   
            Serial.print(e_kd,5);
            bluetooth.println(e_kd,5);
          }
    
   if(c=='i')
    {
      for(int k=40; k<=59;k++){  EEPROM.write(k,0); } 
      e_ki=readString.toFloat();
      EEPROM.put(40, e_ki); //almacenamiento kd 
      Serial.print("dato  ki ");
      Serial.println(e_ki,5);
    }     if(c=='I')
          {
            EEPROM.get(40,e_ki);   
            Serial.print("R");
            Serial.print(e_ki,5);
            bluetooth.println(e_ki,5);
          }

   if(c=='v')//guardando
    {
      for(int k=60; k<=79;k++){    EEPROM.write(k,0); }  // borrando 
      e_velocidad=readString.toFloat();
      EEPROM.put(60, e_velocidad); 
      //Serial.print("dato  velocidad ");
      Serial.println(e_velocidad);
     }   if(c=='V') //recuperando 
         {
            EEPROM.get(60,e_velocidad);   
            Serial.print("R");
            Serial.print(e_velocidad);
            int vel=e_velocidad;
            bluetooth.println(vel);
         }
         
//FRENO ATRAS   
    if(c=='t')//guardando
    {
      for(int k=80; k<=99;k++){    EEPROM.write(k,0); }  // borrando 
      e_freno_atras=readString.toInt();
      EEPROM.put(80, e_freno_atras); 
      //Serial.print("dato  velocidad ");
      Serial.println(e_freno_atras);
     }   if(c=='T') //recuperando 
         {
            EEPROM.get(80,e_freno_atras);
            Serial.print("R");   
            Serial.print(e_freno_atras);
            bluetooth.println(e_freno_atras);
         }
//FRENO ADELANTE   
    if(c=='a')//guardando
    {
      for(int k=100; k<=119;k++){    EEPROM.write(k,0); }  // borrando 
      e_freno_adelante=readString.toInt();
      EEPROM.put(100, e_freno_adelante); 
      //Serial.print("dato  velocidad ");
      Serial.println(e_freno_adelante);
     }   if(c=='A') //recuperando 
         {
            EEPROM.get(100,e_freno_adelante);   
            Serial.print("R");
            Serial.print(e_freno_adelante);
            bluetooth.println(e_freno_adelante);
         }        
//FRENO FLANCO COMPARACION 
    if(c=='c')//guardando
    {
      for(int k=120; k<=139;k++){    EEPROM.write(k,0); }  // borrando 
      e_flanco_comparacion=readString.toInt();
      EEPROM.put(120, e_flanco_comparacion); 
      //Serial.print("dato  velocidad ");
      Serial.println(e_flanco_comparacion);
     }   if(c=='C') //recuperando 
         {
            EEPROM.get(120,e_flanco_comparacion);   
            Serial.print("R");
            Serial.print(e_flanco_comparacion);
            bluetooth.println(e_flanco_comparacion);
         }
//FLANCO COLOR 
    if(c=='n')//guardando
    {
      for(int k=140; k<=159;k++){    EEPROM.write(k,0); }  // borrando 
      e_flanco_color=readString.toInt();
      EEPROM.put(140, e_flanco_color); 
      //Serial.print("dato  velocidad ");
      Serial.println(e_flanco_color);
     }   if(c=='N') //recuperando 
         {
            EEPROM.get(140,e_flanco_color);
            Serial.print("R");   
            Serial.print(e_flanco_color);
            bluetooth.println(e_flanco_color);
         }
//EN LINEA
    if(c=='l')//guardando
    {
      for(int k=160; k<=179;k++){    EEPROM.write(k,0); }  // borrando 
      e_en_linea=readString.toInt();
      EEPROM.put(160, e_en_linea); 
      //Serial.print("dato  velocidad ");
      Serial.println(e_en_linea);
     }   if(c=='L') //recuperando 
         {
            EEPROM.get(160,e_en_linea);
           Serial.print("R");   
            Serial.print(e_en_linea);
            bluetooth.println(e_en_linea);
         }
//EN RUIDO
    if(c=='r')//guardando
    {
      for(int k=180; k<=199;k++){    EEPROM.write(k,0); }  // borrando 
      e_ruido=readString.toInt();
      EEPROM.put(180,e_ruido); 
      //Serial.print("dato  velocidad ");
      Serial.println(e_ruido);
     }   if(c=='R') //recuperando 
         {
            EEPROM.get(180,e_ruido);   
            Serial.print(e_ruido);
            bluetooth.println(e_ruido);
         }
   
   readString+=c; //concatena datos que llegan
  }
}

void leer_eeprom()
{
    EEPROM.get(0,e_kp);
    EEPROM.get(20,e_kd);
    EEPROM.get(40,e_ki);
    EEPROM.get(60,e_velocidad);  
    EEPROM.get(80,e_freno_atras);
    EEPROM.get(100,e_freno_adelante);
    EEPROM.get(120,e_flanco_comparacion);
    EEPROM.get(140,e_flanco_color);
    EEPROM.get(160,e_en_linea); 
    EEPROM.get(180,e_ruido); 
    Serial.println("datos recuperados!!");
 //   Serial.println(e_kp,5);
   // Serial.print(" ");
   // Serial.println(e_kd,5);
    //Serial.print(" ");
    //Serial.println(e_ki,5);
    //Serial.print(" ");
    //Serial.println(e_velocidad,5);
    //Serial.print(" ");
}

void test(int flanco_color,int en_line,int ruido)
{

recepcion_data();
position = qtra.readLine(sensorValues,QTR_EMITTERS_ON, 0, flanco_color, en_linea, ruido);

  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    
     recepcion_data();
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position); // comment this line out if you are using raw values
}

