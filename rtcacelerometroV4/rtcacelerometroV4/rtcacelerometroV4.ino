//////CODIGO SENSOR MOVIMENTO DE MASSA 26/03/2018
/*Considerações:   O endereço de comunicação do RTC e do MPU6050 pode ser os mesmos 0x68.   
 * para mudar o endereço do MPU6050, deve-se conetar o pino AD0 PARA 3.3V. Isso fara com que 
 * o enderço do sensor mude para 0x69.
 * Para verificar todos os dispositivos conectados ao I2C (PINOS A4 E A5) utilize o código do link
 * https://gist.github.com/tfeldmann/5411375
 * Devem aparecer 3 endereços (dois pelo RTC e um pelo sensor - 0x69 - )
 * 
 * O código implementa um SD para guardar os dados coletados
 */
////CODIGO MPU6050

// Pinagem Arduino UNO
// GND - GND
// VCC - 5 V
// SDA - A4
// SCL - A5

// Pinagem Arduino MEGA
// GND - GND
// VCC - 5 V
// SDA - 20
// SCL - 21

//Biblioteca necessária para controlar o MPU6050  UTILIZAR AS LIBRARIAS COMPARTILHADAS NO GOOGLE DRIVE

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#include <SD.h>

//Biblioteca do RTC

//#include <DS3231.h>
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC

MPU6050 sensor(0x69);  //Define o endereço 0x69 como o endereço do sensor MPU6050


// Init the DS3231 using the hardware interface
//DS3231  rtc(SDA, SCL); //SDA, SCL   

// Init a Time-data structure
//Time  t;

// Criando variavel MPU6050 sensor

//Declarando as Variaveis 

//Variaveis para angulo de inclinação

int minVal=265;
int maxVal=402;

double x;
double y;
double z;

//Variaveis do Acelerometro e Giroscopio sem processar.
int axL, ay, az;
int gx, gy, gz;

// Variaveis para o calculo do angulo rotaçao 
long tempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;
float ax_m_s2,ay_m_s2,az_m_s2;

const int chipSelect = 4;  // Define o pin que ativa o cartão de memoria
 
 void setup() {
  Serial.begin(57600);    //Iniciando o monitor serial 57600
 
  sensor.initialize();    //Iniciando o sensor de aceleracion
 

 
  //rtc.begin();  // Initialize the rtc object
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if(timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");

  
  // The following lines can be uncommented to set the date and time
  
  //rtc.setTime(15,35, 0);     // Define a hora (formato de 24h)
  //rtc.setDate(27,11, 2018);   // Define a data
  
  if (sensor.testConnection()) Serial.println(F("Sensor iniciado corretamente"));
  else Serial.println(F("Erro ao iniciar o sensor"));

  // Verifica se o cartão de memoria pode ser inicializado
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Falha ao abrir o cartão / cartão não presente"));
    // don't do anything more:
    while (1);
  }
  Serial.println(F("Cartão inizializado"));
  delay(1000);
  
}

void loop() {
  
  LerAcelera(); //Faz as leituras do MPU6050.  Armazena as informações nas variaveis globais
  //t = rtc.getTime(); // Faz leitura do horario atual  
  PrintarInforma();  //Printa na tela as informações     
  //GravarNoSD(); 
  delay(1000);      //Aguarda um segundo entre as informações
  
}
/************************************************************************************/
/*
 * 
 * FUNÇÕES DE APOIO AO PROGRAMA
 * 
 */
/************************************************************************************/
// Função para gravar os dados no cartão SD
void GravarNoSD()
{

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(String(year()));
    dataFile.print(",");
    dataFile.print(month());
    dataFile.print(",");
    dataFile.print(day());
    dataFile.print(",");
    dataFile.print(hour());
    dataFile.print(",");
    dataFile.print(minute());
    dataFile.print(",");
    dataFile.print(String(axL));
    dataFile.print(",");
    dataFile.print(String(ay));
    dataFile.print(",");
    dataFile.print(String(az));
    dataFile.print(",");
    dataFile.print(String(gx));
    dataFile.print(",");
    dataFile.print(String(gy));
    dataFile.print(",");
    dataFile.println(String(gz));      
    dataFile.close();
    // print to the serial port too:
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println(F("error Abrindo o arquivo"));
  }
}


/*Função para ler os valores da aceleração*/
void LerAcelera()  
{
  //Ler os dados do Acelerometro 
  sensor.getAcceleration(&axL, &ay, &az);
  
  //Ler os dados do Giroscopio  
  sensor.getRotation(&gx, &gy, &gz);

  //Calculos 
  //Calcular os Angulos de Inclinaçao (X,Y,Z)
    int xAng = map(axL,minVal,maxVal,-90,90);
    int yAng = map(ay,minVal,maxVal,-90,90);
    int zAng = map(az,minVal,maxVal,-90,90);

       x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
       y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
       z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
   
  // Transformar os valores do acelerometro inicialmente lidos em bits para m/s²
  
   ax_m_s2 = axL * (9.81/16384.0);
   ay_m_s2 = ay * (9.81/16384.0);
   az_m_s2 = az * (9.81/16384.0);

  
  // Angulos de Rotaçao com filtro de interferencia 
  // Angulos com o acelerometro em radiano
  float acel_ang_x=atan(ay/sqrt(pow(axL,2) + pow(az,2)))*(180.0/PI);
  float acel_ang_y=atan(-axL/sqrt(pow(ay,2) + pow(az,2)))*(180.0/PI);

  //Calcular o angulo de rotaçao com o giroscopio e filtro de complemento

  
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*acel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*acel_ang_y;

  //Substituindo od valores
}


void PrintarInforma()
{
  /*
   Serial.print("Data: ");
  Serial.print(t.date, DEC); 
  Serial.print(" / ");
  Serial.print(rtc.getMonthStr());
  Serial.print(" / ");
  Serial.print(t.year, DEC);
  Serial.println(".");
  
  // Send time
  Serial.print(t.hour, DEC);
  Serial.print(" hora(s):, ");
  Serial.print(t.min, DEC);
  Serial.print(" minuto(s) e ");
  Serial.print(t.sec, DEC);
  Serial.println(" segundo(s).");
*/

  digitalClockDisplay();
  // Mostrar os angulos (X,Y,Z)
  Serial.println(" Angulo de Inclinacao");
  Serial.print(" X: "); Serial.print(x); Serial.print("º"); 
  Serial.print(" Y: ");  Serial.print(y); Serial.print("º");
  Serial.print(" Z: ");  Serial.print(z); Serial.println("º");
  
 // Mostrar os angulos de rotaçao Giroscopio
  Serial.println(" Angulo de Rotacao");
  Serial.print(" X: "); Serial.print(ang_x);  Serial.print("º");
  Serial.print(" Y: "); Serial.print(ang_y); Serial.println("º");

 // Aceleraçoes Angulares
 
  Serial.println(" Aceleracao Angular");
  Serial.print(" X:");Serial.print(ax_m_s2); Serial.print(" m/s2"); Serial.print("\t");
  Serial.print(" Y:");Serial.print(ay_m_s2); Serial.print(" m/s2");  Serial.print("\t");
  Serial.print(" Z:");Serial.print(az_m_s2); Serial.print(" m/s2"); Serial.println("\t");
  
  Serial.println(" Aceleracao Original");
  Serial.print(" AX:");Serial.print(axL); Serial.print(" m/s2"); Serial.print("\t");
  Serial.print(" AY:");Serial.print(ay); Serial.print(" m/s2"); Serial.print("\t");
  Serial.print(" AZ:");Serial.print(az); Serial.print(" m/s2"); Serial.println("\t");
  Serial.println("  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -");
  
}

void digitalClockDisplay()
{
    // digital clock display of the time
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(' ');
    Serial.print(day());
    Serial.print(' ');
    Serial.print(month());
    Serial.print(' ');
    Serial.print(year());
    Serial.println();
}

void printDigits(int digits)
{
    // utility function for digital clock display: prints preceding colon and leading 0
    Serial.print(':');
    if(digits < 10)
        Serial.print('0');
    Serial.print(digits);
}

  /*
  Wire.write(zero);
  Wire.endTransmission();
  Wire.requestFrom(DS1307_ADDRESS, 7);
  int segundos = ConverteparaDecimal(Wire.read());
  int minutos = ConverteparaDecimal(Wire.read());
  int horas = ConverteparaDecimal(Wire.read() & 0b111111);
  int diadasemana = ConverteparaDecimal(Wire.read()); 
  int diadomes = ConverteparaDecimal(Wire.read());
  int mes = ConverteparaDecimal(Wire.read());
  int ano = ConverteparaDecimal(Wire.read());
*/
/*
byte ConverteParaBCD(byte val)
{ 
  //Converte o número de decimal para BCD
  return ( (val/10*16) + (val%10) );
}

byte ConverteparaDecimal(byte val)  
{ 
  //Converte de BCD para decimal
  return ( (val/16*10) + (val%16) );
} 
*/ 
