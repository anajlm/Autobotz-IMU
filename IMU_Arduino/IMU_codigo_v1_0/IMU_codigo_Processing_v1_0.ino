
#include "Wire.h"
#include "I2Cdev.h"
#include "helper_3dmath.h"
//#include "MPU9150_9Axis_MotionApps41.h" Essa biblioteca não está compilando
#include "MPU9150.h"     //Incluidas as bibliotecas necessárias

MPU9150 pt_acel_giro_mag;       //Cria uma variavel que podera acessar as funções da biblioteca MPU9150 e também o Mpu
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;             //Cria as variáveis que recebera os valores das medições (poderá ser substituido depois por um vetor para facilitar as contas)

int16_t b_ax, b_ay, b_az;
int16_t b_gx, b_gy, b_gz;
int16_t b_mx, b_my, b_mz;

float last_x_angle=0;
float last_y_angle=0;
float last_z_angle=0;

unsigned long t_last;

void calibrar(){
    int8_t x;
    b_ax=0; b_ay=0; b_az=0;
    b_gx=0; b_gy=0; b_gz=0;
    b_mx=0; b_my=0; b_mz=0;
    for(x=0;x<20;x++){
        pt_acel_giro_mag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        b_ax+=ax; b_ay+=ay; b_az+=az;
        b_gx+=gx; b_gy+=gy; b_gz+=gz;
        b_mx+=mx; b_my+=my; b_mz+=mz;
        delay(50);
    }
    b_ax=b_ax/20; b_ay=b_ay/20; b_az=b_az/20;
    b_gx=b_gx/20; b_gy=b_gy/20; b_gz=b_gz/20;
    b_mx=b_mx/20; b_my=b_my/20; b_mz=b_mz/20;               //Faz uma média dos valores quando o sensor está parado, depois esses calculos serão feitos na dmp
                                                            //Esses valores são base, referencia.
}
/*
void usa_dmp_(int16_t ax,int16_t ay,int16_t az){
    int16_t data[3]={ax,ay,az};          //Cira uma memoria para recener os valores
    const uint8_t* packet;
//    pt_acel_giro_mag.dmpGetAccel(data,packet=0);    //Realiza as contas na dmp
    Serial.print("Valores do acelerometro com tratamento");
    Serial.print(data[0]);
    Serial.print("\t");
    Serial.print(data[1]);
    Serial.print("\t");
    Serial.print(data[2]);
    Serial.print("\n");         //Imprime os valorres com alguma conta que a dmp faz
    delay(50);
}
*/     //Usa a dmp que usa a biblioteca que não compila
void calibra_com_offset_acelerometro(){
    b_ax=pt_acel_giro_mag.getXGyroOffset(); b_ay=pt_acel_giro_mag.getYGyroOffset(); b_az=pt_acel_giro_mag.getZGyroOffset();
}


void setup(){
    Wire.begin();                   // Inicia a comunicação com o Mpu
    Serial.begin(115200);             //Inicia a comunicação com a ide do arduino
    pt_acel_giro_mag.initialize();                                      //Inicia o MPU
    Serial.println("Testando conecao");
    Serial.println(pt_acel_giro_mag.testConnection() ? "coneccao com MPU9150 feita" : "Falha a coneccao");
    //calibrar();
    t_last = millis(); 
    
}

void loop(){
    pt_acel_giro_mag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);       //Coleta as medidas do acelerometro, giroscopio e magnetometro
  
  // Converte os valores do gyro para graus/segundos
  float FS_SEL = 131;
  unsigned long t_now = millis(); 
  float gyro_x = (gx - b_gx)/FS_SEL;
  float gyro_y = (gy - b_gy)/FS_SEL;
  float gyro_z = (gz - b_gz)/FS_SEL;
  
  
  // Obtem os valores brutos do acelerometro
  //float G_CONVERT = 16384;
  float accel_x = ax;
  float accel_y = ay;
  float accel_z = az;
  
  // Obtem os valores dos angulos do acelerometro
  float RADIANS_TO_DEGREES = 180/3.14159;
//  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_z = 0;
  
  float dt =(t_now - t_last)/1000.0;
  float gyro_angle_x = gyro_x*dt + last_x_angle;
  float gyro_angle_y = gyro_y*dt + last_y_angle;
  float gyro_angle_z = gyro_z*dt + last_z_angle;
  
  // Aplica o filtro complementar nos valores dos angulos dos dois sensores usados - a escolha
  // do alpha foi estimado. 
  float alpha = 0.96;
  float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  float angle_z = gyro_angle_z;  //Acelerometro nao da o valor do angulo z

  last_x_angle=angle_x;
  last_y_angle=angle_y;
  last_z_angle=angle_z;  
  t_last=t_now;
  
  // Send the data to the serial port
  Serial.print(F("DEL:"));              //Delta T
  Serial.print(dt, DEC);
  Serial.print(F("#ACC:"));              //Angulos dos acelerometros
  Serial.print(accel_angle_x, 2);
  Serial.print(F(","));
  Serial.print(accel_angle_y, 2);
  Serial.print(F(","));
  Serial.print(accel_angle_z, 2);
  Serial.print(F("#GYR:"));
  Serial.print(gyro_angle_x, 2);        //Angulos do Giroscopio
  Serial.print(F(","));
  Serial.print(gyro_angle_y, 2);
  Serial.print(F(","));
  Serial.print(gyro_angle_z, 2);
  Serial.print(F("#FIL:"));             //Angulos filtrados
  Serial.print(angle_x, 2);
  Serial.print(F(","));
  Serial.print(angle_y, 2);
  Serial.print(F(","));
  Serial.print(angle_z, 2);
  Serial.println(F(""));
  
  
    delay(5);
}

