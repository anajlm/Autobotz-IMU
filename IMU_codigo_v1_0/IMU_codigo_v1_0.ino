
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

float bussola;

unsigned long t_last;

void calibrar(){
    int8_t x;
    b_gx=0; b_gy=0; b_gz=0;
    for(x=0;x<20;x++){
        pt_acel_giro_mag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        b_gx+=gx; b_gy+=gy; b_gz+=gz;
        delay(5);
    }
    b_gx=b_gx/20; b_gy=b_gy/20; b_gz=b_gz/20;               //Faz uma média dos valores quando o sensor está parado, depois esses calculos serão feitos na dmp
                                                            //Esses valores são base, referencia.
}

void setup(){
    Wire.begin();                   // Inicia a comunicação com o Mpu
    Serial.begin(115200);             //Inicia a comunicação com a ide do arduino
    pt_acel_giro_mag.initialize();                                      //Inicia o MPU
    Serial.println("Testando conecao");
    Serial.println(pt_acel_giro_mag.testConnection() ? "coneccao com MPU9150 feita" : "Falha a coneccao");
    calibrar();
    t_last = millis(); 
    
}

void loop(){
    pt_acel_giro_mag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);       //Coleta as medidas do acelerometro, giroscopio e magnetometro
   
  // Converte os valores do gyro para graus/segundos
  float FS_SEL = 131.072;
  unsigned long t_now = millis(); 
  float gyro_x = (gx-b_gx)/FS_SEL;
  float gyro_y = (gy-b_gy)/FS_SEL;
  float gyro_z = (gz-b_gz)/FS_SEL;
  
  float dt =(t_now - t_last)/1050.0;
  float gyro_angle_x = gyro_x*dt + last_x_angle;
  float gyro_angle_y = gyro_y*dt + last_y_angle;
  float gyro_angle_z = gyro_z*dt + last_z_angle;
  
  // Obtem os valores brutos do acelerometro
  float G_CONVERT = 16384; //Pode ser usado para futuras aplicaçoes
  float accel_x = ax;
  float accel_y = ay;
  float accel_z = az;
  // Obtem os valores dos angulos do acelerometro (Pitch e Roll)
  float RADIANS_TO_DEGREES = 180/3.14159;
  float accel_angle_x = atan(accel_y/accel_z)*RADIANS_TO_DEGREES; //Mede a Inclinaçao em relaçao ao eixo x  (Roll)
  float accel_angle_y = -atan(accel_x/accel_z)*RADIANS_TO_DEGREES; //Mede a inclinaçao em relaçao ao eixo y (Pitch)
  float accel_angle_z = 0; // A inclinaçao em relaçao ao eixo z ser medida pelo giroscopio  
  
  //Trata os valores do magnetrometro
  float mag_x=mx;
  float mag_y=my;
  bussola=atan(mag_x/mag_y)*180/3.14159;
  
  
  // Aplica o filtro complementar nos valores dos angulos dos dois sensores usados - a escolha
  // do alpha foi estimado. 
  float alpha = 0.9;
  float roll = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  float pitch = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  float yaw = alpha*gyro_angle_z + (1.0 - alpha)*bussola;  //Acelerometro nao da o valor do angulo z, faz um filtro entre os valores do giroscopio e magnetrometro

  last_x_angle=roll;
  last_y_angle=pitch;
  last_z_angle=gyro_angle_z;  
  t_last=t_now;      //Atualiza os valores referentes o tratamento dos dados do giroscopio
  
  // Imprime os valores
  Serial.print("\nValores do acelerometro ");
  Serial.print(accel_angle_x);
  Serial.print(" , ");
  Serial.print(accel_angle_y);
  Serial.print(" , ");
  Serial.print(accel_angle_z);
  Serial.print("\nValores do girocopio ");
  Serial.print(gyro_angle_x);        //Angulos do Giroscopio
  Serial.print(" , ");
  Serial.print(gyro_angle_y);
  Serial.print(" , ");
  Serial.print(gyro_angle_z);        //Angulos filtrados*/
  Serial.print("\nAngulos filtrados ");
  Serial.print(roll);
  Serial.print(" , ");
  Serial.print(pitch);
  Serial.print(" , ");
  Serial.print(yaw);
  Serial.print("\nBussola do magnetrometro ");
  Serial.print(bussola);
  delay(5);
}

