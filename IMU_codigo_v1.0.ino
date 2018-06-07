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

void calibrar(){
    int8_t x;
    b_ax=0; b_ay=0; b_az=0;
    b_gx=0; b_gy=0; b_gz=0;
    b_mx=0; b_my=0; b_mz=0;
    for(x=0;x<2;x++){
        pt_acel_giro_mag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        b_ax+=ax; b_ay+=ay; b_az+=az;
        b_gx+=gx; b_gy+=gy; b_gz+=gz;
        b_mx+=mx; b_my+=my; b_mz+=mz;
        delay(50);
    }
    b_ax=b_ax/2; b_ay=b_ay/2; b_az=b_az/2;
    b_gx=b_gx/2; b_gy=b_gy/2; b_gz=b_gz/2;
    b_mx=b_mx/2; b_my=b_my/2; b_mz=b_mz/2;               //Faz uma média dos valores quando o sensor está parado, depois esses calculos serão feitos na dmp
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
    Serial.println("Testando conecção");
    Serial.println(pt_acel_giro_mag.testConnection() ? "conecção com MPU9150 feita" : "Falha a conecção");
   // Serial.println(pt_acel_giro_mag.dmpInitialize() ? "dmp inicializada" : "Falha na inicializacao");   //Inicia os valores da dmp e configura os valores de offset
   calibrar();
   Serial.println(b_az);
}

void loop(){
    pt_acel_giro_mag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);       //Coleta as medidas do acelerometro, giroscopio e magnetometro
    /*Serial.print("Valores do acelerometro sem tratamento");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\n");
    delay(100);*/
/*    Serial.print("Valores do giroscopio sem tratamento");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.print(gz);
    Serial.print("\n");
    Serial.print("Valores do magnetometro sem tratamento");
    Serial.print(mx);
    Serial.print("\t");
    Serial.print(my);
    Serial.print("\t");
    Serial.print(mz);
    Serial.print("\n");                                         //Printa os valores brutos dos sensores
    delay(50);
    pt_acel_giro_mag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);*/       //Coleta as medidas do acelerometro, giroscopio e magnetometro
    Serial.println("Valores do acelerometro com tratamento\t");
    Serial.print(ax-b_ax);
    Serial.print("\t");
    Serial.print(ay-b_ay);
    Serial.print("\t");
    //imprime az
    Serial.print(az);
    Serial.print("\t");
    //
    Serial.print(az+b_az);
    Serial.print("\t\t\t");
    /*Serial.print("Valores do giroscopio com tratamento\t");
    Serial.print(gx-b_gx);
    Serial.print("\t");
    Serial.print(gy-b_gy);
    Serial.print("\t");
    Serial.print(gz-b_gz);
    Serial.print("\n");*/
    /*Serial.print("Valores do magnetometro com tratamento\t");
    Serial.print(mx-b_mx);
    Serial.print("\t");
    Serial.print(my-b_my);
    Serial.print("\t");
    Serial.print(mz-b_mz);
    Serial.print("\n"); */                                       //Printa os valores com o tratamento de calibração
    delay(500);
}

