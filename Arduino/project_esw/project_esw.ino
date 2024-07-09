#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>
#include "model.h"
#include <Adafruit_MPU6050.h> //#include <Adafruit_ADXL345_U.h>

Adafruit_MPU6050 mpu; //Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

#define N_INPUTS  450*12
#define N_OUTPUTS  3
// in future projects you may need to tweak this value: it's a trial and error process
#define TENSOR_ARENA_SIZE 25000

Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf;
float test_data[N_INPUTS] = {0};

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;    

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESW FALL"); //Bluetooth device name
    
      if (!mpu.begin()) {//if(!accel.begin()) while(1); //if not intialised, code will not go further ahead
    SerialBT.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  } 
    
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G); //accel.setRange(ADXL345_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG); //8.73 is rad/sec of 500 degree/sec
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    delay(4000);
    tf.begin(model_data);
    if (!tf.isOk()) {
        Serial.print("ERROR: ");
        Serial.println(tf.getErrorMessage());
        
        while (true) delay(1000);
    }
    SerialBT.print("Done");
}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

   for(int i=0; i<(N_INPUTS/6) - 1; i++) 
   {
    for(int j = 0; j <= 5; j++)
      test_data[i*6 + j] = test_data[(i+1)*6 + j];
   }
   test_data[((N_INPUTS/6) - 1)*6 + 0] = a.acceleration.x/9.8;
   test_data[((N_INPUTS/6) - 1)*6 + 1] = a.acceleration.y/9.8;
   test_data[((N_INPUTS/6) - 1)*6 + 2] = a.acceleration.z/9.8;
   test_data[((N_INPUTS/6) - 1)*6 + 3] = g.gyro.x;
   test_data[((N_INPUTS/6) - 1)*6 + 4] = g.gyro.y;
   test_data[((N_INPUTS/6) - 1)*6 + 5] = g.gyro.z;

//    SerialBT.println();
//   for(int j = 0; j <= 5; j++)
//   {
//    SerialBT.println(test_data[1494+j]);
//   }SerialBT.println();
    
   float y_pred[3] = {0}; 
   

   if((test_data[(N_INPUTS/2)]*test_data[(N_INPUTS/2)])+(test_data[(N_INPUTS/2) + 1]*test_data[(N_INPUTS/2) + 1])+(test_data[(N_INPUTS/2)+2]*test_data[(N_INPUTS/2)+2])>20.25)
   {
    uint32_t start = micros();
      tf.predict(test_data,y_pred);
      uint32_t timeit = micros() - start;

   SerialBT.print("It took ");
   SerialBT.print(timeit);
   SerialBT.println(" micros to run inference");
    SerialBT.println((test_data[(N_INPUTS/2)]*test_data[(N_INPUTS/2)])+(test_data[(N_INPUTS/2) + 1]*test_data[(N_INPUTS/2) + 1])+(test_data[(N_INPUTS/2)+2]*test_data[(N_INPUTS/2)+2]));
    for (int i = 0; i < 3; i++) {
        SerialBT.print(y_pred[i]);
        SerialBT.print(i == 2 ? '\n' : ',');
    }
   }
}
