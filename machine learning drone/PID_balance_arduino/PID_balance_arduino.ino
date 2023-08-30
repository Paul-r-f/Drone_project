#include <Wire.h>
#include <Servo.h>


Servo fr_prop;
Servo fl_prop;
Servo br_prop;
Servo bl_prop;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float Total_angle_pt1[2];

float time, timePrev, motorTime, timer;
double elapsedTime;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmFront, pwmBack, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

/////////////////PID CONSTANTS/////////////////
double kp = 0.7; //0.65
double ki = 0.0001;
double kd = 0.02;

int kp_int, ki_int, kd_int;
///////////////////////////////////////////////

double throttle=1150; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady

///////////////////////GA////////////////////////
float fitness;

void setup() {
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  Serial.setTimeout(100);
  fr_prop.attach(5);
  fl_prop.attach(4);
  br_prop.attach(6);
  bl_prop.attach(7);

  time = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/

  fr_prop.writeMicroseconds(1000);
  fl_prop.writeMicroseconds(1000);
  br_prop.writeMicroseconds(1000);
  bl_prop.writeMicroseconds(1000);
  
  delay(2000);

  fr_prop.writeMicroseconds(throttle);
  fl_prop.writeMicroseconds(throttle);
  br_prop.writeMicroseconds(throttle);
  bl_prop.writeMicroseconds(throttle);

  delay(3000); /*Give some delay, 3s, to have time to connect
                *the propellers and let everything start up*/ 
}//end of setup void

void loop() {
  fitness = 0;
  timer = millis();

  while(millis()-timer < 5000) //Test 5 seconds
  {
    /////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = double (time - timePrev) / 1000.0 ;
  

    /*Reed the values that the accelerometre gives.
    * We know that the slave adress for this IMU is 0x68 in
    * hexadecimal. For that in the RequestFrom and the 
    * begin functions we have to put this value.*/
      
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true); 
      
    /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them  
    * and just make then sum of each pair. For that we shift to the left the high values 
    * register (<<) and make an or (|) operation to add the low values.*/
    
    Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
    Acc_rawY=Wire.read()<<8|Wire.read();
    Acc_rawZ=Wire.read()<<8|Wire.read();


    /*///This is the part where you need to calculate the angles using Euler equations///*/
    
    /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
    * values that we have just read by 16384.0 because that is the value that the MPU6050 
    * datasheet gives us.*/
    /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
    * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
    * to calculate this value in each loop we have done that just once before the setup void.
    */

    /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
    *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
    *  will calculate the rooth square.*/
    /*---X---*/
    Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    /*---Y---*/
    Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

    /*Now we read the Gyro data in the same way as the Acc data. The adress for the
      * gyro data starts at 0x43. We can see this adresses if we look at the register map
      * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for 
      * the Z axis (YAW).*/
      
    Wire.beginTransmission(0x68);
    Wire.write(0x43); //Gyro data first adress
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true); //Just 4 registers
      
    Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
    Gyr_rawY=Wire.read()<<8|Wire.read();

    /*Now in order to obtain the gyro data in degrees/second we have to divide first
    the raw value by 131 because that's the value that the datasheet gives us*/

    /*---X---*/
    Gyro_angle[0] = Gyr_rawX/131.0; 
    /*---Y---*/
    Gyro_angle[1] = Gyr_rawY/131.0;

    /*Finnaly we can apply the final filter where we add the acceleration
    *part that afects the angles and ofcourse multiply by 0.98 */

    /*---X axis angle---*/
    Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
    /*---Y axis angle---*/
    Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

    Total_angle_pt1[0] = Total_angle_pt1[0] * 0.8 + Total_angle[0] * 0.2;
    Serial.println(Total_angle_pt1[0]);
    //Serial.println(Total_angle_pt1[0]);

    //Serial.print(Total_angle[0]);
    //Serial.print(";  ");
    //Serial.println(Total_angle_pt1[0]);
    
    /*Serial.print(Total_angle[0]);
    Serial.print(", ");
    Serial.println(Total_angle[1]);*/
    /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
      //Serial.println(Total_angle[1]);

    
      
    /*///////////////////////////P I D///////////////////////////////////*/
    /*Remember that for the balance we will use just one axis. I've choose the x angle
    to implement the PID with. That means that the x axis of the IMU has to be paralel to
    the balance*/

    /*First calculate the error between the desired angle and 
    *the real measured angle*/
    error = Total_angle_pt1[0] - desired_angle;
    
        
    /*Next the proportional value of the PID is just a proportional constant
    *multiplied by the error*/

    pid_p = kp*error;

    /*To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point*/

    pid_i = pid_i+(ki*error);  


    /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time. 
    Finnaly we multiply the result by the derivate constant*/

    pid_d = kd*((error - previous_error)/elapsedTime);

    /*The final PID values is the sum of each of this 3 parts*/
    PID = pid_p + pid_i + pid_d;

    if(PID < -50)
    {
      PID=-50;
    }
    if(PID > 50)
    {
      PID=50;
    }

    /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
    pwmFront = throttle + 1.0*(PID);
    pwmBack = throttle - 1.0*PID; 

    //Back
    if(pwmBack < 1100)
    {
      pwmBack= 1100;
    }
    if(pwmBack > 1300)
    {
      pwmBack=1300;
    }
    //Front
    if(pwmFront < 1100)
    {
      pwmFront= 1100;
    }
    if(pwmFront > 1300)
    {
      pwmFront=1300;
    }

    /*Finnaly using the servo function we create the PWM pulses with the calculated
    width for each pulse*/
    //Serial.println(pwmFront);
    //Serial.println(pwmBack);
    if (millis() - motorTime >= 20) //50Hz for motor control
    {
      fr_prop.writeMicroseconds(pwmFront);
      fl_prop.writeMicroseconds(pwmFront);
      br_prop.writeMicroseconds(pwmBack);
      bl_prop.writeMicroseconds(pwmBack);

      motorTime = millis();
      
    }

   /* 
    Serial.print(pwmFront);
    Serial.print(";  ");
    Serial.println(pwmBack);*/

    previous_error = error; //Remember to store the previous error.
  }

}