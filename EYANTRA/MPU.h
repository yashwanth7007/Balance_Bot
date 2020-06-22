const float pi= 3.142;

const int MPU = 0x68;


int16_t ax, ay, az;
int16_t gx, gy, gz;

float axs, ays, azs;
float gxs, gys, gzs;

float az_oldf,ax_oldf,acc,alpha=0.01,alphaf=0.864,az_init=-2;

float dT=0.005;


void MPU_initialize()
{
  Wire.begin(MPU);                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  Serial.println("done");
}


void MPU_read(){
Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true ); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  ax = (Wire.read() << 8 | Wire.read()) ; // X-axis value
  ay = (Wire.read() << 8 | Wire.read()) ; // Y-axis value
  az = (Wire.read() << 8 | Wire.read()) ; // Z-axis value

   Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  gx = (Wire.read() << 8 | Wire.read()); // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  gy = (Wire.read() << 8 | Wire.read());
  gz = (Wire.read() << 8 | Wire.read());
}




        int i=5;


void complimentary(float &pitch,float &roll, float ax,float ay,float az,float gx,float gy,float gz)
{
        
        
        axs=(ax/32767.0)*2;//Acceleration in x direction scaled
        azs=(az/32767.0)*2;
        //Serial.println(azs);
        gxs=(gx/131.068);
        gys=(gy/131.068);
        

        while(i>0)
        {
          azs=az_init;
          i--;
        }
        
        

        acc=180*atan(axs/abs(azs))/pi;
        pitch=(1- alpha)*(pitch + gys * dT) + (alpha)*(acc);
  
}


