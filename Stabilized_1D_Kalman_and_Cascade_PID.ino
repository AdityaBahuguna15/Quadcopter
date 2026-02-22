#include <Wire.h>
float RateRoll, RatePitch, RateYaw;
float RateCalRoll, RateCalPitch, RateCalYaw;
int RateCalNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

#include <PulsePosition.h>
PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber=0;
float Voltage;

uint32_t LoopTimer;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

//PID VARIABLES:
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0, 0, 0};
float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;

//For Cascade Controller (Outer PID loop):
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll=2; float PAnglePitch=PAngleRoll;
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0; float DAnglePitch=DAngleRoll;

//KALMAN FILTER VARIABLES:
float KalAngRoll = 0, KalUncAngRoll = 2*2;   // Predicted and Uncertainty (Roll)
float KalAngPitch = 0, KalUncAngPitch = 2*2; // Predicted and Uncertainty (Pitch)
float Kal1DOutput[] = {0,0};                 // Output of Filter

void KalmanFilter1D(float S, float P, float I, float M){
  // S = State vector (Kalman Angle), P = Prediction Uncertainty (Uncert angle)
  // I = Input variable (Rate), M = Kalman Measurement (Measurement vector = angle)
  float F = 1;                     // State Transition Matrix
  float G = 0.004;                 // Control Matrix = Time of 1 iteration (0.004s = 250Hz)
  float Q = 0.004 * 0.004 * 4 * 4; // Process Uncertainty = Standard deviation of rotation rate measurement error is 4 deg/s (An estimation)
  float R = 3 * 3;                 // Measurement Uncertainity
  float H = 1;                     // Mapping Matrix (1)
  
  S = S * F + G * I;
  P = F * P + Q;
  float L = H * P + R;
  float K = P * 1/(L);             // Kalman Gain
  
  // Update step
  S = S + K * (M - (H * S));
  P = (1 - (K * F)) * P;

  Kal1DOutput[0] = S;
  Kal1DOutput[1] = P;
}

void PID(float Error, float P, float I, float D, float PrevError, float PrevIterm){
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004/2;
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;
  float Dterm = D * (Error - PrevError)/0.004;

  float PIDoutput = Pterm + Iterm + Dterm;
  if (PIDoutput > 400) PIDoutput = 400;
  else if (PIDoutput < -400) PIDoutput = -400;

  PIDReturn[0] = PIDoutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void PIDreset(void){
  PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;
  //For Outer PID loop:
  PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
}

// Receiver Function
void read_receiver(void){
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
    for (int i=1; i<=ChannelNumber;i++){
      ReceiverValue[i-1]=ReceiverInput.read(i);
    }
  }
}

void gyro_signals(void){
  // Low Pass Filter Configuration
  Wire.beginTransmission(0x68); // Start I2C Comm
  Wire.write(0x1A); // Register for Low Pass Filter
  Wire.write(0x05); // Cutoff Freq of 10Hz
  Wire.endTransmission();

  // Accelerometer Configuration
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // Accelerometer Config register
  Wire.write(0x10); // Full Scale range: 8g, LSB Sensitivity of 4096 LSB/g
  Wire.endTransmission();

  // Accelerometer measurements
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Start writing to address to indicate first register. Stores recent gyro measurments
  Wire.endTransmission();

  // Request 6 bytes from MPU-6050 so u can pull info of 6 registers 43-48 (2 for each axis (8 bits each)) from sensor
  Wire.requestFrom(0x68,6);
  
  // Measurement of accel of each axis is spread over 2 registers, each 8 bits, merge this info by calling Wire.read() twice
  int16_t AccXLSB = Wire.read()<<8 | Wire.read();
  int16_t AccYLSB = Wire.read()<<8 | Wire.read();
  int16_t AccZLSB = Wire.read()<<8 | Wire.read();

  // Gyroscope Configuration: Sensitivity Scale Factor
  Wire.beginTransmission(0x68); 
  Wire.write(0x1B); 
  Wire.write(0x08); // LSB Sensitivity of 65.5 LSB/°/s
  Wire.endTransmission();

  // Gyroscope Measurements
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Start writing to first register we use. Stores recent gyro measurments
  Wire.endTransmission();

  // Request 6 bytes from MPU-6050 so u can pull info of 6 registers 43-48 (2 for each axis (8 bits each)) from sensor
  Wire.requestFrom(0x68,6); 

  // Measurement of rot rate of each axis is spread over 2 registers, each 8 bits, merge this info by calling Wire.read() twice
  int16_t GyroX = Wire.read()<<8 | Wire.read();
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();

  // Gyro measurements expressed in LSB, convert to °/s
  RateRoll = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;

  // Acc measurements expressed in LSB, convert to g (Calibration values added after)
  AccX = (float)AccXLSB/4096 + 0.01;
  AccY = (float)AccYLSB/4096 - 0.03;
  AccZ = (float)AccZLSB/4096 + 0.04;

  // Absolute angles
  AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))*1/(3.142/180);
  AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))*1/(3.142/180);
}

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Wire.setClock(400000); // Supports comm with all registers using I2C upto 400kHz
  Wire.begin();
  delay(250); // Gives MPU time to start

  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Power management register
  Wire.write(0x00); // Set all bits to 0 to start device and continue in power mode
  Wire.endTransmission();

  // Perform Gyro Calibration (Wait 2 Seconds)
  for(RateCalNumber=0; RateCalNumber<2000; RateCalNumber++){
    gyro_signals();
    RateCalRoll += RateRoll;
    RateCalPitch += RatePitch;
    RateCalYaw += RateYaw;
    delay(1);
  }
  RateCalRoll /= 2000;
  RateCalPitch /= 2000;
  RateCalYaw /= 2000;

  //Set PWM frequency to 250Hz and resolution to 12 bit for all motors
  analogWriteFrequency(1, 250);
  analogWriteFrequency(2, 250);
  analogWriteFrequency(3, 250);
  analogWriteFrequency(4, 250);
  analogWriteResolution(12);

  //Avoid accidental liftoff
  ReceiverInput.begin(14);
  while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050) {
    read_receiver();
    delay(4);
  }
  //Start timer
  LoopTimer = micros();
}

void loop() {
  gyro_signals();
  RateRoll -= RateCalRoll;
  RatePitch -= RateCalPitch;
  RateYaw -= RateCalYaw;

//Kalman Filter
  KalmanFilter1D(KalAngRoll, KalUncAngRoll, RateRoll, AngleRoll);
  KalAngRoll = Kal1DOutput[0];
  KalUncAngRoll = Kal1DOutput[1];
  KalmanFilter1D(KalAngPitch, KalUncAngPitch, RatePitch, AnglePitch);
  KalAngPitch = Kal1DOutput[0];
  KalUncAngPitch = Kal1DOutput[1];

  read_receiver();

//Choose Stable or Rate Control:

//For Stable Control{
  //Calculate Desired Angles (Controller):
  DesiredAngleRoll=0.10*(ReceiverValue[0]-1500);
  DesiredAnglePitch=0.10*(ReceiverValue[1]-1500);
  InputThrottle=ReceiverValue[2];
  DesiredRateYaw=0.15*(ReceiverValue[3]-1500);
//}

//For Rate Control{
  /* For Roll Pitch and Yaw, take desired rate values from reciever, and directly use these desired rate values
     from the reciever to find the error between this and gyroscope values and then use inner PID to get motorinputs*/
  //Caluclate Desired Rates (Controller):
  /*
  DesiredRateRoll = 0.15 * (ReceiverValue[0] - 1500);
  DesiredRatePitch = 0.15 * (ReceiverValue[1] - 1500);
  InputThrottle = ReceiverValue[2];
  DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500); */
//}

//For Stable Control{
  /* For Roll and Pitch, we take Desired angles from reciever, get the error between it and Kalman values
     and using that Error in angle, use outer PID to get desired rate and then find error btwn that
     desired rate and gyroscope reading thus using inner PID to get motorinputs*/
  //Difference between desired and actual angles
  ErrorAngleRoll = DesiredAngleRoll - KalAngRoll;
  ErrorAnglePitch = DesiredAnglePitch - KalAngPitch;

  //Calculate the desired angles through outer PID loop
  PID(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll=PIDReturn[0];
  PrevErrorAngleRoll=PIDReturn[1];
  PrevItermAngleRoll=PIDReturn[2];
  
  PID(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch=PIDReturn[0];
  PrevErrorAnglePitch=PIDReturn[1];
  PrevItermAnglePitch=PIDReturn[2];
//}

  //Error for PID between Controller and Gyroscope reading:
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  PID(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  PID(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  PID(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  if (InputThrottle > 1800) InputThrottle = 1800; // Limit throttle to 80%

  //Quadcopter Dynamics:
  MotorInput1= 1.024*(InputThrottle - InputRoll - InputPitch - InputYaw);
  MotorInput2= 1.024*(InputThrottle - InputRoll + InputPitch + InputYaw);
  MotorInput3= 1.024*(InputThrottle + InputRoll + InputPitch - InputYaw);
  MotorInput4= 1.024*(InputThrottle + InputRoll - InputPitch + InputYaw);

  //Limit max power commands:
  if (MotorInput1 > 2000)MotorInput1 = 1999;
  if (MotorInput2 > 2000)MotorInput2 = 1999;
  if (MotorInput3 > 2000)MotorInput3 = 1999;
  if (MotorInput4 > 2000)MotorInput4 = 1999;

  //Keep motors running at minimum % during flight:
  int ThrottleIdle=1180; // Currently set at 18%
  if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

  // To turn off motors:
  int ThrottleCutOff=1000;
  if (ReceiverValue[2]<1050) {
    MotorInput1=ThrottleCutOff;
    MotorInput2=ThrottleCutOff;
    MotorInput3=ThrottleCutOff;
    MotorInput4=ThrottleCutOff;
    PIDreset();
  }
  analogWrite(1,MotorInput1);
  analogWrite(2,MotorInput2);
  analogWrite(3,MotorInput3);
  analogWrite(4,MotorInput4);

  // Wait 4000us or 0.004s, Reset timer to actual time once condition is met, thus a 250Hz control loop
  while(micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
