#include <fp_accel.h>
#include <Wire.h>

const int DEVICE_ADDRESS = 0x1D;

byte fp::readRegister(byte register_address, int numBytes) {
  Wire.beginTransmission (DEVICE_ADDRESS);
  Wire.write (register_address);
  Wire.endTransmission (false);  // repeated start
  Wire.requestFrom(register_address, 1);
  byte c = Wire.read();
  Serial.println(c);
  Serial.print("0x");
  Serial.print(register_address, HEX);
  Serial.print("\t");
  Serial.println(c);
  return c;
}

int fp::writeRegister(byte register_address, byte val) {
  int numBytes;

  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(register_address);
  Wire.write(val);
  Wire.endTransmission();

  return numBytes;
}

fp::fp(void) {
  Wire.begin();

  accel.x = 0;
  accel.y = 0;
  accel.z = 0;
}

void fp::init() {
  writeRegister(fp::CTRL1, 0x57); //sample rate, activate axes
  writeRegister(fp::CTRL2, 0x00); //+/- 2g
  writeRegister(fp::CTRL5, 0x64); //magnet resolution and sample rate
  writeRegister(fp::CTRL6, 0x20); //+/- 4 gauss
  writeRegister(fp::CTRL7, 0x00); //high-pass filter state

  Serial.println(readRegister(0x0F, 1));
}

fp::vector_d fp::accelRaw() {
  fp::vector_d accelRaw;
  
  byte xla = readRegister(fp::OUT_X_L_A, 1);
  byte xha = readRegister(fp::OUT_X_H_A, 1);
  byte yla = readRegister(fp::OUT_Y_L_A, 1);
  byte yha = readRegister(fp::OUT_Y_H_A, 1);
  byte zla = readRegister(fp::OUT_Z_L_A, 1);
  byte zha = readRegister(fp::OUT_Z_H_A, 1);

  accelRaw.x = xla | xha << 8;
  accelRaw.y = yla | yha << 8;
  accelRaw.z = zla | zha << 8;

  Serial.print("xla\t");
  Serial.println(xla);
  Serial.print("xha\t");
  Serial.println(xha);

  Serial.print("accel");
  Serial.print("\t");
  Serial.println(accelRaw.x);

  return accelRaw;
}

fp::vector_d fp::magnRaw() {
  vector_d magnRaw;
 
  byte xlm = readRegister(fp::OUT_X_L_M, 1);
  byte xhm = readRegister(fp::OUT_X_H_M, 1);
  byte ylm = readRegister(fp::OUT_Y_L_M, 1);
  byte yhm = readRegister(fp::OUT_Y_H_M, 1);
  byte zlm = readRegister(fp::OUT_Z_L_M, 1);
  byte zhm = readRegister(fp::OUT_Z_H_M, 1);

  magnRaw.x = xlm | xhm << 8;
  magnRaw.y = ylm | yhm << 8;
  magnRaw.z = zlm | zhm << 8;

  Serial.print("magn");
  Serial.print("\t");
  Serial.println(magnRaw.x);

  return magnRaw;
}

void fp::read() {
  vector_d accelFiltered;
  const double alpha_a = 0.5;
  fp::vector_d rawAccel = accelRaw();

  accelFiltered.x = alpha_a * rawAccel.x - (1 - alpha_a) * rawAccel.x;
  accelFiltered.y = alpha_a * rawAccel.y - (1 - alpha_a) * rawAccel.y;
  accelFiltered.z = alpha_a * rawAccel.z - (1 - alpha_a) * rawAccel.z;
  
  fp::accel = accelFiltered;
  
  vector_d magnFiltered;
  const double alpha_m = 0.5;
  fp::vector_d rawMagn = magnRaw();

  magnFiltered.x = alpha_m * rawMagn.x - (1 - alpha_m) * rawMagn.x;
  magnFiltered.y = alpha_m * rawMagn.y - (1 - alpha_m) * rawMagn.y;
  magnFiltered.z = alpha_m * rawMagn.z - (1 - alpha_m) * rawMagn.z;
  
  fp::magn = magnFiltered;
}

fp::vector_d fp::normalize(fp::vector_d a)  {
  vector_d unitVect;
  double magnitude = sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
  unitVect.x = a.x / magnitude;
  unitVect.y = a.y / magnitude;
  unitVect.z = a.z / magnitude;
  return unitVect;
}

template <typename T, typename U> 
fp::vector<T> operator+(fp::vector<T> const& a, fp::vector<U> const& b) {
  fp::vector<T> out;
  out.x = a.x + b.x;
  out.y = a.y + b.y;
  out.z = a.z + b.z;
  return out;
}

template <typename T, typename U>
fp::vector<T> operator-(fp::vector<T> const& a, fp::vector<U> const& b) {
  fp::vector<T> out;
  out.x = a.x - b.x;
  out.y = a.y - b.y;
  out.z = a.z - b.z;
  return out;
}

template <typename T>
fp::vector<T> scalarDivide(fp::vector<T> const& v, double s) {
  fp::vector<T> quotient;
  quotient.x = v.x / s;
  quotient.y = v.y / s;
  quotient.z = v.z / s;
  return quotient;
}

double fp::heading(vector_d reference) {
  vector_d calibratedMagn = 
    scalarDivide(fp::magn - calibration_max - calibration_min, 2.0);

  vector_d north;
  vector_d east;
  double foo = east.x + 1.0;
  east = crossProd(calibratedMagn, fp::accel);
  east = normalize(east);
  north = crossProd(fp::accel, east);
  north = normalize(north);

  double heading = atan2(dotProd(east, reference),
			 dotProd(north, reference)) * 180 / M_PI;
  if (heading < 0)
    heading += 180;

  return heading;
}
