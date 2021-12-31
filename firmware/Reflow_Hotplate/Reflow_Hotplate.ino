//Robu.in
#include <Wire.h> 
#include <math.h>

//IO Pins
const int analogInPin           = A4;       //NTC Therm. sensor connected to A4 pin
const int pwmOutPin             = 3;       //SSR connected to D3 pin
//Functions
double temp_sense();  //Function to measure Temperature in degree C.
int    pid_output();  //Function to calculate the PID value.
double ssr_pwm_out(); //Function to control the SSR with PWM output.

void setup() 
{
  Serial.begin(115200);
}

void loop() 
{
  double Temp_C = 0;
  
  Temp_C = temp_sense();
  Serial.print("\n Temperature in C:\t"); 
  Serial.print(Temp_C);

  ssr_pwm_out();

}

double temp_sense()
{
//Temperature Sensor Calculation Constants
const double BETA               = 3990.00;   //Beta value of Thermister
const double RESISTOR_ROOM_TEMP = 100000.00; //Thermister Resistance at 25 C 
const double ROOM_TEMP          = 298.15;   //room temperature in Kelvin
const double R_bal              = 100000.00;//100k Balance Resister
const int SAMPLE_NUMBER         = 100;       //Total no. of samples to average

//Temperature Sensor Calculation Variables
long int sensorValue  = 0;
double Vadc           = 0.00;
double Rt             = 0.00;
double KTemp          = 0.00;
double CTemp          = 0.00;
int adcSamples[SAMPLE_NUMBER];

  for(int i=0; i<SAMPLE_NUMBER; i++)
  {
    adcSamples[i]= analogRead(analogInPin);
    delay(10); // interval of fetching data from sensor
    sensorValue += adcSamples[i];  
  }
  sensorValue /= SAMPLE_NUMBER;
  //Vadc = (sensorValue * 5.00)/1023;  
  Rt   = (((R_bal*1023.00)/sensorValue) - R_bal);
  KTemp = (BETA * ROOM_TEMP)/(BETA+(ROOM_TEMP * log(Rt/ RESISTOR_ROOM_TEMP)));
  CTemp = KTemp - 273.15;
//  Serial.print("\n Sensor Value:\t"); 
//  Serial.print(sensorValue); //printing value of sensor on the serial monitor
//  Serial.print("\n ADC Voltage:\t"); 
//  Serial.print(Vadc); // printing temperature on the serial monitor
//  Serial.print("\n R Thermister:\t"); 
//  Serial.print(Rt); 
//  Serial.print("\n Temperature in K:\t"); 
//  Serial.print(KTemp);
//  Serial.print("\n Temperature in C:\t"); 
//  Serial.print(CTemp);  
  return CTemp;
}

int pid_output()
{
//PID Constants
int kp = 90;  int ki = 30;  int kd = 80;

//PID Variables

  return 0;
}

double ssr_pwm_out()
{
  int value = 30;
  analogWrite(pwmOutPin, value);  //value can vary from 0 to 255
  //delay(100);

}
