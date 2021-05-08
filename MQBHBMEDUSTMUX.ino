///////////////////////////////////////////////////////////////////////////////////
//16C Analog Multiplexer definitions
#define     S0                          D0
#define     S1                          D1
#define     S2                          D2
#define     S3                          D3
#define     analogpin                   A0
///////////////////////////////////////////////////////////////////////////////////
//Dust sensor Specifications
int         dustPin               =     analogpin;
#define     ledPower                    D5
int         delayTime             =     280;
int         delayTime2            =     40;
float       offTime               =     9680;

int         dustVal               =     0;
int         i                     =     0;
float       ppm                   =     0;
char        s[32];
float       voltage               =     0;
float       dustdensity           =     0;
float       ppmpercf              =     0;
/////////////////////////////////////////////////////////////////////////////////////
//Light sensor specifications
#include    <Wire.h>
#include    <BH1750.h>
BH1750      lightMeter;
////////////////////////////////////////////////////////////////////////////////////
//BME680 sensor specifications
#include    "bsec.h"
#define     BME680_ADDRESS               0x77                    //i2c BME680 
Bsec        iaqSensor;
////////////////////////////////////////////////////////////////////////////////////
//MQ Gas sensors definitions
int         MQ_PIN                =      analogpin;             //MQ Pin designation in calculation
int         RL_VALUE              =      10;                    //load resistance on the board, in kilo ohms
float       Ro;                                                //Ro is extracted from calibratiom
float       GAS[3];                                            //Targeted gas
int         READ_SAMPLE_INTERVAL  =      50;                   //number of samples in normal operation
int         READ_SAMPLE_TIMES     =      5;                    //time interval(in millisecond) between each samples in normal operation
/*****************************Globals***********************************************/
float       LPGCurve[3]           =      {2.3,0.20,-0.46};     //calibration curve of LPG from the MQ2 datasheet, Curve data format:{ x, y, slope}
float       COCurve[3]            =      {2.3,0.72,-0.34};     //calibration curve of CO from the MQ2 datasheet
float       SmokeCurve[3]         =      {2.3,0.53,-0.44};     //calibration curve of Smoke from the MQ2 datasheet
float       CH4Curve[3]           =      {3.3, 0,  -0.38};     //calibration curve of CH4 from the MQ6 datasheet
float       H2Curve[3]            =      {2.3,0.93,-1.44};     //calibration curve of H2 from the MQ8 datasheet
/****************** MQResistanceCalculation ****************************************/
float MQResistanceCalculation(int raw_adc) {
  return (((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
/*****************************  MQRead *********************************************/
float MQRead(int mq_pin) {
  int i;
  float rs=0;
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;
  return rs;  
}
/*****************************  MQGetPercentage **********************************/
int  MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
///////////////////////////////////////////////////////////////////////////////////
void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(analogpin, INPUT);
  pinMode(S0,OUTPUT);
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);
  pinMode(S3,OUTPUT);  

  //Start Dust Sensor
  pinMode(ledPower,OUTPUT);

  //Start Light Sensor
  lightMeter.begin();
  Serial.println("Running Light Sensor");
  delay(100);
  
  //Start BME680 Sensor
  iaqSensor.begin(BME680_ADDRESS, Wire);
  //iaqSensor.setTemperatureOffset(temp_offset);
  Serial.println("\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix));
  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };
  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  delay(100);
}

void loop() {

  {//LIGHT Readings
    Serial.println("Light: " + String(lightMeter.readLightLevel()) + " lx");
  }
  
  {//BME Readings
    iaqSensor.run(); // If new data is available
    Serial.println("Temperature: " + String(iaqSensor.temperature) + " *C");
    Serial.println("Pressure: " + String(iaqSensor.pressure / 100.0) + " hPa");
    Serial.println("Humidity: " + String(iaqSensor.humidity) + " %");
    Serial.println("Static IAQ: " + String(iaqSensor.staticIaq));
    Serial.println("CO2eq: " + String(iaqSensor.co2Equivalent) + " ppm");
    Serial.println("bVOCeq: " + String(iaqSensor.breathVocEquivalent) + " ppm");
  }
  
  digitalWrite(S0,LOW);
  digitalWrite(S1,LOW);
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW); {
    //Serial.print("2#MQ2: "); 
    Ro = 8.60;
    Serial.print( "LPG: " );
    Serial.print(MQGetPercentage(MQRead(MQ_PIN)/Ro,LPGCurve));
    Serial.println( "ppm" );
    Serial.print( "CO: " );
    Serial.print(MQGetPercentage(MQRead(MQ_PIN)/Ro,COCurve));
    Serial.println( "ppm" );
    Serial.print( "Smoke: " );
    Serial.print(MQGetPercentage(MQRead(MQ_PIN)/Ro,SmokeCurve));
    Serial.println( "ppm" ); 
  }
  
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW); {
   //Serial.print("2#MQ6: ");
   Ro = 0.70;
   Serial.print( "CH4: " );
   Serial.print(MQGetPercentage(MQRead(MQ_PIN)/Ro,CH4Curve));
   Serial.println( "ppm" );
  }
  
  digitalWrite(S0,LOW);
  digitalWrite(S1,HIGH);
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW); { 
    //Serial.print("3#MQ8: ");
    Ro = 3.89;
    Serial.print( "Hydrogen: " );
    Serial.print(MQGetPercentage(MQRead(MQ_PIN)/Ro,H2Curve));
    Serial.println( "ppm" );
  }

  digitalWrite(S0,HIGH);
  digitalWrite(S1,HIGH);
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW); {
    //DUST Readings
    i=i+1;
    digitalWrite(ledPower,LOW);                                         // power on the LED
    delayMicroseconds(delayTime);
    dustVal=analogRead(dustPin);                                        // read the dust value
    ppm = ppm + dustVal;
    delayMicroseconds(delayTime2);
    digitalWrite(ledPower,HIGH);                                        // turn the LED off
    delayMicroseconds(offTime);
  
    voltage = ppm/i*0.0049;                                             //0-5V conversion
    dustdensity = 0.17*voltage-0.1;
    ppmpercf = (voltage-0.0256)*120000;                                 //Check here later
    if (ppmpercf < 0)
        ppmpercf = 0;
    if (dustdensity < 0)
        dustdensity = 0;
    if (dustdensity > 0.5)
        dustdensity = 0.5;
      i=0;
      ppm=0;
      Serial.print("Dust Density : ");
      Serial.print(dustdensity*1000);
      Serial.print(" ug/m3, ");
      Serial.print("PPM/ft3 : ");
      Serial.println(ppmpercf);
  }
  
  Serial.println( " " );
  delay(2000);
  
  }
