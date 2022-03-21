#include <Arduino.h>
#define ESP32_CAN_TX_PIN GPIO_NUM_16 // Uncomment this and set right CAN TX pin definition, if you use ESP32 and do not have TX on default IO 16
#define ESP32_CAN_RX_PIN GPIO_NUM_17 // Uncomment this and set right CAN RX pin definition, if you use ESP32 and do not have RX on default IO 4
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>

HardwareSerial Display(1);

#define DEBUG 0
int then;

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void SystemTime(const tN2kMsg &N2kMsg);
void Rudder(const tN2kMsg &N2kMsg);
//void EngineRapid(const tN2kMsg &N2kMsg);
//void EngineDynamicParameters(const tN2kMsg &N2kMsg);
//void TransmissionParameters(const tN2kMsg &N2kMsg);
//void TripFuelConsumption(const tN2kMsg &N2kMsg);
void Speed(const tN2kMsg &N2kMsg);
void WaterDepth(const tN2kMsg &N2kMsg);
//void BinaryStatus(const tN2kMsg &N2kMsg);
//void FluidLevel(const tN2kMsg &N2kMsg);
void OutsideEnvironmental(const tN2kMsg &N2kMsg);
void Temperature(const tN2kMsg &N2kMsg);
void TemperatureExt(const tN2kMsg &N2kMsg);
void DCStatus(const tN2kMsg &N2kMsg);
void BatteryConfigurationStatus(const tN2kMsg &N2kMsg);
void COGSOG(const tN2kMsg &N2kMsg);
void GNSS(const tN2kMsg &N2kMsg);
void LocalOffset(const tN2kMsg &N2kMsg);
void Attitude(const tN2kMsg &N2kMsg);
void Heading(const tN2kMsg &N2kMsg);
void Humidity(const tN2kMsg &N2kMsg);
void Pressure(const tN2kMsg &N2kMsg);
void UserDatumSettings(const tN2kMsg &N2kMsg);
void GNSSSatsInView(const tN2kMsg &N2kMsg);
void Wind(const tN2kMsg &N2kMsg);

void nextionSetValue(String object, int val);

tNMEA2000Handler NMEA2000Handlers[]={
  {126992L,&SystemTime},
  {127245L,&Rudder },
  {127250L,&Heading},
  {127257L,&Attitude},
  //{127488L,&EngineRapid},
  //{127489L,&EngineDynamicParameters},
  //{127493L,&TransmissionParameters},
  //{127497L,&TripFuelConsumption},
  //{127501L,&BinaryStatus},
  //{127505L,&FluidLevel},
  {127506L,&DCStatus},
  {127513L,&BatteryConfigurationStatus},
  {128259L,&Speed},
  {128267L,&WaterDepth},
  {129026L,&COGSOG},
  //{129029L,&GNSS},
  {129033L,&LocalOffset},
  {129045L,&UserDatumSettings},
  //{129540L,&GNSSSatsInView},
  {130310L,&OutsideEnvironmental},
  {130312L,&Temperature},
  {130313L,&Humidity},
  {130314L,&Pressure},
  {130316L,&TemperatureExt},
  {130306L,&Wind},
  {0,0}
};

Stream *OutputStream;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

void setup() {

  // Set Product information
  NMEA2000.SetProductInformation("00000002", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "MFDisplay",  // Manufacturer's Model ID
                                 "0.0.0.1 (2022-03-19)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2022-03-19)", // Manufacturer's Model version
                                 0xff, //load equivalency
                                 0xffff, //nmea2k version
                                 0xff, // cert level
                                 0  // internal device ID
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                130, // Device function=Display. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                120, // Device class=Display. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2731, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                                4, //marine
                                0 //internal device ID
                               );

  Serial.begin(115200);
  OutputStream=&Serial;
  Display.begin(115200, SERIAL_8N1, 35, 32); 
  while(!Display);
  
  while(Display.available()){
    OutputStream->print(Display.read());
  }
  //  NMEA2000.SetN2kCANReceiveFrameBufSize(50);
  // Do not forward bus messages at all
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,22);
  // NMEA2000.SetForwardStream(OutputStream);
  // Set false below, if you do not want to see messages parsed to HEX withing library
  NMEA2000.EnableForward(true);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  //  NMEA2000.SetN2kCANMsgBufSize(2);
  NMEA2000.Open();
  OutputStream->print("Running...");
}

//*****************************************************************************
template<typename T> void PrintLabelValWithConversionCheckUnDef(const char* label, T val, double (*ConvFunc)(double val)=0, bool AddLf=false, int8_t Desim=-1 ) {
  OutputStream->print(label);
  if (!N2kIsNA(val)) {
    if ( Desim<0 ) {
      if (ConvFunc) { OutputStream->print(ConvFunc(val)); } else { OutputStream->print(val); }
    } else {
      if (ConvFunc) { OutputStream->print(ConvFunc(val),Desim); } else { OutputStream->print(val,Desim); }
    }
  } else OutputStream->print("not available");
  if (AddLf) OutputStream->println();
}

//*****************************************************************************
void SystemTime(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    uint16_t SystemDate;
    double SystemTime;
    tN2kTimeSource TimeSource;
    
    if (ParseN2kSystemTime(N2kMsg,SID,SystemDate,SystemTime,TimeSource) ) {
                      OutputStream->println("System time:");
      PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
      PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ",SystemDate,0,true);
      PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ",SystemTime,0,true);
                        OutputStream->print("  time source: "); PrintN2kEnumType(TimeSource,OutputStream);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void Rudder(const tN2kMsg &N2kMsg) {
    unsigned char Instance;
    tN2kRudderDirectionOrder RudderDirectionOrder;
    double RudderPosition;
    double AngleOrder;
    
    if (ParseN2kRudder(N2kMsg,RudderPosition,Instance,RudderDirectionOrder,AngleOrder) ) {
      PrintLabelValWithConversionCheckUnDef("Rudder: ",Instance,0,true);
      PrintLabelValWithConversionCheckUnDef("  position (deg): ",RudderPosition,&RadToDeg,true);
                        OutputStream->print("  direction order: "); PrintN2kEnumType(RudderDirectionOrder,OutputStream);
      PrintLabelValWithConversionCheckUnDef("  angle order (deg): ",AngleOrder,&RadToDeg,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}


//*****************************************************************************
void Heading(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    tN2kHeadingReference HeadingReference;
    double Heading;
    double Deviation;
    double Variation;
    
    if (ParseN2kHeading(N2kMsg,SID,Heading,Deviation,Variation,HeadingReference) ) {
                      OutputStream->println("Heading:");
      PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
                        OutputStream->print("  reference: "); PrintN2kEnumType(HeadingReference,OutputStream);
      PrintLabelValWithConversionCheckUnDef("  Heading (deg): ",Heading,&RadToDeg,true);
      PrintLabelValWithConversionCheckUnDef("  Deviation (deg): ",Deviation,&RadToDeg,true);
      PrintLabelValWithConversionCheckUnDef("  Variation (deg): ",Variation,&RadToDeg,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void COGSOG(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    tN2kHeadingReference HeadingReference;
    double COG;
    double SOG;
    
    if (ParseN2kCOGSOGRapid(N2kMsg,SID,HeadingReference,COG,SOG) ) {
    //                  OutputStream->println("COG/SOG:");
    //  PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
    //                    OutputStream->print("  reference: "); PrintN2kEnumType(HeadingReference,OutputStream);
    //  PrintLabelValWithConversionCheckUnDef("  COG (deg): ",COG,&RadToDeg,true);
    //  PrintLabelValWithConversionCheckUnDef("  SOG (m/s): ",SOG,0,true);
    //} else {
    //  OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    nextionSetValue("Track", RadToDeg(COG));
    }
}

//*****************************************************************************
void GNSS(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    uint16_t DaysSince1970;
    double SecondsSinceMidnight; 
    double Latitude;
    double Longitude;
    double Altitude; 
    tN2kGNSStype GNSStype;
    tN2kGNSSmethod GNSSmethod;
    unsigned char nSatellites;
    double HDOP;
    double PDOP;
    double GeoidalSeparation;
    unsigned char nReferenceStations;
    tN2kGNSStype ReferenceStationType;
    uint16_t ReferenceSationID;
    double AgeOfCorrection;

    if (ParseN2kGNSS(N2kMsg,SID,DaysSince1970,SecondsSinceMidnight,
                  Latitude,Longitude,Altitude,
                  GNSStype,GNSSmethod,
                  nSatellites,HDOP,PDOP,GeoidalSeparation,
                  nReferenceStations,ReferenceStationType,ReferenceSationID,
                  AgeOfCorrection) ) {
                      OutputStream->println("GNSS info:");
      PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
      PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ",DaysSince1970,0,true);
      PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ",SecondsSinceMidnight,0,true);
      PrintLabelValWithConversionCheckUnDef("  latitude: ",Latitude,0,true,9);
      PrintLabelValWithConversionCheckUnDef("  longitude: ",Longitude,0,true,9);
      PrintLabelValWithConversionCheckUnDef("  altitude: (m): ",Altitude,0,true);
                        OutputStream->print("  GNSS type: "); PrintN2kEnumType(GNSStype,OutputStream);
                        OutputStream->print("  GNSS method: "); PrintN2kEnumType(GNSSmethod,OutputStream);
      PrintLabelValWithConversionCheckUnDef("  satellite count: ",nSatellites,0,true);
      PrintLabelValWithConversionCheckUnDef("  HDOP: ",HDOP,0,true);
      PrintLabelValWithConversionCheckUnDef("  PDOP: ",PDOP,0,true);
      PrintLabelValWithConversionCheckUnDef("  geoidal separation: ",GeoidalSeparation,0,true);
      PrintLabelValWithConversionCheckUnDef("  reference stations: ",nReferenceStations,0,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void UserDatumSettings(const tN2kMsg &N2kMsg) {
  if (N2kMsg.PGN!=129045L) return;
  int Index=0;
  double val;

  OutputStream->println("User Datum Settings: ");
  val=N2kMsg.Get4ByteDouble(1e-2,Index);
  PrintLabelValWithConversionCheckUnDef("  delta x (m): ",val,0,true);
  val=N2kMsg.Get4ByteDouble(1e-2,Index);
  PrintLabelValWithConversionCheckUnDef("  delta y (m): ",val,0,true);
  val=N2kMsg.Get4ByteDouble(1e-2,Index);
  PrintLabelValWithConversionCheckUnDef("  delta z (m): ",val,0,true);
  val=N2kMsg.GetFloat(Index);
  PrintLabelValWithConversionCheckUnDef("  rotation in x (deg): ",val,&RadToDeg,true,5);
  val=N2kMsg.GetFloat(Index);
  PrintLabelValWithConversionCheckUnDef("  rotation in y (deg): ",val,&RadToDeg,true,5);
  val=N2kMsg.GetFloat(Index);
  PrintLabelValWithConversionCheckUnDef("  rotation in z (deg): ",val,&RadToDeg,true,5);
  val=N2kMsg.GetFloat(Index);
  PrintLabelValWithConversionCheckUnDef("  scale: ",val,0,true,3);
}

//*****************************************************************************
void GNSSSatsInView(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  tN2kRangeResidualMode Mode;
  uint8_t NumberOfSVs;
  tSatelliteInfo SatelliteInfo;

  if (ParseN2kPGNSatellitesInView(N2kMsg,SID,Mode,NumberOfSVs) ) {
    OutputStream->println("Satellites in view: ");
                      OutputStream->print("  mode: "); OutputStream->println(Mode);
                      OutputStream->print("  number of satellites: ");  OutputStream->println(NumberOfSVs);
    for ( uint8_t i=0; i<NumberOfSVs && ParseN2kPGNSatellitesInView(N2kMsg,i,SatelliteInfo); i++) {
                        OutputStream->print("  Satellite PRN: ");  OutputStream->println(SatelliteInfo.PRN);
      PrintLabelValWithConversionCheckUnDef("    elevation: ",SatelliteInfo.Elevation,&RadToDeg,true,1);
      PrintLabelValWithConversionCheckUnDef("    azimuth:   ",SatelliteInfo.Azimuth,&RadToDeg,true,1);
      PrintLabelValWithConversionCheckUnDef("    SNR:       ",SatelliteInfo.SNR,0,true,1);
      PrintLabelValWithConversionCheckUnDef("    residuals: ",SatelliteInfo.RangeResiduals,0,true,1);
                        OutputStream->print("    status: "); OutputStream->println(SatelliteInfo.UsageStatus);
    }
  }
}

//*****************************************************************************
void LocalOffset(const tN2kMsg &N2kMsg) {
    uint16_t SystemDate;
    double SystemTime;
    int16_t Offset;
    
    if (ParseN2kLocalOffset(N2kMsg,SystemDate,SystemTime,Offset) ) {
                      OutputStream->println("Date,time and local offset: ");
      PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ",SystemDate,0,true);
      PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ",SystemTime,0,true);
      PrintLabelValWithConversionCheckUnDef("  local offset (min): ",Offset,0,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void OutsideEnvironmental(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    double WaterTemperature;
    double OutsideAmbientAirTemperature;
    double AtmosphericPressure;
    
    if (ParseN2kOutsideEnvironmentalParameters(N2kMsg,SID,WaterTemperature,OutsideAmbientAirTemperature,AtmosphericPressure) ) {
      PrintLabelValWithConversionCheckUnDef("Water temp: ",WaterTemperature,&KelvinToC);
      PrintLabelValWithConversionCheckUnDef(", outside ambient temp: ",OutsideAmbientAirTemperature,&KelvinToC);
      PrintLabelValWithConversionCheckUnDef(", pressure: ",AtmosphericPressure,0,true);
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void Temperature(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    unsigned char TempInstance;
    tN2kTempSource TempSource;
    double ActualTemperature;
    double SetTemperature;
    
    if (ParseN2kTemperature(N2kMsg,SID,TempInstance,TempSource,ActualTemperature,SetTemperature) ) {
                        OutputStream->print("Temperature source: "); PrintN2kEnumType(TempSource,OutputStream,false);
      PrintLabelValWithConversionCheckUnDef(", actual temperature: ",ActualTemperature,&KelvinToC);
      PrintLabelValWithConversionCheckUnDef(", set temperature: ",SetTemperature,&KelvinToC,true);
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void Humidity(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    unsigned char Instance;
    tN2kHumiditySource HumiditySource;
    double ActualHumidity,SetHumidity;
    
    if ( ParseN2kHumidity(N2kMsg,SID,Instance,HumiditySource,ActualHumidity,SetHumidity) ) {
                        OutputStream->print("Humidity source: "); PrintN2kEnumType(HumiditySource,OutputStream,false);
      PrintLabelValWithConversionCheckUnDef(", humidity: ",ActualHumidity,0,false);
      PrintLabelValWithConversionCheckUnDef(", set humidity: ",SetHumidity,0,true);
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void Pressure(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    unsigned char Instance;
    tN2kPressureSource PressureSource;
    double ActualPressure;
    
    if ( ParseN2kPressure(N2kMsg,SID,Instance,PressureSource,ActualPressure) ) {
                        OutputStream->print("Pressure source: "); PrintN2kEnumType(PressureSource,OutputStream,false);
      PrintLabelValWithConversionCheckUnDef(", pressure: ",ActualPressure,&PascalTomBar,true);
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void TemperatureExt(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    unsigned char TempInstance;
    tN2kTempSource TempSource;
    double ActualTemperature;
    double SetTemperature;
    
    if (ParseN2kTemperatureExt(N2kMsg,SID,TempInstance,TempSource,ActualTemperature,SetTemperature) ) {
                        OutputStream->print("Temperature source: "); PrintN2kEnumType(TempSource,OutputStream,false);
      PrintLabelValWithConversionCheckUnDef(", actual temperature: ",ActualTemperature,&KelvinToC);
      PrintLabelValWithConversionCheckUnDef(", set temperature: ",SetTemperature,&KelvinToC,true);
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void BatteryConfigurationStatus(const tN2kMsg &N2kMsg) {
    unsigned char BatInstance;
    tN2kBatType BatType;
    tN2kBatEqSupport SupportsEqual;
    tN2kBatNomVolt BatNominalVoltage;
    tN2kBatChem BatChemistry;
    double BatCapacity;
    int8_t BatTemperatureCoefficient;
    double PeukertExponent; 
    int8_t ChargeEfficiencyFactor;

    if (ParseN2kBatConf(N2kMsg,BatInstance,BatType,SupportsEqual,BatNominalVoltage,BatChemistry,BatCapacity,BatTemperatureCoefficient,PeukertExponent,ChargeEfficiencyFactor) ) {
      PrintLabelValWithConversionCheckUnDef("Battery instance: ",BatInstance,0,true);
                        OutputStream->print("  - type: "); PrintN2kEnumType(BatType,OutputStream);
                        OutputStream->print("  - support equal.: "); PrintN2kEnumType(SupportsEqual,OutputStream);
                        OutputStream->print("  - nominal voltage: "); PrintN2kEnumType(BatNominalVoltage,OutputStream);
                        OutputStream->print("  - chemistry: "); PrintN2kEnumType(BatChemistry,OutputStream);
      PrintLabelValWithConversionCheckUnDef("  - capacity (Ah): ",BatCapacity,&CoulombToAh,true);
      PrintLabelValWithConversionCheckUnDef("  - temperature coefficient (%): ",BatTemperatureCoefficient,0,true);
      PrintLabelValWithConversionCheckUnDef("  - peukert exponent: ",PeukertExponent,0,true);
      PrintLabelValWithConversionCheckUnDef("  - charge efficiency factor (%): ",ChargeEfficiencyFactor,0,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void DCStatus(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    unsigned char DCInstance;
    tN2kDCType DCType;
    unsigned char StateOfCharge;
    unsigned char StateOfHealth;
    double TimeRemaining;
    double RippleVoltage;
    double Capacity;

    if (ParseN2kDCStatus(N2kMsg,SID,DCInstance,DCType,StateOfCharge,StateOfHealth,TimeRemaining,RippleVoltage,Capacity) ) {
      OutputStream->print("DC instance: ");
      OutputStream->println(DCInstance);
      OutputStream->print("  - type: "); PrintN2kEnumType(DCType,OutputStream);
      OutputStream->print("  - state of charge (%): "); OutputStream->println(StateOfCharge);
      OutputStream->print("  - state of health (%): "); OutputStream->println(StateOfHealth);
      OutputStream->print("  - time remaining (h): "); OutputStream->println(TimeRemaining/60);
      OutputStream->print("  - ripple voltage: "); OutputStream->println(RippleVoltage);
      OutputStream->print("  - capacity: "); OutputStream->println(Capacity);
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void Speed(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    double SOW;
    double SOG;
    tN2kSpeedWaterReferenceType SWRT;

    if (ParseN2kBoatSpeed(N2kMsg,SID,SOW,SOG,SWRT) ) {
      OutputStream->print("Boat speed:");
      nextionSetValue("SpeedB",(int)(msToKnots(SOW)*10));
      //PrintLabelValWithConversionCheckUnDef(" SOW:",N2kIsNA(SOW)?SOW:msToKnots(SOW));
      //PrintLabelValWithConversionCheckUnDef(", SOG:",N2kIsNA(SOG)?SOG:msToKnots(SOG));
      //OutputStream->print(", ");
      //PrintN2kEnumType(SWRT,OutputStream,true);
    }
}

//*****************************************************************************
void WaterDepth(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    double DepthBelowTransducer;
    double Offset;

    if (ParseN2kWaterDepth(N2kMsg,SID,DepthBelowTransducer,Offset) ) {
      nextionSetValue("Depth",(int)(DepthBelowTransducer+Offset)*10);
    }
}

void Wind(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double WindSpeed, WindAngle;
  tN2kWindReference WindReference;
  if(ParseN2kPGN130306(N2kMsg,SID,WindSpeed,WindAngle,WindReference)){
    nextionSetValue("WindA", (int)RadToDeg(WindAngle));
    nextionSetValue("SpeedW",(int)(msToKnots(WindSpeed)*10));
    #if DEBUG == 1
    OutputStream->println("===>Wind update<===");
    OutputStream->printf("    Wrote WindSpeed %i\n          WindAngle %i\n",(int)msToKnots(WindSpeed),(int)RadToDeg(WindAngle));  
    #endif
    }
    
}

//*****************************************************************************
void printLLNumber(Stream *OutputStream, unsigned long long n, uint8_t base=10){
  unsigned char buf[16 * sizeof(long)]; // Assumes 8-bit chars.
  unsigned long long i = 0;

  if (n == 0) {
    OutputStream->print('0');
    return;
  }

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    OutputStream->print((char) (buf[i - 1] < 10 ?
      '0' + buf[i - 1] :
      'A' + buf[i - 1] - 10));
}

//*****************************************************************************
void Attitude(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    double Yaw;
    double Pitch;
    double Roll;
    
    if (ParseN2kAttitude(N2kMsg,SID,Yaw,Pitch,Roll) ) {
                      OutputStream->println("Attitude:");
      PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
      PrintLabelValWithConversionCheckUnDef("  Yaw (deg): ",Yaw,&RadToDeg,true);
      PrintLabelValWithConversionCheckUnDef("  Pitch (deg): ",Pitch,&RadToDeg,true);
      PrintLabelValWithConversionCheckUnDef("  Roll (deg): ",Roll,&RadToDeg,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  
  // Find handler
  //OutputStream->print("In Main Handler: "); OutputStream->println(N2kMsg.PGN);
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}

//*****************************************************************************

void nextionSetValue(String object, int val){
  String cmd;
  cmd=object+"="+String(val);
  nextionSendCmd(cmd);
}

void nextionSendCmd(String cmd){
  Display.print(cmd);
  Display.write("\xFF\xFF\xFF");
  #if DEBUG == 1
    if(cmd.length() > 0)
    {
      Serial.print("Sending command : ");
      Serial.println(cmd);
    } else
    {
      Serial.println("Empty command issued to clear the buffer"); 
    }
  #endif
}

void loop() 
{ 
  NMEA2000.ParseMessages();
  if (millis()-then>1000){
    then=millis();
    OutputStream->println("tick");
    }
}
