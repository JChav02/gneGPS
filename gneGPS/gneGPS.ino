#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>
#include <TM1637Display.h>

static NMEAGPS  gps;

static gps_fix  fix;

#define CLK1 22
#define DIO1 23

#define CLK2 24
#define DIO2 25

#define CLK3 26
#define DIO3 27

#define CLK4 28
#define DIO4 29

#define CLK5 30
#define DIO5 31

#define CLK6 32
#define DIO6 33

#define CLK7 34
#define DIO7 35

TM1637Display display1(CLK1, DIO1);
TM1637Display display2(CLK2, DIO2);
TM1637Display display3(CLK3, DIO3);
TM1637Display display4(CLK4, DIO4);
TM1637Display display5(CLK5, DIO5);
TM1637Display display6(CLK6, DIO6);
TM1637Display display7(CLK7, DIO7);

const uint8_t N[] = {
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F
};

const uint8_t NNE[] = {
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F,
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F,
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G
};

const uint8_t NE[] = {
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F,
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G
};

const uint8_t ENE[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F,
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G
};

const uint8_t E[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G
};

const uint8_t ESE[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G
};

const uint8_t SOUTHEAST[] = {
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G
};

const uint8_t SSE[] = {
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G
};

const uint8_t S[] = {
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G
};

const uint8_t SSW[] = {
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F
};

const uint8_t SW[] = {
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F
};

const uint8_t WSW[] = {
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F
};

const uint8_t W[] = {
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F
};

const uint8_t WNW[] = {
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F,
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F
};

const uint8_t NW[] = {
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F,
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F
};

const uint8_t NNW[] = {
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F,
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F,
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F
};

const uint8_t STOP[] = {
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
  SEG_D | SEG_E | SEG_F | SEG_G,
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,
  SEG_A | SEG_B | SEG_E | SEG_F | SEG_G
};

const uint8_t NOTIME[]{
  SEG_G,
  SEG_G,
  SEG_G,
  SEG_G
};

uint8_t data[] = { 0x0, 0x0, 0x0, 0x0} ;
uint8_t nodata[] = { 0x002D, 0x002D, 0x002D, 0x002D};

float latitudeValue, longitudeValue, altitudeValue, speedValue, courseValue;
float Lat, Lon, Tlat, Tlon, Operation, Counter, Result, Latval, Lonval;
float prevSpeed, currentSpeed, slope;
int Tct, ledPin = 2;
int tct2 = 0, tct3 = 0, tct4 = 0;

const int buzzer = 36;
bool isAllowed = false, onceTriggered = false, twiceTriggered = false, thriceTriggered = false;


static const int32_t          zone_hours   = -5L; // EST
static const int32_t          zone_minutes =  0L; // usually zero
static const NeoGPS::clock_t  zone_offset  =
                                zone_hours   * NeoGPS::SECONDS_PER_HOUR +
                                zone_minutes * NeoGPS::SECONDS_PER_MINUTE;

void adjustTime( NeoGPS::time_t & dt )
{
  NeoGPS::clock_t seconds = dt;

  seconds += zone_offset;

  dt = seconds;
}

static void doSomeWork()
{
  lightup();
  print_latitude();
  print_longitude();
  print_altitude();
  print_speed();
  if(fix.valid.time && fix.valid.date){
    adjustTime(fix.dateTime);
    print_time(fix.dateTime);
  }
  print_distance();
  print_course();
  soundmaker();
  print_divider();
}

static void GPSloop()
{
  while (gps.available( gpsPort )) {
    fix = gps.read();
    doSomeWork();
  }
}

static void print_latitude()
{
  latitudeValue = fix.latitude();
  DEBUG_PORT.print("Latitude: ");
  DEBUG_PORT.println(latitudeValue, 6);
  display1.showNumberDec(latitudeValue); // Reminder: Convert to showNumberDecEx later
}

static void print_longitude()
{
  longitudeValue = fix.longitude();
  DEBUG_PORT.print("Longitude: ");
  DEBUG_PORT.println(longitudeValue, 6);
  display2.showNumberDec(longitudeValue);
}

static void print_altitude()
{
  altitudeValue = fix.altitude();
  DEBUG_PORT.print("Altitude: ");
  DEBUG_PORT.println(altitudeValue, 2);
  display3.showNumberDec(altitudeValue);
}

static void print_speed()
{
  speedValue = fix.speed_kph();
  DEBUG_PORT.print("Speed: ");
  DEBUG_PORT.println(speedValue, 2);
  display4.showNumberDec(speedValue);
}

static void print_time(const NeoGPS::time_t & dt)
{
  if(fix.valid.time){
    DEBUG_PORT.print("Hour: ");
    DEBUG_PORT.println(dt.hours);
    DEBUG_PORT.print("Minute: ");
    DEBUG_PORT.println(dt.minutes);
    display5.showNumberDecEx(dt.hours, 0b01000000, false, 2, 0);
    display5.showNumberDec(dt.minutes, true, 2, 2);
  } else {
    display5.setSegments(NOTIME);
  }
}

static void print_distance(){ // NOTA: CORREGIR INCONSISTENCIAS
  if(fix.speed_kph() > 2.5){
    latitudeValue = abs(fix.latitude());
    longitudeValue = abs(fix.longitude());

    if(Tct < 1){
      Latval = latitudeValue;
      Lonval = longitudeValue;
      Tlon = abs(longitudeValue - Lonval);
      Tlat = abs(latitudeValue - Latval); 
      Tct++;
    }
    Tlat = abs(abs(latitudeValue) - abs(Latval));
    Tlon = abs(abs(longitudeValue) - abs(Lonval));
    Operation = (sqrt((Tlat * Tlat) + (Tlon * Tlon)))/1000.0;
    Result = Operation * 111150.0;
    Counter = Counter + Result;

    Latval = latitudeValue;
    Lonval = longitudeValue;
  }

  DEBUG_PORT.print("Distance: ");
  DEBUG_PORT.println(Counter);

  display6.showNumberDec(Counter);
}

static void print_course()
{
  courseValue = fix.heading();
  display7.setSegments(data);
  if(fix.speed_kph() > 2.5){
    if(courseValue > 350.0 || courseValue <= 10.0){
      display7.setSegments(N, 1, 3);
    } else if (courseValue > 10.0 && courseValue <= 35.0){
      display7.setSegments(NNE, 3, 1);
    } else if (courseValue > 35.0 && courseValue <= 55.0){
      display7.setSegments(NE, 2, 2);
    } else if (courseValue > 55.0 && courseValue <= 80.0){
      display7.setSegments(ENE, 3, 1);
    } else if (courseValue > 80.0 && courseValue <= 100.0){
      display7.setSegments(E, 1, 3);
    } else if (courseValue > 100.0 && courseValue <= 125.0){
      display7.setSegments(ESE, 3, 1);
    } else if (courseValue > 125.0 && courseValue <= 145.0){
      display7.setSegments(SOUTHEAST, 2, 2);
    } else if (courseValue > 145.0 && courseValue <= 170.0){
      display7.setSegments(SSE, 3, 1);
    } else if (courseValue > 170.0 && courseValue <=190.0){
      display7.setSegments(S, 1, 3);
    } else if (courseValue > 190.0 && courseValue <= 215.0){
      display7.setSegments(SSW, 3, 1);
    } else if (courseValue > 215.0 && courseValue <= 235.0){
      display7.setSegments(SW, 2, 2);
    } else if (courseValue > 235.0 && courseValue <= 260.0){
      display7.setSegments(WSW, 3, 1);
    } else if (courseValue > 260.0 && courseValue <= 280.0){
      display7.setSegments(W, 1, 3);
    } else if (courseValue > 280.0 && courseValue <= 305.0){
      display7.setSegments(WNW, 3, 1);
    } else if (courseValue > 305.0 && courseValue <= 325.0){
      display7.setSegments(NW, 2, 2);
    } else if (courseValue > 325.0 && courseValue <= 350.0){
      display7.setSegments(NNW, 3, 1);
    } else {
      //
    }
  } else {
    display7.setSegments(STOP);
  }
}

static void lightup()
{
  if (fix.valid.altitude) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}

static void soundmaker(){
  currentSpeed = fix.speed_kph();

  slope = currentSpeed - prevSpeed;

  if(slope > 0.00){
    isAllowed = true;
  } else {
    isAllowed = false;
  }

  if(isAllowed){
    if(fix.speed_kph() >= 30.00 && fix.speed_kph() < 50.00){
      for(tct2; tct2 < 1; tct2++){
        // Buzz, set frequency
        tone(buzzer, 1000);
        delay(100);
        noTone(buzzer);
        tone(buzzer, 1000);
        delay(100);
        noTone(buzzer);
        tct2++;
      }
    } else if(fix.speed_kph() >= 50.00 && fix.speed_kph() < 90.00) {
      for(tct3; tct3 < 1; tct3++){
        tone(buzzer, 1000);
        delay(100);
        noTone(buzzer);
        tone(buzzer, 1000);
        delay(100);
        noTone(buzzer);
        tct3++;
      }
    } else if(fix.speed_kph() >= 90.00){
      for(tct4; tct4 < 1; tct4++){
        tone(buzzer, 1000);
        delay(100);
        noTone(buzzer);
        tone(buzzer, 1000);
        delay(100);
        noTone(buzzer);
        tct4++;
      }
    } else {
      //
    }
  } else if(isAllowed == false && fix.speed_kph() < 30.00){
    tct2 = 0;
  } else if(isAllowed == false && fix.speed_kph() < 50.00 && fix.speed_kph() > 30.00){
    tct3 = 0;
  } else if(isAllowed == false && fix.speed_kph() < 90.00 && fix.speed_kph() > 50.00){
    tct4 = 0;
  } else {
    //
  }

  // Update value
  prevSpeed = fix.speed_kph();
}

static void print_divider()
{
  DEBUG_PORT.println(' ');
}

void setup()
{
  DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT)
    ;

  DEBUG_PORT.print( F("NeoGPS_Prueba1.ino: started\n") );
  DEBUG_PORT.print( F("  Fix object size = ") );
  DEBUG_PORT.println( sizeof(gps.fix()) );
  DEBUG_PORT.print( F("  GPS object size = ") );
  DEBUG_PORT.println( sizeof(gps) );
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );

  #ifndef NMEAGPS_RECOGNIZE_ALL
    #error You must define NMEAGPS_RECOGNIZE_ALL in NMEAGPS_cfg.h!
  #endif

  #ifdef NMEAGPS_INTERRUPT_PROCESSING
    #error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
  #endif

  #if !defined( NMEAGPS_PARSE_GGA ) & !defined( NMEAGPS_PARSE_GLL ) & \
      !defined( NMEAGPS_PARSE_GSA ) & !defined( NMEAGPS_PARSE_GSV ) & \
      !defined( NMEAGPS_PARSE_RMC ) & !defined( NMEAGPS_PARSE_VTG ) & \
      !defined( NMEAGPS_PARSE_ZDA ) & !defined( NMEAGPS_PARSE_GST )

    DEBUG_PORT.println( F("\nWARNING: No NMEA sentences are enabled: no fix data will be displayed.") );

  #else
    if (gps.merging == NMEAGPS::NO_MERGING) {
      DEBUG_PORT.print  ( F("\nWARNING: displaying data from ") );
      DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
      DEBUG_PORT.print  ( F(" sentences ONLY, and only if ") );
      DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
      DEBUG_PORT.println( F(" is enabled.\n"
                            "  Other sentences may be parsed, but their data will not be displayed.") );
    }
  #endif

  DEBUG_PORT.println("GPS found, loading program...");
  DEBUG_PORT.println(' ');

  pinMode(ledPin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  DEBUG_PORT.print  ( F("\nGPS quiet time is assumed to begin after a ") );
  DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
  DEBUG_PORT.println( F(" sentence is received.\n") );

  DEBUG_PORT.flush();

  gpsPort.begin( 9600 );

  display1.setBrightness(0x0f);
  display1.setSegments(data);
  display2.setBrightness(0x0f);
  display2.setSegments(data);
  display3.setBrightness(0x0f);
  display3.setSegments(data);
  display4.setBrightness(0x0f);
  display4.setSegments(data);
  display5.setBrightness(0x0f);
  display5.setSegments(data);
  display6.setBrightness(0x0f);
  display6.setSegments(data);
  display7.setBrightness(0x0f);
  display7.setSegments(data);

  DEBUG_PORT.println("GPS and Displays initialized");
  DEBUG_PORT.println(' ');
  DEBUG_PORT.println("Program start.");
  DEBUG_PORT.println(' ');
}

void loop()
{
  GPSloop();
}
