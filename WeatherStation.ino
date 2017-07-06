/**********************************************************************
 * Arduino code decode several Acurite devices OTA data stream.
 * 
 * Decoding protocol and prototype code from these sources:
 * Ray Wang (Rayshobby LLC) 
 *   http://rayshobby.net/?p=8998
 * Benjamin Larsson (RTL_433) 
 *   https://github.com/merbanan/rtl_433
 * Brad Hunting (Acurite_00592TX_sniffer) 
 *   https://github.com/bhunting/Acurite_00592TX_sniffer
 *
 * Written and tested on an Arduino Uno R3 Compatible Board
 * using a RXB6 433Mhz Superheterodyne Wireless Receiver Module
 * 
 * This works with these devices but more could be added
 *   Acurite Pro 5-in-1 (8 bytes)
 *    https://tinyurl.com/zwsl9oj
 *   Acurite Ligtning Detector 06045M  (9 bytes)
 *    https://tinyurl.com/y7of6dq4
 *   Acurite Room Temp and Humidity 06044M (7 bytes)
 *    https://tinyurl.com/yc9fpx8q
 */

// ring buffer size has to be large enough to fit
// data and sync signal, at least 120
// round up to 128 for now
#define RING_BUFFER_SIZE  152

#define SYNC_HIGH       600
#define SYNC_LOW        600

#define PULSE_LONG      400
#define PULSE_SHORT     220

#define PULSE_RANGE     100

//#define BIT1_HIGH       PULSE_LONG
//#define BIT1_LOW        PULSE_SHORT
//#define BIT0_HIGH       PULSE_SHORT
//#define BIT0_LOW        PULSE_LONG

//BIT1
#define BIT1_HIGH_MIN  (PULSE_LONG-PULSE_RANGE)
#define BIT1_HIGH_MAX  (PULSE_LONG+PULSE_RANGE)
#define BIT1_LOW_MIN   (PULSE_SHORT-PULSE_RANGE)
#define BIT1_LOW_MAX   (PULSE_SHORT+PULSE_RANGE)

//BIT0
#define BIT0_HIGH_MIN  (PULSE_SHORT-PULSE_RANGE)
#define BIT0_HIGH_MAX  (PULSE_SHORT+PULSE_RANGE)
#define BIT0_LOW_MIN   (PULSE_LONG-PULSE_RANGE)
#define BIT0_LOW_MAX   (PULSE_LONG+PULSE_RANGE)


// On the arduino connect the data pin, the pin that will be 
// toggling with the incomming data from the RF module, to
// digital pin 3. Pin D3 is interrupt 1 and can be configured
// for interrupt on change, change to high or low.
// The squelch pin in an input to the radio that squelches, or
// blanks the incoming data stream. Use the squelch pin to 
// stop the data stream and prevent interrupts between the 
// data packets if desired.
//
#define DATAPIN         (3)             // D3 is interrupt 1
#define LAPIN           (5)             //logic Analyzer PIN
#define SQUELCHPIN      (4)
#define SYNCPULSECNT    (4)             // 4 pulses (8 edges)
#define SYNCPULSEEDGES  (SYNCPULSECNT*2)

#define DATABYTESCNT_MIN (7) // Minimum number of data bytes
#define DATABITSCNT_MIN     (DATABYTESCNT_MIN*8)// 7 bytes * 8 bits
#define DATABITSEDGES_MIN   (DATABITSCNT_MIN*2)

#define DATABYTESCNT_MID 128 //8 Bytes

#define DATABYTESCNT_MAX (9) // 9 Bytes
#define DATABITSCNT_MAX     (DATABYTESCNT_MAX*8)// 7 bytes * 8 bits
#define DATABITSEDGES_MAX   (DATABITSCNT_MAX*2)

// 5n1 Tower Message Types
#define  MT_WS_WD_RF  49    // wind speed, wind direction, rainfall
#define  MT_WS_T_RH   56    // wind speed, temp, RH

#define eventTimeoutms 7200000  //Timeout in miliseconds before an event is over

// macros from DateTime.h 
/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)  


// The pulse durations are the measured time in micro seconds between
// pulse edges.
unsigned long pulseDurations[RING_BUFFER_SIZE];
unsigned int syncIndex  = 0;    // index of the last bit time of the sync signal
unsigned int dataIndex  = 0;    // index of the first bit time of the data bits (syncIndex+1)
bool         syncFound = false; // true if sync pulses found
bool         received  = false; // true if sync plus enough bits found
unsigned int changeCount = 0;
unsigned int bytesReceived = 0;

int acurite_5n1_raincounter = 0;
int rainWrapOffset = 0;
int lastRainCount = 0;
bool activeRain = false;
unsigned long rainLast = 0;

int strikeTot = 0;
int strikeWrapOffset = 0;
int lastStrikeCount = 0;
bool activeStrikes = false;
unsigned long strikeLast = 0;


//const unsigned int tempOffset = 2.4;  // offset in degrees C
//const unsigned int tempOffset = 0;  // offset in degrees C
//const unsigned int tempOffset10th = 24;  // offset in 10th degrees C
const unsigned int tempOffset10th = 0;  // offset in 10th degrees C

const float winddirections[] = { 315.0, 247.5, 292.5, 270.0, 
                                 337.5, 225.0, 0.0, 202.5,
                                 67.5, 135.0, 90.0, 112.5,
                                 45.0, 157.5, 22.5, 180.0 };
                                 
char * acurite_5n1_winddirection_str[] =
    {"NW",  // 0  315
     "WSW", // 1  247.5
     "WNW", // 2  292.5
     "W",   // 3  270
     "NNW", // 4  337.5
     "SW",  // 5  225
     "N",   // 6  0
     "SSW", // 7  202.5
     "ENE", // 8  67.5
     "SE",  // 9  135
     "E",   // 10 90
     "ESE", // 11 112.5
     "NE",  // 12 45
     "SSE", // 13 157.5
     "NNE", // 14 22.5
     "S"};  // 15 180
 
/*
 * helper code to print formatted hex 
 */
void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex
{
 char tmp[length*2+1];
 byte first;
 int j = 0;
 for (uint8_t i = 0; i < length; i++) 
 {
   first = (data[i] >> 4) | 48;
   if (first > 57) tmp[j] = first + (byte)39;
   else tmp[j] = first ;
   j++;

   first = (data[i] & 0x0F) | 48;
   if (first > 57) tmp[j] = first + (byte)39; 
   else tmp[j] = first;
   j++;
 }
 tmp[length*2] = 0;
 Serial.print(tmp);
}


/*
 * Look for the sync pulse train, 4 high-low pulses of
 * 600 uS high and 600 uS low.
 * idx is index of last captured bit duration.
 * Search backwards 8 times looking for 4 pulses
 * approximately 600 uS long.
 */
bool isSync(unsigned int idx) 
{
   // check if we've received 4 pulses of matching timing
   for( int i = 0; i < SYNCPULSEEDGES; i += 2 )
   {
      unsigned long t1 = pulseDurations[(idx+RING_BUFFER_SIZE-i) % RING_BUFFER_SIZE];
      unsigned long t0 = pulseDurations[(idx+RING_BUFFER_SIZE-i-1) % RING_BUFFER_SIZE];    
      
      // any of the preceeding 8 pulses are out of bounds, short or long,
      // return false, no sync found
      if( t0<(SYNC_HIGH-PULSE_RANGE) || t0>(SYNC_HIGH+PULSE_RANGE) ||
          t1<(SYNC_LOW-PULSE_RANGE)  || t1>(SYNC_LOW+PULSE_RANGE) )
      {
         return false;
      }
   }
   return true;
}

/* Interrupt 1 handler 
 * Tied to pin 3 INT1 of arduino.
 * Set to interrupt on edge (level change) high or low transition.
 * Change the state of the Arduino LED (pin 13) on each interrupt. 
 * This allows scoping pin 13 to see the interrupt / data pulse train.
 */
void handler() 
{
   static unsigned long duration = 0;
   static unsigned long lastTime = 0;
   static unsigned int ringIndex = 0;
   static unsigned int syncCount = 0;
   static unsigned int bitState  = 0;

   bitState = digitalRead(DATAPIN);
   digitalWrite(13, bitState);

   // ignore if we haven't finished processing the previous 
   // received signal in the main loop.
   if( received == true )
   {
      return;
   }

   // calculating timing since last change
   long time = micros();
   duration = time - lastTime;
   lastTime = time;

   // Known error in bit stream is runt/short pulses.
   // If we ever get a really short, or really long, 
   // pulse we know there is an error in the bit stream
   // and should start over.
   if( (duration > (PULSE_LONG+PULSE_RANGE)) || (duration < (PULSE_SHORT-PULSE_RANGE)) )
   {
     //Check to see if we have received a minimum number of bits we could take
     
     if (syncFound  && changeCount >= DATABITSEDGES_MIN)
     {
       if (changeCount >= DATABYTESCNT_MID)
       {
         bytesReceived = 8;
       } else {
         bytesReceived = 7;
       }
       received = true;
       return;

     } else {
      received = false;
      syncFound = false;
      changeCount = 0;  // restart looking for data bits
     }
     

      digitalWrite(LAPIN, LOW);
   }

   // store data in ring buffer
   ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
   pulseDurations[ringIndex] = duration;
   changeCount++; // found another edge

   // detect sync signal
   if( isSync(ringIndex) )
   {
      syncFound = true;
      changeCount = 0;  // restart looking for data bits
      syncIndex = ringIndex;
      dataIndex = (syncIndex + 1)%RING_BUFFER_SIZE;
      digitalWrite(LAPIN, HIGH);
   }

   // If a sync has been found the start looking for the
   // DATABITSEDGES data bit edges.
   if( syncFound )
   {
      // if not enough bits yet, no message received yet
      if( changeCount < DATABITSEDGES_MAX )
      {
        
         received = false;
      }
      else if( changeCount > DATABITSEDGES_MAX )
      {
        // if too many bits received then reset and start over
         received = false;
         syncFound = false;
         digitalWrite(LAPIN, LOW);
      }
      else
      {
         received = true;
         bytesReceived = 9;
         digitalWrite(LAPIN, LOW);
      }
   }
}


void setup()
{
   Serial.begin(9600);
   Serial.println("Started.");
   pinMode(DATAPIN, INPUT);             // data interrupt input
   pinMode(13, OUTPUT);                 // LED output
   pinMode(LAPIN, OUTPUT);
   digitalWrite(LAPIN, LOW);
   attachInterrupt(1, handler, CHANGE);
   pinMode(SQUELCHPIN, OUTPUT);         // data squelch pin on radio module
   digitalWrite(SQUELCHPIN, HIGH);      // UN-squelch data
   
}

/*
 * Convert pulse durations to bits.
 * 
 * 1 bit ~0.4 msec high followed by ~0.2 msec low
 * 0 bit ~0.2 msec high followed by ~0.4 msec low
 */
int convertTimingToBit(unsigned int t0, unsigned int t1) 
{
   if( t0 > (BIT1_HIGH_MIN) && t0 < (BIT1_HIGH_MAX) && t1 > (BIT1_LOW_MIN) && t1 < (BIT1_LOW_MAX) )
   {
      return 1;
   }
   else if( t0 > (BIT0_HIGH_MIN) && t0 < (BIT0_HIGH_MAX) && t1 > (BIT0_LOW_MIN) && t1 < (BIT0_LOW_MAX) )
   {
      return 0;
   }
   return -1;  // undefined
}

/*
 * Validate the CRC value to validate the packet
 *
 *
 */
bool acurite_crc(volatile byte row[]) {
      // sum of first n-1 bytes modulo 256 should equal nth byte
      int sum = 0;

      for (int i = 0; i < bytesReceived -1; i++) {
        sum += row[i];
      }    
      if (sum != 0 && sum % 256 == row[bytesReceived -1]) {
        return true;
      } else {
        
        Serial.print("Expected: ");
        Serial.print(row[bytesReceived-1], HEX);
        Serial.print(" Got: ");
        Serial.println(sum%256, HEX);
        return false;
      }
}

/*
 * Acurite 06045 Lightning sensor Temperature encoding
 * 12 bits of temperature after removing parity and status bits.
 * Message native format appears to be in 1/10 of a degree Fahrenheit
 * Device Specification: -40 to 158 F  / -40 to 70 C
 * Available range given encoding with 12 bits -150.0 F to +259.6 F
 */
float acurite_6045_getTemp (uint8_t highbyte, uint8_t lowbyte) {
    int rawtemp = ((highbyte & 0x1F) << 7) | (lowbyte & 0x7F);
    float temp = (rawtemp - 1500) / 10.0;
    return temp;
}

float acurite_getTemp_6044M(byte hibyte, byte lobyte) {
  // range -40 to 158 F
  int highbits = (hibyte & 0x0F) << 7;
  int lowbits = lobyte & 0x7F;
  int rawtemp = highbits | lowbits;
  float temp = (rawtemp / 10.0) - 100;
  return temp;
}

float convCF(float c) {
  return ((c * 1.8) +32);
}

int acurite_6045_strikeCnt(byte strikeByte)
{
  //int strikeTot = 0;
  //int strikeWrapOffset = 0;
  int strikeCnt = strikeByte & 0x7f;
  if (strikeTot == 0)
  {
    //Initialize Strike Counter
    strikeTot = strikeCnt;
    strikeWrapOffset = 0;
    strikeLast = millis();
    activeStrikes = false;
    return 0;
  } else if (strikeCnt < strikeTot && strikeCnt > 0){
    /*Strikes wrapped around  
     *  Setting strikeTot to 1 as zero would cause a reset, 
     *   Need to make sure strikeCnt isn't 0 so we don't get 
     *   127 false strikes added to our wrap around
     */
    strikeWrapOffset = (127 - strikeTot) + strikeWrapOffset;
    strikeTot = 1;
    strikeLast = millis();
    activeStrikes = true;
  } else if ( strikeCnt == strikeTot) {
    if (millis() - strikeLast > eventTimeoutms)    {
      //Reset the Lightning event time its been more than the eventTiemoutms
      strikeTot = strikeCnt;
      strikeWrapOffset = 0;
      activeStrikes = false;
      //strikeLast = millis();
    }
  } else {
    //strike occured increase lastStrike
    strikeLast = millis();
    activeStrikes = true;
  }
  return (strikeCnt - strikeTot) + strikeWrapOffset;
  
}

uint8_t acurite_6045_strikeRange(uint8_t strikeRange)
{
  return strikeRange & 0x1f;
}

uint16_t acurite_txr_getSensorId(uint8_t hibyte, uint8_t lobyte){
    return ((hibyte & 0x3f) << 8) | lobyte;
}

int acurite_5n1_getBatteryLevel(uint8_t byte){
    return (byte & 0x40) >> 6;
}

int acurite_getHumidity (uint8_t byte) {
    // range: 1 to 99 %RH
    int humidity = byte & 0x7F;
    return humidity;
}

float acurite_getWindSpeed_kph (uint8_t highbyte, uint8_t lowbyte) {
    // range: 0 to 159 kph
    // raw number is cup rotations per 4 seconds
    // http://www.wxforum.net/index.php?topic=27244.0 (found from weewx driver)
	 int highbits = ( highbyte & 0x1F) << 3;
   int lowbits = ( lowbyte & 0x70 ) >> 4;
   int rawspeed = highbits | lowbits;
   float speed_kph = 0;
   if (rawspeed > 0) {
   speed_kph = rawspeed * 0.8278 + 1.0;
    }
    return speed_kph;
}

char * getWindDirection_Descr(byte b) {
  // 16 compass points, ccw from (NNW) to 15 (N), 
        // { "NW", "WSW", "WNW", "W", "NNW", "SW", "N", "SSW",
        //   "ENE", "SE", "E", "ESE", "NE", "SSE", "NNE", "S" };
  int direction = b & 0x0F;
  return acurite_5n1_winddirection_str[direction];
}


float acurite_getTemp_5n1(byte highbyte, byte lowbyte) {
    // range -40 to 158 F
    int highbits = (highbyte & 0x0F) << 7 ;
    int lowbits = lowbyte & 0x7F;
    int rawtemp = highbits | lowbits;
    float temp_F = (rawtemp - 400) / 10.0;
    return temp_F;
}

float acurite_getRainfall(uint8_t hibyte, uint8_t lobyte) {
    // range: 0 to 99.99 in, 0.01 in incr., rolling counter?
	int raincounter = ((hibyte & 0x7f) << 7) | (lobyte & 0x7F);
	if (acurite_5n1_raincounter > 0)
	{
	 if (raincounter < acurite_5n1_raincounter)
	  {
	    rainWrapOffset= lastRainCount - acurite_5n1_raincounter;
	    acurite_5n1_raincounter = 1;
	    rainLast = millis();
	    activeRain = true;
	  } else if (acurite_5n1_raincounter == raincounter){
	    if ((millis() - rainLast) >= eventTimeoutms)  {
	    	    lastRainCount = raincounter;
	    	    rainWrapOffset = 0;
	    	    activeRain = false;
	    	  }  
	  } else {
	    rainLast = millis();
	    activeRain = false;
	  }
	  return (raincounter - acurite_5n1_raincounter + rainWrapOffset) * .01;
	} else {
	  acurite_5n1_raincounter = raincounter;
	  lastRainCount = raincounter;
	  rainLast = millis();
	  activeRain = false;
    return 0.0;
	}
}

String getTimeSpan(unsigned long startMillis, unsigned long endMillis)
{
  String outString;
  long span = (endMillis - startMillis) / 1000;
  outString.concat(numberOfHours(span));
  outString.concat(":");
  outString.concat(numberOfMinutes(span));
  outString.concat(":");
  outString.concat(numberOfSeconds(span));
  return outString;
}

float convKphMph(float kph) {
  return kph * 0.62137;
}

void decode_5n1(byte dataBytes[])
{
    Serial.print("Acurite 5n1 Tower - ");
    Serial.print(acurite_txr_getSensorId(dataBytes[0], dataBytes[1]), HEX);
    Serial.print("; Windspeed - ");
    Serial.print(convKphMph(acurite_getWindSpeed_kph(dataBytes[3], dataBytes[4])));
  if ((dataBytes[2] & 0x3F) == MT_WS_WD_RF)
  {
    // Wind Speed, Direction and Rainfall
    Serial.print("; Direction - ");
    Serial.print(getWindDirection_Descr(dataBytes[4]));
    Serial.print(" Rainfall - ");
    Serial.print(acurite_getRainfall(dataBytes[5], dataBytes[6]));
    if (activeRain){
    Serial.print("; Last rain event - ");
    Serial.print(getTimeSpan(rainLast, millis()));
    }
    Serial.print(";");
  } else {
    // Wind speed, Temp, Humidity
    Serial.print("; Temp - ");
    Serial.print(acurite_getTemp_5n1(dataBytes[4], dataBytes[5]));
    Serial.print("; Humidity - ");
    Serial.print(acurite_getHumidity(dataBytes[6]));
    Serial.print("%");
  }
  
  if ((dataBytes[4] & 0x20) == 0x20 )
  {
    Serial.print(" Battery Low;");
  }
  Serial.println();
  
}



void decode_Acurite_6044(byte dataBytes[])
{
  Serial.print("Acurite 6044 Tower - ");
  Serial.print(acurite_txr_getSensorId(dataBytes[0], dataBytes[1]), HEX);
  Serial.print("; Temp - ");
  Serial.print(convCF(acurite_getTemp_6044M(dataBytes[4], dataBytes[5])));
  Serial.print("; Humidity - ");
  Serial.print(acurite_getHumidity(dataBytes[3]));
  Serial.print(" %;");
  if ((dataBytes[4] & 0x20) == 0x20 )
  {
    Serial.print("Battery Low;");
  }
  Serial.println();
}

void decode_Acurite_6045(byte dataBytes[])
{
  Serial.print("Acurite 6045 Lightning - ");
  Serial.print(acurite_txr_getSensorId(dataBytes[0], dataBytes[1]), HEX);
  Serial.print("; Temp - ");
  Serial.print(acurite_6045_getTemp (dataBytes[4], dataBytes[5]));
  Serial.print("; Humidity - ");
  Serial.print(acurite_getHumidity(dataBytes[3]));
  Serial.print(" %; Lightning - ");
  if (((dataBytes[7] & 0x40) == 0x40) || activeStrikes)
  {
    if ((dataBytes[7] & 0x20) == 0x20)
    {
      Serial.print("Interference");
    } else {
    Serial.print("Detected; Dist - ");
    Serial.print(acurite_6045_strikeRange(dataBytes[7]));
    Serial.print(" miles; Count - ");
    Serial.print(acurite_6045_strikeCnt(dataBytes[6]));
    Serial.print("; Last Strike -  ");
    Serial.print(getTimeSpan(strikeLast, millis()));
    Serial.print(";");
    }
  } else
  {
    Serial.print("None; ");
  }
  if ((dataBytes[4] & 0x20) == 0x20 )
  {
    Serial.print(" Battery Low;");
  }
  Serial.println();
}



// Print the bit stream for debugging. 
// Generates a lot of chatter, normally disable this.
void displayBitTiming()
{
  unsigned int ringIndex;
  
        Serial.print("syncFound = ");
      Serial.println(syncFound);
      Serial.print("changeCount = ");
      Serial.println(changeCount);
      Serial.print("bytesReceived = ");
      Serial.println(bytesReceived);
      Serial.print("syncIndex = ");
      Serial.println(syncIndex);

      Serial.print("dataIndex = ");
      Serial.println(dataIndex);

      ringIndex = (syncIndex - (SYNCPULSEEDGES-1))%RING_BUFFER_SIZE;

      for( int i = 0; i < (SYNCPULSECNT+(bytesReceived*8)); i++ )
      {
         int bit = convertTimingToBit( pulseDurations[ringIndex%RING_BUFFER_SIZE], 
                                       pulseDurations[(ringIndex+1)%RING_BUFFER_SIZE] );

         Serial.print("bit ");
         Serial.print( i );
         Serial.print(" = ");
         Serial.print(bit);
         Serial.print(" t1 = ");
         Serial.print(pulseDurations[ringIndex%RING_BUFFER_SIZE]);
         Serial.print(" t2 = ");
         Serial.println(pulseDurations[(ringIndex+1)%RING_BUFFER_SIZE]);

         ringIndex += 2;
      }
  
}

/*
 * Main Loop
 * Wait for received to be true, meaning a sync stream plus
 * all of the data bit edges have been found.
 * Convert all of the pulse timings to bits and calculate
 * the results.
 */
void loop()
{
   if( received == true )
   {
      // disable interrupt to avoid new data corrupting the buffer
      detachInterrupt(1);

      // extract temperature value
      unsigned int startIndex, stopIndex, ringIndex;
      unsigned long temperature = 0;
      bool fail = false;

//define DISPLAY_BIT_TIMING 
#ifdef DISPLAY_BIT_TIMING
  displayBitTiming();
#endif // DISPLAY_BIT_TIMING

//Decode to Hex Bytes
      byte dataBytes[bytesReceived];
      fail = false; // reset bit decode error flag

      // clear the data bytes array
      for( int i = 0; i < bytesReceived; i++ )
      {
        dataBytes[i] = 0;
      }
        
      ringIndex = (syncIndex+1)%RING_BUFFER_SIZE;

      for( int i = 0; i < bytesReceived * 8; i++ )
      {
         int bit = convertTimingToBit( pulseDurations[ringIndex%RING_BUFFER_SIZE], 
                                       pulseDurations[(ringIndex+1)%RING_BUFFER_SIZE] );
                                       
         if( bit < 0 )
         {  
            fail = true;
            break;      // exit loop
         }
         else
         {
            dataBytes[i/8] |= bit << (7-(i%8));
         }
         
         ringIndex += 2;
      }

     

// Display the raw data received in hex
#define DISPLAY_DATA_BYTES
#ifdef DISPLAY_DATA_BYTES
 
 if (fail)
      {
         Serial.println("Data Byte Display : Decoding error.");

      } else {
              for( int i = 0; i < bytesReceived; i++ )
              {
                PrintHex8(&dataBytes[i], 1);
                //Serial.print(dataBytes[i], HEX);
                Serial.print(",");
              }

      
//              for( int i = 0; i < bytesReceived; i++ )
//              {
//                Serial.print(dataBytes[i], BIN);
//                Serial.print(",");
//              }
//              //Serial.println();
   }

#endif  

if (fail)
  {

  } else if (bytesReceived == 7)  {
    //Small Tower sensor with Temp and Humidity Only
    decode_Acurite_6044(dataBytes);
  } else if (bytesReceived==8) {
    //5n1 tower sensor 
    decode_5n1(dataBytes);
  } else if (bytesReceived==9) {
    //Lightening detector
    decode_Acurite_6045(dataBytes);  
  }       
       // delay for 1 second to avoid repetitions
      delay(1000);
      received = false;
      syncFound = false;

      // re-enable interrupt
      attachInterrupt(1, handler, CHANGE);
   }
}

