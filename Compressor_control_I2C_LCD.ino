/*
  TO DO: 
       * Change max running time & algorithms to limit running time 
         to N minutes in every last P minutes period. E. g. dont'r 
         run for more than 15 minutes every 20 minutes. This would 
         call for a P-element circular array containing cummulative
         ON times for each minute. Every cycle all P elements would
         be added and a rest period would be started. The rest period
         should be long enough to ensure that the compressor wil 
         not fall in a ON-OFF-ON-OFF cycle because of the time limits
         being marginally met with the rest period.
          
       * Add off switch to the circuit 
       * Test actual pump temperatures and specs; adjust defaults and
         limits.
*/
  
//
// ARDUINO UNO CONNECTIONS
// =======================
//
//     D2   DS18B20 Temperature sensor data (4K7 pullup)
//     D3   CONFIG MODE ON/OFF INTERRUPT (ENCODER BUTTON) (4K7 PULLUP)
//     D4   ENCODER PIN A (consider using debounce circuit)
//     D5   ENCODER PIN B
//     D6   COMPRESSOR PUMP RELAY
//
//     A0 (D14)  PRESSURE TRANSDUCER 
//     
//     D18  I2C SDA
//     D19  I2C SCL
//
#define PIN_ONE_WIRE_BUS   2      // One or more DS18B20 sensors Dallas Library
//
#define PIN_CONFIG_BUTTON  3      // Config mode on-off (rotary encoder pushbutton)
#define PIN_ENCODER_A      4      // Rotary encoder A 
#define PIN_ENCODER_B      5      // Rotary encoder B
//
#define PIN_PUMP_RELAY     6      // Commpressor pump on/off relay 
#define PIN_PRESSURE      A0      // Pressure sensor, analog data

// LOOP DELAY 


unsigned long MAIN_LOOP_DELAY = 10; // SLEEPING TIME BETWEEN LOOPS 


//----------------------------------------------------------------------------------
// EEPROM Configuration
// 
#include <EEPROM.h>

const unsigned long EEPROM_MAGIC_NUMBER = 913572365; // V4 magic 

// Limits
const float  MAX_TARGET_PSI        =  87.0;   // Reject target pressure greater than 5 bar
const float  MAX_PUMP_TEMP         = 120.0;   // Max configurable pump temperature 
const float  MIN_RESTART_DELTA_T   =   3.0;   // Min difference between max temp and restart temp
const float  MAX_RUNNING_TIME      =  7200;   // Never let the pump run for more than 2 h
const float  MIN_RUNNING_TIME      =    20;   // ** Never let the pump run for LESS than 20 sec
const float  MAX_REST_TIME         =  3600;   // Never let the pump rest for more than 1h 
const float  MIN_REST_TIME         =    10;   // ** If ran for too long, rest for at least 1'40"
const float  MAX_SENSOR_PSI_RATING =   800;   // config limits of sensor rating
const float  MIN_SENSOR_PSI_RATING =    50;   // config limits of sensor rating
//
// Defaults
const float  DEFAULT_PSI_MAX       =   200;   // 200 PSI MODEL
const float  DEFAULT_TARGET_PSI    =  30.0;   // Aprox. 2 bar. Set on first startup
const float  DEFAULT_MAX_TEMP      = 110.0;   // *** VALUES FOR TESTING PURPOSES! ***
const float  DEFAULT_RESTART_TEMP  =  90.0;   // *** USE SOMETHING REALISTIC ~100 *** 
// Cheap car tyre pump cannot run continuously for too long 
const int    DEFAULT_MAX_RUNNING_TIME = 360;  // 67 min
const int    DEFAULT_REST_TIME        = 120;  // 2 min

bool EEPROM_valid = false;

struct config_data_s {  // V4
  unsigned long   magic_number;
  float           target_psi;
  float           sensor_psi_max; // V4 - Sensor max psi 
  float           max_pump_temp;
  float           restart_temp;
  unsigned int    max_running_time;
  unsigned int    rest_time;
};

struct config_data_s GL_config_data;

bool loadEEPROMConfig();    // Load configuration from EEPROM (forward)
bool updateEEPROMConfig();  // Save configuration from EEPRO (forward)


//----------------------------------------------------------------------------------
// Configuration interface
//
// 0-Normal operation 
// 1-Configure target pressure
// 2-Configure max temp
// 3-Configure restart temp ( < max temp  )
// 3-Configure max running time
// 4-Configure rest time
//
volatile int GL_config_mode = 0;  
int GL_old_config_mode = 0;

bool GL_config_requested = false;  // Configuration requested
bool GL_config_active = false;     // Configuration in process
bool GL_parameter_changed = false; // One parameters has been modified during configuration
bool GL_config_changed = false;    // One or more parameters chaged -> update EEPROM

unsigned long GL_button_time = 0UL;         // time ms config button was pressed
int GL_last_state_A = 0;                    // last state of ENCODER_A signal
unsigned long GL_last_encoder_change = 0;   // time the encoder knob was last turned

unsigned int GL_button_counter = 0;

void handle_button_int()  {                 // Handle interrupt, toggle config mode
  if ((millis() - GL_button_time) > 300) { 
    if (!GL_config_mode) {
      GL_config_requested = true; // Run->Config
    } else  {
      GL_config_mode = (++GL_config_mode % 6); 
    }
    GL_button_time = millis(); 
    GL_button_counter ++;  
  }  
}

char *GL_MODELABELS[] = {
      "** END CONFIG **",  // NOT USED
      "SET TARGET PSI  ",
      "SENSOR MAX PSI  ",
      "SET MAX TEMP (C)",
      "SET RESTART TEMP",
      "SET MAX TIME (S)",
      "SET REST SECONDS"
};
char INVALID_MODE_LABEL[] = "(INVALID MODE!)";
const int NUM_MODES = sizeof(GL_MODELABELS) / sizeof(char *);

// Encoder interface, prompts user depending on the current GL_config_mode 
// value and updates the corresponondig configuration parameter

int GL_value_set, GL_value_max, GL_value_min, GL_value_old; // Encoder values 

const int MAX_KNOB_INCREMENT = 8;
unsigned int GL_knob_increment = 1; 

void prepare_config_interface(int mode); // Prepare interface and values acording to mode
void read_encoder(int mode);             // Update values


//----------------------------------------------------------------------------------
// I2C LCD Module
//
// WARNING, REMEMBER TO ADJUST THE DISPLAY'S CONTRAST POTENTIOMENTER.
//
//  Nano, UNO
//  =========
//  SDA - D18 
//  SCL - D19
//  
#include <Wire.h>                     // I2C
#include <LiquidCrystal_I2C.h>        // set the LCD address to 0x27 for a 16 chars and 2 line display

LiquidCrystal_I2C lcd(0x3f, 16, 2);     // Use i2c scanner example if necessary
// Forward

void LCD_refresh();

unsigned long GL_last_display_refresh_time = 0;
const unsigned long DISPLAY_REFRESH_TIME = 500;

//
//----------------------------------------------------------------------------------
// DS18B20 Temperature sensors library 
// Requires OneWire library
// All data pins are conected in parallel to digital pin 2 and collectivelly pulled
// up with a 4.7K resistor
//

#include <OneWire.h>
#include <DallasTemperature.h>

// Set up a oneWire instance to communicate with temp sensnor
OneWire oneWire(PIN_ONE_WIRE_BUS);  

// Pass oneWire buses reference to DallasTemperature library
DallasTemperature tempSensor(&oneWire);
int GL_temp_sensor_count = 0;

float GL_current_temperature = 0;

const unsigned long TMP_MEASUREMENT_INTERVAL = 1000;  // 1s 
unsigned long GL_last_tmp_measurement_time = 0; // Use millis() after each read


//
//----------------------------------------------------------------------------------
// EARU PRESSURE TRANSDUCER (models 30, 100, 150, 200, 300 & 500 max psi)
// https://www.aliexpress.com/item/33059909689.html
//
// Al models are 5V Vin
// Transducer output at 0 psi    -  0.5 V
// Transducer output at max psi  -  4.5 V
// 
// ADC Values calculated for 
//
//    ADC offset   =    0  at 0V input
//    ADC reading  = 1024  at 5V input
//
// 
// Pmax   Pmax                                      ADC val   ADC val
//  psi    bar    V/psi    V/bar   psi/V    bar/V   per psi	  per bar     
// ----  -----   ------   ------   -----   ------   -------  --------
//   30   2.07   0.1333   1.9338    7.50   0.5171    0,0293    0,0020
//  100   6.89   0.0400   0.5802   25.00   1.7237    0,0977    0,0067
//  150  10.34   0.0267   0.3868   37.50   2.5855    0,1465    0,0101
//  200  13.79   0.0200   0.2901   50.00   3.4474    0,1953    0,0135
//  300  20.68   0.0133   0.1934   75.00   5.1711    0,2930    0,0202
//  500  34.47   0.0080   0.1160  125.00   8.6185    0,4883    0,0337
//

// ### Using 300 max psi model ###

const float  BAR_PER_PSI       = 0.0689476;
const float  SENSOR_V_AT_ZERO  =       0.5; // output volts at 0 psi
const float  SENSOR_V_AT_MAX   =       4.5; // output volts a max psi
const float  SENSOR_V_PRESENT  =  SENSOR_V_AT_ZERO / 2.0;  // v4. iF LESS THAN HALF THE 0 PSI READING ASSUME  SENSOR N/A
const int    ADC_TICS_0V       =         0; // ADC reading at 0V
const float  ADC_V_MAX         =       5.0; // ADC max input voltage
const int    ADC_MAX_TICS      =      1024; // ADC reading at ADC_V_MAX

const float  ADC_V_PER_TIC = ADC_V_MAX /(ADC_MAX_TICS - ADC_TICS_0V);

const float PRESSURE_MARGIN = 1.10;        // Let pressure oscillate +/- 10% of target

// Translate ADC reading to volts
inline float adc2volt(float adcval) { 
  return (adcval - ADC_TICS_0V) * ADC_V_PER_TIC; 
}

//
// translate ADC reading to PSI
// Unambiguously return -1.0 when no sensor present (very low voltage < SENSOR_V_PRESENT)
// Return 0.0 between low voltage and 0 PSI reading (SENSOR_V_AT_ZERO)
inline float adc2psi(float adcval)  { 
  float psi_per_v = GL_config_data.sensor_psi_max / (SENSOR_V_AT_MAX - SENSOR_V_AT_ZERO);  // UPDATED ON RECONFIGURATION
  float volts = adc2volt(adcval);
  float psi =  psi_per_v * (volts - SENSOR_V_AT_ZERO); 
  if (psi >= 0.0) {
      return psi;
  } else {
    if (volts >= SENSOR_V_PRESENT)  {
      return 0.001;
    } else {
      return -1.0;
    }
  }
}

// Convert PSI to bar
inline float psi2bar(float psi) { 
  return psi * BAR_PER_PSI;  
}

float GL_current_pressure = 0.0;

const unsigned long PSI_MEASUREMENT_INTERVAL = 1000;  // 1s 
unsigned long GL_last_psi_measurement_time = 0; // Use millis() after each read


//
//----------------------------------------------------------------------------------
// PUMP RELAY
//
bool GL_pump_state = false;      // true=pump on, false=pump_off

// if pressure has reached target*factor, wait for it to go down to target/factor 

bool GL_overpressure = false;    // target not reached or already recovered

// If temperature reaches  GL_config_data.max_pump_temp then wait to go down to GL_config_data.restart_temp

bool  GL_cooling = false;        // pump is GL_cooling 

// Cheap car tyre pump cannot run continuously for too long 
bool            GL_rest_mode        = false;  // rest has been requested
unsigned long   GL_started_millis   = 0UL;    // Time the pump was switched on
unsigned long   GL_stopped_millis   = 0UL;    // Time the pump was switched off
unsigned long   GL_rest_start_tme   = 0UL;    // When rest begun

inline void switchPump(bool onoff)      {     // Switch compressor pump on/off
  unsigned long mili = millis();
  if (onoff) {
    GL_started_millis = mili; 
  } else { 
    GL_stopped_millis = millis();
  }
  digitalWrite(PIN_PUMP_RELAY, onoff? HIGH : LOW); GL_pump_state = onoff; 
}


//
//----------------------------------------------------------------------------------
// Initial configuration
//
//
void setDefaultConfig() {
      GL_config_data.magic_number = EEPROM_MAGIC_NUMBER;
      GL_config_data.target_psi = DEFAULT_TARGET_PSI;
      GL_config_data.sensor_psi_max = DEFAULT_PSI_MAX; // Transducer max psi (model used)
      GL_config_data.max_pump_temp = DEFAULT_MAX_TEMP;
      GL_config_data.restart_temp = DEFAULT_RESTART_TEMP;
      GL_config_data.max_running_time = DEFAULT_RESTART_TEMP;
      GL_config_data.rest_time = DEFAULT_RESTART_TEMP;
};


// **********************************************************************************
// SETUP
// **********************************************************************************
void setup() {
  //
  // LED is on while pump is on
  //
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //
  // Serial
  //
  Serial.begin(9600);
  Serial.println("Compressor control unit");
  Serial.println("STARTING");

  //
  // lcd setup
  //
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("STARTING");

  //
  // EEPROM
  //
  loadEEPROMConfig();
  Serial.println("Press CONFIG button to modify configuration");
  
  //
  // DS18B20 TEMPERATURE SENSOR (DALLAs LIBRARY)
  //
  tempSensor.begin();  // Start up the library
  GL_temp_sensor_count = tempSensor.getDeviceCount();
  lcd.setCursor(0,1);
  lcd.print("#OF DS18B20: ");lcd.print(GL_temp_sensor_count);
  Serial.print("FOUND ");Serial.print(GL_temp_sensor_count);Serial.println(" DS18B20 sensors");

  //
  // CONFIG MODE 
  //
  GL_config_mode = 0;
  GL_config_requested = false;
  GL_button_time = millis();
  pinMode(PIN_CONFIG_BUTTON, INPUT_PULLUP); // Remember to pull up the pin w/4K7 resistor  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< CHECK IF INTERNAL PULLUP CAUSES PROBLEMS
  attachInterrupt(digitalPinToInterrupt(PIN_CONFIG_BUTTON), handle_button_int, FALLING);
  
  //  pinMode (PIN_ENCODER_A, INPUT_PULLUP);
  //  pinMode (PIN_ENCODER_B, INPUT_PULLUP);
  pinMode (PIN_ENCODER_A, INPUT); // w/debouncing circuitry
  pinMode (PIN_ENCODER_B, INPUT);
  GL_last_state_A = digitalRead(PIN_ENCODER_A);  // Read the initial state of the ENCODER_A

  //
  // PUMP RELAY 
  // 
  pinMode(PIN_PUMP_RELAY, OUTPUT);
  switchPump(false);

  //
  // END SETUP
  // 
  lcd.clear();
  
  digitalWrite(LED_BUILTIN, LOW);
  lcd.setCursor(0,0);
  Serial.println("END SETUP. PUMP IS OFF");
  lcd.print("END SETUP");
  lcd.setCursor(0,1);
  lcd.print("PUMP IS OFF");
  
  delay(600);

}

// **********************************************************************************
// LOOP
// **********************************************************************************

void loop() {
  //
  // Perform configuration if requested
  //
  if (GL_config_requested) {
    noInterrupts();
    GL_config_mode = 1;
    interrupts();
    GL_old_config_mode = 1;
    lcd.clear();
    switchPump(false);
    GL_config_changed = false;
    GL_config_requested = false; 
    GL_config_active = true;
    Serial.println("CONFIGURATION");
    prepare_config_interface(GL_config_mode); 
  }
  if (GL_config_active) {
    GL_parameter_changed = false;  // Set by read_encoder()
    read_encoder(GL_old_config_mode);
    if (GL_parameter_changed) {
          GL_config_changed = true;
    }
    if (GL_config_mode != GL_old_config_mode) {
      if (GL_config_mode == 0) { // configuration ended
        GL_config_active = false;
        if (GL_config_changed) {
          Serial.println("\nNew configuration:\n");
          displayConfig();
          updateEEPROMConfig();
        } else {
          Serial.println("No changes\n");
        }
      }
      GL_old_config_mode = GL_config_mode;
      prepare_config_interface(GL_old_config_mode);       
    }
    return; // keep looping during configuration
  }

  // 
  // Read temperature
  // 
  if ((millis() - GL_last_tmp_measurement_time) >= TMP_MEASUREMENT_INTERVAL) {
    if (GL_temp_sensor_count > 0) 
    {
      tempSensor.requestTemperatures(); 
      GL_current_temperature = tempSensor.getTempCByIndex(0);
    }
    GL_last_tmp_measurement_time = millis();
  }

  //
  // Read pressure
  // 
  if ((millis() - GL_last_psi_measurement_time) >= PSI_MEASUREMENT_INTERVAL) {
    unsigned int adcval = analogRead(PIN_PRESSURE);
    GL_current_pressure = adc2psi(adcval);
    GL_last_psi_measurement_time = millis();
  }
  
  //
  // Update pump relay
  // Assume pump must be on and handle transitions
  //
  //
  bool prev_state = GL_pump_state;
  GL_pump_state = true; 
  
  float c_p = GL_current_pressure<0.0? 0.000001: GL_current_pressure;
  
  // #define trace_(x) Serial.print(">" x " ")
  #define trace_(x) 
  #define reportChange(msg) { Serial.print("\n"); Serial.print(msg); Serial.print(": "); }

  unsigned int running_s = (millis() - GL_started_millis) / 1000;
  unsigned int rest_elapsed = (millis() - GL_rest_start_tme) /1000;
  
  // --- If no ressure reading, pump off
  if (GL_current_pressure < -0.0001) {                                              trace_("U");
    GL_pump_state = false;
  // --- Check max running time, pause if necessary
  } else if (prev_state && (running_s > GL_config_data.max_running_time)) {         trace_("V");
    GL_rest_mode = true;
    GL_rest_start_tme = millis();
    GL_pump_state = false;
    reportChange("Continuous operation time exceeded");
  } else if (GL_rest_mode) {                                                        trace_("W");
    if (rest_elapsed > GL_config_data.rest_time) {                                  trace_("X");
      reportChange("Rest time complete"); 
      GL_rest_mode = false; // remember new pump state defaults to true
    } else {                                                                        trace_("Y");
      GL_pump_state = false;
    }
  // --- Check pressure vs target PSI (-/+ 5%)
  } else if ( c_p >= (GL_config_data.target_psi * PRESSURE_MARGIN ) )  {            trace_("A");
    if (prev_state) {                                                               trace_("B");
      reportChange("Pressure target reached");
      GL_overpressure = true;
    }
    GL_pump_state = false;
  // --- Still in  GL_overpressure recovery area?
  } else if ( c_p > (GL_config_data.target_psi / PRESSURE_MARGIN ) )  {             trace_("C");
    if (GL_overpressure) {                                                          trace_("D");
        GL_pump_state = false; // lower bound nor reached yet
    } 
  // --- Crossed lower pressure limit after GL_overpressure?
  } else if (GL_overpressure) {                                                     trace_("E");
      // current pressure is under the lower bound, restart  
      reportChange("Restart pressure reached");
      GL_overpressure = false;
  } else {                                                                          trace_("F");
    GL_overpressure = false;
    // --- Pressure is OK, check overheating
    if (GL_current_temperature > GL_config_data.max_pump_temp) {                    trace_("G");
      if (!GL_cooling) reportChange("Pump overheated. COOLING");
      GL_cooling = true;
      GL_pump_state = false;
    // --- Has GL_cooling finished?
    } else if (GL_current_temperature < GL_config_data.restart_temp) {              trace_("H");
      if (GL_cooling) {                                                             trace_("I");
        GL_cooling = false;
        reportChange("Cooling complete");
      }
    } else {                                                                        trace_("J");
      if (GL_cooling) {                                                             trace_("K");
        // No, keep GL_cooling
        GL_pump_state = false;
      }
    }
  }

  if (prev_state != GL_pump_state) {
    Serial.print("Compressor "); Serial.println(GL_pump_state? "ON": "OFF");
    switchPump(GL_pump_state);
  }

  //
  // Light builtin LED acording to pump state
  //
  digitalWrite(LED_BUILTIN, GL_pump_state? HIGH: LOW);

  
  //
  // UPDATE DISPLAY
  //
  if ((millis() - GL_last_display_refresh_time) >= DISPLAY_REFRESH_TIME) {
    LCD_refresh ();
    GL_last_display_refresh_time = millis();
  }

  //
  // Delay
  //
  delay(MAIN_LOOP_DELAY);

}


// **********************************************************************************
// AUXILIARY 
// **********************************************************************************

// Update 12x2 LCD 
// ---------------------------------------------------------------------
// 
//       0123456789012345
//      +----------------+
//     0|PSI 0.0 TMP 99.9|
//     1|set 0.0 pump OFF|
//      +----------------+
//
void LCD_refresh()
{
  char buf[17];      // buffer for max 16 char display
  char fltbuf1[14];  // float buffer for dtostrf() (arduino's snprintf does not handle floats well)
  char fltbuf2[14];
    
  //lcd.clear();
  const char *notavail = " N/A";
  // int w1 = GL_current_pressure > 99.99999 ? 0 : 1;
  int w1 = 0;
  int w2 = GL_current_temperature  > 99.99999 ? 0 : 1;
  dtostrf(GL_current_pressure, 4, w1, fltbuf1);  
  dtostrf(GL_current_temperature,  5, w2, fltbuf2);  
  char *tmpstr = GL_temp_sensor_count < 1?  notavail : fltbuf2;
  char *prestr = GL_current_pressure < 0.0?  notavail : fltbuf1;
  snprintf(buf, sizeof(buf), "PSI%s TMP%s", prestr, tmpstr);     

  lcd.setCursor(0,0);    
  lcd.print(buf);
  Serial.print(buf);  Serial.print("  ");

  w1 = GL_config_data.target_psi > 10 ? 0 : 1;
  
  dtostrf(GL_config_data.target_psi, 4, w1, fltbuf1);  
  char strstate[18];
  if (GL_overpressure) {
    strcpy(strstate, "MAX PSI");
  } else if (GL_cooling) {
    strcpy(strstate, "COOLING");
  } else if (GL_rest_mode) {
    unsigned int rest_elapsed = (millis() - GL_rest_start_tme) / 1000;
    unsigned int rest_remaining = GL_config_data.rest_time - rest_elapsed;
    snprintf(strstate, sizeof(strstate), "WAIT%4d", rest_remaining);
  } else if (GL_pump_state) {
    unsigned int runnig_s = (millis() - GL_started_millis) / 1000;
    if (runnig_s < 9999) {
      snprintf(strstate, sizeof(strstate), "RUN%5d", runnig_s);
    } else {
      snprintf(strstate, sizeof(strstate), "RUN%4dm", runnig_s/60);
    }
  } else {
    strcpy(strstate, "PUMP OFF");
  }
  snprintf(buf, sizeof(buf), "set%s %s ", fltbuf1, strstate);     

  lcd.setCursor(0,1);    
  lcd.print(buf);
  Serial.println(buf);
}

//
// Load configuration from EEPROM
// ---------------------------------------------------------------------
// 
bool loadEEPROMConfig() {
  #if defined(ESP8266) || defined(ESP32)
  #define ESP__GEN
  #endif

  #ifdef ESP__GEN
    Serial.print(" (ESP32 Version)");
    EEPROM.begin(2048);
  #endif
  Serial.print("EEPROM size is "); Serial.print(EEPROM.length()); Serial.println(" bytes\n");
  lcd.setCursor(0,1);
  lcd.print("EEPROM "); lcd.print(EEPROM.length());
  if (EEPROM.length() < sizeof(config_data_s))  {
    lcd.print(" ERR");
    Serial.print("Error. EEPROM too small (need ");Serial.print(sizeof(config_data_s));Serial.println(")");
    Serial.println("Setting configuration to default values. Configuration will not be saved.");
    setDefaultConfig();
    EEPROM_valid = false;
  } else {
    EEPROM_valid = true;
    // read config from EEPROM
    EEPROM.get(0, GL_config_data); // EEPROM.get() is a template based on 2nd arg type, passed as reference
    if (GL_config_data.magic_number != EEPROM_MAGIC_NUMBER) {  
      // EEPROM is not initialized
      setDefaultConfig();
      // Not initialized
      EEPROM.put(0, GL_config_data);
      Serial.println("EEPROM not initilized. Initializing with default values");
    } else {
      Serial.println("Valid configuration found:\n");
    }
  }
  displayConfig();
  return EEPROM_valid;
}


// Update EEPROM configuration
// ---------------------------------------------------------------------
// 
bool updateEEPROMConfig() {
  GL_config_data.magic_number = EEPROM_MAGIC_NUMBER;
  struct config_data_s conf2;
  EEPROM.put(0, GL_config_data);
  EEPROM.get(0, conf2);
  EEPROM_valid = ((GL_config_data.magic_number == conf2.magic_number) && (GL_config_data.target_psi == conf2.target_psi));
  return EEPROM_valid;
}

// Display value while configuring
// ---------------------------------------------------------------------
void show_value(int v) {
  lcd.setCursor(0, 1);
  lcd.print("> ");
  lcd.print(v);  
  lcd.print("     ");
}

// Prepare interface and globals for a new configuration mode
// ---------------------------------------------------------------------
// 
void prepare_config_interface(int mode) {
  int GL_value_old = GL_value_set;
  char *label = ((mode>=0) && (mode < NUM_MODES)) ? GL_MODELABELS[mode]: INVALID_MODE_LABEL;
  switch (mode) {
    case 0:
      break;
    case 1:
      GL_value_set = (int) GL_config_data.target_psi; 
      GL_value_max = MAX_TARGET_PSI;
      GL_value_min = 0;
      break;
    case 2:
      GL_value_set = (int) GL_config_data.sensor_psi_max; 
      GL_value_max = MAX_SENSOR_PSI_RATING;   
      GL_value_min = MIN_SENSOR_PSI_RATING;
      break;
    case 3:
      GL_value_set = (int) GL_config_data.max_pump_temp; 
      GL_value_max = MAX_PUMP_TEMP;
      GL_value_min = MIN_RESTART_DELTA_T;
      break;
    case 4:
      GL_value_set = (int) GL_config_data.restart_temp ; // Use integer psi
      GL_value_max = MAX_PUMP_TEMP - MIN_RESTART_DELTA_T;
      GL_value_min = 0;
      break;
    case 5:
      GL_value_set = (int) GL_config_data.max_running_time; // Use integer psi
      GL_value_max = MAX_RUNNING_TIME;
      GL_value_min = MIN_RUNNING_TIME;
      break;
    case 6:
      GL_value_set = (int) GL_config_data.rest_time; // Use integer psi
      GL_value_max = MAX_REST_TIME;
      GL_value_min = MIN_REST_TIME;
      break;
    default:
      GL_config_mode = 0;
  }
  lcd.setCursor(0, 0);
  lcd.print(label);
  show_value(GL_value_set);
  if (mode) { Serial.print("Config: ");Serial.print(mode);Serial.print(" "); }
  Serial.print(label);
  // Serial.print("- ");Serial.println(GL_button_counter);
  Serial.println();
}

// Read encoder, update config
// ---------------------------------------------------------------------
// Read encoder. Sets GL_parameter_changed to true if one or more 
// configuration parameters have changed
// 
// Config mode is changed 0-1-2-3-4-0 using the config button ()
//    0-Normal operation 
//    1-Configure target pressure
//    2-Configure max temp
//    3-Configure restart temp ( < max temp  )
//    3-Configure max running time
//    4-Configure rest time
//

void read_encoder(int mode) {  

  int state_A = 0, state_B = 0;             // state of encoder A, B signals

  //
  // Interrupts might change config mode to 0 in the middle of
  // the operation, so we don't use the global mode
  
  GL_value_old = GL_value_set;
  byte direction_cw = false; // is encoder rotating clockwise?
  byte old_direction_cw = direction_cw;
  
  // Read the current state of PIN_ENCODER_A
  state_A = digitalRead(PIN_ENCODER_A);
  // If last and current state of PIN_ENCODER_A are different, then pulse occurred.
  // Value is updated according to the direction of rotation, except whein it
  // changes, to avoid spurious pulses
  while (state_A != GL_last_state_A) { 
    // If the PIN_ENCODER_B state is different than the PIN_ENCODER_A state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(PIN_ENCODER_B) != state_A) {
      direction_cw = false;
      //  GL_value_set ++;
      GL_value_set += GL_knob_increment;
      // Serial.print("+ ");Serial.println(GL_knob_increment); 
    } else {
      direction_cw = true;
      // Encoder is rotating CW so increment
      // GL_value_set --;
      GL_value_set -= GL_knob_increment;
      // Serial.print("- ");Serial.println(GL_knob_increment); 
    }
    //->>Serial.println(GL_value_set);
    // Remember last PIN_ENCODER_A state
    GL_last_encoder_change = millis();
    GL_last_state_A = state_A;
    // Put in a slight delay to help debounce the reading
    delay(1);
    // Read again
    state_A = digitalRead(PIN_ENCODER_A);
    if (old_direction_cw != direction_cw) GL_knob_increment = 1;
    old_direction_cw = direction_cw;
  }
  if (GL_value_set != GL_value_old) {
    GL_parameter_changed = true;   // SIGNAL SOMETHING HAS CHANGED
    if (GL_value_set < GL_value_min) { 
      GL_value_set = GL_value_min;
    } else  {
      if (GL_value_set > GL_value_max) { 
        GL_value_set = GL_value_max;
      }
    }
    switch (mode) {
      case 1: GL_config_data.target_psi         =  GL_value_set; break;
      case 2: GL_config_data.sensor_psi_max     =  GL_value_set; break;
      case 3: GL_config_data.max_pump_temp      =  GL_value_set; break;
      case 4: GL_config_data.restart_temp       =  GL_value_set; break;
      case 5: GL_config_data.max_running_time   =  GL_value_set; break; 
      case 6: GL_config_data.rest_time          =  GL_value_set; break;
    }
    show_value(GL_value_set);
    // debounce and increment speed if turning fast 
    unsigned long t_knob = millis() < GL_last_encoder_change;
    if (t_knob < 150) {
      if ((t_knob > 5) && (GL_knob_increment < MAX_KNOB_INCREMENT)) {
        GL_knob_increment++;
      }
    } else {
      GL_knob_increment = 1;
    }
  }
}


// Display configuration on terminal
// ---------------------------------------------------------------------
// 

void displayConfig() {
  Serial.print("Magic_number (Version ID).........: "); Serial.println(GL_config_data.magic_number    );
  Serial.print("Target pressure (psi) ............: "); Serial.println(GL_config_data.target_psi      );
  Serial.print("Max pump temperature (Celsius) ...: "); Serial.println(GL_config_data.max_pump_temp   );
  Serial.print("Restart temperature (Celsius) ....: "); Serial.println(GL_config_data.restart_temp    );
  Serial.print("Max continuous running time (s) ..: "); Serial.println(GL_config_data.max_running_time);
  Serial.print("Rest time when above limit reached: "); Serial.println(GL_config_data.rest_time       );
return;
}
