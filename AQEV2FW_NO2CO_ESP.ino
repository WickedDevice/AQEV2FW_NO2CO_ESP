#include <Wire.h>
#include <SPI.h>
#include <ESP8266_AT_Client.h>
#include <SdFat.h>
#include <RTClib.h>
#include <RTC_DS3231.h>
#include <Time.h>
#include <TinyWatchdog.h>
#include <SHT25.h>
#include <MCP342x.h>
#include <LMP91000.h>
#include <WildFire_SPIFlash.h>
#include <CapacitiveSensor.h>
#include <LiquidCrystal.h>
#include <PubSubClient.h>
#include <util/crc16.h>
#include <math.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <jsmn.h>

// semantic versioning - see http://semver.org/
#define AQEV2FW_MAJOR_VERSION 2
#define AQEV2FW_MINOR_VERSION 1
#define AQEV2FW_PATCH_VERSION 9

#define WLAN_SEC_AUTO (10) // made up to support auto-config of security

// the start address of the second to last 4k page, where config is backed up off MCU
// the last page is reserved for use by the bootloader
#define SECOND_TO_LAST_4K_PAGE_ADDRESS      0x7E000     

int esp8266_enable_pin = 23; // Arduino digital the pin that is used to reset/enable the ESP8266 module

Stream * at_command_interface = &Serial1;  // Serial1 is the 'stream' the AT command interface is on
Stream * at_debug_interface = &Serial;
ESP8266_AT_Client esp(esp8266_enable_pin, at_command_interface); // instantiate the client object

TinyWatchdog tinywdt;
LMP91000 lmp91000;
MCP342x adc;
SHT25 sht25;
WildFire_SPIFlash flash;
CapacitiveSensor touch = CapacitiveSensor(A0, A1);
LiquidCrystal lcd(A3, A2, 4, 5, 6, 8);
char g_lcd_buffer[2][17] = {0}; // 2 rows of 16 characters each, with space for NULL terminator
byte mqtt_server_ip[4] = { 0 };    
PubSubClient mqtt_client;
char mqtt_client_id[32] = {0};

boolean wifi_can_connect = false;
boolean user_location_override = false;
boolean gps_installed = false;

RTC_DS3231 rtc;
SdFat SD;

TinyGPS gps;
SoftwareSerial gpsSerial(18, 17); // RX, TX
boolean gps_disabled = false;
#define GPS_MQTT_STRING_LENGTH (128)
#define GPS_CSV_STRING_LENGTH (64)
char gps_mqtt_string[GPS_MQTT_STRING_LENGTH] = {0};
char gps_csv_string[GPS_CSV_STRING_LENGTH] = {0}; 

uint32_t update_server_ip32 = 0;
char update_server_name[32] = {0};
unsigned long integrity_num_bytes_total = 0;
unsigned long integrity_crc16_checksum = 0;
uint32_t flash_file_size = 0;
uint16_t flash_signature = 0;
boolean downloaded_integrity_file = false;
boolean integrity_check_succeeded = false;
boolean allowed_to_write_config_eeprom = false;

unsigned long current_millis = 0;
char firmware_version[16] = {0};
uint8_t temperature_units = 'C';
float reported_temperature_offset_degC = 0.0f;
float reported_humidity_offset_percent = 0.0f;

float temperature_degc = 0.0f;
float relative_humidity_percent = 0.0f;
float no2_ppb = 0.0f;
float co_ppm = 0.0f;

float instant_temperature_degc = 0.0f;
float instant_humidity_percent = 0.0f;
float instant_no2_v = 0.0f;
float instant_co_v = 0.0f;

float gps_latitude = TinyGPS::GPS_INVALID_F_ANGLE;
float gps_longitude = TinyGPS::GPS_INVALID_F_ANGLE;
float gps_altitude = TinyGPS::GPS_INVALID_F_ALTITUDE;
unsigned long gps_age = TinyGPS::GPS_INVALID_AGE;

float user_latitude = TinyGPS::GPS_INVALID_F_ANGLE;
float user_longitude = TinyGPS::GPS_INVALID_F_ANGLE;
float user_altitude = TinyGPS::GPS_INVALID_F_ALTITUDE;

#define MAX_SAMPLE_BUFFER_DEPTH (180) // 15 minutes @ 5 second resolution
#define NO2_SAMPLE_BUFFER         (0)
#define CO_SAMPLE_BUFFER          (1)
#define TEMPERATURE_SAMPLE_BUFFER (2)
#define HUMIDITY_SAMPLE_BUFFER    (3)
float sample_buffer[4][MAX_SAMPLE_BUFFER_DEPTH] = {0};
uint16_t sample_buffer_idx = 0;

uint32_t sampling_interval = 0;    // how frequently the sensorss are sampled
uint16_t sample_buffer_depth = 0;  // how many samples are kept in memory for averaging
uint32_t reporting_interval = 0;   // how frequently readings are reported (to wifi or console/sd)

#define TOUCH_SAMPLE_BUFFER_DEPTH (4)
float touch_sample_buffer[TOUCH_SAMPLE_BUFFER_DEPTH] = {0};

#define LCD_ERROR_MESSAGE_DELAY   (4000)
#define LCD_SUCCESS_MESSAGE_DELAY (2000)

jsmn_parser parser;
jsmntok_t json_tokens[16];

boolean no2_ready = false;
boolean co_ready = false;
boolean temperature_ready = false;
boolean humidity_ready = false;

boolean init_sht25_ok = false;
boolean init_co_afe_ok = false;
boolean init_no2_afe_ok = false;
boolean init_co_adc_ok = false;
boolean init_no2_adc_ok = false;
boolean init_spi_flash_ok = false;
boolean init_esp8266_ok = false;
boolean init_sdcard_ok = false;
boolean init_rtc_ok = false;

typedef struct{
  float temperature_degC;     // starting at this temperature 
  float slope_volts_per_degC; // use a line with this slope
  float intercept_volts;      // and this intercept
                              // to calculate the baseline voltage
} baseline_voltage_t;
baseline_voltage_t baseline_voltage_struct; // scratch space for a single baseline_voltage_t entry

#define BACKLIGHT_OFF_AT_STARTUP (0)
#define BACKLIGHT_ON_AT_STARTUP  (1)
#define BACKLIGHT_ALWAYS_ON      (2)
#define BACKLIGHT_ALWAYS_OFF     (3)

boolean g_backlight_turned_on = false;

// the software's operating mode
#define MODE_CONFIG      (1)
#define MODE_OPERATIONAL (2)
// submodes of normal behavior
#define SUBMODE_NORMAL   (3)
// #define SUBMODE_ZEROING  (4) // deprecated for SUBMODE_OFFLINE
#define SUBMODE_OFFLINE  (5)

uint8_t mode = MODE_OPERATIONAL;

// the config mode state machine's return values
#define CONFIG_MODE_NOTHING_SPECIAL  (0)
#define CONFIG_MODE_GOT_INIT         (1)
#define CONFIG_MODE_GOT_EXIT         (2)

#define EEPROM_CONFIG_MEMORY_SIZE (1024)

#define EEPROM_MAC_ADDRESS    (E2END + 1 - 6)    // MAC address, i.e. the last 6-bytes of EEPROM
// more parameters follow, address relative to each other so they don't overlap
#define EEPROM_CONNECT_METHOD     (EEPROM_MAC_ADDRESS - 1)        // connection method encoded as a single byte value 
#define EEPROM_SSID               (EEPROM_CONNECT_METHOD - 32)    // ssid string, up to 32 characters (one of which is a null terminator)
#define EEPROM_NETWORK_PWD        (EEPROM_SSID - 32)              // network password, up to 32 characters (one of which is a null terminator)
#define EEPROM_SECURITY_MODE      (EEPROM_NETWORK_PWD - 1)        // security mode encoded as a single byte value
#define EEPROM_STATIC_IP_ADDRESS  (EEPROM_SECURITY_MODE - 4)      // static ipv4 address, 4 bytes - 0.0.0.0 indicates use DHCP
#define EEPROM_STATIC_NETMASK     (EEPROM_STATIC_IP_ADDRESS - 4)  // static netmask, 4 bytes
#define EEPROM_STATIC_GATEWAY     (EEPROM_STATIC_NETMASK - 4)     // static default gateway ip address, 4 bytes
#define EEPROM_STATIC_DNS         (EEPROM_STATIC_GATEWAY - 4)     // static dns server ip address, 4 bytes
#define EEPROM_MQTT_PASSWORD      (EEPROM_STATIC_DNS - 32)        // password for mqtt server, up to 32 characters (one of which is a null terminator)
#define EEPROM_NO2_SENSITIVITY    (EEPROM_MQTT_PASSWORD - 4)      // float value, 4-bytes, the sensitivity from the sticker
#define EEPROM_NO2_CAL_SLOPE      (EEPROM_NO2_SENSITIVITY - 4)    // float value, 4-bytes, the slope applied to the sensor
#define EEPROM_NO2_CAL_OFFSET     (EEPROM_NO2_CAL_SLOPE - 4)      // float value, 4-btyes, the offset applied to the sensor
#define EEPROM_CO_SENSITIVITY     (EEPROM_NO2_CAL_OFFSET - 4)     // float value, 4-bytes, the sensitivity from the sticker
#define EEPROM_CO_CAL_SLOPE       (EEPROM_CO_SENSITIVITY - 4)     // float value, 4-bytes, the slope applied to the sensor
#define EEPROM_CO_CAL_OFFSET      (EEPROM_CO_CAL_SLOPE - 4)       // float value, 4-bytes, the offset applied to the sensor
#define EEPROM_PRIVATE_KEY        (EEPROM_CO_CAL_OFFSET - 32)     // 32-bytes of Random Data (256-bits)
#define EEPROM_MQTT_SERVER_NAME   (EEPROM_PRIVATE_KEY - 32)       // string, the DNS name of the MQTT server (default mqtt.opensensors.io), up to 32 characters (one of which is a null terminator)
#define EEPROM_MQTT_USERNAME      (EEPROM_MQTT_SERVER_NAME - 32)  // string, the user name for the MQTT server (default wickeddevice), up to 32 characters (one of which is a null terminator)
#define EEPROM_MQTT_CLIENT_ID     (EEPROM_MQTT_USERNAME - 32)     // string, the client identifier for the MQTT server (default SHT25 identifier), between 1 and 23 characters long
#define EEPROM_MQTT_AUTH          (EEPROM_MQTT_CLIENT_ID - 1)     // MQTT authentication enabled, single byte value 0 = disabled or 1 = enabled
#define EEPROM_MQTT_PORT          (EEPROM_MQTT_AUTH - 4)          // MQTT authentication enabled, reserve four bytes, even though you only need two for a port
#define EEPROM_UPDATE_SERVER_NAME (EEPROM_MQTT_PORT - 32)         // string, the DNS name of the Firmware Update server (default update.wickeddevice.com), up to 32 characters (one of which is a null terminator)
#define EEPROM_OPERATIONAL_MODE   (EEPROM_UPDATE_SERVER_NAME - 1) // operational mode encoded as a single byte value (e.g. NORMAL, OFFLINE, etc.)
#define EEPROM_TEMPERATURE_UNITS  (EEPROM_OPERATIONAL_MODE - 1)   // temperature units 'F' for Fahrenheit and 'C' for Celsius
#define EEPROM_UPDATE_FILENAME    (EEPROM_TEMPERATURE_UNITS - 32) // 32-bytes for the update server filename (excluding the implied extension)
#define EEPROM_TEMPERATURE_OFFSET (EEPROM_UPDATE_FILENAME - 4)    // float value, 4-bytes, the offset applied to the sensor for reporting
#define EEPROM_HUMIDITY_OFFSET    (EEPROM_TEMPERATURE_OFFSET - 4) // float value, 4-bytes, the offset applied to the sensor for reporting
#define EEPROM_BACKLIGHT_DURATION (EEPROM_HUMIDITY_OFFSET - 2)    // integer value, 2-bytes, how long, in seconds the backlight should stay on when it turns on
#define EEPROM_BACKLIGHT_STARTUP  (EEPROM_BACKLIGHT_DURATION - 1) // boolean value, whether or not the backlight should turn on at startup
#define EEPROM_SAMPLING_INTERVAL  (EEPROM_BACKLIGHT_STARTUP - 2)  // integer value, number of seconds between sensor samplings
#define EEPROM_REPORTING_INTERVAL (EEPROM_SAMPLING_INTERVAL - 2)  // integer value, number of seconds between sensor reports
#define EEPROM_AVERAGING_INTERVAL (EEPROM_REPORTING_INTERVAL - 2) // integer value, number of seconds of samples averaged
#define EEPROM_ALTITUDE_METERS    (EEPROM_AVERAGING_INTERVAL - 2) // signed integer value, 2-bytes, the altitude in meters above sea level, where the Egg is located
#define EEPROM_MQTT_TOPIC_PREFIX  (EEPROM_ALTITUDE_METERS - 64)   // up to 64-character string, prefix prepended to logical sensor topics
#define EEPROM_USE_NTP            (EEPROM_MQTT_TOPIC_PREFIX - 1)  // 1 means use NTP, anything else means don't use NTP
#define EEPROM_NTP_SERVER_NAME    (EEPROM_USE_NTP - 32)           // 32-bytes for the NTP server to use
#define EEPROM_NTP_TZ_OFFSET_HRS  (EEPROM_NTP_SERVER_NAME - 4)    // timezonEEPROM_CO_BASELINE_VOLTAGE_TABLE e offset as a floating point value
#define EEPROM_NO2_BASELINE_VOLTAGE_TABLE (EEPROM_NTP_TZ_OFFSET_HRS - (5*sizeof(baseline_voltage_t))) // array of (up to) five structures for baseline offset characterization over temperature
#define EEPROM_CO_BASELINE_VOLTAGE_TABLE (EEPROM_NO2_BASELINE_VOLTAGE_TABLE - (5*sizeof(baseline_voltage_t))) // array of (up to) five structures for baseline offset characterization over temperature
#define EEPROM_MQTT_TOPIC_SUFFIX_ENABLED  (EEPROM_CO_BASELINE_VOLTAGE_TABLE - 1)    // a simple flag to indicate whether or not the topic suffix is enabled
#define EEPROM_USER_LATITUDE_DEG  (EEPROM_MQTT_TOPIC_SUFFIX_ENABLED - 4) // float value, 4-bytes, user specified latitude in degrees
#define EEPROM_USER_LONGITUDE_DEG (EEPROM_USER_LATITUDE_DEG - 4)         // float value, 4-bytes, user specified longitude in degrees
#define EEPROM_USER_LOCATION_EN   (EEPROM_USER_LONGITUDE_DEG - 1)        // 1 means user location supercedes GPS location, anything else means GPS or bust
//  /\
//   L Add values up here by subtracting offsets to previously added values
//   * ... and make sure the addresses don't collide and start overlapping!
//   T Add values down here by adding offsets to previously added values
//  \/
#define EEPROM_BACKUP_NTP_TZ_OFFSET_HRS  (EEPROM_BACKUP_HUMIDITY_OFFSET + 4)
#define EEPROM_BACKUP_HUMIDITY_OFFSET    (EEPROM_BACKUP_TEMPERATURE_OFFSET + 4)
#define EEPROM_BACKUP_TEMPERATURE_OFFSET (EEPROM_BACKUP_PRIVATE_KEY + 32)
#define EEPROM_BACKUP_PRIVATE_KEY        (EEPROM_BACKUP_CO_CAL_OFFSET + 4)
#define EEPROM_BACKUP_CO_CAL_OFFSET      (EEPROM_BACKUP_CO_CAL_SLOPE + 4)
#define EEPROM_BACKUP_CO_CAL_SLOPE       (EEPROM_BACKUP_CO_SENSITIVITY + 4)
#define EEPROM_BACKUP_CO_SENSITIVITY     (EEPROM_BACKUP_NO2_CAL_OFFSET + 4)
#define EEPROM_BACKUP_NO2_CAL_OFFSET     (EEPROM_BACKUP_NO2_CAL_SLOPE + 4)
#define EEPROM_BACKUP_NO2_CAL_SLOPE      (EEPROM_BACKUP_NO2_SENSITIVITY + 4)
#define EEPROM_BACKUP_NO2_SENSITIVITY    (EEPROM_BACKUP_MQTT_PASSWORD + 32)
#define EEPROM_BACKUP_MQTT_PASSWORD      (EEPROM_BACKUP_MAC_ADDRESS + 6)
#define EEPROM_BACKUP_MAC_ADDRESS        (EEPROM_BACKUP_CHECK + 2) // backup parameters are added here offset from the EEPROM_CRC_CHECKSUM
#define EEPROM_BACKUP_CHECK              (EEPROM_CRC_CHECKSUM + 2) // 2-byte value with various bits set if backup has ever happened
#define EEPROM_CRC_CHECKSUM              (E2END + 1 - EEPROM_CONFIG_MEMORY_SIZE) // reserve the last 1kB for config
// the only things that need "backup" are those which are unique to a device
// other things can have "defaults" stored in flash (i.e. using the restore defaults command)

// valid connection methods
// only DIRECT is supported initially
#define CONNECT_METHOD_DIRECT        (0)

// backup status bits
#define BACKUP_STATUS_MAC_ADDRESS_BIT             (7)
#define BACKUP_STATUS_MQTT_PASSSWORD_BIT          (6)
#define BACKUP_STATUS_NO2_CALIBRATION_BIT         (5)
#define BACKUP_STATUS_CO_CALIBRATION_BIT          (4)
#define BACKUP_STATUS_PRIVATE_KEY_BIT             (3)
#define BACKUP_STATUS_TEMPERATURE_CALIBRATION_BIT (2)
#define BACKUP_STATUS_HUMIDITY_CALIBRATION_BIT    (1)
#define BACKUP_STATUS_TIMEZONE_CALIBRATION_BIT    (0)

#define BIT_IS_CLEARED(val, b) (!(val & (1UL << b)))
#define CLEAR_BIT(val, b) \
  do { \
    val &= ~(1UL << b); \
  } while(0)

void help_menu(char * arg);
void print_eeprom_value(char * arg);
void initialize_eeprom_value(char * arg);
void restore(char * arg);
void set_mac_address(char * arg);
void set_connection_method(char * arg);
void set_ssid(char * arg);
void set_network_password(char * arg);
void set_network_security_mode(char * arg);
void set_static_ip_address(char * arg);
void use_command(char * arg);
void set_mqtt_password(char * arg);
void set_mqtt_server(char * arg);
void set_mqtt_port(char * arg);
void set_mqtt_username(char * arg);
void set_mqtt_client_id(char * arg);
void set_mqtt_authentication(char * arg);
void set_mqtt_topic_prefix(char * arg);
void backup(char * arg);
void set_no2_slope(char * arg);
void set_no2_offset(char * arg);
void set_no2_sensitivity(char * arg);
void set_co_slope(char * arg);
void set_co_offset(char * arg);
void set_co_sensitivity(char * arg);
void set_reported_temperature_offset(char * arg);
void set_reported_humidity_offset(char * arg);
void set_private_key(char * arg);
void set_operational_mode(char * arg);
void set_temperature_units(char * arg);
void set_update_filename(char * arg);
void force_command(char * arg);
void set_backlight_behavior(char * arg);
void AQE_set_datetime(char * arg);
void list_command(char * arg);
void download_command(char * arg);
void delete_command(char * arg);
void sampling_command(char * arg);
void altitude_command(char * arg);
void set_ntp_server(char * arg);
void set_ntp_timezone_offset(char * arg);
void set_update_server_name(char * arg);
void no2_baseline_voltage_characterization_command(char * arg);
void co_baseline_voltage_characterization_command(char * arg);
void topic_suffix_config(char * arg);
void set_user_latitude(char * arg);
void set_user_longitude(char * arg);
void set_user_location_enable(char * arg);

// Note to self:
//   When implementing a new parameter, ask yourself:
//     should there be a command for the user to set its value directly
//     should 'get' support it (almost certainly the answer is yes)
//     should 'init' support it (is there a way to set it without user intervention)
//     should 'restore' support it directly
//     should 'restore defaults' support it
//   ... and remember, anything that changes the config EEPROM
//       needs to call recomputeAndStoreConfigChecksum after doing so

// the order of the command keywords in this array
// must be kept in index-correspondence with the associated
// function pointers in the command_functions array
//
// these keywords are padded with spaces
// in order to ease printing as a table
// string comparisons should use strncmp rather than strcmp
const char cmd_string_get[] PROGMEM         = "get        ";
const char cmd_string_init[] PROGMEM        = "init       ";
const char cmd_string_restore[] PROGMEM     = "restore    ";
const char cmd_string_mac[] PROGMEM         = "mac        ";
const char cmd_string_method[] PROGMEM      = "method     ";
const char cmd_string_ssid[] PROGMEM        = "ssid       ";
const char cmd_string_pwd[] PROGMEM         = "pwd        ";
const char cmd_string_security[] PROGMEM    = "security   ";
const char cmd_string_staticip[] PROGMEM    = "staticip   ";
const char cmd_string_use[] PROGMEM         = "use        ";
const char cmd_string_mqttsrv[] PROGMEM     = "mqttsrv    ";
const char cmd_string_mqttport[] PROGMEM    = "mqttport   ";
const char cmd_string_mqttuser[] PROGMEM    = "mqttuser   ";
const char cmd_string_mqttpwd[] PROGMEM     = "mqttpwd    ";
const char cmd_string_mqttid[] PROGMEM      = "mqttid     ";
const char cmd_string_mqttauth[] PROGMEM    = "mqttauth   ";
const char cmd_string_mqttprefix[] PROGMEM  = "mqttprefix ";
const char cmd_string_mqttsuffix[] PROGMEM  = "mqttsuffix ";
const char cmd_string_updatesrv[] PROGMEM   = "updatesrv  ";
const char cmd_string_backup[] PROGMEM      = "backup     ";
const char cmd_string_no2_sen[] PROGMEM     = "no2_sen    ";
const char cmd_string_no2_slope[] PROGMEM   = "no2_slope  ";
const char cmd_string_no2_off[] PROGMEM     = "no2_off    ";
const char cmd_string_co_sen[] PROGMEM      = "co_sen     ";
const char cmd_string_co_slope[] PROGMEM    = "co_slope   ";
const char cmd_string_co_off[] PROGMEM      = "co_off     ";
const char cmd_string_temp_off[] PROGMEM    = "temp_off   ";
const char cmd_string_hum_off[] PROGMEM     = "hum_off    ";
const char cmd_string_key[] PROGMEM         = "key        ";
const char cmd_string_opmode[] PROGMEM      = "opmode     ";
const char cmd_string_tempunit[] PROGMEM    = "tempunit   ";
const char cmd_string_updatefile[] PROGMEM  = "updatefile ";
const char cmd_string_force[] PROGMEM       = "force      ";
const char cmd_string_backlight[] PROGMEM   = "backlight  ";
const char cmd_string_datetime[] PROGMEM    = "datetime   ";
const char cmd_string_list[] PROGMEM        = "list       ";
const char cmd_string_download[] PROGMEM    = "download   ";
const char cmd_string_delete[] PROGMEM      = "delete     ";
const char cmd_string_sampling[] PROGMEM    = "sampling   ";
const char cmd_string_altitude[] PROGMEM    = "altitude   ";
const char cmd_string_ntpsrv[] PROGMEM      = "ntpsrv     ";
const char cmd_string_tz_off[] PROGMEM      = "tz_off     ";
const char cmd_string_no2_blv[] PROGMEM     = "no2_blv    ";
const char cmd_string_co_blv[] PROGMEM      = "co_blv     ";
const char cmd_string_usr_lat[] PROGMEM     = "latitude   ";
const char cmd_string_usr_lng[] PROGMEM     = "longitude  ";
const char cmd_string_usr_loc_en[] PROGMEM  = "location   ";
const char cmd_string_null[] PROGMEM        = "";

PGM_P const commands[] PROGMEM = {
  cmd_string_get,
  cmd_string_init,
  cmd_string_restore,
  cmd_string_mac,
  cmd_string_method,
  cmd_string_ssid,
  cmd_string_pwd,
  cmd_string_security,
  cmd_string_staticip,
  cmd_string_use,
  cmd_string_mqttsrv,
  cmd_string_mqttport,
  cmd_string_mqttuser,
  cmd_string_mqttpwd,
  cmd_string_mqttid,
  cmd_string_mqttauth,
  cmd_string_mqttprefix,
  cmd_string_mqttsuffix,
  cmd_string_updatesrv,
  cmd_string_backup,
  cmd_string_no2_sen,
  cmd_string_no2_slope,
  cmd_string_no2_off,
  cmd_string_co_sen,
  cmd_string_co_slope,
  cmd_string_co_off,
  cmd_string_temp_off,
  cmd_string_hum_off,
  cmd_string_key,
  cmd_string_opmode,
  cmd_string_tempunit,
  cmd_string_updatefile,
  cmd_string_force,
  cmd_string_backlight,
  cmd_string_datetime,
  cmd_string_list,
  cmd_string_download,
  cmd_string_delete,
  cmd_string_sampling,
  cmd_string_altitude,
  cmd_string_ntpsrv,
  cmd_string_tz_off,
  cmd_string_no2_blv,
  cmd_string_co_blv, 
  cmd_string_usr_lat,
  cmd_string_usr_lng,
  cmd_string_usr_loc_en,
  cmd_string_null
};

void (*command_functions[])(char * arg) = {
  print_eeprom_value,
  initialize_eeprom_value,
  restore,
  set_mac_address,
  set_connection_method,
  set_ssid,
  set_network_password,
  set_network_security_mode,
  set_static_ip_address,
  use_command,
  set_mqtt_server,
  set_mqtt_port,  
  set_mqtt_username,
  set_mqtt_password,  
  set_mqtt_client_id,
  set_mqtt_authentication,
  set_mqtt_topic_prefix,
  topic_suffix_config,
  set_update_server_name,
  backup,
  set_no2_sensitivity,
  set_no2_slope,
  set_no2_offset,
  set_co_sensitivity,
  set_co_slope,
  set_co_offset,
  set_reported_temperature_offset,
  set_reported_humidity_offset,
  set_private_key,
  set_operational_mode,
  set_temperature_units,
  set_update_filename,
  force_command,
  set_backlight_behavior,
  AQE_set_datetime,
  list_command,
  download_command,
  delete_command,
  sampling_command,
  altitude_command,
  set_ntp_server,
  set_ntp_timezone_offset,
  no2_baseline_voltage_characterization_command,
  co_baseline_voltage_characterization_command,
  set_user_latitude,
  set_user_longitude,
  set_user_location_enable,
  0
};

// tiny watchdog timer intervals
unsigned long previous_tinywdt_millis = 0;
const long tinywdt_interval = 1000;

// sensor sampling timer intervals
unsigned long previous_sensor_sampling_millis = 0;

// touch sampling timer intervals
unsigned long previous_touch_sampling_millis = 0;
const long touch_sampling_interval = 200;

// progress dots timer intervals  
unsigned long previous_progress_dots_millis = 0;
const long progress_dots_interval = 1000;

#define NUM_HEARTBEAT_WAVEFORM_SAMPLES (84)
const uint8_t heartbeat_waveform[NUM_HEARTBEAT_WAVEFORM_SAMPLES] PROGMEM = {
  95, 94, 95, 96, 95, 94, 95, 96, 95, 94,
  95, 96, 95, 94, 95, 96, 95, 97, 105, 112,
  117, 119, 120, 117, 111, 103, 95, 94, 95, 96,
  95, 94, 100, 131, 162, 193, 224, 255, 244, 214,
  183, 152, 121, 95, 88, 80, 71, 74, 82, 90, 
  96, 95, 94, 95, 96, 97, 106, 113, 120, 125, 
  129, 132, 133, 131, 128, 124, 118, 111, 103, 96,
  95, 96, 95, 94, 95, 96, 95, 94, 95, 99, 
  105, 106, 101, 96  
};
uint8_t heartbeat_waveform_index = 0;

#define SCRATCH_BUFFER_SIZE (512)
char scratch[SCRATCH_BUFFER_SIZE] = { 0 };  // scratch buffer, for general use
uint16_t scratch_idx = 0;
#define ESP8266_INPUT_BUFFER_SIZE (1500)
uint8_t esp8266_input_buffer[ESP8266_INPUT_BUFFER_SIZE] = {0};     // sketch must instantiate a buffer to hold incoming data
                                                                   // 1500 bytes is way overkill for MQTT, but if you have it, may as well
                                                                   // make space for a whole TCP packet
char converted_value_string[64] = {0};
char compensated_value_string[64] = {0};
char raw_value_string[64] = {0};
char raw_instant_value_string[64] = {0};
char response_body[256] = {0};

char MQTT_TOPIC_STRING[128] = {0};
char MQTT_TOPIC_PREFIX[64] = "/orgs/wd/aqe/";
uint8_t mqtt_suffix_enabled = 0;

const char * header_row = "Timestamp,"
               "Temperature[degC],"
               "Humidity[percent],"                   
               "NO2[ppb],"                    
               "CO[ppm],"      
               "NO2[V]," 
               "CO[V],"  
               "Latitude[deg],"
               "Longitude[deg],"
               "Altitude[m]";   

void setup() {
  boolean integrity_check_passed = false;
  boolean mirrored_config_mismatch = false;
  boolean valid_ssid_passed = false; 
  
  // initialize hardware
  initializeHardware(); 
  backlightOff();
  
  //  uint8_t tmp[EEPROM_CONFIG_MEMORY_SIZE] = {0};
  //  get_eeprom_config(tmp);
  //  Serial.println(F("EEPROM Config:"));
  //  dump_config(tmp);
  //  Serial.println();
  //  Serial.println(F("Mirrored Config:"));
  //  get_mirrored_config(tmp);  
  //  dump_config(tmp);
  //  Serial.println();
    
  // if a software update introduced new settings
  // they should be populated with defaults as necessary
  initializeNewConfigSettings();


  user_location_override = (eeprom_read_byte((const uint8_t *) EEPROM_USER_LOCATION_EN) == 1) ? true : false;  
  uint8_t target_mode = eeprom_read_byte((const uint8_t *) EEPROM_OPERATIONAL_MODE);  
  boolean ok_to_exit_config_mode = true;     
  
  // config mode processing loop
  do{
    // if the appropriate escape sequence is received within 8 seconds
    // go into config mode
    const long startup_time_period = 12000;
    long start = millis();
    long min_over = 100;
    boolean got_serial_input = false;
    Serial.println(F("Enter 'aqe' for CONFIG mode."));
    Serial.print(F("OPERATIONAL mode automatically begins after "));
    Serial.print(startup_time_period / 1000);
    Serial.println(F(" secs of no input."));
    setLCD_P(PSTR("CONNECT TERMINAL"
                  "FOR CONFIG MODE "));

    g_backlight_turned_on = false; // clear the global flag
    boolean soft_ap_config_activated = false;
    current_millis = millis();     
    while (current_millis < start + startup_time_period) { // can get away with this sort of thing at start up
      current_millis = millis();

      if(g_backlight_turned_on){
        soft_ap_config_activated = true;
        Serial.println();
        Serial.println(F("Info: Entering SoftAP Mode for Configuration"));
        break;
      }

      if(current_millis - previous_touch_sampling_millis >= touch_sampling_interval){
        static uint8_t num_touch_intervals = 0;
        previous_touch_sampling_millis = current_millis;    
        collectTouch();    
        processTouchQuietly();
        
        num_touch_intervals++;
        if(num_touch_intervals == 5){
          petWatchdog(); 
          num_touch_intervals = 0;
        }
        
      }      
    
      if (Serial.available()) {
        if (got_serial_input == false) {
          Serial.println();
        }
        got_serial_input = true;

        start = millis(); // reset the timeout
        if (CONFIG_MODE_GOT_INIT == configModeStateMachine(Serial.read(), false)) {
          mode = MODE_CONFIG;
          allowed_to_write_config_eeprom = true;
          break;
        }
      }

      // output a countdown to the Serial Monitor
      if (millis() - start >= min_over) {
        uint8_t countdown_value_display = (startup_time_period - 500 - min_over) / 1000;
        if (got_serial_input == false) {
          Serial.print(countdown_value_display);
          Serial.print(F("..."));
        }
        
        updateCornerDot();
        
        min_over += 1000;
      }
    }
    Serial.println();

    if(soft_ap_config_activated){
      allowed_to_write_config_eeprom = true;
      configInject("aqe\r");
      doSoftApModeConfigBehavior();
      configInject("exit\r");
    }
    else{
      integrity_check_passed = checkConfigIntegrity();
      // if the integrity check failed, try and undo the damage using the mirror config, if it's valid
      if(!integrity_check_passed){
        Serial.println(F("Info: Startup config integrity check failed, attempting to restore from mirrored configuration."));
        allowed_to_write_config_eeprom = true;
        integrity_check_passed = mirrored_config_restore_and_validate(); 
        allowed_to_write_config_eeprom = false;
      }
      else if(!mirrored_config_matches_eeprom_config()){
        mirrored_config_mismatch = true;
        Serial.println(F("Info: Startup config integrity check passed, but mirrored config differs, attempting to restore from mirrored configuration."));
        allowed_to_write_config_eeprom = true;
        integrity_check_passed = mirrored_config_restore_and_validate();
        allowed_to_write_config_eeprom = false;
      }     
      
      valid_ssid_passed = valid_ssid_config();  
    
      // check for initial integrity of configuration in eeprom
      if((mode != MODE_CONFIG) && mode_requires_wifi(target_mode) && !valid_ssid_passed){
        Serial.println(F("Info: No valid SSID configured, automatically falling back to CONFIG mode."));
        configInject("aqe\r");
        Serial.println();
        
        do{
          setLCD_P(PSTR("PLEASE CONFIGURE"
                        "NETWORK SETTINGS"));
          delay(LCD_ERROR_MESSAGE_DELAY);
          
          allowed_to_write_config_eeprom = true;
          doSoftApModeConfigBehavior();
          valid_ssid_passed = valid_ssid_config();       
                               
        } while(!valid_ssid_passed);
      }
      else if(!integrity_check_passed && !mirrored_config_mismatch) { 
        // if there was not a mirrored config mismatch and integrity check did not pass
        // that means startup config integrity check failed, and restoring from mirror configuration failed
        // to result in a valid configuration as well
        //
        // if, on the other hand, there was a mirrored config mismatch, the logic above *implies* that the eeprom config 
        // is valid and that the mirrored config is not (yet) valid, so we shouldn't go into this case, and instead 
        // we should drop into the else case (i.e. what normally happens on a startup with a valid configuration)
        Serial.println(F("Info: Config memory integrity check failed, automatically falling back to CONFIG mode."));
        configInject("aqe\r");
        Serial.println();
        setLCD_P(PSTR("CONFIG INTEGRITY"
                      "  CHECK FAILED  "));
        mode = MODE_CONFIG;        
      }          
    }
    
    
    Serial.println();
    delayForWatchdog();
    
    if (mode == MODE_CONFIG) {      
      allowed_to_write_config_eeprom = true;   
      const uint32_t idle_timeout_period_ms = 1000UL * 60UL * 5UL; // 5 minutes
      uint32_t idle_time_ms = 0;
      Serial.println(F("-~=* In CONFIG Mode *=~-"));
      
      if(integrity_check_passed && valid_ssid_passed){
        setLCD_P(PSTR("  CONFIG MODE"));
      }          
      
      Serial.print(F("OPERATIONAL mode begins automatically after "));
      Serial.print((idle_timeout_period_ms / 1000UL) / 60UL);
      Serial.println(F(" mins without input."));
      Serial.println(F("Enter 'help' for a list of available commands, "));
            
  
      configInject("get settings\r");
      Serial.println();
      Serial.println(F(" @=============================================================@"));
      Serial.println(F(" # GETTING STARTED                                             #"));
      Serial.println(F(" #-------------------------------------------------------------#"));
      Serial.println(F(" #   First type 'ssid your_ssid_here' and & press <enter>      #"));
      Serial.println(F(" #   Then type 'pwd your_network_password' & press <enter>     #"));
      Serial.println(F(" #   Then type 'get settings' & press <enter> to review config #"));
      Serial.println(F(" #   Finally, type 'exit' to go into OPERATIONAL mode,         #"));
      Serial.println(F(" #     and verify that the Egg connects to your network!       #")); 
      Serial.println(F(" @=============================================================@"));
  
      prompt();
      for (;;) {
        current_millis = millis();
        if(current_millis - previous_touch_sampling_millis >= touch_sampling_interval){
          previous_touch_sampling_millis = current_millis;   
          collectTouch();    
          processTouchQuietly();  
        }

        // check to determine if we have a GPS
        while(!gps_installed && gpsSerial.available()){
          char c = gpsSerial.read();
          if(c == '$'){
            gps_installed = true;
          }
        }        
  
        // stuck in this loop until the command line receives an exit command
        if(mode != MODE_CONFIG){
          break; // if a command changes mode, we're done with config
        }
        
        if (Serial.available()) {
          idle_time_ms = 0;
          // if you get serial traffic, pass it along to the configModeStateMachine for consumption
          if (CONFIG_MODE_GOT_EXIT == configModeStateMachine(Serial.read(), false)) {
            break;
          }
        }
  
        // pet the watchdog once a second      
        if (current_millis - previous_tinywdt_millis >= tinywdt_interval) {
          idle_time_ms += tinywdt_interval;
          petWatchdog();
          previous_tinywdt_millis = current_millis;
        }
  
        if (idle_time_ms >= idle_timeout_period_ms) {
          Serial.println(F("Info: Idle time expired, exiting CONFIG mode."));
          break;
        }
      }
    }
    
    integrity_check_passed = checkConfigIntegrity();
    valid_ssid_passed = valid_ssid_config();    
    ok_to_exit_config_mode = true;
    
    target_mode = eeprom_read_byte((const uint8_t *) EEPROM_OPERATIONAL_MODE);      
       
    if(!integrity_check_passed){
      ok_to_exit_config_mode = false;
    }
    else if(mode_requires_wifi(target_mode) && !valid_ssid_passed){
      ok_to_exit_config_mode = false;
    }
    
  } while(!ok_to_exit_config_mode);

  allowed_to_write_config_eeprom = false;
  
  Serial.println(F("-~=* In OPERATIONAL Mode *=~-"));
  setLCD_P(PSTR("OPERATIONAL MODE"));
  SUCCESS_MESSAGE_DELAY();
  
  // ... but *which* operational mode are we in?
  mode = target_mode;
  
  // ... and what is the temperature and humdidity offset we should use
  reported_temperature_offset_degC = eeprom_read_float((float *) EEPROM_TEMPERATURE_OFFSET);
  reported_humidity_offset_percent = eeprom_read_float((float *) EEPROM_HUMIDITY_OFFSET);

  boolean use_ntp = eeprom_read_byte((uint8_t *) EEPROM_USE_NTP);
  boolean shutdown_wifi = !mode_requires_wifi(mode);
  
  if(mode_requires_wifi(mode) || use_ntp){
    shutdown_wifi = false;
    
    // Scan Networks to show RSSI    
    uint8_t connect_method = eeprom_read_byte((const uint8_t *) EEPROM_CONNECT_METHOD);        
    displayRSSI();         
    delayForWatchdog();
    petWatchdog();
    
    // Try and Connect to the Configured Network
    if(!restartWifi()){
      // technically this code should be unreachable
      // because error conditions internal to the restartWifi function
      // should restart the unit at a finer granularity
      // but this additional report should be harmless at any rate
      Serial.println(F("Error: Failed to connect to configured network. Rebooting."));
      Serial.flush();
      watchdogForceReset();
    }
    delayForWatchdog();
    petWatchdog();
     
    // at this point we have connected to the network successfully
    // it's an opportunity to mirror the eeprom configuration
    // if it's different from what's already there
    // importantly this check only happens at startup    
    commitConfigToMirroredConfig();
  
    // Check for Firmware Updates 
    checkForFirmwareUpdates();
    integrity_check_passed = checkConfigIntegrity();
    if(!integrity_check_passed){
      Serial.println(F("Error: Config Integrity Check Failed after checkForFirmwareUpdates"));
      setLCD_P(PSTR("CONFIG INTEGRITY"
                    "  CHECK FAILED  "));
      for(;;){
        // prevent automatic reset
        delay(1000);
        petWatchdog();
      }      
    }

    if(use_ntp){
      getNetworkTime();
    }

    if(mode_requires_wifi(mode)){
      // Connect to MQTT server
      if(!mqttReconnect()){
        setLCD_P(PSTR("  MQTT CONNECT  "
                      "     FAILED     "));
        lcdFrownie(15, 1);
        ERROR_MESSAGE_DELAY();      
        Serial.println(F("Error: Unable to connect to MQTT server"));
        Serial.flush();
        watchdogForceReset();    
      }
      delayForWatchdog();
      petWatchdog();
    }
    else{
      shutdown_wifi = true;
    }
  }

  if(shutdown_wifi){
    // it's a mode that doesn't require Wi-Fi
    // save settings as necessary
    commitConfigToMirroredConfig();    
    esp.sleep(2); // deep sleep    
  }
  
  // get the temperature units
  temperature_units = eeprom_read_byte((const uint8_t *) EEPROM_TEMPERATURE_UNITS);
  if((temperature_units != 'C') && (temperature_units != 'F')){
    temperature_units = 'C';
  }
  
  // get the sampling, reporting, and averaging parameters
  sampling_interval = eeprom_read_word((uint16_t * ) EEPROM_SAMPLING_INTERVAL) * 1000L;
  reporting_interval = eeprom_read_word((uint16_t * ) EEPROM_REPORTING_INTERVAL) * 1000L;
  sample_buffer_depth = (uint16_t) ((((uint32_t) eeprom_read_word((uint16_t * ) EEPROM_AVERAGING_INTERVAL)) * 1000L) / sampling_interval);
  
  if(mode == SUBMODE_NORMAL){
    setLCD_P(PSTR("TEMP ---  RH ---"
                  "NO2  ---  CO ---"));           
    SUCCESS_MESSAGE_DELAY();                      
  }
  
  resumeGpsProcessing();
}

void loop() {  
  current_millis = millis();
  
  // whenever you come through loop, process a GPS byte if there is one
  // will need to test if this keeps up, but I think it will      
  if(user_location_override){
    // hardware doesn't matter in this case
    updateGpsStrings();
  }
  else if(!gps_disabled){       
    while(gpsSerial.available()){            
      char c = gpsSerial.read();

      if(c == '$'){
        gps_installed = true;
      }
      
      if(gps.encode(c)){
        gps.f_get_position(&gps_latitude, &gps_longitude, &gps_age);
        gps_altitude = gps.f_altitude();
        updateGpsStrings();        
        break;
      }
    }
  }   
  
  if(current_millis - previous_sensor_sampling_millis >= sampling_interval){
    suspendGpsProcessing();
    previous_sensor_sampling_millis = current_millis;    
    //Serial.print(F("Info: Sampling Sensors @ "));
    //Serial.println(millis());
    collectNO2();
    collectCO();
    collectTemperature();
    collectHumidity(); 
    advanceSampleBufferIndex(); 
  }

  if(current_millis - previous_touch_sampling_millis >= touch_sampling_interval){
    suspendGpsProcessing();
    previous_touch_sampling_millis = current_millis;    
    collectTouch();    
    processTouchQuietly();  
  }  

  // the following loop routines *must* return reasonably frequently
  // so that the watchdog timer is serviced
  switch(mode){
    case SUBMODE_NORMAL:
      loop_wifi_mqtt_mode();
      break;
    case SUBMODE_OFFLINE:
      loop_offline_mode();
      break;
    default: // unkown operating mode, nothing to be done 
      break;
  }
  
  // pet the watchdog
  if (current_millis - previous_tinywdt_millis >= tinywdt_interval) {
    suspendGpsProcessing();
    previous_tinywdt_millis = current_millis;
    //Serial.println(F("Info: Watchdog Pet."));
    delayForWatchdog();
    petWatchdog();
  }

  if(gps_disabled){
    resumeGpsProcessing();
  }
}

/****** INITIALIZATION SUPPORT FUNCTIONS ******/
void ERROR_MESSAGE_DELAY(void){
  delay(LCD_ERROR_MESSAGE_DELAY);
}

void SUCCESS_MESSAGE_DELAY(void){
  delay(LCD_SUCCESS_MESSAGE_DELAY);
}

void init_firmware_version(void){
  snprintf(firmware_version, 15, "%d.%d.%d", 
    AQEV2FW_MAJOR_VERSION, 
    AQEV2FW_MINOR_VERSION, 
    AQEV2FW_PATCH_VERSION);
}

void initializeHardware(void) {
  Serial.begin(115200);
  Serial1.begin(115200);
  
  init_firmware_version();
  
  // without this line, if the touch hardware is absent
  // serial input processing grinds to a snails pace
  touch.set_CS_Timeout_Millis(100); 

  Serial.println(F(" +------------------------------------+"));
  Serial.println(F(" |   Welcome to Air Quality Egg 2.0   |"));
  Serial.println(F(" |       NO2 / CO Sensor Suite        |"));  
  Serial.print(F(" |       Firmware Version "));
  Serial.print(firmware_version);
  Serial.println(F("       |"));
  Serial.println(F(" +------------------------------------+"));
  Serial.print(F(" Compiled on: "));
  Serial.println(__DATE__ " " __TIME__);
  Serial.print(F(" Egg Serial Number: "));
  print_eeprom_mqtt_client_id();
  Serial.println();
  
  // Initialize Tiny Watchdog
  Serial.print(F("Info: Tiny Watchdog Initialization..."));
  watchdogInitialize();
  Serial.println(F("OK."));

  pinMode(A6, OUTPUT);
  uint8_t backlight_behavior = eeprom_read_byte((uint8_t *) EEPROM_BACKLIGHT_STARTUP);
  if((BACKLIGHT_ON_AT_STARTUP == backlight_behavior) || (backlight_behavior == BACKLIGHT_ALWAYS_ON)){
    backlightOn();
  }
  else{
    backlightOff();
  }

  // smiley face
  byte smiley[8] = {
          B00000,
          B00000,
          B01010,
          B00000,
          B10001,
          B01110,
          B00000,
          B00000
  };
  
  byte frownie[8] = {
          B00000,
          B00000,
          B01010,
          B00000,
          B01110,
          B10001,
          B00000,
          B00000
  };
  
  byte emptybar[8] = {
          B11111,
          B10001,
          B10001,
          B10001,
          B10001,
          B10001,
          B10001,
          B11111
  };
  
  byte fullbar[8] = {
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111,
          B11111
  };     

  lcd.begin(16, 2);
  
  lcd.createChar(0, smiley);
  lcd.createChar(1, frownie);  
  lcd.createChar(2, emptybar);
  lcd.createChar(3, fullbar);
  
  setLCD_P(PSTR("AIR QUALITY EGG "));
  char tmp[17] = {0};
  snprintf(tmp, 16, "VERSION %d.%d.%d", 
    AQEV2FW_MAJOR_VERSION, 
    AQEV2FW_MINOR_VERSION,
    AQEV2FW_PATCH_VERSION);
    
  updateLCD(tmp, 1);
  
  Wire.begin();

  // Initialize slot select pins
  Serial.print(F("Info: Slot Select Pins Initialization..."));
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  selectNoSlot();
  Serial.println(F("OK."));
  
  // Initialize SPI Flash
  Serial.print(F("Info: SPI Flash Initialization..."));
  if (flash.initialize()) {
    Serial.println(F("OK."));
    init_spi_flash_ok = true;
  }
  else {
    Serial.println(F("Fail."));
    init_spi_flash_ok = false;
  }  
  
  // Initialize SD card
  Serial.print(F("Info: SD Card Initialization..."));        
  if (SD.begin(16)) {
    Serial.println(F("OK."));     
    init_sdcard_ok = true;        
  }
  else{
    Serial.println(F("Fail.")); 
    init_sdcard_ok = false;  
  }

  getCurrentFirmwareSignature();

  // Initialize SHT25
  Serial.print(F("Info: SHT25 Initialization..."));
  if (sht25.begin()) {
    Serial.println(F("OK."));
    init_sht25_ok = true;
  }
  else {
    Serial.println(F("Failed."));
    init_sht25_ok = false;
  }

  // Initialize NO2 Sensor
  Serial.print(F("Info: NO2 Sensor AFE Initialization..."));
  selectSlot2();
  if (lmp91000.configure(
        LMP91000_TIA_GAIN_350K | LMP91000_RLOAD_10OHM,
        LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_67PCT
        | LMP91000_BIAS_SIGN_NEG | LMP91000_BIAS_8PCT,
        LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_AMPEROMETRIC)) {
    Serial.println(F("OK."));
    init_no2_afe_ok = true;
  }
  else {
    Serial.println(F("Failed."));
    init_no2_afe_ok = false;
  }

  Serial.print(F("Info: NO2 Sensor ADC Initialization..."));
  if(MCP342x::errorNone == adc.convert(MCP342x::channel1, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1)){
    Serial.println(F("OK."));
    init_no2_adc_ok = true;    
  }
  else{
    Serial.println(F("Failed."));
    init_no2_adc_ok = false;    
  }

  Serial.print(F("Info: CO Sensor AFE Initialization..."));
  selectSlot1();
  if (lmp91000.configure(
        LMP91000_TIA_GAIN_350K | LMP91000_RLOAD_10OHM,
        LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_20PCT
        | LMP91000_BIAS_SIGN_POS | LMP91000_BIAS_1PCT,
        LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_AMPEROMETRIC)) {
    Serial.println(F("OK."));
    init_co_afe_ok = true;
  }
  else {
    Serial.println(F("Failed."));
    init_co_afe_ok = false;
  }
  
  Serial.print(F("Info: CO Sensor ADC Initialization..."));
  if(MCP342x::errorNone == adc.convert(MCP342x::channel1, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1)){
    Serial.println(F("OK."));
    init_co_adc_ok = true;    
  }
  else{
    Serial.println(F("Failed."));
    init_co_adc_ok = false;    
  }

  // Initialize SD card
  Serial.print(F("Info: RTC Initialization..."));   
  selectSlot3();  
  rtc.begin();
  if (rtc.isrunning()) {
    Serial.println(F("OK."));   
    setSyncProvider(AQE_now);  
    init_rtc_ok = true;       
  }
  else{
    Serial.println(F("Fail.")); 
    init_rtc_ok = false;  
  }

  selectNoSlot();

  uint8_t connect_method = eeprom_read_byte((const uint8_t *) EEPROM_CONNECT_METHOD);
  Serial.print(F("Info: ESP8266 Initialization..."));  
  SUCCESS_MESSAGE_DELAY(); // don't race past the splash screen, and give watchdog some breathing room
  petWatchdog();

  esp.setTcpKeepAliveInterval(10); // 10 seconds
  esp.setInputBuffer(esp8266_input_buffer, ESP8266_INPUT_BUFFER_SIZE); // connect the input buffer up   
  if (esp.reset()) {
    esp.setNetworkMode(1);
    Serial.println(F("OK."));
    init_esp8266_ok = true;
  }
  else {
    Serial.println(F("Failed."));
    init_esp8266_ok = false;
  }
  
  updateLCD("NO2 / CO", 0);
  updateLCD("MODEL", 1);
  SUCCESS_MESSAGE_DELAY();  
  
}

/****** CONFIGURATION SUPPORT FUNCTIONS ******/
void initializeNewConfigSettings(void){  
  char * command_buf = &(scratch[0]);
  clearTempBuffers();
  
  boolean in_config_mode = false; 
  allowed_to_write_config_eeprom = true;
  
  // backlight settings
  uint8_t backlight_startup = eeprom_read_byte((uint8_t *) EEPROM_BACKLIGHT_STARTUP);
  uint16_t backlight_duration = eeprom_read_word((uint16_t *) EEPROM_BACKLIGHT_DURATION);
  if((backlight_startup == 0xFF) || (backlight_duration == 0xFFFF)){
    configInject("aqe\r");    
    configInject("backlight initon\r");
    configInject("backlight 60\r");
    in_config_mode = true;
  }
  
  // sampling settings
  uint16_t l_sampling_interval = eeprom_read_word((uint16_t * ) EEPROM_SAMPLING_INTERVAL);
  uint16_t l_reporting_interval = eeprom_read_word((uint16_t * ) EEPROM_REPORTING_INTERVAL);
  uint16_t l_averaging_interval = eeprom_read_word((uint16_t * ) EEPROM_AVERAGING_INTERVAL);
  if((l_sampling_interval == 0xFFFF) || (l_reporting_interval == 0xFFFF) || (l_averaging_interval == 0xFFFF)){
    if(!in_config_mode){
      configInject("aqe\r");
      in_config_mode = true;
    }
    configInject("sampling 5, 160, 5\r");    
  }    

  // the following two blocks of code are a 'hot-fix' to the slope calculation, 
  // only apply it if the slope is not already self consistent with the sensitivity
  float sensitivity = eeprom_read_float((const float *) EEPROM_NO2_SENSITIVITY);  
  float calculated_slope = convert_no2_sensitivity_to_slope(sensitivity);
  float stored_slope = eeprom_read_float((const float *) EEPROM_NO2_CAL_SLOPE);
  if(calculated_slope != stored_slope){ 
    if(!in_config_mode){
      configInject("aqe\r");
      in_config_mode = true;
    }    
    memset(command_buf, 0, 128);  
    snprintf(command_buf, 127, "no2_sen %8.4f\r", sensitivity);
    configInject(command_buf);
    configInject("backup no2\r");
  }
  
  sensitivity = eeprom_read_float((const float *) EEPROM_CO_SENSITIVITY);  
  calculated_slope = convert_co_sensitivity_to_slope(sensitivity);
  stored_slope = eeprom_read_float((const float *) EEPROM_CO_CAL_SLOPE);
  if(calculated_slope != stored_slope){ 
    if(!in_config_mode){
      configInject("aqe\r");
      in_config_mode = true;
    }    
    memset(command_buf, 0, 128);  
    snprintf(command_buf, 127, "co_sen %8.4f\r", sensitivity);
    configInject(command_buf);  
    configInject("backup co\r");
  }  

  // if necessary, initialize the default mqtt prefix
  // if it's never been set, the first byte in memory will be 0xFF
  uint8_t val = eeprom_read_byte((const uint8_t *) EEPROM_MQTT_TOPIC_PREFIX);  
  if(val == 0xFF){
    if(!in_config_mode){
      configInject("aqe\r");
      in_config_mode = true;
    }    
    memset(command_buf, 0, 128);
    strcat(command_buf, "mqttprefix ");
    strcat(command_buf, MQTT_TOPIC_PREFIX);
    strcat(command_buf, "\r");
    configInject(command_buf);
  }

  // if the mqtt server is set to opensensors.io, change it to mqtt.opensensors.io
  memset(command_buf, 0, 128);
  eeprom_read_block(command_buf, (const void *) EEPROM_MQTT_SERVER_NAME, 31);
  if(strcmp_P(command_buf, PSTR("opensensors.io")) == 0){
    if(!in_config_mode){
      configInject("aqe\r");
      in_config_mode = true;
    }    
    configInject("mqttsrv mqtt.opensensors.io\r");
  }
  
  // if the mqtt suffix enable is neither zero nor one, set it to one (enabled)
  val = eeprom_read_byte((const uint8_t *) EEPROM_MQTT_TOPIC_SUFFIX_ENABLED);  
  if(val == 0xFF){
    if(!in_config_mode){
      configInject("aqe\r");
      in_config_mode = true;
    }    
    memset(command_buf, 0, 128);
    strcat(command_buf, "mqttsuffix enable\r");        
    configInject(command_buf);    
  }
  
  if(in_config_mode){
    configInject("exit\r");
  }
  
  allowed_to_write_config_eeprom = false;  
}

boolean checkConfigIntegrity(void) {
  uint16_t computed_crc = computeEepromChecksum();
  uint16_t stored_crc = getStoredEepromChecksum();
  if (computed_crc == stored_crc) {
    return true;
  }
  else {
    //Serial.print(F("Computed CRC = "));
    //Serial.print(computed_crc, HEX);
    //Serial.print(F(", Stored CRC = "));
    //Serial.println(stored_crc, HEX);
    return false;
  }
}

// this state machine receives bytes and
// returns true if the function is in config mode
uint8_t configModeStateMachine(char b, boolean reset_buffers) {
  static boolean received_init_code = false;
  const uint8_t buf_max_write_idx = 126; // [127] must always have a null-terminator
  static char buf[128] = {0}; // buffer to hold commands / data
  static uint8_t buf_idx = 0;  // current number of bytes in buf
  boolean line_terminated = false;
  char * first_arg = 0;
  uint8_t ret = CONFIG_MODE_NOTHING_SPECIAL;

  if (reset_buffers) {
    buf_idx = 0;
  }

  //  Serial.print('[');
  //  if(isprint(b)) Serial.print((char) b);
  //  Serial.print(']');
  //  Serial.print('\t');
  //  Serial.print("0x");
  //  if(b < 0x10) Serial.print('0');
  //  Serial.println(b, HEX);

  // if you are at the last write-able location in the buffer
  // the only legal characters to accept are a backspace, a newline, or a carriage return
  // reject anything else implicitly
  if((buf_idx == buf_max_write_idx) && (b != 0x7F) && (b != 0x0D) && (b != 0x0A)){
    Serial.println(F("Warn: Input buffer full and cannot accept new characters. Press enter to clear buffers."));
  }
  // the following logic rejects all non-printable characters besides 0D, 0A, and 7F
  else if (b == 0x7F) { // backspace key is special
    if (buf_idx > 0) {
      buf_idx--;
      buf[buf_idx] = '\0';
      Serial.print(b); // echo the character
    }
  }
  else if (b == 0x0D || b == 0x0A) { // carriage return or new line is also special
    buf[buf_idx] = '\0'; // force terminator do not advance write pointer
    line_terminated = true;
    Serial.println(); // echo the character
  }
  else if ((buf_idx <= buf_max_write_idx) && isprint(b)) {
    // otherwise if there's space and the character is 'printable' add it to the buffer
    // silently drop all other non-printable characters
    buf[buf_idx++] = b;
    buf[buf_idx] = '\0';
    Serial.print(b); // echo the character
  }

  char lower_buf[128] = {0};
  if (line_terminated) {
    strncpy(lower_buf, buf, 127);
    lowercase(lower_buf);
  }

  // process the data currently stored in the buffer
  if (received_init_code && line_terminated) {
    // with the exeption of the command "exit"
    // commands are always of the form <command> <argument>
    // they are minimally parsed here and delegated to
    // callback functions that take the argument as a string

    // Serial.print("buf = ");
    // Serial.println(buf);
    
    if (strncmp("aqe", lower_buf, 3) == 0) {
      ret = CONFIG_MODE_GOT_INIT;
    }
    if (strncmp("exit", lower_buf, 4) == 0) {
      Serial.println(F("Exiting CONFIG mode..."));
      ret = CONFIG_MODE_GOT_EXIT;
    }
    else {
      // the string must have one, and only one, space in it
      uint8_t num_spaces = 0;
      char * p;
      for (p = buf; *p != '\0'; p++) { // all lines are terminated by '\r' above
        if (*p == ' ') {
          num_spaces++;
        }

        if ((num_spaces == 1) && (*p == ' ')) {
          // if this is the first space encountered, null the original string here
          // in order to mark the first argument string
          *p = '\0';
        }
        else if ((num_spaces > 0) && (first_arg == 0) && (*p != ' ')) {
          // if we are beyond the first space,
          // and have not encountered the beginning of the first argument
          // and this character is not a space, it is by definition
          // the beginning of the first argument, so mark it as such
          first_arg = p;
        }
      }

      // deal with commands that can legitimately have no arguments first
      if (strncmp("help", lower_buf, 4) == 0) {
        help_menu(first_arg);
      }
      else if(strncmp("pwd", lower_buf, 3) == 0) {
        set_network_password(first_arg);
      }
      else if (first_arg != 0) {
        //Serial.print(F("Received Command: \""));
        //Serial.print(buf);
        //Serial.print(F("\" with Argument: \""));
        //Serial.print(first_arg);
        //Serial.print(F("\""));
        //Serial.println();

        // command with argument was received, determine if it's valid
        // and if so, call the appropriate command processing function
        boolean command_found = false;
        char _temp_command[16] = {0};
        uint8_t ii = 0;
        do{    
          strcpy_P(_temp_command, (PGM_P) pgm_read_word(&(commands[ii])));    
          if (strncmp(_temp_command, lower_buf, strlen(buf)) == 0) {
            command_functions[ii](first_arg);
            command_found = true;
            break;
          }
          ii++;
        } while(_temp_command[0] != 0);

        if (!command_found) {
          Serial.print(F("Error: Unknown command \""));
          Serial.print(buf);
          Serial.println(F("\""));
        }
      }
      else if (strlen(buf) > 0) {
        Serial.print(F("Error: Argument expected for command \""));
        Serial.print(buf);
        Serial.println(F("\", but none was received"));
      }
    }
  }
  else if (line_terminated) {
    // before we receive the init code, the only things
    // we are looking for are an exact match to the strings
    // "AQE\r" or "aqe\r"

    if (strncmp("aqe", lower_buf, 3) == 0) {
      received_init_code = true;
      ret = CONFIG_MODE_GOT_INIT;
    }
    else if (strlen(buf) > 0) {
      Serial.print(F("Error: Expecting Config Mode Unlock Code (\"aqe\"), but received \""));
      Serial.print(buf);
      Serial.println(F("\""));
    }
  }

  // clean up the buffer if you got a line termination
  if (line_terminated) {
    if (ret == CONFIG_MODE_NOTHING_SPECIAL) {
      prompt();
    }
    buf[0] = '\0';
    buf_idx = 0;
  }

  return ret;
}

void prompt(void) {
  Serial.print(F("AQE>: "));
}

// command processing function implementations
void configInject(char * str) {
  boolean reset_buffers = true;
  while (*str != '\0') {
    boolean got_exit = false;
    got_exit = configModeStateMachine(*str++, reset_buffers);
    if (reset_buffers) {
      reset_buffers = false;
    }
  }
}

void lowercase(char * str) {
  uint16_t len = strlen(str);
  if (len < 0xFFFF) {
    for (uint16_t ii = 0; ii < len; ii++) {
      str[ii] = tolower(str[ii]);
    }
  }
}

void note_know_what_youre_doing(){
  Serial.println(F("   note:    Unless you *really* know what you're doing, you should"));
  Serial.println(F("            probably not be using this command."));  
}

void warn_could_break_upload(){
  Serial.println(F("   warning: Using this command incorrectly can prevent your device"));
  Serial.println(F("            from publishing data to the internet."));  
}

void warn_could_break_connect(){
  Serial.println(F("   warning: Using this command incorrectly can prevent your device"));
  Serial.println(F("            from connecting to your network."));  
}

void defaults_help_indent(void){
  Serial.print(F("                     "));
}

void get_help_indent(void){
  Serial.print(F("      "));
}

void help_menu(char * arg) {
  const uint8_t commands_per_line = 3;
  const uint8_t first_dynamic_command_index = 2;

  lowercase(arg);

  if (arg == 0) {
    // list the commands that are legal
    Serial.print(F("help    \texit    \t"));
    char _temp_command[16] = {0};
    uint8_t ii = 0;
    uint8_t jj = first_dynamic_command_index;     
    do{        
      strcpy_P(_temp_command, (PGM_P) pgm_read_word(&(commands[ii])));
      
      if ((jj % commands_per_line) == 0) {
        Serial.println();
      }
      //Serial.print(jj + 1);
      //Serial.print(". ");
      Serial.print(_temp_command);
      Serial.print('\t');

      ii++; 
      jj++;
    } while(_temp_command[0] != 0);
    Serial.println();
  }
  else {
    Serial.println(F("Please visit http://airqualityegg.wickeddevice.com/help for command line usage details"));
  }
}

void print_eeprom_mac(void) {
  uint8_t _mac_address[6] = {0};
  // retrieve the value from EEPROM
  eeprom_read_block(_mac_address, (const void *) EEPROM_MAC_ADDRESS, 6);

  // print the stored value, formatted
  for (uint8_t ii = 0; ii < 6; ii++) {
    if (_mac_address[ii] < 0x10) {
      Serial.print(F("0"));
    }
    Serial.print(_mac_address[ii], HEX);

    // only print colons after the first 5 values
    if (ii < 5) {
      Serial.print(F(":"));
    }
  }
  Serial.println();
}

void print_eeprom_connect_method(void) {
  uint8_t method = eeprom_read_byte((const uint8_t *) EEPROM_CONNECT_METHOD);
  switch (method) {
    case CONNECT_METHOD_DIRECT:
      Serial.println(F("Direct Connect"));
      break;
    default:
      Serial.print(F("Error: Unknown connection method code [0x"));
      if (method < 0x10) {
        Serial.print(F("0"));
      }
      Serial.print(method, HEX);
      Serial.println(F("]"));
      break;
  }
}

boolean valid_ssid_config(void) {
  char ssid[33] = {0};
  boolean ssid_contains_only_printables = true;

  uint8_t connect_method = eeprom_read_byte((const uint8_t *) EEPROM_CONNECT_METHOD);      
  eeprom_read_block(ssid, (const void *) EEPROM_SSID, 32);
  for (uint8_t ii = 0; ii <= 32; ii++) {
    if (ssid[ii] == '\0') {
      break;
    }
    else if (!isprint(ssid[ii])) {
      ssid_contains_only_printables = false;
      break;
    }
  }

  if (!ssid_contains_only_printables || (strlen(ssid) == 0)) {
    return false;
  }

  return true;
}

void print_eeprom_ssid(void) {
  char ssid[33] = {0};
  eeprom_read_block(ssid, (const void *) EEPROM_SSID, 32);

  if (!valid_ssid_config()) {
    Serial.println(F("No SSID currently configured."));
  }
  else {
    Serial.println(ssid);
  }
}

void print_eeprom_security_type(void) {
  uint8_t security = eeprom_read_byte((const uint8_t *) EEPROM_SECURITY_MODE);
  switch (security) {
    case 0:
      Serial.println(F("Open"));
      break;
    case 1:
      Serial.println(F("WEP"));
      break;
    case 2:
      Serial.println(F("WPA"));
      break;
    case 3:
      Serial.println(F("WPA2"));
      break;
    case WLAN_SEC_AUTO:
      Serial.println(F("Automatic - Not Yet Determined"));
      break;
    default:
      Serial.print(F("Error: Unknown security mode code [0x"));
      if (security < 0x10) {
        Serial.print(F("0"));
      }
      Serial.print(security, HEX);
      Serial.println(F("]"));
      break;
  }
}

void print_eeprom_ipmode(void) {
  uint8_t ip[4] = {0};
  uint8_t netmask[4] = {0};
  uint8_t gateway[4] = {0};
  uint8_t dns[4] = {0};
  uint8_t noip[4] = {0};
  eeprom_read_block(ip, (const void *) EEPROM_STATIC_IP_ADDRESS, 4);
  eeprom_read_block(netmask, (const void *) EEPROM_STATIC_NETMASK, 4);
  eeprom_read_block(gateway, (const void *) EEPROM_STATIC_GATEWAY, 4);
  eeprom_read_block(dns, (const void *) EEPROM_STATIC_DNS, 4);
  
  if (memcmp(ip, noip, 4) == 0) {
    Serial.println(F("Configured for DHCP"));
  }
  else {
    Serial.println(F("Configured for Static IP: "));
    for(uint8_t param_idx = 0; param_idx < 4; param_idx++){         
      for (uint8_t ii = 0; ii < 4; ii++) {
        switch(param_idx){
          case 0:
            if(ii == 0){
              Serial.print(F("   IP Address:      "));
            }
            Serial.print(ip[ii], DEC);
            break;
          case 1:
            if(ii == 0){
              Serial.print(F("   Netmask:         "));
            }
            Serial.print(netmask[ii], DEC);
            break;
          case 2:
            if(ii == 0){
              Serial.print(F("   Default Gateway: "));
            }
            Serial.print(gateway[ii], DEC);
            break;
          case 3:
            if(ii == 0){
              Serial.print(F("   DNS Server:      "));
            }         
            Serial.print(dns[ii], DEC); 
            break;
        }   
        
        if( ii != 3 ){        
          Serial.print(F("."));
        }
        else{
          Serial.println(); 
        }
      }      
    }
  }
}

void print_eeprom_float(const float * address) {
  float val = eeprom_read_float(address);
  Serial.println(val, 9);
}

void print_label_with_star_if_not_backed_up(char * label, uint8_t bit_number) {
  uint16_t backup_check = eeprom_read_word((const uint16_t *) EEPROM_BACKUP_CHECK);
  Serial.print(F("  "));
  if (!BIT_IS_CLEARED(backup_check, bit_number)) {
    Serial.print(F("*"));
  }
  else {
    Serial.print(F(" "));
  }
  Serial.print(F(" "));
  Serial.print(label);
}

void print_eeprom_string(const char * address){
  char tmp[32] = {0};
  eeprom_read_block(tmp, (const void *) address, 31);
  Serial.println(tmp);
}

void print_eeprom_string(const char * address, const char * unless_it_matches_this, const char * in_which_case_print_this_instead){
  char tmp[32] = {0};
  eeprom_read_block(tmp, (const void *) address, 31);

  if(strcmp(tmp, unless_it_matches_this) == 0){
    Serial.println(in_which_case_print_this_instead);
  }
  else{
    Serial.println(tmp);
  }
}

void print_eeprom_update_server(){
  print_eeprom_string((const char *) EEPROM_UPDATE_SERVER_NAME, "", "Disabled");
}

void print_eeprom_ntp_server(){
  print_eeprom_string((const char *) EEPROM_NTP_SERVER_NAME, "", "Disabled");
}

void print_eeprom_update_filename(){
  print_eeprom_string((const char *) EEPROM_UPDATE_FILENAME);
}  

void print_eeprom_mqtt_server(){
  print_eeprom_string((const char *) EEPROM_MQTT_SERVER_NAME);
}


void print_eeprom_mqtt_client_id(){
  print_eeprom_string((const char *) EEPROM_MQTT_CLIENT_ID);
}

void print_eeprom_mqtt_topic_prefix(){
  print_eeprom_string((const char *) EEPROM_MQTT_TOPIC_PREFIX);
}

void print_eeprom_mqtt_topic_suffix(){
  uint8_t val = eeprom_read_byte((uint8_t * ) EEPROM_MQTT_TOPIC_SUFFIX_ENABLED);
  if(val == 1){
    Serial.print("Enabled");
  }
  else if(val == 0){
    Serial.print("Disabled");
  }
  else{
    Serial.print("Uninitialized");
  }
  Serial.println();
}

void print_eeprom_mqtt_username(){
  print_eeprom_string((const char *) EEPROM_MQTT_USERNAME);
}

void print_eeprom_mqtt_authentication(){
  uint8_t auth = eeprom_read_byte((uint8_t *) EEPROM_MQTT_AUTH);
  if(auth){
    Serial.println(F("    MQTT Authentication: Enabled"));    
    Serial.print(F("    MQTT Username: "));
    print_eeprom_mqtt_username();              
  }
  else{
    Serial.println(F("    MQTT Authentication: Disabled"));
  }
}

void print_eeprom_operational_mode(uint8_t opmode){   
  switch (opmode) {
    case SUBMODE_NORMAL:
      Serial.println(F("Normal"));
      break;
    case SUBMODE_OFFLINE:
      Serial.println(F("Offline"));
      break;
    default:
      Serial.print(F("Error: Unknown operational mode [0x"));
      if (opmode < 0x10) {
        Serial.print(F("0"));
      }
      Serial.print(opmode, HEX);
      Serial.println(F("]"));
      break;
  }  
}

void print_eeprom_temperature_units(){   
  uint8_t tempunit = eeprom_read_byte((uint8_t *) EEPROM_TEMPERATURE_UNITS);
  switch (tempunit) {
    case 'C':
      Serial.println(F("Celsius"));
      break;
    case 'F':
      Serial.println(F("Fahrenheit"));
      break;
    default:
      Serial.print(F("Error: Unknown temperature units [0x"));
      if (tempunit < 0x10) {
        Serial.print(F("0"));
      }
      Serial.print(tempunit, HEX);
      Serial.println(F("]"));
      break;
  }  
}

void print_altitude_settings(void){
  int16_t l_altitude = (int16_t) eeprom_read_word((uint16_t *) EEPROM_ALTITUDE_METERS);
  if(l_altitude != -1){
    Serial.print(l_altitude);
    Serial.println(F(" meters"));  
  }
  else{
    Serial.println("Not set");
  }
}

void print_latitude_settings(void){
  int16_t l_latitude = (int16_t) eeprom_read_word((uint16_t *) EEPROM_USER_LATITUDE_DEG);
  float f_latitude = eeprom_read_float((float *) EEPROM_USER_LATITUDE_DEG);
  if(l_latitude != -1){
    Serial.print(f_latitude, 6);
    Serial.println(F(" degrees"));  
  }
  else{
    Serial.println("Not set");
  }
}

void print_longitude_settings(void){
  int16_t l_longitude = (int16_t) eeprom_read_word((uint16_t *) EEPROM_USER_LONGITUDE_DEG);
  float f_longitude = eeprom_read_float((float *) EEPROM_USER_LONGITUDE_DEG);
  if(l_longitude != -1){
    Serial.print(f_longitude, 6);
    Serial.println(F(" degrees"));  
  }
  else{
    Serial.println("Not set");
  }
}

void print_eeprom_backlight(){   
  uint16_t backlight_duration = eeprom_read_word((uint16_t *) EEPROM_BACKLIGHT_DURATION);
  uint8_t backlight_startup = eeprom_read_byte((uint8_t *) EEPROM_BACKLIGHT_STARTUP);
  Serial.print(backlight_duration);
  Serial.print(F(" seconds, "));
  switch(backlight_startup){
    case BACKLIGHT_ON_AT_STARTUP:
      Serial.println(F("ON at startup"));
      break;
    case BACKLIGHT_OFF_AT_STARTUP:
      Serial.println(F("OFF at startup"));    
      break;
    case BACKLIGHT_ALWAYS_ON:
      Serial.println(F("always ON"));    
      break;
    case BACKLIGHT_ALWAYS_OFF:
      Serial.println(F("always OFF"));        
      break;
  }   
}

void print_eeprom_value(char * arg) {
  if (strncmp(arg, "mac", 3) == 0) {
    print_eeprom_mac();
  }
  else if (strncmp(arg, "method", 6) == 0) {
    print_eeprom_connect_method();
  }
  else if (strncmp(arg, "ssid", 4) == 0) {
    print_eeprom_ssid();
  }
  else if (strncmp(arg, "security", 8) == 0) {
    print_eeprom_security_type();
  }
  else if (strncmp(arg, "ipmode", 6) == 0) {
    print_eeprom_ipmode();
  }
  else if (strncmp(arg, "no2_sen", 7) == 0) {
    print_eeprom_float((const float *) EEPROM_NO2_SENSITIVITY);
  }
  else if (strncmp(arg, "no2_slope", 9) == 0) {
    print_eeprom_float((const float *) EEPROM_NO2_CAL_SLOPE);
  }
  else if (strncmp(arg, "no2_off", 7) == 0) {
    print_eeprom_float((const float *) EEPROM_NO2_CAL_OFFSET);
  }
  else if (strncmp(arg, "co_sen", 6) == 0) {
    print_eeprom_float((const float *) EEPROM_CO_SENSITIVITY);
  }
  else if (strncmp(arg, "co_slope", 8) == 0) {
    print_eeprom_float((const float *) EEPROM_CO_CAL_SLOPE);
  }
  else if (strncmp(arg, "co_off", 6) == 0) {
    print_eeprom_float((const float *) EEPROM_CO_CAL_OFFSET);
  }
  else if (strncmp(arg, "temp_off", 8) == 0) {
    print_eeprom_float((const float *) EEPROM_TEMPERATURE_OFFSET);
  }
  else if (strncmp(arg, "hum_off", 7) == 0) {
    print_eeprom_float((const float *) EEPROM_HUMIDITY_OFFSET);
  }  
  else if(strncmp(arg, "mqttsrv", 7) == 0) {
    print_eeprom_string((const char *) EEPROM_MQTT_SERVER_NAME);    
  }
  else if(strncmp(arg, "mqttport", 8) == 0) {
    Serial.println(eeprom_read_dword((const uint32_t *) EEPROM_MQTT_PORT));      
  }  
  else if(strncmp(arg, "mqttuser", 8) == 0) {
    print_eeprom_string((const char *) EEPROM_MQTT_USERNAME);    
  }  
  else if(strncmp(arg, "mqttid", 6) == 0) {
    print_eeprom_string((const char *) EEPROM_MQTT_CLIENT_ID);    
  }
  else if(strncmp(arg, "mqttauth", 8) == 0) {
    Serial.println(eeprom_read_byte((const uint8_t *) EEPROM_MQTT_AUTH));    
  }
  else if(strncmp(arg, "opmode", 6) == 0) {
     print_eeprom_operational_mode(eeprom_read_byte((const uint8_t *) EEPROM_OPERATIONAL_MODE));
  }
  else if(strncmp(arg, "tempunit", 8) == 0) {
     print_eeprom_temperature_units();
  }
  else if(strncmp(arg, "backlight", 9) == 0) {
    print_eeprom_backlight();
  }
  else if(strncmp(arg, "timestamp", 9) == 0) {
    printCurrentTimestamp(NULL, NULL);
    Serial.println();
  } 
  else if(strncmp(arg, "updatesrv", 9) == 0) {
    print_eeprom_update_server();    
  }  
  else if(strncmp(arg, "ntpsrv", 6) == 0) {
    print_eeprom_ntp_server();    
  }  
  else if(strncmp(arg, "updatefile", 10) == 0) {
    print_eeprom_string((const char *) EEPROM_UPDATE_FILENAME);    
  }  
  else if(strncmp(arg, "sampleint", 9) == 0) {
    Serial.println(eeprom_read_word((uint16_t *) EEPROM_SAMPLING_INTERVAL));    
  }   
  else if(strncmp(arg, "reportint", 9) == 0) {
    Serial.println(eeprom_read_word((uint16_t *) EEPROM_REPORTING_INTERVAL));    
  }     
  else if(strncmp(arg, "avgint", 6) == 0) {
    Serial.println(eeprom_read_word((uint16_t *) EEPROM_AVERAGING_INTERVAL));    
  } 
  else if(strncmp(arg, "altitude", 8) == 0) {
    Serial.println((int16_t) eeprom_read_word((uint16_t *) EEPROM_ALTITUDE_METERS));    
  }
  else if(strncmp(arg, "latitude", 8) == 0) {
    Serial.println((int16_t) eeprom_read_float((float *) EEPROM_USER_LATITUDE_DEG));    
  }
  else if(strncmp(arg, "longitude", 8) == 0) {
    Serial.println((int16_t) eeprom_read_float((float *) EEPROM_USER_LONGITUDE_DEG));    
  }             
  else if(strncmp(arg, "settings", 8) == 0) {   
    // print all the settings to the screen in an orderly fashion
    Serial.println(F(" +-------------------------------------------------------------+"));
    Serial.println(F(" | Preferences/Options:                                        |"));
    Serial.println(F(" +-------------------------------------------------------------+"));
    Serial.print(F("    Operational Mode: "));
    print_eeprom_operational_mode(eeprom_read_byte((const uint8_t *) EEPROM_OPERATIONAL_MODE));
    Serial.print(F("    Temperature Units: "));
    print_eeprom_temperature_units();
    Serial.print(F("    Backlight Settings: "));
    print_eeprom_backlight();
    Serial.print(F("    Sensor Sampling Interval: "));
    Serial.print(eeprom_read_word((uint16_t *) EEPROM_SAMPLING_INTERVAL));  
    Serial.println(F(" seconds"));
    Serial.print(F("    Sensor Averaging Interval: "));
    Serial.print(eeprom_read_word((uint16_t *) EEPROM_AVERAGING_INTERVAL));  
    Serial.println(F(" seconds"));
    Serial.print(F("    Sensor Reporting Interval: "));
    Serial.print(eeprom_read_word((uint16_t *) EEPROM_REPORTING_INTERVAL));  
    Serial.println(F(" seconds"));

    Serial.println(F(" +-------------------------------------------------------------+"));
    Serial.println(F(" | Location Settings:                                          |"));
    Serial.println(F(" +-------------------------------------------------------------+"));
    Serial.print(F("    User Location: "));
    if(eeprom_read_byte((uint8_t *) EEPROM_USER_LOCATION_EN) == 1){
      Serial.println(F("Enabled"));
    }
    else{
      Serial.println(F("Disabled"));
    }  
    Serial.print(F("    User Latitude: "));
    print_latitude_settings();
    Serial.print(F("    User Longitude: "));
    print_longitude_settings();
    Serial.print(F("    User Altitude: "));
    print_altitude_settings();    
    
    Serial.println(F(" +-------------------------------------------------------------+"));
    Serial.println(F(" | Network Settings:                                           |"));
    Serial.println(F(" +-------------------------------------------------------------+"));
    print_label_with_star_if_not_backed_up("MAC Address: ", BACKUP_STATUS_MAC_ADDRESS_BIT);
    print_eeprom_mac();
    uint8_t connect_method = eeprom_read_byte((const uint8_t *) EEPROM_CONNECT_METHOD);
    Serial.print(F("    Method: "));    
    print_eeprom_connect_method();
    Serial.print(F("    SSID: "));
    print_eeprom_ssid();
    Serial.print(F("    Security Mode: "));
    print_eeprom_security_type();    
    Serial.print(F("    IP Mode: "));
    print_eeprom_ipmode();
    Serial.print(F("    Update Server: "));
    print_eeprom_update_server();    
    Serial.print(F("    Update Filename: "));
    print_eeprom_update_filename();        
    Serial.print(F("    NTP Server: "));
    if(eeprom_read_byte((uint8_t *) EEPROM_USE_NTP) == 1){
      print_eeprom_ntp_server();
    }
    else{
      Serial.println(F("Disabled"));
    }
    print_label_with_star_if_not_backed_up("NTP TZ Offset: ", BACKUP_STATUS_TIMEZONE_CALIBRATION_BIT);
    print_eeprom_float((const float *) EEPROM_NTP_TZ_OFFSET_HRS);
    
    Serial.println(F(" +-------------------------------------------------------------+"));
    Serial.println(F(" | MQTT Settings:                                              |"));
    Serial.println(F(" +-------------------------------------------------------------+"));    
    Serial.print(F("    MQTT Server: "));
    print_eeprom_mqtt_server();   
    Serial.print(F("    MQTT Port: "));
    Serial.println(eeprom_read_dword((const uint32_t *) EEPROM_MQTT_PORT)); 
    Serial.print(F("    MQTT Client ID: "));
    print_eeprom_mqtt_client_id();       
    print_eeprom_mqtt_authentication(); 
    Serial.print(F("    MQTT Topic Prefix: "));
    print_eeprom_mqtt_topic_prefix();
    Serial.print(F("    MQTT Topic Suffix: "));
    print_eeprom_mqtt_topic_suffix();      
    Serial.println(F(" +-------------------------------------------------------------+"));
    Serial.println(F(" | Credentials:                                                |"));
    Serial.println(F(" +-------------------------------------------------------------+"));
    print_label_with_star_if_not_backed_up("MQTT Password backed up? [* means no]", BACKUP_STATUS_MQTT_PASSSWORD_BIT);
    Serial.println();
    print_label_with_star_if_not_backed_up("Private key backed up? [* means no]", BACKUP_STATUS_PRIVATE_KEY_BIT);
    Serial.println();
    Serial.println(F(" +-------------------------------------------------------------+"));
    Serial.println(F(" | Sensor Calibrations:                                        |"));
    Serial.println(F(" +-------------------------------------------------------------+"));

    print_label_with_star_if_not_backed_up("NO2 Sensitivity [nA/ppm]: ", BACKUP_STATUS_NO2_CALIBRATION_BIT);
    print_eeprom_float((const float *) EEPROM_NO2_SENSITIVITY);
    print_label_with_star_if_not_backed_up("NO2 Slope [ppb/V]: ", BACKUP_STATUS_NO2_CALIBRATION_BIT);
    print_eeprom_float((const float *) EEPROM_NO2_CAL_SLOPE);
    print_label_with_star_if_not_backed_up("NO2 Offset [V]: ", BACKUP_STATUS_NO2_CALIBRATION_BIT);
    print_eeprom_float((const float *) EEPROM_NO2_CAL_OFFSET);
    Serial.print(F("    ")); Serial.println(F("NO2 Baseline Voltage Characterization:"));
    print_baseline_voltage_characterization(EEPROM_NO2_BASELINE_VOLTAGE_TABLE);
    
    print_label_with_star_if_not_backed_up("CO Sensitivity [nA/ppm]: ", BACKUP_STATUS_CO_CALIBRATION_BIT);
    print_eeprom_float((const float *) EEPROM_CO_SENSITIVITY);
    print_label_with_star_if_not_backed_up("CO Slope [ppm/V]: ", BACKUP_STATUS_CO_CALIBRATION_BIT);
    print_eeprom_float((const float *) EEPROM_CO_CAL_SLOPE);
    print_label_with_star_if_not_backed_up("CO Offset [V]: ", BACKUP_STATUS_CO_CALIBRATION_BIT);
    print_eeprom_float((const float *) EEPROM_CO_CAL_OFFSET);
    Serial.print(F("    ")); Serial.println(F("CO Baseline Voltage Characterization:"));
    print_baseline_voltage_characterization(EEPROM_CO_BASELINE_VOLTAGE_TABLE);
    
    char temp_reporting_offset_label[64] = {0};
    char temperature_units = (char) eeprom_read_byte((uint8_t *) EEPROM_TEMPERATURE_UNITS);
    snprintf(temp_reporting_offset_label, 63, "Temperature Reporting Offset [deg%c]: ", temperature_units); 
    float temp_reporting_offset_degc = eeprom_read_float((float *) EEPROM_TEMPERATURE_OFFSET);
    float temperature_offset_display = temp_reporting_offset_degc;
    if(temperature_units == 'F'){
      temperature_offset_display = toFahrenheit(temp_reporting_offset_degc) - 32.0f;
    }
    print_label_with_star_if_not_backed_up((char * )temp_reporting_offset_label, BACKUP_STATUS_TEMPERATURE_CALIBRATION_BIT);
    Serial.println(temperature_offset_display, 2);
    
    print_label_with_star_if_not_backed_up("Humidity Reporting Offset [%]: ", BACKUP_STATUS_HUMIDITY_CALIBRATION_BIT);
    Serial.println(eeprom_read_float((float *) EEPROM_HUMIDITY_OFFSET), 2);  
    
    
    Serial.println(F(" +-------------------------------------------------------------+"));
    Serial.println(F(" | note: '*' next to label means the setting is not backed up. |"));
    Serial.println(F(" |     run 'backup all' when you are satisfied                 |"));
    Serial.println(F(" +-------------------------------------------------------------+"));
  }
  else {
    Serial.print(F("Error: Unexpected Variable Name \""));
    Serial.print(arg);
    Serial.println(F("\""));
  }
}

void initialize_eeprom_value(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }

  if (strncmp(arg, "mac", 3) == 0) {
    uint8_t _mac_address[6];
    if (!esp.getMacAddress((uint8_t *) _mac_address)) {
      Serial.println(F("Error: Could not retrieve MAC address from ESP8266"));
    }
    else {
      eeprom_write_block(_mac_address, (void *) EEPROM_MAC_ADDRESS, 6);
      recomputeAndStoreConfigChecksum();
    }
  }
  else {
    Serial.print(F("Error: Unexpected Variable Name \""));
    Serial.print(arg);
    Serial.println(F("\""));
  }
}

void restore(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  char blank[32] = {0};
  uint8_t tmp[32] = {0};
  boolean valid = true;

  // things that must have been backed up before restoring.
  // 1. MAC address              0x80
  // 2. MQTT Password            0x40
  // 3. Private Key              0x20
  // 4. NO2 Calibration Values   0x10
  // 5. CO Calibratino Values    0x80

  uint16_t backup_check = eeprom_read_word((const uint16_t *) EEPROM_BACKUP_CHECK);

  if (strncmp(arg, "defaults", 8) == 0) {
    prompt();
    configInject("method direct\r");
    configInject("security auto\r");
    configInject("use dhcp\r");
    configInject("opmode normal\r");
    configInject("tempunit C\r");    
    configInject("altitude -1\r");    
    configInject("backlight 60\r");
    configInject("backlight initon\r");
    configInject("mqttsrv mqtt.opensensors.io\r");
    configInject("mqttport 1883\r");        
    configInject("mqttauth enable\r");    
    configInject("mqttuser wickeddevice\r");
    configInject("mqttprefix /orgs/wd/aqe/\r");
    configInject("mqttsuffix enable\r");
    configInject("sampling 5, 160, 5\r");   
    configInject("ntpsrv disable\r");
    configInject("ntpsrv pool.ntp.org\r");
    configInject("restore tz_off\r");
    configInject("restore temp_off\r");
    configInject("restore hum_off\r");       
    configInject("restore mqttpwd\r");
    configInject("restore mqttid\r");    
    configInject("restore updatesrv\r");
    configInject("restore updatefile\r");    
    configInject("restore key\r");
    configInject("restore no2\r");
    configInject("restore co\r");
    configInject("restore mac\r");    

    eeprom_write_block(blank, (void *) EEPROM_SSID, 32); // clear the SSID
    eeprom_write_block(blank, (void *) EEPROM_NETWORK_PWD, 32); // clear the Network Password
    mirrored_config_erase(); // erase the mirrored configuration, which will be restored next successful network connect                            
    
    Serial.println();
  }
  else if (strncmp(arg, "mac", 3) == 0) {
    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_MAC_ADDRESS_BIT)) {
      Serial.println(F("Error: MAC address must be backed up  "));
      Serial.println(F("       prior to executing a 'restore'."));
      return;
    }

    uint8_t _mac_address[6] = {0};
    char setmac_string[32] = {0};
    eeprom_read_block(_mac_address, (const void *) EEPROM_BACKUP_MAC_ADDRESS, 6);
    snprintf(setmac_string, 31,
             "mac %02x:%02x:%02x:%02x:%02x:%02x\r",
             _mac_address[0],
             _mac_address[1],
             _mac_address[2],
             _mac_address[3],
             _mac_address[4],
             _mac_address[5]);

    configInject(setmac_string);
    Serial.println();
  }
  else if (strncmp("mqttpwd", arg, 7) == 0) {
    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_MQTT_PASSSWORD_BIT)) {
      Serial.println(F("Error: MQTT Password must be backed up  "));
      Serial.println(F("       prior to executing a 'restore'."));
      return;
    }

    eeprom_read_block(tmp, (const void *) EEPROM_BACKUP_MQTT_PASSWORD, 32);
    eeprom_write_block(tmp, (void *) EEPROM_MQTT_PASSWORD, 32);
  }
  else if (strncmp("mqttid", arg, 6) == 0) {
    // get the 8-byte unique electronic ID from the SHT25 
    // convert it to a string, and store it to EEPROM
    uint8_t serial_number[8];
    sht25.getSerialNumber(serial_number);
    snprintf((char *) tmp, 31, "egg%02X%02X%02X%02X%02X%02X%02X%02X",
      serial_number[0],
      serial_number[1],
      serial_number[2],
      serial_number[3],
      serial_number[4],
      serial_number[5],
      serial_number[6],
      serial_number[7]);

    lowercase((char *) tmp); // for consistency with aqe v1 and airqualityegg.com assumptions
    
    eeprom_write_block(tmp, (void *) EEPROM_MQTT_CLIENT_ID, 32);
  }  
  else if (strncmp("updatesrv", arg, 9) == 0) {
    eeprom_write_block("update.wickeddevice.com", (void *) EEPROM_UPDATE_SERVER_NAME, 32);
  }  
  else if (strncmp("updatefile", arg, 10) == 0) {
    eeprom_write_block("aqev2_no2_co_esp", (void *) EEPROM_UPDATE_FILENAME, 32);
  }  
  else if (strncmp("key", arg, 3) == 0) {
    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_PRIVATE_KEY_BIT)) {
      Serial.println(F("Error: Private key must be backed up  "));
      Serial.println(F("       prior to executing a 'restore'."));
      return;
    }

    eeprom_read_block(tmp, (const void *) EEPROM_BACKUP_PRIVATE_KEY, 32);
    eeprom_write_block(tmp, (void *) EEPROM_PRIVATE_KEY, 32);
  }
  else if (strncmp("no2", arg, 3) == 0) {
    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_NO2_CALIBRATION_BIT)) {
      Serial.println(F("Error: NO2 calibration must be backed up  "));
      Serial.println(F("       prior to executing a 'restore'."));
      return;
    }

    eeprom_read_block(tmp, (const void *) EEPROM_BACKUP_NO2_SENSITIVITY, 4);
    eeprom_write_block(tmp, (void *) EEPROM_NO2_SENSITIVITY, 4);
    eeprom_read_block(tmp, (const void *) EEPROM_BACKUP_NO2_CAL_SLOPE, 4);
    eeprom_write_block(tmp, (void *) EEPROM_NO2_CAL_SLOPE, 4);
    eeprom_read_block(tmp, (const void *) EEPROM_BACKUP_NO2_CAL_OFFSET, 4);
    eeprom_write_block(tmp, (void *) EEPROM_NO2_CAL_OFFSET, 4);
  }
  else if (strncmp("co", arg, 5) == 0) {
    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_CO_CALIBRATION_BIT)) {
      Serial.println(F("Error: CO calibration must be backed up  "));
      Serial.println(F("       prior to executing a 'restore'."));
      return;
    }

    eeprom_read_block(tmp, (const void *) EEPROM_BACKUP_CO_SENSITIVITY, 4);
    eeprom_write_block(tmp, (void *) EEPROM_CO_SENSITIVITY, 4);
    eeprom_read_block(tmp, (const void *) EEPROM_BACKUP_CO_CAL_SLOPE, 4);
    eeprom_write_block(tmp, (void *) EEPROM_CO_CAL_SLOPE, 4);
    eeprom_read_block(tmp, (const void *) EEPROM_BACKUP_CO_CAL_OFFSET, 4);
    eeprom_write_block(tmp, (void *) EEPROM_CO_CAL_OFFSET, 4);
  }
  else if (strncmp("temp_off", arg, 8) == 0) {
    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_TEMPERATURE_CALIBRATION_BIT)) {
      Serial.println(F("Error: Temperature reporting offset should be backed up  "));
      Serial.println(F("       prior to executing a 'restore'. Setting to 0.0"));
      eeprom_write_float((float *) EEPROM_TEMPERATURE_OFFSET, 0.0f);  
    }
    else{
      eeprom_read_block(tmp, (const void *) EEPROM_BACKUP_TEMPERATURE_OFFSET, 4);
      eeprom_write_block(tmp, (void *) EEPROM_TEMPERATURE_OFFSET, 4);
    }
  }
  else if (strncmp("hum_off", arg, 7) == 0) {   
    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_HUMIDITY_CALIBRATION_BIT)) {
      Serial.println(F("Warning: Humidity reporting offset should be backed up  "));
      Serial.println(F("         prior to executing a 'restore'. Setting to 0.0."));   
      eeprom_write_float((float *) EEPROM_HUMIDITY_OFFSET, 0.0f);
    }
    else{
      eeprom_read_block(tmp, (const void *) EEPROM_BACKUP_HUMIDITY_OFFSET, 4);
      eeprom_write_block(tmp, (void *) EEPROM_HUMIDITY_OFFSET, 4);
    }
  }
  else if(strncmp("tz_off", arg, 6) == 0) {   
    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_TIMEZONE_CALIBRATION_BIT)) {
      Serial.println(F("Warning: Timezone offset should be backed up  "));
      Serial.println(F("         prior to executing a 'restore'. Setting to 0.0."));   
      eeprom_write_float((float *) EEPROM_NTP_TZ_OFFSET_HRS, 0.0f);
    }
    else{
      eeprom_read_block(tmp, (const void *) EEPROM_BACKUP_NTP_TZ_OFFSET_HRS, 4);
      eeprom_write_block(tmp, (void *) EEPROM_NTP_TZ_OFFSET_HRS, 4);
    }
  }
  else {
    valid = false;
    Serial.print(F("Error: Unexpected paramater name \""));
    Serial.print(arg);
    Serial.println(F("\""));
  }

  if (valid) {
    recomputeAndStoreConfigChecksum();
  }

}

void set_backlight_behavior(char * arg){
  boolean valid = true;  
  
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  lowercase(arg);
  
  if(strncmp(arg, "initon", 6) == 0){
    eeprom_write_byte((uint8_t *) EEPROM_BACKLIGHT_STARTUP, BACKLIGHT_ON_AT_STARTUP);
  }
  else if(strncmp(arg, "initoff", 7) == 0){
    eeprom_write_byte((uint8_t *) EEPROM_BACKLIGHT_STARTUP, BACKLIGHT_OFF_AT_STARTUP);
  }
  else if(strncmp(arg, "alwayson", 8) == 0){
    eeprom_write_byte((uint8_t *) EEPROM_BACKLIGHT_STARTUP, BACKLIGHT_ALWAYS_ON);
  }
  else if(strncmp(arg, "alwaysoff", 9) == 0){
    eeprom_write_byte((uint8_t *) EEPROM_BACKLIGHT_STARTUP, BACKLIGHT_ALWAYS_OFF);
  }  
  else{ 
    boolean arg_contains_only_digits = true;
    char * ptr = arg;
    uint16_t arglen = strlen(arg);    
    for(uint16_t ii = 0; ii < arglen; ii++){
      if(!isdigit(arg[ii])){
        arg_contains_only_digits = true;
        break; 
      }
    }
    
    if(arg_contains_only_digits){
      uint32_t duration = (uint32_t) strtoul(arg, NULL, 10);
      if(duration < 0xFFFF){
        eeprom_write_word((uint16_t *) EEPROM_BACKLIGHT_DURATION, (uint16_t) duration);
      }
    }
    else{
      valid = false;
      Serial.print(F("Error: Unexpected paramater name \""));
      Serial.print(arg);
      Serial.println(F("\""));      
    }
  }
  
  if (valid) {
    recomputeAndStoreConfigChecksum();
  }  
}

void altitude_command(char * arg){
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }

  char * endptr = NULL;
  trim_string(arg);
  int16_t l_altitude = (int16_t) strtol(arg, &endptr, 10);
  if(*endptr == NULL){
    eeprom_write_word((uint16_t *) EEPROM_ALTITUDE_METERS, l_altitude);
    recomputeAndStoreConfigChecksum();
  }
  else{
    Serial.println(F("Error: altitude must be a numeric"));
  }
}

void sampling_command(char * arg){
  if(!configMemoryUnlocked(__LINE__)){
    return;
  } 

  uint16_t len = strlen(arg);
  uint8_t num_commas = 0;  
  
  for(uint16_t ii = 0; ii < len; ii++){
    // if any character is not a space, comma, or digit it's unparseable
    if((!isdigit(arg[ii])) && (arg[ii] != ' ') && (arg[ii] != ',')){
      Serial.print(F("Error: Found invalid character '"));
      Serial.print((char) arg[ii]);
      Serial.print(F("'"));
      Serial.println();
      return;
    }

    if(arg[ii] == ','){
      num_commas++;
    }
  }    
    
  if(num_commas != 2){
    Serial.print(F("Error: sampling expects exactly 3 values separated by commas, but received "));
    Serial.print(num_commas - 1);
    Serial.println();
    return; 
  }  
  
  // ok we have 3 numeric arguments separated by commas, parse them
  // ok we have six numeric arguments separated by commas, parse them
  char tmp[32] = {0};  
  strncpy(tmp, arg, 31); // copy the string so you don't mutilate the argument
  char * token = strtok(tmp, ",");
  uint8_t token_number = 0;  
  uint16_t l_sample_interval = 0;
  uint16_t l_averaging_interval = 0;
  uint16_t l_reporting_interval = 0;
  
  while (token != NULL) {
    switch(token_number++){
      case 0:
        l_sample_interval = (uint16_t) strtoul(token, NULL, 10);
        if(l_sample_interval < 3){
          Serial.print(F("Error: Sampling interval must be greater than 2 [was ")); 
          Serial.print(l_sample_interval);
          Serial.print(F("]"));
          Serial.println();
          return;
        }
        break;      
      case 1:
        l_averaging_interval = (uint16_t) strtoul(token, NULL, 10);
        if(l_averaging_interval < 1){
          Serial.print(F("Error: Averaging interval must be greater than 0 [was ")); 
          Serial.print(l_averaging_interval);
          Serial.print(F("]"));
          Serial.println();
          return;
        }
        else if((l_averaging_interval % l_sample_interval) != 0){
          Serial.print(F("Error: Averaging interval must be an integer multiple of the Sampling interval"));
          Serial.println();
          return;          
        }
        else if((l_averaging_interval / l_sample_interval) > MAX_SAMPLE_BUFFER_DEPTH){
          Serial.print(F("Error: Insufficient memory available for averaging interval @ sampling interval."));
          Serial.print(F("       Must require no more than "));          
          Serial.print(MAX_SAMPLE_BUFFER_DEPTH);
          Serial.print(F(" samples, but requires"));
          Serial.print(l_averaging_interval / l_sample_interval);
          Serial.println(F(" samples"));
          Serial.println();
          return;                    
        }
        break;
      case 2:
        l_reporting_interval = (uint16_t) strtoul(token, NULL, 10);
        if(l_reporting_interval < 1){
          Serial.print(F("Error: Reporting interval must be greater than 0 [was ")); 
          Serial.print(l_reporting_interval);
          Serial.print(F("]"));
          Serial.println();
          return;
        }
        else if((l_reporting_interval % l_sample_interval) != 0){
          Serial.print(F("Error: Reporting interval must be an integer multiple of the Sampling interval"));
          Serial.println();
          return;          
        }        
        break;
    }    
    
    token = strtok(NULL, ",");
  } 
  
  // we got through all the checks! save these parameters to config memory
  eeprom_write_word((uint16_t *) EEPROM_SAMPLING_INTERVAL, l_sample_interval);
  eeprom_write_word((uint16_t *) EEPROM_REPORTING_INTERVAL, l_reporting_interval);
  eeprom_write_word((uint16_t *) EEPROM_AVERAGING_INTERVAL, l_averaging_interval);
  recomputeAndStoreConfigChecksum();  
}

void AQE_set_datetime(char * arg){
  if(!configMemoryUnlocked(__LINE__)){
    return;
  } 
  
  uint16_t len = strlen(arg);
  uint8_t num_commas = 0;  
  
  for(uint16_t ii = 0; ii < len; ii++){
    // if any character is not a space, comma, or digit it's unparseable
    if((!isdigit(arg[ii])) && (arg[ii] != ' ') && (arg[ii] != ',')){
      Serial.print(F("Error: Found invalid character '"));
      Serial.print((char) arg[ii]);
      Serial.print(F("'"));
      Serial.println();
      return;
    }
    
    if(arg[ii] == ','){
      num_commas++;
    }
  }
 
  if(num_commas != 5){
    Serial.print(F("Error: datetime expects exactly 6 values separated by commas, but received "));
    Serial.print(num_commas - 1);
    Serial.println();
    return; 
  }
  
  // ok we have six numeric arguments separated by commas, parse them
  char tmp[32] = {0};  
  strncpy(tmp, arg, 31); // copy the string so you don't mutilate the argument
  char * token = strtok(tmp, ",");
  uint8_t token_number = 0;  
  uint8_t mo = 0, dy = 0, hr = 0, mn = 0, sc = 0;
  uint16_t yr = 0;
  
  while (token != NULL) {
    switch(token_number++){
      case 0:
        yr = (uint16_t) strtoul(token, NULL, 10);
        if(yr < 2015){
          Serial.print(F("Error: Year must be no earlier than 2015 [was ")); 
          Serial.print(yr);
          Serial.print(F("]"));
          Serial.println();
          return;
        }
        break;      
      case 1:
        mo = (uint8_t) strtoul(token, NULL, 10);
        if(mo < 1 || mo > 12){
          Serial.print(F("Error: Month must be between 1 and 12 [was ")); 
          Serial.print(mo);
          Serial.print(F("]"));
          Serial.println();
          return;
        }
        break;
      case 2:
        dy = (uint8_t) strtoul(token, NULL, 10);
        if(dy < 1 || dy > 31){
          Serial.print(F("Error: Day must be between 1 and 31 [was ")); 
          Serial.print(dy);
          Serial.print(F("]"));
          Serial.println();
          return;
        }        
        break;
      case 3:
        hr = (uint8_t) strtoul(token, NULL, 10);    
        if(hr > 23){
          Serial.print(F("Error: Hour must be between 0 and 23 [was ")); 
          Serial.print(hr);
          Serial.print(F("]"));
          Serial.println();
          return;
        }        
        break;
      case 4:
        mn = (uint8_t) strtoul(token, NULL, 10);  
        if(mn > 59){
          Serial.print(F("Error: Minute must be between 0 and 59 [was ")); 
          Serial.print(mn);
          Serial.print(F("]"));
          Serial.println();
          return;
        }        
        break;
      case 5:
        sc = (uint8_t) strtoul(token, NULL, 10);      
        if(mn > 59){
          Serial.print(F("Error: Second must be between 0 and 59 [was ")); 
          Serial.print(sc);
          Serial.print(F("]"));
          Serial.println();
          return;
        }          
        break;        
    }
    token = strtok(NULL, ",");
  }
  
  // if we have an RTC set the time in the RTC
  DateTime datetime(yr,mo,dy,hr,mn,sc);
  
  // it's not harmful to do this
  // even if the RTC is not present
  selectSlot3();
  rtc.adjust(datetime);
  
  // also clear the Oscillator Stop Flag
  // this should really be folded into the RTCLib code
  rtcClearOscillatorStopFlag();
  
      
  // at any rate sync the time to this
  setTime(datetime.unixtime());
  
}

void set_mac_address(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }  
  
  uint8_t _mac_address[6] = {0};
  char tmp[32] = {0};
  
  strncpy(tmp, arg, 31); // copy the string so you don't mutilate the argument
  char * token = strtok(tmp, ":");
  uint8_t num_tokens = 0;

  // parse the argument string, expected to be of the form ab:01:33:51:c8:77
  while (token != NULL) {
    if(num_tokens > 5){
      Serial.println(F("Error: Too many octets passed to setmac: "));
      Serial.print(F("       "));
      Serial.println(arg);
      Serial.println();
      return;
    }    
    
    if ((strlen(token) == 2) && isxdigit(token[0]) && isxdigit(token[1]) && (num_tokens < 6)) {
      _mac_address[num_tokens++] = (uint8_t) strtoul(token, NULL, 16);
    }
    else {
      Serial.print(F("Error: MAC address parse error on input \""));
      Serial.print(arg);
      Serial.println(F("\""));
      return; // return early
    }

    
    token = strtok(NULL, ":");
  }

  if (num_tokens == 6) {
    eeprom_write_block(_mac_address, (void *) EEPROM_MAC_ADDRESS, 6);
    recomputeAndStoreConfigChecksum();    
  }
  else {
    Serial.println(F("Error: MAC address must contain 6 bytes, with each separated by ':'"));
  }
}

void set_connection_method(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }  
  
  lowercase(arg);
  boolean valid = true;
  if (strncmp(arg, "direct", 6) == 0) {
    eeprom_write_byte((uint8_t *) EEPROM_CONNECT_METHOD, CONNECT_METHOD_DIRECT);
  }
  else {
    Serial.print(F("Error: Invalid connection method entered - \""));
    Serial.print(arg);
    Serial.println(F("\""));
    Serial.println(F("       valid options are: 'direct'"));
    valid = false;
  }
  
  if (valid) {
    recomputeAndStoreConfigChecksum();
  }
}

void set_ssid(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  // we've reserved 32-bytes of EEPROM for an SSID
  // so the argument's length must be <= 31
  char ssid[33] = {0};
  uint16_t len = strlen(arg);
  if (len <= 32) {
    strncpy(ssid, arg, len);
    eeprom_write_block(ssid, (void *) EEPROM_SSID, 32);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.println(F("Error: SSID must be less than 33 characters in length"));
  }
}

void set_network_password(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  // we've reserved 32-bytes of EEPROM for a network password
  // so the argument's length must be <= 31
  char password[32] = {0};
  uint16_t len = arg ? strlen(arg) : 0;
  if (len < 32) {
    if(arg){
      strncpy(password, arg, len);
    }
    eeprom_write_block(password, (void *) EEPROM_NETWORK_PWD, 32);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.println(F("Error: Network password must be less than 32 characters in length"));
  }
}

void set_network_security_mode(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  boolean valid = true;
  if (strncmp("open", arg, 4) == 0) {
    eeprom_write_byte((uint8_t *) EEPROM_SECURITY_MODE, 0);
    set_network_password(NULL);
  }
  else if (strncmp("wep", arg, 3) == 0) {
    eeprom_write_byte((uint8_t *) EEPROM_SECURITY_MODE, 1);
  }
  else if (strncmp("wpa2", arg, 4) == 0) {
    eeprom_write_byte((uint8_t *) EEPROM_SECURITY_MODE, 2);
  }
  else if (strncmp("wpa", arg, 3) == 0) {
    eeprom_write_byte((uint8_t *) EEPROM_SECURITY_MODE, 3);
  }
  else if(strncmp("auto", arg, 4) == 0) {
    eeprom_write_byte((uint8_t *) EEPROM_SECURITY_MODE, WLAN_SEC_AUTO);
  }
  else {
    Serial.print(F("Error: Invalid security mode entered - \""));
    Serial.print(arg);
    Serial.println(F("\""));
    Serial.println(F("       valid options are: 'open', 'wep', 'wpa', and 'wpa2'"));
    valid = false;
  }

  if (valid) {
    recomputeAndStoreConfigChecksum();
  }
}

void set_operational_mode(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  boolean valid = true;
  if (strncmp("normal", arg, 6) == 0) {
    eeprom_write_byte((uint8_t *) EEPROM_OPERATIONAL_MODE, SUBMODE_NORMAL);
  }
  else if (strncmp("offline", arg, 7) == 0) {
    eeprom_write_byte((uint8_t *) EEPROM_OPERATIONAL_MODE, SUBMODE_OFFLINE);
  }
  else {
    Serial.print(F("Error: Invalid operational mode entered - \""));
    Serial.print(arg);
    Serial.println(F("\""));
    Serial.println(F("       valid options are: 'normal', 'offline'"));
    valid = false;
  }

  if(valid) {
    recomputeAndStoreConfigChecksum();
  }
}

void set_temperature_units(char * arg){
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }  
  
  if((strlen(arg) != 1) || ((arg[0] != 'C') && (arg[0] != 'F'))){
    Serial.print(F("Error: temperature unit must be 'C' or 'F', but received '"));
    Serial.print(arg);
    Serial.println(F("'"));
    return;
  }  
  
  eeprom_write_byte((uint8_t *) EEPROM_TEMPERATURE_UNITS, arg[0]);
  recomputeAndStoreConfigChecksum();
}

void set_static_ip_address(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }  
  
  uint8_t _ip_address[4] = {0};
  uint8_t _gateway_ip[4] = {0};
  uint8_t _dns_ip[4] = {0}; 
  uint8_t _netmask[4] = {0};
  
  char tmp[128] = {0};
  strncpy(tmp, arg, 127); // copy the string so you don't mutilate the argument
  char * params[4] = {0};
  uint8_t param_idx = 0;
  
  // first tokenize on spaces, you should end up with four strings     
  char * token = strtok(tmp, " ");
  uint8_t num_tokens = 0;

  while (token != NULL) {
    if(param_idx > 3){
      Serial.println(F("Error: Too many parameters passed to staticip"));
      Serial.print(F("       "));
      Serial.println(arg);
      configInject("help staticip\r");
      Serial.println();
      return;
    }
    params[param_idx++] = token;   
    token = strtok(NULL, " "); 
  }
  
  if(param_idx != 4){
     Serial.println(F("Error: Too few parameters passed to staticip"));
     Serial.print(F("       "));
     Serial.println(arg);
     configInject("help staticip\r");   
     Serial.println();     
     return;
  }

  for(param_idx = 0; param_idx < 4; param_idx++){
    token = strtok(params[param_idx], ".");
    num_tokens = 0;    
    
    // parse the parameter string, expected to be of the form 192.168.1.52
    while (token != NULL) {
      uint8_t tokenlen = strlen(token);
      if ((tokenlen < 4) && (num_tokens < 4)) {
        for (uint8_t ii = 0; ii < tokenlen; ii++) {
          if (!isdigit(token[ii])) {
            Serial.print(F("Error: IP address octets must be integer values [@param "));
            Serial.print(param_idx + 1);
            Serial.println(F("]"));
            return;
          }
        }
        uint32_t octet = (uint8_t) strtoul(token, NULL, 10);
        if (octet < 256) {
          switch(param_idx){
            case 0:
              _ip_address[num_tokens++] = octet;
              break;
            case 1:
              _netmask[num_tokens++] = octet;
              break;
            case 2:
              _gateway_ip[num_tokens++] = octet;
              break;
            case 3:
              _dns_ip[num_tokens++] = octet;
              break; 
            default:
              break;            
          }
        }
        else {
          Serial.print(F("Error: IP address octets must be between 0 and 255 inclusive [@param "));
          Serial.print(param_idx + 1);
          Serial.println(F("]"));
          return;
        }
      }
      else {
        Serial.print(F("Error: IP address parse error on input \""));
        Serial.print(token);
        Serial.println(F("\""));
        return; 
      }
      
      token = strtok(NULL, ".");
    }    
      
    if (num_tokens != 4){
      Serial.print(F("Error: IP Address must contain 4 valid octets separated by '.' [@param "));
      Serial.print(param_idx + 1);
      Serial.println(F("]"));
      return;
    }

  }

  // if we got this far, it means we got 4 valid IP addresses, and they
  // are stored in their respective local variables    
  uint32_t ipAddress = esp.IpArrayToIpUint32((uint8_t *) _ip_address);
  uint32_t netMask = esp.IpArrayToIpUint32((uint8_t *) _netmask);
  uint32_t defaultGateway = esp.IpArrayToIpUint32((uint8_t *) _gateway_ip);  
  
  eeprom_write_block(_ip_address, (void *) EEPROM_STATIC_IP_ADDRESS, 4);  
  eeprom_write_block(_netmask, (void *) EEPROM_STATIC_NETMASK, 4);
  eeprom_write_block(_gateway_ip, (void *) EEPROM_STATIC_GATEWAY, 4);
  //eeprom_write_block(_dns_ip, (void *) EEPROM_STATIC_DNS, 4);
  recomputeAndStoreConfigChecksum();  
}

void use_command(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  const uint8_t noip[4] = {0};
  if (strncmp("dhcp", arg, 4) == 0) {    
    eeprom_write_block(noip, (void *) EEPROM_STATIC_IP_ADDRESS, 4);
    recomputeAndStoreConfigChecksum();  
  }
  else if (strncmp("ntp", arg, 3) == 0) {
    eeprom_write_byte((uint8_t *) EEPROM_USE_NTP, 1);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.print(F("Error: Invalid parameter provided to 'use' command - \""));
    Serial.print(arg);
    Serial.println("\"");
    return;
  }
}

void force_command(char * arg){
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  if (strncmp("update", arg, 6) == 0) {    
    Serial.println(F("Info: Erasing last flash page"));
    SUCCESS_MESSAGE_DELAY();                      
    invalidateSignature();    
    configInject("opmode normal\r");
    mode = SUBMODE_NORMAL;
    configInject("exit\r");
  }
  else {
    Serial.print(F("Error: Invalid parameter provided to 'force' command - \""));
    Serial.print(arg);
    Serial.println(F("\""));
    return;
  }  
}

void printDirectory(File dir, int numTabs) {
   for(;;){     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print(F("\t"));
     }
     char tmp[16] = {0};
     entry.getName(tmp, 16);
     Serial.print(tmp);
     if (entry.isDirectory()) {
       Serial.println(F("/"));
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print(F("\t"));
       Serial.print(F("\t"));       
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}

void list_command(char * arg){
  if (strncmp("files", arg, 5) == 0){
    if(init_sdcard_ok){
      File root = SD.open("/", FILE_READ);
      printDirectory(root, 0);   
      root.close();       
    }
    else{
      Serial.println(F("Error: SD Card is not initialized, can't list files."));
    }    
  }
  else{
    Serial.print(F("Error: Invalid parameter provided to 'list' command - \""));
    Serial.print(arg);
    Serial.println(F("\""));
  }  
}

void download_one_file(char * filename){
  if(filename != NULL && init_sdcard_ok){    
    File dataFile = SD.open(filename, FILE_READ);
    char last_char_read = NULL;
    if (dataFile) {
      while (dataFile.available()) {
        last_char_read = dataFile.read();
        Serial.write(last_char_read);
      }
      dataFile.close();      
    }
    //else {
    //  Serial.print("Error: Failed to open file named \"");
    //  Serial.print(filename);
    //  Serial.print(F("\""));
    //}    
    if(last_char_read != '\n'){
      Serial.println();        
    }
  }  
}

void crack_datetime_filename(char * filename, uint8_t target_array[4]){
  char temp_str[3] = {0, 0, 0};   
  for(uint8_t ii = 0; ii < 4; ii++){
    strncpy(temp_str, &(filename[ii * 2]), 2);  
    target_array[ii] = atoi(temp_str);
  }

  target_array[0] += 30; // YY is offset from 2000, but epoch time is offset from 1970
}

void make_datetime_filename(uint8_t src_array[4], char * target_filename, uint8_t max_len){
  snprintf(target_filename, max_len, "%02d%02d%02d%02d.csv", 
    src_array[0] - 30, // YY is offset from 2000, but epoch time is offset from 1970
    src_array[1],
    src_array[2],
    src_array[3]);  
}

void advanceByOneHour(uint8_t src_array[4]){

  tmElements_t tm;
  tm.Year   = src_array[0];
  tm.Month  = src_array[1];
  tm.Day    = src_array[2];
  tm.Wday   = 0;
  tm.Hour   = src_array[3];
  tm.Minute = 0;
  tm.Second = 0;
  
  time_t seconds_since_epoch = makeTime(tm);
  seconds_since_epoch += SECS_PER_HOUR; 
  breakTime(seconds_since_epoch, tm);

  src_array[0] = tm.Year;
  src_array[1] = tm.Month;
  src_array[2] = tm.Day;
  src_array[3] = tm.Hour;
}

// does the behavior of executing the one_file_function on a single file
// or on each file in a range of files 
void fileop_command_delegate(char * arg, void (*one_file_function)(char *)){
  char * first_arg = NULL;
  char * second_arg = NULL;
  
  trim_string(arg);
  
  first_arg = strtok(arg, " ");
  second_arg = strtok(NULL, " ");

  if(second_arg == NULL){   
    one_file_function(first_arg);
  }
  else{    
    uint8_t cur_date[4] = {0,0,0,0};
    uint8_t end_date[4] = {0,0,0,0};
    crack_datetime_filename(first_arg, cur_date);
    crack_datetime_filename(second_arg, end_date);

    // starting from cur_date, download the file with that name
    char cur_date_filename[16] = {0};
    boolean finished_last_file = false;
    unsigned long previousMillis = millis();
    const long interval = 1000;
    while(!finished_last_file){
      unsigned long currentMillis = millis();
      if(currentMillis - previousMillis >= interval) {        
        previousMillis = currentMillis;   
        petWatchdog();
      }
      memset(cur_date_filename, 0, 16);
      make_datetime_filename(cur_date, cur_date_filename, 15);
      one_file_function(cur_date_filename);
      if(memcmp(cur_date, end_date, 4) == 0){      
        finished_last_file = true;
      }
      else{
        advanceByOneHour(cur_date);      
      }
    } 
    delayForWatchdog();    
  }  
}

void download_command(char * arg){
  Serial.println(header_row);
  fileop_command_delegate(arg, download_one_file);
  Serial.println("Info: Done downloading.");
}

void delete_one_file(char * filename){
  if(filename != NULL && init_sdcard_ok){   
    if (SD.remove(filename)) {
      Serial.print("Info: Removed file named \"");
      Serial.print(filename);
      Serial.println(F("\""));
    }
//    else {
//      Serial.print("Error: Failed to delete file named \"");
//      Serial.print(filename);
//      Serial.println(F("\""));
//    }    
  }  
}

void delete_command(char * arg){
  fileop_command_delegate(arg, delete_one_file);
  Serial.println("Info: Done deleting.");
}

void set_mqtt_password(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  // we've reserved 32-bytes of EEPROM for a MQTT password
  // so the argument's length must be <= 31
  char password[32] = {0};
  uint16_t len = strlen(arg);
  if (len < 32) {
    strncpy(password, arg, len);
    eeprom_write_block(password, (void *) EEPROM_MQTT_PASSWORD, 32);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.println(F("Error: MQTT password must be less than 32 characters in length"));
  }
}

void set_mqtt_topic_prefix(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  // we've reserved 64-bytes of EEPROM for a MQTT prefix
  // so the argument's length must be <= 63
  char prefix[64] = {0};
  uint16_t len = strlen(arg);
  if (len < 64) {
    strncpy(prefix, arg, len);
    eeprom_write_block(prefix, (void *) EEPROM_MQTT_TOPIC_PREFIX, 64);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.println(F("Error: MQTT prefix must be less than 64 characters in length"));
  }
}

void set_user_latitude(char * arg){
  set_float_param(arg, (float *) EEPROM_USER_LATITUDE_DEG, NULL);
}

void set_user_longitude(char * arg){
  set_float_param(arg, (float *) EEPROM_USER_LONGITUDE_DEG, NULL);  
}

void set_user_location_enable(char * arg){
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }

  lowercase(arg);
  
  if (strcmp(arg, "enable") == 0){
    eeprom_write_byte((uint8_t *) EEPROM_USER_LOCATION_EN, 1);    
    recomputeAndStoreConfigChecksum();
  }
  else if (strcmp(arg, "disable") == 0){
    eeprom_write_byte((uint8_t *) EEPROM_USER_LOCATION_EN, 0);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.print(F("Error: expected 'enable' or 'disable' but got '"));
    Serial.print(arg);
    Serial.println("'");
  }  

  user_location_override = eeprom_read_byte((uint8_t *) EEPROM_USER_LOCATION_EN) == 1 ? true : false;
}

void topic_suffix_config(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }

  lowercase(arg);
  
  if (strcmp(arg, "enable") == 0){
    eeprom_write_byte((uint8_t *) EEPROM_MQTT_TOPIC_SUFFIX_ENABLED, 1);
    recomputeAndStoreConfigChecksum();
  }
  else if (strcmp(arg, "disable") == 0){
    eeprom_write_byte((uint8_t *) EEPROM_MQTT_TOPIC_SUFFIX_ENABLED, 0);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.print(F("Error: expected 'enable' or 'disable' but got '"));
    Serial.print(arg);
    Serial.println("'");
  }
}


void set_mqtt_server(char * arg){
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  // we've reserved 32-bytes of EEPROM for an MQTT server name
  // so the argument's length must be <= 31
  char server[32] = {0};
  uint16_t len = strlen(arg);
  if (len < 32) {
    strncpy(server, arg, len);
    eeprom_write_block(server, (void *) EEPROM_MQTT_SERVER_NAME, 32);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.println(F("Error: MQTT server name must be less than 32 characters in length"));
  }  
}

void set_mqtt_username(char * arg){
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  // we've reserved 32-bytes of EEPROM for an MQTT username
  // so the argument's length must be <= 31
  char username[32] = {0};
  uint16_t len = strlen(arg);
  if (len < 32) {
    strncpy(username, arg, len);
    eeprom_write_block(username, (void *) EEPROM_MQTT_USERNAME, 32);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.println(F("Error: MQTT username must be less than 32 characters in length"));
  }    
}
void set_mqtt_client_id(char * arg){
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  // we've reserved 32-bytes of EEPROM for an MQTT client ID
  // but in fact an MQTT client ID must be between 1 and 23 characters
  // and must start with an letter
  char client_id[32] = {0};  
  
  uint16_t len = strlen(arg);
  if ((len >= 1) && (len <= 23)) {
    strncpy(client_id, arg, len);
    eeprom_write_block(client_id, (void *) EEPROM_MQTT_CLIENT_ID, 32);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.println(F("Error: MQTT client ID must be less between 1 and 23 characters in length"));
  }
}

void set_mqtt_authentication(char * arg) {   
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  if (strncmp("enable", arg, 6) == 0) { 
    eeprom_write_byte((uint8_t *) EEPROM_MQTT_AUTH, 1);
    recomputeAndStoreConfigChecksum();
  }
  else if(strncmp("disable", arg, 7) == 0) { 
    eeprom_write_byte((uint8_t *) EEPROM_MQTT_AUTH, 0);
    recomputeAndStoreConfigChecksum();    
  }
  else {
    Serial.print(F("Error: Invalid parameter provided to 'mqttauth' command - \""));
    Serial.print(arg);
    Serial.println("\", must be either \"enable\" or \"disable\"");
    return;
  }
}

void set_mqtt_port(char * arg) {  
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  uint16_t len = strlen(arg);
  boolean valid = true;
  
  for(uint16_t ii = 0; ii < len; ii++){
    if(!isdigit(arg[ii])){
      valid = false;
    } 
  }
  
  uint32_t port = 0xFFFFFFFF;
  if(valid){
    port = (uint32_t) strtoul(arg, NULL, 10);
  }
  
  if(valid && (port < 0x10000) && (port > 0)){
    eeprom_write_dword((uint32_t *) EEPROM_MQTT_PORT, port);
    recomputeAndStoreConfigChecksum();        
  }
  else {
    Serial.print(F("Error: Invalid parameter provided to 'mqttport' command - \""));
    Serial.print(arg);
    Serial.println("\", must a number between 1 and 65535 inclusive");
    return;
  }
}

void set_update_server_name(char * arg){

  static char server[32] = {0};
  memset(server, 0, 32);
  
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  trim_string(arg); // leading and trailing spaces are not relevant 
  uint16_t len = strlen(arg);   
  
  // we've reserved 32-bytes of EEPROM for an update server name
  // so the argument's length must be <= 31      
  if (len < 32) {        
    strncpy(server, arg, 31); // copy the argument as a case-sensitive server name
    lowercase(arg);           // in case it's the "disable" special case, make arg case insensitive
    
    if(strncmp(arg, "disable", 7) == 0){      
      memset(server, 0, 32); // wipe out the update server name 
    }
    
    eeprom_write_block(server, (void *) EEPROM_UPDATE_SERVER_NAME, 32);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.println(F("Error: Update server name must be less than 32 characters in length"));
  }
}

void set_update_filename(char * arg){
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  // we've reserved 32-bytes of EEPROM for an update server name
  // so the argument's length must be <= 31
  char filename[32] = {0};
  uint16_t len = strlen(arg);
  if (len < 32) {
    strncpy(filename, arg, len);
    eeprom_write_block(filename, (void *) EEPROM_UPDATE_FILENAME, 32);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.println(F("Error: Update filename must be less than 32 characters in length"));
  }  
}

void set_ntp_server(char * arg){

  static char server[32] = {0};
  memset(server, 0, 32);
  
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  trim_string(arg); // leading and trailing spaces are not relevant 
  uint16_t len = strlen(arg);   
  
  // we've reserved 32-bytes of EEPROM for an NTP server name
  // so the argument's length must be <= 31      
  if (len < 32) {        
    strncpy(server, arg, 31); // copy the argument as a case-sensitive server name
    lowercase(arg);           // in case it's the "disable" special case, make arg case insensitive
    
    if(strncmp(arg, "disable", 7) == 0){     
      eeprom_write_byte((uint8_t *) EEPROM_USE_NTP, 0); 
      memset(server, 0, 32); // wipe out the NTP server name 
    }
    
    eeprom_write_block(server, (void *) EEPROM_NTP_SERVER_NAME, 32);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.println(F("Error: NTP server name must be less than 32 characters in length"));
  }
}

void backup(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  boolean valid = true;
  char tmp[32] = {0};
  uint16_t backup_check = eeprom_read_word((const uint16_t *) EEPROM_BACKUP_CHECK);

  if (strncmp("mac", arg, 3) == 0) {
    configInject("init mac\r"); // make sure the ESP8266 mac address is in EEPROM
    Serial.println();
    eeprom_read_block(tmp, (const void *) EEPROM_MAC_ADDRESS, 6);
    eeprom_write_block(tmp, (void *) EEPROM_BACKUP_MAC_ADDRESS, 6);

    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_MAC_ADDRESS_BIT)) {
      CLEAR_BIT(backup_check, BACKUP_STATUS_MAC_ADDRESS_BIT);
      eeprom_write_word((uint16_t *) EEPROM_BACKUP_CHECK, backup_check);
    }
  }
  else if (strncmp("mqttpwd", arg, 7) == 0) {
    eeprom_read_block(tmp, (const void *) EEPROM_MQTT_PASSWORD, 32);
    eeprom_write_block(tmp, (void *) EEPROM_BACKUP_MQTT_PASSWORD, 32);

    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_MQTT_PASSSWORD_BIT)) {
      CLEAR_BIT(backup_check, BACKUP_STATUS_MQTT_PASSSWORD_BIT);
      eeprom_write_word((uint16_t *) EEPROM_BACKUP_CHECK, backup_check);
    }
  }
  else if (strncmp("key", arg, 3) == 0) {
    eeprom_read_block(tmp, (const void *) EEPROM_PRIVATE_KEY, 32);
    eeprom_write_block(tmp, (void *) EEPROM_BACKUP_PRIVATE_KEY, 32);

    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_PRIVATE_KEY_BIT)) {
      CLEAR_BIT(backup_check, BACKUP_STATUS_PRIVATE_KEY_BIT);
      eeprom_write_word((uint16_t *) EEPROM_BACKUP_CHECK, backup_check);
    }
  }
  else if (strncmp("no2", arg, 3) == 0) {
    eeprom_read_block(tmp, (const void *) EEPROM_NO2_SENSITIVITY, 4);
    eeprom_write_block(tmp, (void *) EEPROM_BACKUP_NO2_SENSITIVITY, 4);
    eeprom_read_block(tmp, (const void *) EEPROM_NO2_CAL_SLOPE, 4);
    eeprom_write_block(tmp, (void *) EEPROM_BACKUP_NO2_CAL_SLOPE, 4);
    eeprom_read_block(tmp, (const void *) EEPROM_NO2_CAL_OFFSET, 4);
    eeprom_write_block(tmp, (void *) EEPROM_BACKUP_NO2_CAL_OFFSET, 4);

    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_NO2_CALIBRATION_BIT)) {
      CLEAR_BIT(backup_check, BACKUP_STATUS_NO2_CALIBRATION_BIT);
      eeprom_write_word((uint16_t *) EEPROM_BACKUP_CHECK, backup_check);
    }
  }
  else if (strncmp("co", arg, 2) == 0) {
    eeprom_read_block(tmp, (const void *) EEPROM_CO_SENSITIVITY, 4);
    eeprom_write_block(tmp, (void *) EEPROM_BACKUP_CO_SENSITIVITY, 4);
    eeprom_read_block(tmp, (const void *) EEPROM_CO_CAL_SLOPE, 4);
    eeprom_write_block(tmp, (void *) EEPROM_BACKUP_CO_CAL_SLOPE, 4);
    eeprom_read_block(tmp, (const void *) EEPROM_CO_CAL_OFFSET, 4);
    eeprom_write_block(tmp, (void *) EEPROM_BACKUP_CO_CAL_OFFSET, 4);

    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_CO_CALIBRATION_BIT)) {
      CLEAR_BIT(backup_check, BACKUP_STATUS_CO_CALIBRATION_BIT);
      eeprom_write_word((uint16_t *) EEPROM_BACKUP_CHECK, backup_check);
    }
  }
  else if (strncmp("temp", arg, 4) == 0) {
    eeprom_read_block(tmp, (const void *) EEPROM_TEMPERATURE_OFFSET, 4);
    eeprom_write_block(tmp, (void *) EEPROM_BACKUP_TEMPERATURE_OFFSET, 4);

    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_TEMPERATURE_CALIBRATION_BIT)) {
      CLEAR_BIT(backup_check, BACKUP_STATUS_TEMPERATURE_CALIBRATION_BIT);
      eeprom_write_word((uint16_t *) EEPROM_BACKUP_CHECK, backup_check);
    }
  }  
  else if (strncmp("hum", arg, 3) == 0) {
    eeprom_read_block(tmp, (const void *) EEPROM_HUMIDITY_OFFSET, 4);
    eeprom_write_block(tmp, (void *) EEPROM_BACKUP_HUMIDITY_OFFSET, 4);

    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_HUMIDITY_CALIBRATION_BIT)) {
      CLEAR_BIT(backup_check, BACKUP_STATUS_HUMIDITY_CALIBRATION_BIT);
      eeprom_write_word((uint16_t *) EEPROM_BACKUP_CHECK, backup_check);
    }
  }  
  else if (strncmp("tz", arg, 2) == 0) {
    eeprom_read_block(tmp, (const void *) EEPROM_NTP_TZ_OFFSET_HRS, 4);
    eeprom_write_block(tmp, (void *) EEPROM_BACKUP_NTP_TZ_OFFSET_HRS, 4);

    if (!BIT_IS_CLEARED(backup_check, BACKUP_STATUS_TIMEZONE_CALIBRATION_BIT)) {
      CLEAR_BIT(backup_check, BACKUP_STATUS_TIMEZONE_CALIBRATION_BIT);
      eeprom_write_word((uint16_t *) EEPROM_BACKUP_CHECK, backup_check);
    }
  }    
  else if (strncmp("all", arg, 3) == 0) {
    valid = false;    
    configInject("backup mqttpwd\r");
    configInject("backup key\r");
    configInject("backup no2\r");
    configInject("backup co\r");
    configInject("backup temp\r");
    configInject("backup hum\r");    
    configInject("backup mac\r");
    configInject("backup tz\r");
    Serial.println();
  }
  else {
    valid = false;
    Serial.print(F("Error: Invalid parameter provided to 'backup' command - \""));
    Serial.print(arg);
    Serial.println(F("\""));
  }

  if (valid) {
    recomputeAndStoreConfigChecksum();
  }
}

boolean convertStringToFloat(char * str_to_convert, float * target) {
  char * end_ptr;
  *target = strtod(str_to_convert, &end_ptr);
  if (end_ptr != (str_to_convert + strlen(str_to_convert))) {
    return false;
  }
  return true;
}

void set_float_param(char * arg, float * eeprom_address, float (*conversion)(float)) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }

  // read the value at that address from eeprom
  float current_value = eeprom_read_float((const float *) eeprom_address);  
  
  float value = 0.0;
  if (convertStringToFloat(arg, &value)) {
    if (conversion) {
      value = conversion(value);
    }

    if(current_value != value){
      eeprom_write_float(eeprom_address, value);
      recomputeAndStoreConfigChecksum();
    }
  }
  else {
    Serial.print(F("Error: Failed to convert string \""));
    Serial.print(arg);
    Serial.println(F("\" to decimal number."));
  }
}

void set_ntp_timezone_offset(char * arg){
  set_float_param(arg, (float *) EEPROM_NTP_TZ_OFFSET_HRS, NULL);
}

// convert from nA/ppm to ppb/V
// from SPEC Sensors, Sensor Development Kit, User Manual, Rev. 1.5
// M[V/ppb] = Sensitivity[nA/ppm] * TIA_Gain[kV/A] * 10^-9[A/nA] * 10^3[V/kV] * 10^-3[ppb/ppm]
// TIA_Gain[kV/A] for NO2 = 350
// slope = 1/M
float convert_no2_sensitivity_to_slope(float sensitivity) {
  float ret = 1.0e9f;
  ret /= sensitivity;
  ret /= 350.0f;
  return ret;
}

// sets both sensitivity and slope
void set_no2_sensitivity(char * arg) {
  set_float_param(arg, (float *) EEPROM_NO2_SENSITIVITY, 0);
  set_float_param(arg, (float *) EEPROM_NO2_CAL_SLOPE, convert_no2_sensitivity_to_slope);
}

void set_no2_slope(char * arg) {
  set_float_param(arg, (float *) EEPROM_NO2_CAL_SLOPE, 0);
}

void set_no2_offset(char * arg) {
  set_float_param(arg, (float *) EEPROM_NO2_CAL_OFFSET, 0);
}

void set_reported_temperature_offset(char * arg) {
  set_float_param(arg, (float *) EEPROM_TEMPERATURE_OFFSET, 0);
}

void set_reported_humidity_offset(char * arg) {
  set_float_param(arg, (float *) EEPROM_HUMIDITY_OFFSET, 0);
}

// convert from nA/ppm to ppb/V
// from SPEC Sensors, Sensor Development Kit, User Manual, Rev. 1.5
// M[V/ppm] = Sensitivity[nA/ppm] * TIA_Gain[kV/A] * 10^-9[A/nA] * 10^3[V/kV]
// TIA_Gain[kV/A] for CO = 350
// slope = 1/M
float convert_co_sensitivity_to_slope(float sensitivity) {
  float ret = 1.0e6f;
  ret /= sensitivity;
  ret /= 350.0f;
  return ret;
}

void set_co_sensitivity(char * arg) {
  set_float_param(arg, (float *) EEPROM_CO_SENSITIVITY, 0);
  set_float_param(arg, (float *) EEPROM_CO_CAL_SLOPE, convert_co_sensitivity_to_slope);
}

void set_co_slope(char * arg) {
  set_float_param(arg, (float *) EEPROM_CO_CAL_SLOPE, 0);
}

void set_co_offset(char * arg) {
  set_float_param(arg, (float *) EEPROM_CO_CAL_OFFSET, 0);
}

void set_private_key(char * arg) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  // we've reserved 32-bytes of EEPROM for a private key
  // only exact 64-character hex representation is accepted
  uint8_t key[32] = {0};
  uint16_t len = strlen(arg);
  if (len == 64) {
    // process the characters as pairs
    for (uint8_t ii = 0; ii < 32; ii++) {
      char tmp[3] = {0};
      tmp[0] = arg[ii * 2];
      tmp[1] = arg[ii * 2 + 1];
      if (isxdigit(tmp[0]) && isxdigit(tmp[1])) {
        key[ii] = (uint8_t) strtoul(tmp, NULL, 16);
      }
      else {
        Serial.print(F("Error: Invalid hex value found ["));
        Serial.print(tmp);
        Serial.println(F("]"));
        return;
      }
    }

    eeprom_write_block(key, (void *) EEPROM_PRIVATE_KEY, 32);
    recomputeAndStoreConfigChecksum();
  }
  else {
    Serial.println(F("Error: Private key must be exactly 64 characters long, "));
    Serial.print(F("       but was "));
    Serial.print(len);
    Serial.println(F(" characters long."));
  }
}

void recomputeAndStoreConfigChecksum(void) {
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }
  
  uint16_t crc = computeEepromChecksum();
  eeprom_write_word((uint16_t *) EEPROM_CRC_CHECKSUM, crc);
}

uint16_t computeEepromChecksum(void) {
  uint16_t crc = 0;

  // there are EEPROM_CONFIG_MEMORY_SIZE - 2 bytes to compute the CRC16 over
  // the first byte is located at EEPROM_CRC_CHECKSUM + 2
  // the last byte is located at EEPROM_CONFIG_MEMORY_SIZE - 1
  for (uint16_t ii = 0; ii < EEPROM_CONFIG_MEMORY_SIZE - 2; ii++) {
    uint8_t value = eeprom_read_byte((uint8_t *) (EEPROM_CRC_CHECKSUM + 2 + ii));
    crc = _crc16_update(crc, value);
  }
  return crc;
}

uint16_t getStoredEepromChecksum(void){
  return eeprom_read_word((const uint16_t *) EEPROM_CRC_CHECKSUM);  
}

uint16_t computeFlashChecksum(void) {
  uint16_t crc = 0;
  // there are EEPROM_CONFIG_MEMORY_SIZE - 2 bytes to compute the CRC16 over
  // the first byte is located at SECOND_TO_LAST_4K_PAGE_ADDRESS + 2
  // the last byte is located at EEPROM_CONFIG_MEMORY_SIZE - 1  
  for (uint16_t ii = 0; ii < EEPROM_CONFIG_MEMORY_SIZE - 2; ii++) {
    uint8_t value = flash.readByte(((uint32_t) SECOND_TO_LAST_4K_PAGE_ADDRESS) + 2UL + ((uint32_t) ii));
    crc = _crc16_update(crc, value);
  }
  return crc;
}

uint16_t getStoredFlashChecksum(void){
  uint16_t stored_crc = flash.readByte(((uint32_t) SECOND_TO_LAST_4K_PAGE_ADDRESS) + 1UL);
  stored_crc <<= 8;
  stored_crc |= flash.readByte(((uint32_t) SECOND_TO_LAST_4K_PAGE_ADDRESS) + 0UL);
  return stored_crc;
}

/****** GAS SENSOR SUPPORT FUNCTIONS ******/

void selectNoSlot(void) {
  digitalWrite(7, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
}

void selectSlot1(void) {
  selectNoSlot();
  digitalWrite(10, HIGH);
}

void selectSlot2(void) {
  selectNoSlot();
  digitalWrite(9, HIGH);
}

void selectSlot3(void){
  selectNoSlot();
  digitalWrite(7, HIGH); 
}

boolean add_baseline_voltage_characterization(char * arg, uint32_t eeprom_table_base_address){
  // there should be three values provided
  // a temperature in degC        [float]  
  // a slope in Volts / degC      [float]
  // an intercept in Volts        [float]
  // tokenization should already be in progress when this function is called
  
  char * token = strtok(NULL, " "); // advance to the next token  
  if(token == NULL){
    Serial.println(F("Error: No temperature provided"));
    return false;  
  }
    
  if (!convertStringToFloat(token, &(baseline_voltage_struct.temperature_degC))) {
    Serial.print(F("Error: Failed to convert temperature string \""));
    Serial.print(token);
    Serial.println(F("\" to decimal number."));
    return false;
  }

  token = strtok(NULL, " "); // advance to the next token  
  if(token == NULL){
    Serial.println(F("Error: No slope provided"));
    return false;  
  }
  
  if (!convertStringToFloat(token, &(baseline_voltage_struct.slope_volts_per_degC))) {
    Serial.print(F("Error: Failed to convert slope string \""));
    Serial.print(token);
    Serial.println(F("\" to decimal number."));
    return false;
  }


  token = strtok(NULL, " "); // advance to the next token  
  if(token == NULL){
    Serial.println(F("Error: No intercpet provided"));
    return false;  
  }  
  
  if (!convertStringToFloat(token, &(baseline_voltage_struct.intercept_volts))) {
    Serial.print(F("Error: Failed to convert intercept string \""));
    Serial.print(token);
    Serial.println(F("\" to decimal number."));
    return false;
  }  

  // if you got this far, you've managed to parse three numbers and save them to the temp struct
  // and we should commit the results to EEPROM, and we should do so at the first available
  // index where the temperature is currently NaN, if there are no such spaces then report an 
  // error instructing the user to clear the table because it's full
  baseline_voltage_t tmp;

  // keep track of the highest temperature we've seen until we find an empty slot
  // and enforce the constraint that the temperature's in the table are monotonically increasing
  // any large negative number would do to initialize, but this is as cold as it gets, because Physics
  float temperature_of_first_valid_entry_degC = -273.15; 
  boolean empty_location_found = false;
  for(uint8_t ii = 0; ii < 5; ii++){
    eeprom_read_block((void *) &tmp, (void *) (eeprom_table_base_address + (ii*sizeof(baseline_voltage_t))), sizeof(baseline_voltage_t));
    if(!valid_temperature_characterization_struct(&tmp)){
      // ok we've found a slot where our new entry might be able to go, but first we have to enforce the monotinicity constraint
      // so that the search process is easier later
      if(temperature_of_first_valid_entry_degC >= baseline_voltage_struct.temperature_degC){
        // monotonicity contraint violation
        Serial.println(F("Error: Entries must be added in increasing order of temperature"));
        return false;
      }
      empty_location_found = true;
      // write the newly parsed struct at this index
      eeprom_write_block((void *) &baseline_voltage_struct, (void *) (eeprom_table_base_address + (ii*sizeof(baseline_voltage_t))), sizeof(baseline_voltage_t));
      break;
    }
    else{
      temperature_of_first_valid_entry_degC = tmp.temperature_degC;
    }
  }

  if(!empty_location_found){
    Serial.print(F("Error: Table is full, please run '"));             
    if(eeprom_table_base_address == EEPROM_CO_BASELINE_VOLTAGE_TABLE){
      Serial.print(F("co"));
    }
    else{
      Serial.print(F("no2"));
    }
    Serial.print(F("_blv clear' first"));
    Serial.println();
    return false;
  }

  return true;
  
}

boolean clear_baseline_voltage_characterization(uint32_t eeprom_table_base_address){
  baseline_voltage_t tmp;
  uint32_t erase_float = 0xFFFFFFFF;
  boolean deleted_at_least_one_entry = false;
  for(uint8_t ii = 0; ii < 5; ii++){
    eeprom_read_block((void *) &tmp, (void *) (eeprom_table_base_address + (ii*sizeof(baseline_voltage_t))), sizeof(baseline_voltage_t));
    if(!isnan(tmp.temperature_degC)){
      deleted_at_least_one_entry = true;
      tmp.temperature_degC = *((float *) (&erase_float));
      tmp.slope_volts_per_degC = *((float *) (&erase_float));
      tmp.intercept_volts = *((float *) (&erase_float));     
      eeprom_write_block((const void *) &tmp, (void *) (eeprom_table_base_address + (ii*sizeof(baseline_voltage_t))), sizeof(baseline_voltage_t));
    }
  }

  if(deleted_at_least_one_entry){
    return true;
  }
  return false;
}

void baseline_voltage_characterization_command(char * arg, uint32_t eeprom_table_base_address){
  boolean valid = true;  
  
  if(!configMemoryUnlocked(__LINE__)){
    return;
  }

  trim_string(arg);
  lowercase(arg);

  // make sure there's at least one argument
  char * token = strtok(arg, " "); // tokenize the string on spaces
  if(token == NULL){
    Serial.println(F("Error: no arguments provided"));
    return;  
  }

  // the first argument should be either "add", "show", or "clear"
  if(strcmp(token, "add") == 0){
    valid = add_baseline_voltage_characterization(token, eeprom_table_base_address); // tokenization in progress!
  }
  else if(strcmp(token, "clear") == 0){
    valid = clear_baseline_voltage_characterization(eeprom_table_base_address);
  }
  else if(strcmp(token, "show") == 0){
    valid = false;
    print_baseline_voltage_characterization(eeprom_table_base_address);
  }
  else{
    Serial.print(F("Error: valid sub-commands are [add, clear] but got '"));
    Serial.print(token);
    Serial.println("'");
    return;
  }
  
  if (valid) {
    recomputeAndStoreConfigChecksum();
  }    
}

void co_baseline_voltage_characterization_command(char * arg){
  baseline_voltage_characterization_command(arg, EEPROM_CO_BASELINE_VOLTAGE_TABLE);
}

void no2_baseline_voltage_characterization_command(char * arg){
  baseline_voltage_characterization_command(arg, EEPROM_NO2_BASELINE_VOLTAGE_TABLE);
}

void load_temperature_characterization_entry(uint32_t eeprom_table_base_address, uint8_t index){
  eeprom_read_block((void *) &baseline_voltage_struct, (void *) (eeprom_table_base_address + (index*sizeof(baseline_voltage_t))), sizeof(baseline_voltage_t));  
}

boolean load_and_validate_temperature_characterization_entry(uint32_t eeprom_table_base_address, uint8_t index){
  return valid_temperature_characterization_entry(eeprom_table_base_address, index);
}

boolean find_and_load_temperature_characterization_entry(uint32_t eeprom_table_base_address, float target_temperature_degC){  
  // this search requires that table entries are monotonically increasing in temperature
  // which is to say entry[0].temperature_degC < entry[1].temperature_degC < ... < entry[4].temperature_degC
  // this is enforced by the add entry mechanism

  // finds the first entry that has a temperature that is <= target_temperature
  // as a side-effect, if such an entry is found, it is loaded into baseline_voltage_struct
  
  int8_t index_of_highest_temperature_that_is_less_than_or_equal_to_target_temperature = -1;
  for(uint8_t ii = 0; ii < 5; ii++){
    if(load_and_validate_temperature_characterization_entry(eeprom_table_base_address, ii)){
      if(baseline_voltage_struct.temperature_degC <= target_temperature_degC){
        index_of_highest_temperature_that_is_less_than_or_equal_to_target_temperature = ii;
      }
      else{
        break;
      }
    }    
  }

  if(index_of_highest_temperature_that_is_less_than_or_equal_to_target_temperature >= 0){
    load_and_validate_temperature_characterization_entry(eeprom_table_base_address, 
      index_of_highest_temperature_that_is_less_than_or_equal_to_target_temperature);
    return true;  
  }

  // if we got to here it means that the target temperature is colder than the coldest characterized value
  // or there are no valid entries 
  return false; 
  
}

boolean valid_temperature_characterization_struct(baseline_voltage_t * temperature_characterization_struct_p){
  if(isnan(temperature_characterization_struct_p->temperature_degC)){
    return false;
  }
  
  if(isnan(temperature_characterization_struct_p->slope_volts_per_degC)){
    return false;
  }
  
  if(isnan(temperature_characterization_struct_p->intercept_volts)){
    return false;
  }

  if(temperature_characterization_struct_p->temperature_degC < -273.15){
    return false;
  }

  if(temperature_characterization_struct_p->temperature_degC > 60.0){
    return false;
  }
  
  return true;  
}

boolean valid_temperature_characterization_entry(uint32_t eeprom_table_base_address, uint8_t index){
  // the entry is valid only if none of the three fields are NaN
  // read the requested table entry into RAM, note side effect is baseline_voltage_struct is loaded with the data    
  load_temperature_characterization_entry(eeprom_table_base_address, index);
  return valid_temperature_characterization_struct(&baseline_voltage_struct);
}

boolean valid_temperature_characterization(uint32_t eeprom_table_base_address){
  // the table is valid only if the first entry is valid
  if(load_and_validate_temperature_characterization_entry(eeprom_table_base_address, 0)){
    return true;
  }
  return false;
}

void print_baseline_voltage_characterization_entry(uint32_t eeprom_table_base_address, uint8_t index){ 
  
  if(valid_temperature_characterization_entry(eeprom_table_base_address, index)){
    Serial.print(F("        ")); 
    Serial.print(index);
    Serial.print(F("\t"));
    Serial.print(baseline_voltage_struct.temperature_degC,8);
    Serial.print(F("\t"));
    Serial.print(baseline_voltage_struct.slope_volts_per_degC,8);
    Serial.print(F("\t"));
    Serial.print(baseline_voltage_struct.intercept_volts,8);
    Serial.println();
  }
  
}

void print_baseline_voltage_characterization(uint32_t eeprom_table_base_address){
  Serial.print(F("        ")); 
  Serial.println(F("idx\ttemp [degC]\tslope [V/degC]\tintercept [V]"));
  Serial.print(F("        ")); 
  Serial.println(F("---------------------------------------------------------"));
  if(!load_and_validate_temperature_characterization_entry(eeprom_table_base_address, 0)){
    Serial.print(F("        "));   
    Serial.println(F("No valid entries found.")); 
  }
  else{
    for(uint8_t ii = 0; ii < 5; ii++){
      if(load_and_validate_temperature_characterization_entry(eeprom_table_base_address, ii)){
        print_baseline_voltage_characterization_entry(eeprom_table_base_address, ii);
      }
      else{
        break;
      }
    }
  }
}

/****** LCD SUPPORT FUNCTIONS ******/
void safe_dtostrf(float value, signed char width, unsigned char precision, char * target_buffer, uint16_t target_buffer_length){
  char meta_format_string[16] = "%%.%df";
  char format_string[16] = {0};

  if((target_buffer != NULL) && (target_buffer_length > 0)){  
    snprintf(format_string, 15, meta_format_string, precision); // format string should come out to something like "%.2f"
    snprintf(target_buffer, target_buffer_length - 1, format_string, value);
  }

}

void backlightOn(void) {
  g_backlight_turned_on = true; // set a global flag
  uint8_t backlight_behavior = eeprom_read_byte((uint8_t *) EEPROM_BACKLIGHT_STARTUP);
  if(backlight_behavior != BACKLIGHT_ALWAYS_OFF){
    digitalWrite(A6, HIGH);
  }
}

void backlightOff(void) {
  uint8_t backlight_behavior = eeprom_read_byte((uint8_t *) EEPROM_BACKLIGHT_STARTUP);
  if(backlight_behavior != BACKLIGHT_ALWAYS_ON){
    digitalWrite(A6, LOW);
  }
}

void lcdFrownie(uint8_t pos_x, uint8_t pos_y){
  if((pos_x < 16) && (pos_y < 2)){
    lcd.setCursor(pos_x, pos_y);
    lcd.write((byte) 1); 
  }
}

void lcdSmiley(uint8_t pos_x, uint8_t pos_y){
  if((pos_x < 16) && (pos_y < 2)){
    lcd.setCursor(pos_x, pos_y);
    lcd.write((byte) 0); 
  }
}

void lcdBars(uint8_t numBars){  
  for(uint8_t ii = 0; ii < numBars && ii < 5; ii++){
    lcd.setCursor(5+ii, 1);
    lcd.write(3); // full bar
  }
  
  if(numBars < 5){
    for(uint8_t ii = 0; ii <  5 - numBars; ii++){
      lcd.setCursor(5 + numBars + ii, 1); 
      lcd.write((byte) 2); // empty bar
    }
  }
}

void setLCD_P(const char * str PROGMEM){  
  char tmp[33] = {0};
  strncpy_P(tmp, str, 32);
  setLCD(tmp);
}

//void dumpDisplayBuffer(void){
//  Serial.print(F("Debug: Line 1: "));
//  for(uint8_t ii = 0; ii < 17; ii++){
//    Serial.print(F("0x"));
//    Serial.print((uint8_t) g_lcd_buffer[0][ii],HEX);
//    if(ii != 16){
//      Serial.print(F(","));
//    }
//  } 
//  Serial.println();
//  Serial.print(F("Debug: Line 2: "));
//  for(uint8_t ii = 0; ii < 17; ii++){
//    Serial.print(F("0x"));
//    Serial.print((uint8_t) g_lcd_buffer[1][ii],HEX);
//    if(ii != 16){
//      Serial.print(F(","));
//    }
//  }
//  Serial.println();  
//}

void repaintLCD(void){
  static char last_painted[2][17] = {
    "                ",
    "                "
  };
  
  //dumpDisplayBuffer();      
  //  if(strlen((char *) &(g_lcd_buffer[0])) <= 16){
  //    lcd.setCursor(0,0);    
  //    lcd.print((char *) &(g_lcd_buffer[0]));
  //  }
  //  
  //  if(strlen((char *) &(g_lcd_buffer[1])) <= 16){
  //    lcd.setCursor(0,1);    
  //    lcd.print((char *) &(g_lcd_buffer[1])); 
  //  }    

  char tmp[2] = " ";
  for(uint8_t line = 0; line < 2; line++){
    for(uint8_t column = 0; column < 16; column++){
      if(last_painted[line][column] != g_lcd_buffer[line][column]){
        tmp[0] = g_lcd_buffer[line][column];
        lcd.setCursor(column, line);
        lcd.print(tmp);
      }
      last_painted[line][column] = g_lcd_buffer[line][column];
    } 
  }  
  
  g_lcd_buffer[0][16] = '\0'; // ensure null termination
  g_lcd_buffer[1][16] = '\0'; // ensure null termination  
}

void setLCD(const char * str){
  clearLCD();
  uint16_t original_length = strlen(str);  
  strncpy((char *) &(g_lcd_buffer[0]), str, 16);  
  if(original_length > 16){   
    strncpy((char *) &(g_lcd_buffer[1]), str + 16, 16);    
  }   
  repaintLCD();
}

void updateLCD(const char * str, uint8_t pos_x, uint8_t pos_y, uint8_t num_chars){
  uint16_t len = strlen(str);
  char * ptr = 0;
  if((pos_y == 0) || (pos_y == 1)){
    ptr = (char *) &(g_lcd_buffer[pos_y]);   
  }
  
  uint8_t x = 0;  // display buffer index
  uint8_t ii = 0; // input string index
  for(x = pos_x, ii = 0;  (x < 16) && (ii < len) && (ii < num_chars); x++, ii++){
    // don't allow the injection of non-printable characters into the display buffer
    if(isprint(str[ii])){    
      ptr[x] = str[ii];
    }
    else{
      ptr[x] = ' ';
    }
  }
  
  repaintLCD();
}

void clearLCD(){
  memset((uint8_t *) &(g_lcd_buffer[0]), ' ', 16);
  memset((uint8_t *) &(g_lcd_buffer[1]), ' ', 16);
  g_lcd_buffer[0][16] = '\0';
  g_lcd_buffer[1][16] = '\0';  
  lcd.clear();
  repaintLCD();
}

boolean index_of(char ch, char * str, uint16_t * index){
  uint16_t len = strlen(str);
  for(uint16_t ii = 0; ii < len; ii++){
    if(str[ii] == ch){
      *index = ii;
      return true;     
    }
  }
  
  return false;
}

void ltrim_string(char * str){
  uint16_t num_leading_spaces = 0;
  uint16_t len = strlen(str);
  for(uint16_t ii = 0; ii < len; ii++){
    if(!isspace(str[ii])){
      break;      
    }     
    num_leading_spaces++;
  }
  
  if(num_leading_spaces > 0){
    // copy the string left, including the null terminator
    // which is why this loop is <= len
    for(uint16_t ii = 0; ii <= len; ii++){
      str[ii] = str[ii + num_leading_spaces];
    }
  }
}

void rtrim_string(char * str){  
  // starting at the last character in the string
  // overwrite space characters with null characteres
  // until you reach a non-space character
  // or you overwrite the entire string
  int16_t ii = strlen(str) - 1;  
  while(ii >= 0){
    if(isspace(str[ii])){
      str[ii] = '\0';
    }
    else{
      break;
    }
    ii--;
  }
}

void trim_string(char * str){
  ltrim_string(str);
  
  //Serial.print(F("ltrim: "));
  //Serial.println(str);
  
  rtrim_string(str);
  
  //Serial.print(F("rtrim: "));
  //Serial.println(str);  
}

void replace_nan_with_null(char * str){
  if(strcmp(str, "nan") == 0){    
    strcpy(str, "null");
  }
}

void replace_character(char * str, char find_char, char replace_char){
  uint16_t len = strlen(str);
  for(uint16_t ii = 0; ii < len; ii++){
    if(str[ii] == find_char){
      str[ii] = replace_char;
    }
  }
}

// returns false if truncating the string to the field width
// would result in alter the whole number part of the represented value
// otherwise truncates the string to the field_width and returns true
boolean truncate_float_string(char * str, uint8_t field_width){
  // if there is a decimal point in the string after the field_width character
  // the value doesn't fit on a line at all, let alone after truncation
  // examples for field_width := 3
  //             v-- field boundardy (for field_width := 3)
  // Case 0:  0.3|4  is ok (and will be truncated to 0.3)
  // Case 0b: 0. |   is ok (and will be truncated to 0)    
  // Case 0c: -0.|3  is ok (and will be truncated to 0)            
  // Case 1:  1.2|4  is ok (and will be truncated to 1.2)
  // Case 1b: 1. |   is ok (and will be truncated to 1)
  // Case 1c: 1  |   is ok (and will be truncated to 1)
  // Case 2b: 12.|5  is ok (and will be truncated to 13)  
  // Cas3 2c: 12.|   is ok (and will be truncated to 12)
  // Cas3 2d: 12 |   is ok (and will be truncated to 12)  
  // Case 3:  123|.4 is ok (and will be truncated to 123)
  // Case 3b: 123|.  is ok (and will be truncated to 123)
  // Case 3c: 123|   is ok (and will be truncated to 123)
  // Case 3d: -12|3  is not ok (because it would be truncated to -12)  
  // Case 3f: -12|3. is not ok (because it would be truncated to -12)  
  // Case 4:  123|4  is not ok (because it would be truncated to 123)
  // Case 4b: 123|4. is not ok (because it would be truncated to 123)
  //          012|345678901234567 (index for reference)
  
  uint16_t period_index = 0;

  // first trim the string to remove leading and trailing ' ' characters
  trim_string(str);  

  uint16_t len = strlen(str);
  boolean string_contains_decimal_point = index_of('.', str, &period_index);

  //Serial.print(F("len > field_width: "));
  //Serial.print(len);
  //Serial.print(F(" > "));
  //Serial.print(field_width);
  //Serial.print(F(", string_contains_decimal_point = "));
  //Serial.print(string_contains_decimal_point);
  //Serial.print(F(", period_index = "));
  //Serial.println(period_index);
  
  if(len > field_width){   
    if(string_contains_decimal_point){
      // there's a decimal point in the string somewhere
      // and the string is longer than the field width
      if(period_index > field_width){
        // the decimal point occurs at least 
        // two characters past the field boundary
        return false;
      }
    }
    else{
      // it's a pure whole number
      // and there's not enough room in the field to hold it
      return false;
    }            
  } 
  
  // first truncate the string to the field width if it's longer than the field width
  if(len > field_width){
    str[field_width] = '\0';
    //Serial.print(F("truncated step 1: "));
    //Serial.println(str);
  }
  
  len = strlen(str);
  // if the last character in the string is a decimal point, lop it off
  if((len > 0) && (str[len-1] == '.')){
     str[len-1] = '\0';
     //Serial.print(F("truncated step 2:"));
     //Serial.println(str);
  }    
      
  // it's already adequately truncated if len <= field_width
  return true;
}

// the caller must ensure that there is adequate memory
// allocated to str, so that it can be safely padded 
// to target length
void leftpad_string(char * str, uint16_t target_length){
  uint16_t len = strlen(str);  
  if(len < target_length){
    uint16_t pad_amount = target_length - len;

    // shift the string (including the null temrinator) right by the pad amount
    // by walking backwards from the end to the start
    // and copying characters over (copying the null terminator is why it starts at len)
    for(int16_t ii = len; ii >= 0; ii--){
      str[ii + pad_amount] = str[ii];
    } 
    
    // then put spaces in the front 
    for(uint16_t ii = 0; ii < pad_amount; ii++){
      str[ii] = ' ';
    }
  }
}

void updateLCD(float value, uint8_t pos_x, uint8_t pos_y, uint8_t field_width){
  static char tmp[64] = {0};
  static char asterisks_field[17] = {0};
  memset(tmp, 0, 64);
  memset(asterisks_field, 0, 17);
  
  for(uint8_t ii = 0; (ii < field_width) && (ii < 16); ii++){
     asterisks_field[ii] = '*'; 
  }  
  
  //Serial.print(F("value: "));
  //Serial.println(value,8);  
  
  safe_dtostrf(value, -16, 6, tmp, 16);
  
  //Serial.print(F("dtostrf: "));
  //Serial.println(tmp);
  
  if(!truncate_float_string(tmp, field_width)){
    updateLCD(asterisks_field, pos_x, pos_y, field_width);
    return;
  }
  
  //Serial.print(F("truncate: "));
  //Serial.println(tmp);
  
  leftpad_string(tmp, field_width); 
  //Serial.print(F("leftpad_string: "));
  //Serial.println(tmp);
      
  updateLCD(tmp, pos_x, pos_y, field_width);  
}

void updateLCD(uint32_t ip, uint8_t line_number){
  char tmp[17] = {0};
  snprintf(tmp, 16, "%d.%d.%d.%d", 
    (uint8_t)(ip >> 24),
    (uint8_t)(ip >> 16),
    (uint8_t)(ip >> 8),       
    (uint8_t)(ip >> 0));    
  
  updateLCD(tmp, line_number);
}

void updateLCD(const char * str, uint8_t line_number){
  // center the string on the line
  char tmp[17] = {0};  
  uint16_t original_len = strlen(str);
  if(original_len < 16){
    uint8_t num_empty_chars_on_line = 16 - original_len;
    // pad the front of the string with spaces
    uint8_t half_num_empty_chars_on_line = num_empty_chars_on_line / 2;
    for(uint8_t ii = 0; ii < half_num_empty_chars_on_line; ii++){
      tmp[ii] = ' '; 
    }    
  }
  uint16_t len = strlen(tmp);  // length of the front padding    
  if((original_len + len) <= 16){
    strcat(tmp, str); // concatenate the string into the front padding-
  }
  
  len = strlen(tmp);
  if(len < 16){
    // pad the tail of the string with spaces
    uint8_t num_trailing_spaces = 16 - len;
    for(uint8_t ii = 0; ii < num_trailing_spaces; ii++){
      tmp[len + ii] = ' ';
    }
  }
  
  if(line_number < 2){
    updateLCD(tmp, 0, line_number, 16);
  }
}

void updateLCD(int32_t value, uint8_t pos_x, uint8_t pos_y, uint8_t num_chars){
  char tmp[17] = {0};
  snprintf(tmp, num_chars, "%ld", value);
  updateLCD(tmp, pos_x, pos_y, num_chars);
}

void updateLCD(char value, uint8_t pos_x, uint8_t pos_y, uint8_t num_chars){
  char tmp[17] = {0};
  tmp[0] = value;
  updateLCD(tmp, pos_x, pos_y, num_chars);
}

void updateCornerDot(void){
  static uint8_t on = 0;
  on = 1 - on;
  if(on == 1){
    updateLCD('.', 15, 1, 1);
  } 
  else{
    updateLCD(' ', 15, 1, 1); 
  }
}

void updateLcdProgressDots(void){
  static uint8_t cnt = 0;
  cnt++;
  uint8_t num_dots = cnt % 4;
  switch(num_dots){
    case 0: 
      updateLCD("   ", 1); 
      break;
    case 1:
      updateLCD(".  ", 1); 
      break;
    case 2:
      updateLCD(".. ", 1); 
      break;
    case 3:
      updateLCD("...", 1); 
      break;          
  } 
}

/****** WIFI SUPPORT FUNCTIONS ******/
void displayRSSI(void){ 
  char ssid[33] = {0};
  static ap_scan_result_t res = {0};    
  int8_t max_rssi = -256;
  boolean found_ssid = false;
  uint8_t target_network_secMode = 0;  
  uint8_t network_security_mode = eeprom_read_byte((const uint8_t *) EEPROM_SECURITY_MODE);   
  uint8_t num_results_found = 0;  
  
  eeprom_read_block(ssid, (const void *) EEPROM_SSID, 32);
  
  setLCD_P(PSTR(" SCANNING WI-FI "
                "                "));
  Serial.println(F("Info: Beginning Network Scan..."));                
  SUCCESS_MESSAGE_DELAY();

  boolean foundSSID = false; 
  uint8_t num_scan_attempts = 0;
  do{
    foundSSID = esp.scanForAccessPoint(ssid, &res, &num_results_found);
    num_scan_attempts++;
    if(!foundSSID){
      delay(100);
    }
  } while(!foundSSID && num_scan_attempts <= 5);
  
  Serial.print(F("Info: Network Scan found "));
  Serial.print(num_results_found);
  Serial.println(F(" networks"));
  
  Serial.print(F("Info: Access Point \""));
  Serial.print(ssid);
  Serial.print(F("\", ")); 
  if(foundSSID){
    
    Serial.print(F("RSSI = "));
    Serial.println(res.rssi);    
    int8_t rssi_dbm = res.rssi;
    lcdBars(rssi_to_bars(rssi_dbm));
    lcdSmiley(15, 1); // lower right corner
    
    if(network_security_mode == WLAN_SEC_AUTO){
      allowed_to_write_config_eeprom = true;
      if(target_network_secMode == 0){
        set_network_security_mode("open");
      }
      else if(target_network_secMode == 1){
        set_network_security_mode("wep");
      }
      else if(target_network_secMode == 2){
        set_network_security_mode("wpa");
      }
      else if((target_network_secMode == 3) || (target_network_secMode == 4)){
        set_network_security_mode("wpa2");
      }
      allowed_to_write_config_eeprom = false;      
    }
    
    ERROR_MESSAGE_DELAY(); // ERROR is intentional here, to get a longer delay
  }
  else{
    Serial.println(F("Not Found.")); 
    Serial.println(F("Info: Attempting to connect anyway."));
    updateLCD("NOT FOUND", 1);
    lcdFrownie(15, 1); // lower right corner
    ERROR_MESSAGE_DELAY();
  }
}

uint8_t rssi_to_bars(int8_t rssi_dbm){
  uint8_t num_bars = 0;
  if (rssi_dbm < -87){
    num_bars = 0;
  }
  else if (rssi_dbm < -82){
    num_bars = 1;
  }
  else if (rssi_dbm < -77){
    num_bars = 2;
  }
  else if (rssi_dbm < -72){
    num_bars = 3;
  }
  else if (rssi_dbm < -67){
    num_bars = 4;
  }
  else{
    num_bars = 5;
  }  
  
  return num_bars;
}

boolean restartWifi(){     
  if(!connectedToNetwork()){        
    delayForWatchdog();
    petWatchdog();
    current_millis = millis();   
    reconnectToAccessPoint();
    current_millis = millis();
    delayForWatchdog();
    petWatchdog();    
    acquireIpAddress(); 
    current_millis = millis();
    delayForWatchdog();
    petWatchdog();    
    displayConnectionDetails();
    
    clearLCD();
  }
  
  return connectedToNetwork();
}

bool displayConnectionDetails(void){
  uint32_t ipAddress, netmask, gateway;
 
  if(!esp.getIPAddress(&ipAddress, &gateway, &netmask))
  {
    Serial.println(F("Error: Unable to retrieve the IP Address!"));
    return false;
  }
  else
  {
    char ip_str[16] = {0};
    
    Serial.print(F("Info: IP Addr: ")); 
    memset(ip_str, 0, 16);
    esp.IpUint32ToString(ipAddress, &(ip_str[0])); 
    Serial.print((char *) ip_str);
    Serial.println();
    
    Serial.print(F("Info: Netmask: ")); 
    memset(ip_str, 0, 16);
    esp.IpUint32ToString(netmask, &(ip_str[0])); 
    Serial.print((char *) ip_str);
    Serial.println();
    
    Serial.print(F("Info: Gateway: ")); 
    memset(ip_str, 0, 16);
    esp.IpUint32ToString(gateway, &(ip_str[0])); 
    Serial.print((char *) ip_str);
    Serial.println();    
    
    updateLCD(ipAddress, 1);
    lcdSmiley(15, 1); // lower right corner
    SUCCESS_MESSAGE_DELAY();  
  
    return true;
  }
}

void reconnectToAccessPoint(void){
  static char ssid[33] = {0};
  static char network_password[32] = {0};
  static uint8_t connect_method = 0;
  static uint8_t network_security_mode = 0;
  static boolean first_access = true;
  static uint8_t mac_address[6] = {0};
  if(first_access){
    first_access = false;
    connect_method = eeprom_read_byte((const uint8_t *) EEPROM_CONNECT_METHOD);
    network_security_mode = eeprom_read_byte((const uint8_t *) EEPROM_SECURITY_MODE);  
    eeprom_read_block(ssid, (const void *) EEPROM_SSID, 32);
    eeprom_read_block(network_password, (const void *) EEPROM_NETWORK_PWD, 31); 
    eeprom_read_block(mac_address, (const void *) EEPROM_MAC_ADDRESS, 6);
  }

  esp.reset();
  esp.setNetworkMode(1);

  if (!esp.setMacAddress(mac_address)) {
    Serial.println(F("Error: Failed to write MAC address to ESP8266"));
  }
  
  switch(connect_method){
    case CONNECT_METHOD_DIRECT:
      Serial.print(F("Info: Connecting to Access Point with SSID \""));
      Serial.print(ssid);
      Serial.print(F("\"..."));
      setLCD_P(PSTR("CONNECTING TO AP"));
      updateLCD(ssid, 1);
      delayForWatchdog();
      petWatchdog();
      
      if(!esp.connectToNetwork((char *) ssid, (char *) network_password)){
        Serial.print(F("Error: Failed to connect to Access Point with SSID: "));
        Serial.println(ssid);
        Serial.flush();
        updateLCD("FAILED", 1);
        lcdFrownie(15, 1);
        ERROR_MESSAGE_DELAY();
        watchdogForceReset();
      }

      // if your configured for static ip address then setStaticIP
      // otherwise setDHCP
      
      Serial.println(F("OK."));
      updateLCD("CONNECTED", 1);
      lcdSmiley(15, 1);
      SUCCESS_MESSAGE_DELAY();
      break;   
    default:
      Serial.println(F("Error: Connection method not currently supported"));
      break;
  }  
}

void acquireIpAddress(void){
  static boolean first_access = true;  
  static uint32_t static_ip = 0;
  static uint32_t static_gateway = 0;
  static uint32_t static_netmask = 0;
  
  uint8_t noip[4] = {0};

  if(first_access){
    uint8_t ip[4] = {0};
    first_access = false;
    eeprom_read_block(ip, (const void *) EEPROM_STATIC_IP_ADDRESS, 4);
    static_ip = esp.IpArrayToIpUint32((uint8_t *) &(ip[0]));
    eeprom_read_block(ip, (const void *) EEPROM_STATIC_NETMASK, 4);
    static_netmask = esp.IpArrayToIpUint32((uint8_t *) &(ip[0]));
    eeprom_read_block(ip, (const void *) EEPROM_STATIC_GATEWAY, 4);
    static_gateway = esp.IpArrayToIpUint32((uint8_t *) &(ip[0]));
  }
  
  // if it's DHCP we're configured for, engage DHCP process
  if (static_ip == 0){
    /* Wait for DHCP to complete */
    Serial.print(F("Info: Request DHCP..."));
    setLCD_P(PSTR(" REQUESTING IP  "));   
    
    const long dhcp_timeout_duration_ms = 60000L;
    unsigned long previous_dhcp_timeout_millis = current_millis;
    uint32_t ii = 0;
    if(esp.setDHCP()){
      Serial.println(F("OK.")); 
    }
    else{
      Serial.println(F("Error: Failed to acquire IP address via DHCP. Rebooting."));
      Serial.flush();
      updateLCD("FAILED", 1);
      lcdFrownie(15, 1);
      ERROR_MESSAGE_DELAY();    
      watchdogForceReset();
    }
  }
  else{
    Serial.print(F("Info: Setting Static IP configuration..."));
    if(!esp.setStaticIPAddress(static_ip, static_netmask, static_gateway, 0)){
      Serial.println(F("Failed.")); 
    }
    else{
      Serial.println(F("OK.")); 
    }
  }
}

boolean connectedToNetwork(void){
  return esp.connectedToNetwork();
}

void espIpToArray(uint32_t ip, uint8_t * ip_array){
  for(uint8_t ii = 0; ii < 4; ii++){
    ip_array[ii] = ip & 0xff;
    ip >>= 8;
  }
}

uint32_t arrayToESP8266Ip(uint8_t * ip_array){
  uint32_t ip = 0;
  for(int8_t ii = 3; ii > 0; ii++){
    ip |= ip_array[ii];    
    ip <<= 8;
  }  
  return ip;
}

/****** ADC SUPPORT FUNCTIONS ******/
// returns the measured voltage in Volts
// 62.5 microvolts resolution in 16-bit mode
boolean burstSampleADC(float * result){
  #define NUM_SAMPLES_PER_BURST (8)

  // autoprobe to find out which address the ADC is at in this slot
  static uint8_t viable_addresses[8] = {0};
  static boolean first_access = true;
  if(first_access){
    first_access = false;
    // viable addresses for MCP3421 are 0x68...0x6F
    for(uint8_t ii = 0; ii < 8; ii++){
      viable_addresses[ii] = 0x68 + ii;
    }
  }

  adc.autoprobe(viable_addresses, 8);
  
  MCP342x::Config status;
  int32_t burst_sample_total = 0;
  uint8_t num_samples = 0;
  int32_t value = 0;
  for(uint8_t ii = 0; ii < NUM_SAMPLES_PER_BURST; ii++){    
    uint8_t err = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot, 
      MCP342x::resolution16, MCP342x::gain1, 75000, value, status);          
    if(err == 0){
      burst_sample_total += value;
      num_samples++;
    }
    else{  
      Serial.print(F("Error: ADC Read Error ["));    
      Serial.print((uint32_t) err, HEX); 
      Serial.println(F("]"));
    }
  }
  
  if(num_samples > 0){
    *result = (62.5e-6f * burst_sample_total) / num_samples;  
    return true;
  }

  return false;
}

/****** MQTT SUPPORT FUNCTIONS ******/
void clearTempBuffers(void){
  memset(converted_value_string, 0, 64);
  memset(compensated_value_string, 0, 64);
  memset(raw_instant_value_string, 0, 64);  
  memset(raw_value_string, 0, 64);
  memset(raw_instant_value_string, 0, 64);
  memset(scratch, 0, SCRATCH_BUFFER_SIZE);
  scratch_idx = 0;
  memset(MQTT_TOPIC_STRING, 0, 128);
  memset(response_body, 0, 256); 
}

boolean mqttResolve(void){
  uint32_t ip = 0;
  static boolean resolved = false;
  
  char mqtt_server_name[32] = {0};
  if(!resolved){
    eeprom_read_block(mqtt_server_name, (const void *) EEPROM_MQTT_SERVER_NAME, 31);
    setLCD_P(PSTR("   RESOLVING"));
    updateLCD("MQTT SERVER", 1);
    SUCCESS_MESSAGE_DELAY();    
    if  (!esp.getHostByName(mqtt_server_name, &ip) || (ip == 0))  {      
      Serial.print(F("Error: Couldn't resolve '"));
      Serial.print(mqtt_server_name);
      Serial.println(F("'"));
      
      updateLCD("FAILED", 1);
      lcdFrownie(15, 1);
      ERROR_MESSAGE_DELAY();
      return false;
    }  
    else{
      resolved = true;
      espIpToArray(ip, mqtt_server_ip);      
      Serial.print(F("Info: Resolved \""));
      Serial.print(mqtt_server_name);
      Serial.print(F("\" to IP address "));
      char ip_str[16] = {0};
      esp.IpUint32ToString(ip, &(ip_str[0]));
      Serial.print((char *) ip_str);
      
      updateLCD(ip, 1);      
      lcdSmiley(15, 1);
      SUCCESS_MESSAGE_DELAY();          
      Serial.println();    
    }
  }    
  return true;
}

boolean mqttReconnect(void){
   static boolean first_access = true;
   static char mqtt_username[32] = {0};
   static char mqtt_password[32] = {0};
   static uint8_t mqtt_auth_enabled = 0;
   static uint32_t mqtt_port = 0;
   
   boolean loop_return_flag = true;
   
   if(!mqttResolve()){
     return false;
   }
     
   if(first_access){
     first_access = false;
     loop_return_flag = false;
     eeprom_read_block(mqtt_username, (const void *) EEPROM_MQTT_USERNAME, 31);
     eeprom_read_block(mqtt_client_id, (const void *) EEPROM_MQTT_CLIENT_ID, 31);
     eeprom_read_block(mqtt_password, (const void *) EEPROM_MQTT_PASSWORD, 31);
     eeprom_read_block(MQTT_TOPIC_PREFIX, (const void *) EEPROM_MQTT_TOPIC_PREFIX, 63);
     mqtt_suffix_enabled = eeprom_read_byte((const uint8_t *) EEPROM_MQTT_TOPIC_SUFFIX_ENABLED);
     mqtt_auth_enabled = eeprom_read_byte((const uint8_t *) EEPROM_MQTT_AUTH);
     mqtt_port = eeprom_read_dword((const uint32_t *) EEPROM_MQTT_PORT);
     
     mqtt_client.setServer(mqtt_server_ip, mqtt_port);
     mqtt_client.setClient(esp);          
   }
   else{
     loop_return_flag = mqtt_client.loop();
   }           
   
   if(!loop_return_flag){
     Serial.print(F("Info: Connecting to MQTT Broker with Client ID \""));
     Serial.print(mqtt_client_id);
     Serial.print(F("\" "));
     boolean connect_status = false;
     if(mqtt_auth_enabled){
       Serial.print(F("using Authentication..."));
       connect_status = mqtt_client.connect(mqtt_client_id, mqtt_username, mqtt_password);
     }
     else{
       Serial.print(F("Without Authentication..."));
       connect_status = mqtt_client.connect(mqtt_client_id);
     }

     clearLCD();         
     if (connect_status) {
       Serial.println(F("OK."));
       return true;
     }
     else{
       Serial.println(F("Failed.")); 
       return false;       
     }    
   }  

   return loop_return_flag;
}

boolean mqttPublish(char * topic, char *str){
  boolean response_status = true;
  
  Serial.print(F("Info: MQTT publishing to topic "));
  Serial.print(topic);
  Serial.print(F("..."));
  
  uint32_t space_required = 5;
  space_required += strlen(topic);
  space_required += strlen(str);
  if(space_required >= 1023){
    Serial.println(F("Aborted."));
    response_status = false;
  } 
  else if(mqtt_client.publish(topic, str)){
    Serial.println(F("OK."));
    response_status = true;
  } 
  else {
    Serial.println(F("Failed."));
    response_status = false;
  }

  mqtt_client.loop();  
  
  return response_status;
}


boolean publishHeartbeat(){
  clearTempBuffers();
  static uint32_t post_counter = 0;  
  uint8_t sample = pgm_read_byte(&heartbeat_waveform[heartbeat_waveform_index++]); 
  
  snprintf(scratch, SCRATCH_BUFFER_SIZE-1, 
  "{"
  "\"serial-number\":\"%s\","
  "\"converted-value\":%d,"
  "\"firmware-version\":\"%s\","
  "\"publishes\":[\"no2\",\"co\",\"temperature\",\"humidity\"],"
  "\"counter\":%lu"
  "}", mqtt_client_id, sample, firmware_version, post_counter++);  
  
  if(heartbeat_waveform_index >= NUM_HEARTBEAT_WAVEFORM_SAMPLES){
     heartbeat_waveform_index = 0;
  }

  replace_character(scratch, '\'', '\"');
  
  strcat(MQTT_TOPIC_STRING, MQTT_TOPIC_PREFIX);
  strcat(MQTT_TOPIC_STRING, "heartbeat");    
  if(mqtt_suffix_enabled){
    strcat(MQTT_TOPIC_STRING, "/");      
    strcat(MQTT_TOPIC_STRING, mqtt_client_id);      
  }
  return mqttPublish(MQTT_TOPIC_STRING, scratch);
}

float toFahrenheit(float degC){
  return  (degC * 9.0f / 5.0f) + 32.0f;
}

boolean publishTemperature(){
  clearTempBuffers();
  float temperature_moving_average = calculateAverage(&(sample_buffer[TEMPERATURE_SAMPLE_BUFFER][0]), sample_buffer_depth);
  temperature_degc = temperature_moving_average;
  float raw_temperature = temperature_degc;
  float reported_temperature = temperature_degc - reported_temperature_offset_degC;
  if(temperature_units == 'F'){
    reported_temperature = toFahrenheit(reported_temperature);
    raw_temperature = toFahrenheit(raw_temperature);
    safe_dtostrf(toFahrenheit(instant_temperature_degc), -6, 2, raw_instant_value_string, 16);
  }
  else{
    safe_dtostrf(instant_temperature_degc, -6, 2, raw_instant_value_string, 16);
  }
  safe_dtostrf(reported_temperature, -6, 2, converted_value_string, 16);
  safe_dtostrf(raw_temperature, -6, 2, raw_value_string, 16);  
  
  trim_string(converted_value_string);
  trim_string(raw_value_string);
  trim_string(raw_instant_value_string);

  replace_nan_with_null(converted_value_string);
  replace_nan_with_null(raw_value_string);
  replace_nan_with_null(raw_instant_value_string);
  
  snprintf(scratch, SCRATCH_BUFFER_SIZE-1,
    "{"
    "\"serial-number\":\"%s\","
    "\"converted-value\":%s,"
    "\"converted-units\":\"deg%c\","
    "\"raw-value\":%s,"
    "\"raw-instant-value\":%s,"
    "\"raw-units\":\"deg%c\","
    "\"sensor-part-number\":\"SHT25\""
    "%s"
    "}", 
    mqtt_client_id, 
    converted_value_string, 
    temperature_units, 
    raw_value_string, 
    raw_instant_value_string,
    temperature_units, 
    gps_mqtt_string);

  replace_character(scratch, '\'', '\"');
    
  strcat(MQTT_TOPIC_STRING, MQTT_TOPIC_PREFIX);
  strcat(MQTT_TOPIC_STRING, "temperature");    
  if(mqtt_suffix_enabled){
    strcat(MQTT_TOPIC_STRING, "/");      
    strcat(MQTT_TOPIC_STRING, mqtt_client_id);      
  }
       
  return mqttPublish(MQTT_TOPIC_STRING, scratch);
}

boolean publishHumidity(){
  clearTempBuffers();
  float humidity_moving_average = calculateAverage(&(sample_buffer[HUMIDITY_SAMPLE_BUFFER][0]), sample_buffer_depth);
  relative_humidity_percent = humidity_moving_average;
  float raw_humidity = constrain(relative_humidity_percent, 0.0f, 100.0f);
  float reported_humidity = constrain(relative_humidity_percent - reported_humidity_offset_percent, 0.0f, 100.0f);
  
  safe_dtostrf(reported_humidity, -6, 2, converted_value_string, 16);
  safe_dtostrf(raw_humidity, -6, 2, raw_value_string, 16);
  safe_dtostrf(instant_humidity_percent, -6, 2, raw_instant_value_string, 16);
  
  trim_string(converted_value_string);
  trim_string(raw_value_string);
  trim_string(raw_instant_value_string);

  replace_nan_with_null(converted_value_string);
  replace_nan_with_null(raw_value_string);
  replace_nan_with_null(raw_instant_value_string);
  
  snprintf(scratch, SCRATCH_BUFFER_SIZE-1, 
    "{"
    "\"serial-number\":\"%s\","    
    "\"converted-value\":%s,"
    "\"converted-units\":\"percent\","
    "\"raw-value\":%s,"
    "\"raw-instant-value\":%s,"
    "\"raw-units\":\"percent\","  
    "\"sensor-part-number\":\"SHT25\""
    "%s"
    "}", 
    mqtt_client_id, 
    converted_value_string, 
    raw_value_string, 
    raw_instant_value_string, 
    gps_mqtt_string);  

  replace_character(scratch, '\'', '\"');

  strcat(MQTT_TOPIC_STRING, MQTT_TOPIC_PREFIX);
  strcat(MQTT_TOPIC_STRING, "humidity");    
  if(mqtt_suffix_enabled){
    strcat(MQTT_TOPIC_STRING, "/");      
    strcat(MQTT_TOPIC_STRING, mqtt_client_id);      
  }  
  return mqttPublish(MQTT_TOPIC_STRING, scratch);
}

void collectTemperature(void){  
  if(init_sht25_ok){
    if(sht25.getTemperature(&instant_temperature_degc)){
      addSample(TEMPERATURE_SAMPLE_BUFFER, instant_temperature_degc);       
      if(sample_buffer_idx == (sample_buffer_depth - 1)){
        temperature_ready = true;
      }
    }
  }
}

void collectHumidity(void){  
  if(init_sht25_ok){
    if(sht25.getRelativeHumidity(&instant_humidity_percent)){
      addSample(HUMIDITY_SAMPLE_BUFFER, instant_humidity_percent);       
      if(sample_buffer_idx == (sample_buffer_depth - 1)){
        humidity_ready = true;
      }
    }
  }
}

// TODO: create a vector to collect a touch sample every half second or so
//       and move all calls to collectTouch out of main processing into vector

void collectTouch(void){
  static uint8_t sample_write_index = 0;
  touch_sample_buffer[sample_write_index++] = touch.capacitiveSensor(30);
  
  if(sample_write_index == TOUCH_SAMPLE_BUFFER_DEPTH){
    sample_write_index = 0; 
  }
}

void processTouchVerbose(boolean verbose_output){
  const uint32_t touch_event_threshold = 50UL;  
  static boolean first_time = true;
  static unsigned long touch_start_millis = 0UL;
  long backlight_interval = 60000L; 
  static boolean backlight_is_on = false;
 
  if(first_time){
    first_time = false;
    backlight_interval = ((long) eeprom_read_word((uint16_t *) EEPROM_BACKLIGHT_DURATION)) * 1000;    
  }
  
  float touch_moving_average = calculateAverage(touch_sample_buffer, TOUCH_SAMPLE_BUFFER_DEPTH); 
  if(verbose_output){
    Serial.print(F("Info: Average Touch Reading: "));
    Serial.println(touch_moving_average);
  }
  
  if(touch_moving_average > touch_event_threshold){
    backlightOn();
    backlight_is_on = true;
    if(verbose_output){
      Serial.println(F("Info: Turning backlight on."));
    }
    touch_start_millis = current_millis;
  } 
  
  if((current_millis - touch_start_millis) >= backlight_interval) {        
    if(backlight_is_on){      
      if(verbose_output){
        Serial.println(F("Info: Turning backlight off (timer expired)."));   
      }
      backlightOff();      
      backlight_is_on = false;      
    }
  }  
}

void processTouch(void){
  processTouchVerbose(true);
}

void processTouchQuietly(void){
  processTouchVerbose(false);
}

void advanceSampleBufferIndex(void){
  sample_buffer_idx++;
  if((sample_buffer_idx >= sample_buffer_depth) || (sample_buffer_idx >= MAX_SAMPLE_BUFFER_DEPTH)){
    sample_buffer_idx = 0;
  }
}

void addSample(uint8_t sample_type, float value){
  if((sample_type < 4) && (sample_buffer_idx < MAX_SAMPLE_BUFFER_DEPTH)){
    sample_buffer[sample_type][sample_buffer_idx] = value;    
  }
}

void collectNO2(void){  
  if(init_no2_afe_ok && init_no2_adc_ok){
    selectSlot2();  
    if(burstSampleADC(&instant_no2_v)){      
      addSample(NO2_SAMPLE_BUFFER, instant_no2_v);
      if(sample_buffer_idx == (sample_buffer_depth - 1)){
        no2_ready = true;
      }
    }
  }
    
  selectNoSlot(); 
}

void collectCO(void ){    
  if(init_co_afe_ok && init_co_adc_ok){
    selectSlot1();  
    if(burstSampleADC(&instant_co_v)){   
      addSample(CO_SAMPLE_BUFFER, instant_co_v);      
      if(sample_buffer_idx == (sample_buffer_depth - 1)){
        co_ready = true;
      }      
    }
  }
    
  selectNoSlot();     
}

float pressure_scale_factor(void){
  float ret = 1.0f;
  
  static boolean first_access = true;
  static int16_t altitude_meters = 0.0f;
  
  if(first_access){
    first_access = false;
    altitude_meters = (int16_t) eeprom_read_word((uint16_t *) EEPROM_ALTITUDE_METERS);
  }

  if(altitude_meters != -1){
    // calculate scale factor of altitude and temperature
    const float kelvin_offset = 273.15f;
    const float lapse_rate_kelvin_per_meter = -0.0065f;
    const float pressure_exponentiation_constant = 5.2558774324f;
    
    float outside_temperature_kelvin = kelvin_offset + (temperature_degc - reported_temperature_offset_degC);
    float outside_temperature_kelvin_at_sea_level = outside_temperature_kelvin - lapse_rate_kelvin_per_meter * altitude_meters; // lapse rate is negative
    float pow_arg = 1.0f + ((lapse_rate_kelvin_per_meter * altitude_meters) / outside_temperature_kelvin_at_sea_level);
    ret = powf(pow_arg, pressure_exponentiation_constant);
  }
  
  return ret;
}

void no2_convert_from_volts_to_ppb(float volts, float * converted_value, float * temperature_compensated_value){
  static boolean first_access = true;
  static float no2_zero_volts = 0.0f;
  static float no2_slope_ppb_per_volt = 0.0f;
  float temperature_coefficient_of_span = 0.0f;
  float temperature_compensated_slope = 0.0f;
  if(first_access){
    // NO2 has negative slope in circuit, more negative voltages correspond to higher levels of NO2
    no2_slope_ppb_per_volt = eeprom_read_float((const float *) EEPROM_NO2_CAL_SLOPE); 
    no2_zero_volts = eeprom_read_float((const float *) EEPROM_NO2_CAL_OFFSET);
    first_access = false;
  }

  // apply piecewise linear regressions
  // to signal scaling effect curve
  float scaling_slope = 0.0f;
  float scaling_intercept = 0.0f;  
  if(temperature_degc < 0.0f){                 // < 0C  
    scaling_slope = -0.0355739076f;
    scaling_intercept = 97.9865525718f;
  }
  else if(temperature_degc < 20.0f){           // 0C .. 20C   
    scaling_slope = 0.1702484721f;
    scaling_intercept = 97.9953985672f; 
  }
  else{                                        // > 20C   
    scaling_slope = 0.3385634354f;
    scaling_intercept = 94.6638669473f;
  }
  float signal_scaling_factor_at_temperature = ((scaling_slope * temperature_degc) + scaling_intercept)/100.0f;
  // divide by 100 becauset the slope/intercept graphs have scaling factors in value

  // apply piecewise linear regressions
  // to baseline offset effect curve
  float baseline_offset_ppm_slope = 0.0f;
  float baseline_offset_ppm_intercept = 0.0f;
                                                                     
  if(temperature_degc < 33.0f){                          // < 33C
    baseline_offset_ppm_slope = -0.0007019288f;
    baseline_offset_ppm_intercept = 0.0177058403f;
  }
  else if(temperature_degc < 38.0f){                     // 33C .. 38C
    baseline_offset_ppm_slope = -0.0085978946f;
    baseline_offset_ppm_intercept = 0.2777254052f;
  }
  else if(temperature_degc < 42.0f){                     // 38C .. 42C
    baseline_offset_ppm_slope = -0.0196092331f;
    baseline_offset_ppm_intercept = 0.6994563331f;    
  }
  else if(temperature_degc < 46.0f){                     // 42C .. 46C
    baseline_offset_ppm_slope = -0.0351416006f;
    baseline_offset_ppm_intercept = 1.3566041809f;          
  }
  else{                                                  // > 46C
    baseline_offset_ppm_slope = -0.0531894279f;
    baseline_offset_ppm_intercept =  2.1948987152f;        
  }  
  float baseline_offset_ppm_at_temperature = ((baseline_offset_ppm_slope * temperature_degc) + baseline_offset_ppm_intercept); 
  float baseline_offset_ppb_at_temperature = baseline_offset_ppm_at_temperature * 1000.0f;
  // multiply by 1000 because baseline offset graph shows NO2 in ppm  
  float baseline_offset_voltage_at_temperature = -1.0f * baseline_offset_ppb_at_temperature / no2_slope_ppb_per_volt;
  // multiply by -1 because the ppm curve goes negative but the voltage actually *increases*
 
  float signal_scaling_factor_at_altitude = pressure_scale_factor();
  
  *converted_value = (no2_zero_volts - volts) * no2_slope_ppb_per_volt;
  if(*converted_value <= 0.0f){
    *converted_value = 0.0f;
  }

  if(valid_temperature_characterization(EEPROM_NO2_BASELINE_VOLTAGE_TABLE)){
    // do the math a little differently in this case
    // first figure out what baseline_offset_voltage_at_temperature is based on the characterization
    // (1) find the first entry in the table where temperature_degc is >= the table temperature
    // (2) if no such entry exists, then it is colder than the coldest characterized temperature
    // (3) use the associated slope and intercept to determine baseline_offset_voltage_at_temperature
    //     using the formula baseline_offset_voltage_at_temperature = slope * temperature_degc + intercept
    if(find_and_load_temperature_characterization_entry(EEPROM_NO2_BASELINE_VOLTAGE_TABLE, temperature_degc)){
      // great we have a useful entry in the table for this temperature, pull the slope and intercept
      // and use them to evaluate the baseline voltage at the current temperature
      baseline_offset_voltage_at_temperature = 
        temperature_degc * baseline_voltage_struct.slope_volts_per_degC + baseline_voltage_struct.intercept_volts;
    }
    else{
      // it's colder than the coldest characterized temperature
      // in this special case, instead of extrapolating, we will just evaluate the entry line
      // at coldest characterized temperature, and use that
      load_temperature_characterization_entry(EEPROM_NO2_BASELINE_VOLTAGE_TABLE, 0);
      baseline_offset_voltage_at_temperature = 
        baseline_voltage_struct.temperature_degC * baseline_voltage_struct.slope_volts_per_degC + baseline_voltage_struct.intercept_volts;
    }

    
    // then do the math with that number, if we did the characterization properly, then 
    // volts should always be *smaller* than baseline_offset_voltage_at_temperature in the presense of NO2 gas
    *temperature_compensated_value = (baseline_offset_voltage_at_temperature - volts) * no2_slope_ppb_per_volt 
                                     / signal_scaling_factor_at_temperature 
                                     / signal_scaling_factor_at_altitude;     
  }
  else{
    // otherwise the static data-sheet based method prevails and we do the same math as before
    *temperature_compensated_value = (no2_zero_volts - volts - baseline_offset_voltage_at_temperature) * no2_slope_ppb_per_volt 
                                     / signal_scaling_factor_at_temperature 
                                     / signal_scaling_factor_at_altitude; 
  }                                     
                                                                       
  if(*temperature_compensated_value <= 0.0f){
    *temperature_compensated_value = 0.0f;
  }
}

boolean publishNO2(){
  clearTempBuffers();
  float converted_value = 0.0f, compensated_value = 0.0f;    
  float no2_moving_average = calculateAverage(&(sample_buffer[NO2_SAMPLE_BUFFER][0]), sample_buffer_depth);
  no2_convert_from_volts_to_ppb(no2_moving_average, &converted_value, &compensated_value);
  no2_ppb = compensated_value;  
  safe_dtostrf(no2_moving_average, -8, 5, raw_value_string, 16);
  safe_dtostrf(converted_value, -4, 2, converted_value_string, 16);
  safe_dtostrf(compensated_value, -4, 2, compensated_value_string, 16); 
  safe_dtostrf(instant_no2_v, -8, 5, raw_instant_value_string, 16);
  
  trim_string(raw_value_string);
  trim_string(converted_value_string);
  trim_string(compensated_value_string);  
  trim_string(raw_instant_value_string);
  
  replace_nan_with_null(raw_value_string);
  replace_nan_with_null(converted_value_string);
  replace_nan_with_null(compensated_value_string);
  replace_nan_with_null(raw_instant_value_string);
  
  snprintf(scratch, SCRATCH_BUFFER_SIZE-1, 
    "{"
    "\"serial-number\":\"%s\","       
    "\"raw-value\":%s,"
    "\"raw-instant-value\":%s,"
    "\"raw-units\":\"volt\","
    "\"converted-value\":%s,"
    "\"converted-units\":\"ppb\","
    "\"compensated-value\":%s,"
    "\"sensor-part-number\":\"3SP-NO2-20-PCB\""
    "%s"
    "}",
    mqtt_client_id,
    raw_value_string, 
    raw_instant_value_string,
    converted_value_string, 
    compensated_value_string,
    gps_mqtt_string);  

  replace_character(scratch, '\'', '\"'); // replace single quotes with double quotes
  
  strcat(MQTT_TOPIC_STRING, MQTT_TOPIC_PREFIX);
  strcat(MQTT_TOPIC_STRING, "no2");    
  if(mqtt_suffix_enabled){
    strcat(MQTT_TOPIC_STRING, "/");      
    strcat(MQTT_TOPIC_STRING, mqtt_client_id);      
  } 
  return mqttPublish(MQTT_TOPIC_STRING, scratch);      
}

void co_convert_from_volts_to_ppm(float volts, float * converted_value, float * temperature_compensated_value){
  static boolean first_access = true;
  static float co_zero_volts = 0.0f;
  static float co_slope_ppm_per_volt = 0.0f;
  float temperature_coefficient_of_span = 0.0f;
  float temperature_compensated_slope = 0.0f;  
  if(first_access){
    // CO has positive slope in circuit, more positive voltages correspond to higher levels of CO
    co_slope_ppm_per_volt = eeprom_read_float((const float *) EEPROM_CO_CAL_SLOPE);
    co_zero_volts = eeprom_read_float((const float *) EEPROM_CO_CAL_OFFSET);
    first_access = false;
  }

  // apply piecewise linear regressions
  // to signal scaling effect curve
  float scaling_slope = 0.0f;
  float scaling_intercept = 0.0f;  
  if(temperature_degc < 0.0f){       // < 0C
    scaling_slope = 0.926586438f;
    scaling_intercept = 88.2942019565f;
  }
  else if(temperature_degc < 20.0f){ // 0C .. 20C
    scaling_slope = 0.6072408915f;
    scaling_intercept = 87.9176593244f; 
  }
  else{                              // > 20C
    scaling_slope = 0.2600853674f;
    scaling_intercept = 95.6168149016f;
  }
  float signal_scaling_factor_at_temperature = ((scaling_slope * temperature_degc) + scaling_intercept)/100.0f;
  // divide by 100 becauset the slope/intercept graphs have scaling factors in value
  
  // apply piecewise linear regressions
  // to baseline offset effect curve
  float baseline_offset_ppm_slope = 0.0f;
  float baseline_offset_ppm_intercept = 0.0f;
                
  if(temperature_degc < 15.5f){                          // no correction for < 15.5C
    baseline_offset_ppm_slope = 0.0f;
    baseline_offset_ppm_intercept = 0.0f;
  }
  else if(temperature_degc < 25.0f){                      // 15.5C .. 25C
    baseline_offset_ppm_slope = 0.2590260005f;
    baseline_offset_ppm_intercept = -4.0290395187f;
  }
  else if(temperature_degc < 32.0f){                      // 25C .. 32C
    baseline_offset_ppm_slope = 0.5387700048f;
    baseline_offset_ppm_intercept = -11.0899532317f;
  }
  else{                                                   // > 32C
    baseline_offset_ppm_slope = 0.824964228f;
    baseline_offset_ppm_intercept = -20.3665881995f;        
  }  
  float baseline_offset_ppm_at_temperature = (baseline_offset_ppm_slope * temperature_degc) + baseline_offset_ppm_intercept;  
  float baseline_offset_voltage_at_temperature = baseline_offset_ppm_at_temperature / co_slope_ppm_per_volt;

  float signal_scaling_factor_at_altitude = pressure_scale_factor();
    
  *converted_value = (volts - co_zero_volts) * co_slope_ppm_per_volt;
  if(*converted_value <= 0.0f){
    *converted_value = 0.0f; 
  }

  if(valid_temperature_characterization(EEPROM_CO_BASELINE_VOLTAGE_TABLE)){
    // do the math a little differently in this case
    // first figure out what baseline_offset_voltage_at_temperature is based on the characterization
    // (1) find the first entry in the table where temperature_degc is >= the table temperature
    // (2) if no such entry exists, then it is colder than the coldest characterized temperature
    // (3) use the associated slope and intercept to determine baseline_offset_voltage_at_temperature
    //     using the formula baseline_offset_voltage_at_temperature = slope * temperature_degc + intercept
    if(find_and_load_temperature_characterization_entry(EEPROM_CO_BASELINE_VOLTAGE_TABLE, temperature_degc)){
      // great we have a useful entry in the table for this temperature, pull the slope and intercept
      // and use them to evaluate the baseline voltage at the current temperature
      baseline_offset_voltage_at_temperature = 
        temperature_degc * baseline_voltage_struct.slope_volts_per_degC + baseline_voltage_struct.intercept_volts;
    }
    else{
      // it's colder than the coldest characterized temperature
      // in this special case, instead of extrapolating, we will just evaluate the entry line
      // at coldest characterized temperature, and use that
      load_temperature_characterization_entry(EEPROM_CO_BASELINE_VOLTAGE_TABLE, 0);
      baseline_offset_voltage_at_temperature = 
        baseline_voltage_struct.temperature_degC * baseline_voltage_struct.slope_volts_per_degC + baseline_voltage_struct.intercept_volts;
    }

    
    // then do the math with that number, if we did the characterization properly, then 
    // volts should always be *larger* than baseline_offset_voltage_at_temperature in the presense of CO gas
    *temperature_compensated_value = (volts - baseline_offset_voltage_at_temperature) * co_slope_ppm_per_volt 
                                     / signal_scaling_factor_at_temperature 
                                     / signal_scaling_factor_at_altitude;     
  }
  else{  
    // otherwise the static data-sheet based method prevails and we do the same math as before
    *temperature_compensated_value = (volts - co_zero_volts - baseline_offset_voltage_at_temperature) * co_slope_ppm_per_volt 
                                     / signal_scaling_factor_at_temperature
                                     / signal_scaling_factor_at_altitude;
  }                                     
                                   
  if(*temperature_compensated_value <= 0.0f){
    *temperature_compensated_value = 0.0f;
  }
}

boolean publishCO(){
  clearTempBuffers();  
  float converted_value = 0.0f, compensated_value = 0.0f;   
  float co_moving_average = calculateAverage(&(sample_buffer[CO_SAMPLE_BUFFER][0]), sample_buffer_depth);
  co_convert_from_volts_to_ppm(co_moving_average, &converted_value, &compensated_value);
  co_ppm = compensated_value;  
  safe_dtostrf(co_moving_average, -8, 5, raw_value_string, 16);
  safe_dtostrf(converted_value, -4, 2, converted_value_string, 16);
  safe_dtostrf(compensated_value, -4, 2, compensated_value_string, 16);    
  safe_dtostrf(instant_co_v, -8, 5, raw_instant_value_string, 16);
  
  trim_string(raw_value_string);
  trim_string(converted_value_string);
  trim_string(compensated_value_string);    
  trim_string(raw_instant_value_string);    

  replace_nan_with_null(raw_value_string);
  replace_nan_with_null(converted_value_string);
  replace_nan_with_null(compensated_value_string);
  replace_nan_with_null(raw_instant_value_string);
  
  snprintf(scratch, SCRATCH_BUFFER_SIZE-1, 
    "{"
    "\"serial-number\":\"%s\","      
    "\"raw-value\":%s,"
    "\"raw-instant-value\":%s,"
    "\"raw-units\":\"volt\","
    "\"converted-value\":%s,"
    "\"converted-units\":\"ppm\","
    "\"compensated-value\":%s,"
    "\"sensor-part-number\":\"3SP-CO-1000-PCB\""
    "%s"
    "}",
    mqtt_client_id,
    raw_value_string, 
    raw_instant_value_string,
    converted_value_string, 
    compensated_value_string,
    gps_mqtt_string);  

  replace_character(scratch, '\'', '\"'); // replace single quotes with double quotes

  strcat(MQTT_TOPIC_STRING, MQTT_TOPIC_PREFIX);
  strcat(MQTT_TOPIC_STRING, "co");    
  if(mqtt_suffix_enabled){
    strcat(MQTT_TOPIC_STRING, "/");      
    strcat(MQTT_TOPIC_STRING, mqtt_client_id);      
  } 
  return mqttPublish(MQTT_TOPIC_STRING, scratch);      
}

void petWatchdog(void){
  tinywdt.pet(); 
}

void delayForWatchdog(void){
  delay(120); 
}

void watchdogForceReset(void){
  tinywdt.force_reset(); 
  Serial.println(F("Error: Watchdog Force Restart failed. Manual reset is required."));
  setLCD_P(PSTR("AUTORESET FAILED"
                " RESET REQUIRED "));                
  backlightOn();                
  ERROR_MESSAGE_DELAY();
  for(;;){
    delay(1000);
  }
}

void watchdogInitialize(void){
  tinywdt.begin(100, 65000); 
}

// modal operation loop functions
void loop_wifi_mqtt_mode(void){
  static uint8_t num_mqtt_connect_retries = 0;
  static uint8_t num_mqtt_intervals_without_wifi = 0; 
  
  // mqtt publish timer intervals
  static unsigned long previous_mqtt_publish_millis = 0;
  
  if(current_millis - previous_mqtt_publish_millis >= reporting_interval){   
    suspendGpsProcessing();    
    previous_mqtt_publish_millis = current_millis;      
    
    printCsvDataLine();
    
    if(connectedToNetwork()){
      num_mqtt_intervals_without_wifi = 0;
      
      if(mqttReconnect()){         
        updateLCD("TEMP ", 0, 0, 5);
        updateLCD("RH ", 10, 0, 3);         
        updateLCD("NO2 ", 0, 1, 4);
        updateLCD("CO ", 10, 1, 3);
                      
        //connected to MQTT server and connected to Wi-Fi network        
        num_mqtt_connect_retries = 0;   
        if(!publishHeartbeat()){
          Serial.println(F("Error: Failed to publish Heartbeat."));  
        }
        
        if(init_sht25_ok){
          if(temperature_ready){
            if(!publishTemperature()){          
              Serial.println(F("Error: Failed to publish Temperature."));          
            }
            else{
              float reported_temperature = temperature_degc - reported_temperature_offset_degC;
              if(temperature_units == 'F'){
                reported_temperature = toFahrenheit(reported_temperature);
              }
              updateLCD(reported_temperature, 5, 0, 3);             
            }
          }
          else{
            updateLCD("---", 5, 0, 3);
          }        
        }
        else{
          // sht25 is not ok
          updateLCD("XXX", 5, 0, 3);
        }
        
        if(init_sht25_ok){
          if(humidity_ready){
            if(!publishHumidity()){
              Serial.println(F("Error: Failed to publish Humidity."));         
            }
            else{
              float reported_relative_humidity_percent = relative_humidity_percent - reported_humidity_offset_percent;
              updateLCD(reported_relative_humidity_percent, 13, 0, 3);  
            }
          }
          else{
            updateLCD("---", 13, 0, 3);
          }
        }
        else{
          updateLCD("XXX", 13, 0, 3);
        }
        
        if(init_no2_afe_ok && init_no2_adc_ok){
          if(no2_ready){
            if(!publishNO2()){
              Serial.println(F("Error: Failed to publish NO2."));          
            }
            else{
              updateLCD(no2_ppb, 5, 1, 3);  
            }
          }
          else{
            updateLCD("---", 5, 1, 3); 
          }
        }
        else{
          updateLCD("XXX", 5, 1, 3); 
        }
        
        if(init_co_afe_ok && init_co_adc_ok){
          if(co_ready){
            if(!publishCO()){
              Serial.println(F("Error: Failed to publish CO."));         
            }
            else{
              updateLCD(co_ppm, 13, 1, 3); 
            }
          }
          else{
            updateLCD("---", 13, 1, 3);  
          }
        }
        else{
          updateLCD("XXX", 13, 1, 3);
        }
    
      }
      else{
        // not connected to MQTT server
        num_mqtt_connect_retries++;
        Serial.print(F("Warn: Failed to connect to MQTT server "));
        Serial.print(num_mqtt_connect_retries);
        Serial.print(F(" consecutive time"));
        if(num_mqtt_connect_retries > 1){
          Serial.print(F("s"));           
        }
        Serial.println();
        
        if(num_mqtt_connect_retries >= 5){
          Serial.println(F("Error: MQTT Connect Failed 5 consecutive times. Forcing reboot."));
          Serial.flush();
          setLCD_P(PSTR("  MQTT SERVER   "
                        "    FAILURE     "));
          lcdFrownie(15, 1);
          ERROR_MESSAGE_DELAY();            
          watchdogForceReset();  
        }
      }
    }
    else{
      // not connected to Wi-Fi network
      num_mqtt_intervals_without_wifi++;
      Serial.print(F("Warn: Failed to connect to Wi-Fi network "));
      Serial.print(num_mqtt_intervals_without_wifi);
      Serial.print(F(" consecutive time"));
      if(num_mqtt_intervals_without_wifi > 1){
        Serial.print(F("s"));
      }
      Serial.println();      
      if(num_mqtt_intervals_without_wifi >= 5){
        Serial.println(F("Error: Wi-Fi Re-connect Failed 5 consecutive times. Forcing reboot."));
        Serial.flush();
        setLCD_P(PSTR(" WI-FI NETWORK  "
                      "    FAILURE     "));
        lcdFrownie(15, 1);
        ERROR_MESSAGE_DELAY();         
        watchdogForceReset();  
      }
      
      restartWifi();
    }
  }    
}

void loop_offline_mode(void){
  
  // write record timer intervals
  static unsigned long previous_write_record_millis = 0;

  if(current_millis - previous_write_record_millis >= reporting_interval){ 
    suspendGpsProcessing();   
    previous_write_record_millis = current_millis;
    printCsvDataLine();
  }  
}

/****** SIGNAL PROCESSING MATH SUPPORT FUNCTIONS ******/

float calculateAverage(float * buf, uint16_t num_samples){
  float average = 0.0f;
  for(uint16_t ii = 0; ii < num_samples; ii++){
    average += buf[ii];
  } 
  
  return average / num_samples;
}

void printCsvDataLine(){
  static boolean first = true;
  char * dataString = &(scratch[0]);  
  clearTempBuffers();
  
  uint16_t len = 0;
  uint16_t dataStringRemaining = SCRATCH_BUFFER_SIZE-1;
  
  if(first){
    first = false;      
    Serial.print(F("csv: "));    
    Serial.print(header_row);    
    Serial.println();    
  }  
  
  Serial.print(F("csv: "));
  printCurrentTimestamp(dataString, &dataStringRemaining);
  Serial.print(F(","));
  appendToString("," , dataString, &dataStringRemaining);
  
  if(temperature_ready){
    temperature_degc = calculateAverage(&(sample_buffer[TEMPERATURE_SAMPLE_BUFFER][0]), sample_buffer_depth);
    float reported_temperature = temperature_degc - reported_temperature_offset_degC;
    if(temperature_units == 'F'){
      reported_temperature = toFahrenheit(reported_temperature);
    }
    Serial.print(reported_temperature, 2);
    appendToString(reported_temperature, 2, dataString, &dataStringRemaining);
  }
  else{
    Serial.print(F("---"));
    appendToString("---", dataString, &dataStringRemaining);
  }
  
  Serial.print(F(","));
  appendToString("," , dataString, &dataStringRemaining);
  
  if(humidity_ready){
    relative_humidity_percent = calculateAverage(&(sample_buffer[HUMIDITY_SAMPLE_BUFFER][0]), sample_buffer_depth);
    float reported_relative_humidity = relative_humidity_percent - reported_humidity_offset_percent;        
    Serial.print(reported_relative_humidity, 2);
    appendToString(reported_relative_humidity, 2, dataString, &dataStringRemaining);
  }
  else{
    Serial.print(F("---"));
    appendToString("---", dataString, &dataStringRemaining);
  }    
  
  Serial.print(F(","));
  appendToString("," , dataString, &dataStringRemaining);
  
  float no2_moving_average = 0.0f;
  if(no2_ready){
    float converted_value = 0.0f, compensated_value = 0.0f;    
    no2_moving_average = calculateAverage(&(sample_buffer[NO2_SAMPLE_BUFFER][0]), sample_buffer_depth);
    no2_convert_from_volts_to_ppb(no2_moving_average, &converted_value, &compensated_value);
    no2_ppb = compensated_value;      
    Serial.print(no2_ppb, 2);
    appendToString(no2_ppb, 2, dataString, &dataStringRemaining);
  }
  else{
    Serial.print(F("---"));
    appendToString("---", dataString, &dataStringRemaining);
  }
  
  Serial.print(F(","));
  appendToString("," , dataString, &dataStringRemaining);
  
  float co_moving_average = 0.0f;
  if(co_ready){    
    float converted_value = 0.0f, compensated_value = 0.0f;   
    co_moving_average = calculateAverage(&(sample_buffer[CO_SAMPLE_BUFFER][0]), sample_buffer_depth);
    co_convert_from_volts_to_ppm(co_moving_average, &converted_value, &compensated_value);
    co_ppm = compensated_value;     
    Serial.print(co_ppm, 2);
    appendToString(co_ppm, 2, dataString, &dataStringRemaining);
  }
  else{
    Serial.print(F("---"));
    appendToString("---", dataString, &dataStringRemaining);
  }   
  
  Serial.print(F(","));
  appendToString("," , dataString, &dataStringRemaining);
  
  Serial.print(no2_moving_average, 6);
  appendToString(no2_moving_average, 6, dataString, &dataStringRemaining);
  
  Serial.print(F(","));
  appendToString("," , dataString, &dataStringRemaining);
  
  
  Serial.print(co_moving_average, 6);
  appendToString(co_moving_average, 6, dataString, &dataStringRemaining);
  
  Serial.print(gps_csv_string);
  appendToString(gps_csv_string, dataString, &dataStringRemaining);
  
  Serial.println();
  appendToString("\n", dataString, &dataStringRemaining);   
  
  if((mode == SUBMODE_OFFLINE) && init_sdcard_ok){
    char filename[16] = {0};
    getNowFilename(filename, 15);     
    File dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
      dataFile.print(dataString);
      dataFile.close();
      setLCD_P(PSTR("  LOGGING DATA  "
                    "   TO SD CARD   "));   
    }
    else {
      Serial.print("Error: Failed to open SD card file named \"");
      Serial.print(filename);
      Serial.println(F("\""));
      setLCD_P(PSTR("  SD CARD FILE  "
                    "  OPEN FAILED   "));
      lcdFrownie(15, 1);      
    }
  }
  else if((mode == SUBMODE_OFFLINE) && !init_sdcard_ok){
    setLCD_P(PSTR("  LOGGING DATA  "
                  "  TO USB-SERIAL "));
  }
}

boolean mode_requires_wifi(uint8_t opmode){
  boolean requires_wifi = false;
  
  if(opmode == SUBMODE_NORMAL){
    requires_wifi = true;
  } 
  
  return requires_wifi;
}

/****** INITIALIZATION SUPPORT FUNCTIONS ******/

// the following defines are what goes in the SPI flash where to signal to the bootloader
#define LAST_4K_PAGE_ADDRESS      0x7F000     // the start address of the last 4k page
#define MAGIC_NUMBER              0x00ddba11  // this word at the end of SPI flash
                                              // is a signal to the bootloader to 
                                              // think about loading it
#define MAGIC_NUMBER_ADDRESS      0x7FFFC     // the last 4 bytes are the magic number
#define CRC16_CHECKSUM_ADDRESS    0x7FFFA     // the two bytes before the magic number
                                              // are the expected checksum of the file
#define FILESIZE_ADDRESS          0x7FFF6     // the four bytes before the checksum
                                              // are the stored file size

#define IDLE_TIMEOUT_MS  10000     // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.

void invalidateSignature(void){
  flash_file_size = 0;
  flash_signature = 0;  
  while(flash.busy()){;}   
  flash.blockErase4K(LAST_4K_PAGE_ADDRESS);
  while(flash.busy()){;}   
}

uint16_t download_body_crc16_checksum = 0;
uint32_t download_body_bytes_received = 0;
boolean download_past_header = false; 
uint32_t download_content_length = 0;

void downloadHandleHeader(char * key, char * value){
//  Serial.print("\"");
//  Serial.print(key);
//  Serial.print("\" => \"");
//  Serial.print(value);
//  Serial.println("\"");
  
  if(strcmp(key, "Content-Length") == 0){
    download_content_length = strtoul(value, NULL, 10);   
  }
}

uint32_t downloadProcessHeader(uint8_t * data, uint32_t data_length){
  uint32_t start_index = 0;         
  static uint8_t header_guard_index = 0; 
  static boolean past_first_line = false;
  static char key[64] = {0};
  static char value[64] = {0};
  static uint8_t key_or_value = 0;
  static uint8_t keyval_index = 0;
  
  if(!download_past_header){
    for(uint32_t ii = 0; ii < data_length; ii++){                 
      switch(header_guard_index){
      case 0:
        if(data[ii] == '\r') header_guard_index++;
        else if(data[ii] == ':'){
          key_or_value = 1;
          keyval_index = 0;
        }
        else if(past_first_line){
          if(keyval_index < 63){
            if(!((keyval_index == 0) && (data[ii] == ' '))){ // strip leading spaces
              if(key_or_value == 0) key[keyval_index++] = data[ii];
              else value[keyval_index++] = data[ii];
            }
          }
          else{
            // warning the key string doesn't fit in 64 characters
          }
        }
        break;
      case 1:
        if(data[ii] == '\n'){
          header_guard_index++;
          
          if(past_first_line){
            downloadHandleHeader((char *) key, (char *) value);
          }
          
          past_first_line = true;
          key_or_value = 0;
          keyval_index = 0;
          memset(key, 0, 64);
          memset(value, 0, 64);          
        }
        else header_guard_index = 0;        
        break;
      case 2:
        if(data[ii] == '\r') header_guard_index++;
        else{
          key[keyval_index++] = data[ii];
          header_guard_index = 0;         
        }
        break;
      case 3:
        if(data[ii] == '\n') header_guard_index++;
        else header_guard_index = 0;         
        break;
      case 4:        
        download_past_header = true;
        start_index = ii;
        header_guard_index = 0;
        break;
      }      
    }
  }  

  return start_index;
}

void downloadFile(char * hostname, uint16_t port, char * filename, void (*responseBodyProcessor)(uint8_t *, uint32_t)){      
  unsigned long total_bytes_read = 0;
  uint8_t mybuffer[64] = {0};
  
  // re-initialize the globals
  download_body_crc16_checksum = 0;
  download_body_bytes_received = 0;   
  download_past_header = false;  
  download_content_length = 0;
  
  /* Try connecting to the website.
     Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
  */ 
  esp.connect(hostname, port);
  if (esp.connected()) {   
    memset(scratch, 0, 512);
    snprintf(scratch, SCRATCH_BUFFER_SIZE-1, "GET /%s HTTP/1.1\r\nHost: %s\r\n\r\n\r\n", filename, hostname);        
    esp.print(scratch);
    //Serial.print(scratch);    
  } else {
    Serial.println(F("Error: Server Connection failed"));    
    return;
  }

  Serial.println(F("Info: -------------------------------------"));
  
  /* Read data until either the connection is closed, or the idle timeout is reached. */ 
  unsigned long lastRead = millis(); 
  unsigned long num_bytes_read = 0;  
  unsigned long start_time = millis();
  uint32_t loop_counter = 0;
  
  while (esp.connected(false) && (millis() - lastRead < IDLE_TIMEOUT_MS)) {        
    while (esp.available()) {
      //char c = esp.read();
      num_bytes_read = esp.read(mybuffer, 64);     
      total_bytes_read += num_bytes_read;

      if(loop_counter == 4096){
        Serial.print("Info: ");
      }
      
      loop_counter++;
      if((loop_counter % 4096) == 0){
        Serial.print(".");
        updateCornerDot();
        petWatchdog();
      }

      if(responseBodyProcessor != 0){
        responseBodyProcessor(mybuffer, num_bytes_read); // signal end of stream
      }        
        
      lastRead = millis();
    }
  }
  
  esp.stop();
  
  Serial.println();  
  Serial.println("Info: Download Complete");
  Serial.print("Info: Total Bytes: ");
  Serial.println(total_bytes_read);
  Serial.print("Info: File Size: ");
  Serial.println(download_body_bytes_received);
  Serial.print("Info: Checksum: ");
  Serial.println(download_body_crc16_checksum);  
  Serial.print("Info: Duration: ");
  Serial.println(millis() - start_time);   
    
}

void processChkResponseData(uint8_t * data, uint32_t data_length){
  uint32_t start_index = downloadProcessHeader(data, data_length);
  char * endPtr;
  static char buff[64] = {0};
  static uint8_t buff_idx = 0;  
    
  if(download_past_header){       
    for(uint32_t ii = start_index; ii < data_length; ii++){     
      download_body_bytes_received++;
      download_body_crc16_checksum = _crc16_update(download_body_crc16_checksum, data[ii]);
      if(buff_idx < 63){
        buff[buff_idx++] = data[ii];         
      }      
    }

    if(download_body_bytes_received == download_content_length){
      integrity_num_bytes_total = strtoul(buff, &endPtr, 10);
      if(endPtr != 0){
        integrity_crc16_checksum = strtoul(endPtr, 0, 10);
        downloaded_integrity_file = true;
      }
      Serial.println("Info: Integrity Checks: ");
      Serial.print(  "Info:    File Size: ");
      Serial.println(integrity_num_bytes_total);
      Serial.print(  "Info:    CRC16 Checksum: ");
      Serial.println(integrity_crc16_checksum);    
    }
    
  }
    
}

void processHexResponseData(uint8_t * data, uint32_t data_length){
  uint32_t start_index = downloadProcessHeader(data, data_length);
  static uint8_t page[256] = {0};
  static uint16_t page_idx = 0;
  static uint32_t page_address = 0;
  static uint32_t local_download_body_bytes_received = 0;
  
  if(download_past_header){   
    for(uint32_t ii = start_index; ii < data_length; ii++){     
      download_body_bytes_received++;
      download_body_crc16_checksum = _crc16_update(download_body_crc16_checksum, data[ii]);

      if(page_idx < 256){
        page[page_idx++] = data[ii];  
        if(page_idx >= 256){
           page_idx = 0;
        }
      }     

      if((download_body_bytes_received == download_content_length) || (page_idx == 0)){       
        uint16_t top_bound = 256;
        if(page_idx != 0){
          top_bound = page_idx;
        }
        flash.writeBytes(page_address, page, top_bound);
                
        // clear the page
        memset(page, 0, 256);
        
        // advance the page address
        page_address += 256;        
      }       
    }

    if(download_body_bytes_received == download_content_length){
      if((download_body_bytes_received == integrity_num_bytes_total) && (download_body_crc16_checksum == integrity_crc16_checksum)){
        integrity_check_succeeded = true;
        Serial.println(F("Info: Integrity Check Succeeded!"));
      }
      else{
        Serial.println(F("Error: Integrity Check Failed!"));
        Serial.print(F("Error: Expected Checksum: "));
        Serial.print(integrity_crc16_checksum);
        Serial.print(F(", Actual Checksum: "));
        Serial.println(download_body_crc16_checksum);
        Serial.print(F("Error: Expected Filesize: "));
        Serial.print(integrity_num_bytes_total);
        Serial.print(F(", Actual Filesize: "));
        Serial.println(download_body_bytes_received);
      }
    }

  }
    
}

void checkForFirmwareUpdates(){ 
  static char filename[64] = {0};
  memset(filename, 0, 64);
  
  if(updateServerResolve()){
    // try and download the integrity check file up to three times    
    setLCD_P(PSTR("  CHECKING FOR  "
                  "    UPDATES     "));
    eeprom_read_block(filename, (const void *) EEPROM_UPDATE_FILENAME, 31);
    strncat_P(filename, PSTR(".chk"), 4);    
    
    downloadFile(update_server_name, 80, filename, processChkResponseData);   
    if(downloaded_integrity_file){
      lcdSmiley(15, 1);
      SUCCESS_MESSAGE_DELAY();
      delayForWatchdog();        
      petWatchdog();              
    }    

    if(downloaded_integrity_file){
      // compare the just-retrieved signature file contents 
      // to the signature already stored in flash
      if((flash_file_size != integrity_num_bytes_total) || 
        (flash_signature != integrity_crc16_checksum)){                 
        
        setLCD_P(PSTR("UPDATE AVAILABLE"
                      "  DOWNLOADING   "));       
        SUCCESS_MESSAGE_DELAY(); 
        delayForWatchdog();        
        petWatchdog();
       
        
        memset(filename, 0, 64); // switch to the hex extension
        eeprom_read_block(filename, (const void *) EEPROM_UPDATE_FILENAME, 31);
        strncat_P(filename, PSTR(".hex"), 4); 
        
        setLCD_P(PSTR("     PLEASE     "
                      "      WAIT      "));  
                      
        Serial.print(F("Info: Downloading \""));
        Serial.print(filename);
        Serial.print(F("\""));
        Serial.println();

        //before starting the download of the hex file, first erase the flash memory up to the second to last page        
        for(uint32_t page = 0; page < SECOND_TO_LAST_4K_PAGE_ADDRESS; page+=4096){
          while(flash.busy()){;}    
          flash.blockErase4K(page); 
          while(flash.busy()){;}           
        }
        
        downloadFile(update_server_name, 80, filename, processHexResponseData);    
        while(flash.busy()){;}           
        if(integrity_check_succeeded){ 
          // also write these parameters to their rightful place in the SPI flash
          // for consumption by the bootloader
          invalidateSignature();                                 
          
          flash.writeByte(CRC16_CHECKSUM_ADDRESS + 0, (integrity_crc16_checksum >> 8) & 0xff);
          flash.writeByte(CRC16_CHECKSUM_ADDRESS + 1, (integrity_crc16_checksum >> 0) & 0xff);
          
          flash.writeByte(FILESIZE_ADDRESS + 0, (integrity_num_bytes_total >> 24) & 0xff);
          flash.writeByte(FILESIZE_ADDRESS + 1, (integrity_num_bytes_total >> 16) & 0xff);    
          flash.writeByte(FILESIZE_ADDRESS + 2, (integrity_num_bytes_total >> 8)  & 0xff);
          flash.writeByte(FILESIZE_ADDRESS + 3, (integrity_num_bytes_total >> 0)  & 0xff);                
          
          flash.writeByte(MAGIC_NUMBER_ADDRESS + 0, MAGIC_NUMBER >> 24); 
          flash.writeByte(MAGIC_NUMBER_ADDRESS + 1, MAGIC_NUMBER >> 16); 
          flash.writeByte(MAGIC_NUMBER_ADDRESS + 2, MAGIC_NUMBER >> 8); 
          flash.writeByte(MAGIC_NUMBER_ADDRESS + 3, MAGIC_NUMBER >> 0); 
          Serial.println(F("Info: Wrote Magic Number"));          
          
          Serial.println(F("Info: Firmware Update Complete. Reseting to apply changes."));
          setLCD_P(PSTR("APPLYING UPDATES"
                        "WAIT ONE MINUTE "));          
          lcdSmiley(15, 1);
          SUCCESS_MESSAGE_DELAY();
          watchdogForceReset();        
        }
        else{
          Serial.println(F("Error: Firmware Update Failed. Try again later by resetting."));
          setLCD_P(PSTR(" UPDATE FAILED  "
                        "  RETRY LATER   "));
          lcdFrownie(15, 1);
          ERROR_MESSAGE_DELAY();         
        }
      }
      else{
        Serial.println("Info: Signature matches, skipping HEX download.");
        setLCD_P(PSTR("SOFTWARE ALREADY"
                      "   UP TO DATE   "));       
        SUCCESS_MESSAGE_DELAY();   
        delayForWatchdog();        
        petWatchdog();        
      }
    }
    else{
      Serial.println("Error: Failed to download integrity check file, skipping Hex file download");
    }    
  }  
}

boolean updateServerResolve(void){
  static boolean resolved = false;
  
  if(connectedToNetwork()){ 
    if(!resolved){
      eeprom_read_block(update_server_name, (const void *) EEPROM_UPDATE_SERVER_NAME, 31);      

      if(strlen(update_server_name) == 0){
        return false; // this is as indication that OTA updates are disabled
      }
      
      setLCD_P(PSTR("   RESOLVING"));
      updateLCD("UPDATE SERVER", 1);
      SUCCESS_MESSAGE_DELAY();
      
      if  (!esp.getHostByName(update_server_name, &update_server_ip32) || (update_server_ip32 == 0)){
        Serial.print(F("Error: Couldn't resolve '"));
        Serial.print(update_server_name);
        Serial.println(F("'"));
        
        updateLCD("FAILED", 1);
        lcdFrownie(15, 1);
        ERROR_MESSAGE_DELAY();
        return false;
      }
      else{
        resolved = true;    
        Serial.print(F("Info: Resolved \""));
        Serial.print(update_server_name);
        Serial.print(F("\" to IP address "));
        char ip_str[16] = {0};
        esp.IpUint32ToString(update_server_ip32, &(ip_str[0]));
        Serial.print((char *) ip_str);
        
        updateLCD(update_server_ip32, 1);      
        lcdSmiley(15, 1);
        SUCCESS_MESSAGE_DELAY();          
        Serial.println();    
      }
    }
     
    // connected to network and resolution succeeded 
    return true;
  }
  
  // not connected to network
  return false;
}

void getCurrentFirmwareSignature(void){  
  // retrieve the current signature parameters  
  flash_file_size = flash.readByte(FILESIZE_ADDRESS);
  flash_file_size <<= 8;
  flash_file_size |= flash.readByte(FILESIZE_ADDRESS+1);
  flash_file_size <<= 8;
  flash_file_size |= flash.readByte(FILESIZE_ADDRESS+2);  
  flash_file_size <<= 8;
  flash_file_size |= flash.readByte(FILESIZE_ADDRESS+3);  

  flash_signature = flash.readByte(CRC16_CHECKSUM_ADDRESS);
  flash_signature <<= 8;
  flash_signature |= flash.readByte(CRC16_CHECKSUM_ADDRESS+1);

  Serial.print(F("Info: Current firmware signature: "));
  Serial.print(flash_file_size);
  Serial.print(F(" "));
  Serial.print(flash_signature);
  Serial.println();   
}

/****** CONFIGURATION MIRRORING SUPPORT FUNCTIONS ******/
void commitConfigToMirroredConfig(void){
  if(!mirrored_config_matches_eeprom_config()){      
    mirrored_config_copy_from_eeprom(); // create a valid mirrored config from the current settings             
    if(!mirrored_config_integrity_check()){
      Serial.println(F("Error: Mirrored configuration commit failed to validate.")); 
      //TODO: should something be written to the LCD here?
    }
  }
  else{
    Serial.println(F("Info: Mirrored configuration already matches current configuration.")); 
  } 
}

boolean mirrored_config_matches_eeprom_config(void){
  boolean ret = true;

  // compare each corresponding byte of the Flash into the EEPROM
  for (uint16_t ii = 0; ii < EEPROM_CONFIG_MEMORY_SIZE; ii++) {
    uint8_t flash_value = flash.readByte(((uint32_t) SECOND_TO_LAST_4K_PAGE_ADDRESS) + ((uint32_t) ii));
    uint8_t eeprom_value = eeprom_read_byte((uint8_t *) (EEPROM_CRC_CHECKSUM + ii));
    if(flash_value != eeprom_value){
      ret = false;
      break;
    }
  }
  
  return ret;
} 

boolean configMemoryUnlocked(uint16_t call_id){
  if(!allowed_to_write_config_eeprom){
    Serial.print(F("Error: Config Memory is not unlocked, called from line number "));
    Serial.println(call_id);
    return false; 
  }
  
  return allowed_to_write_config_eeprom;
}

boolean mirrored_config_integrity_check(){
  boolean ret = false;
  uint16_t computed_crc = computeFlashChecksum();
  
  // interpret the CRC, little endian
  uint16_t stored_crc = getStoredFlashChecksum();
  
  if(stored_crc == computed_crc){
    ret = true; 
  }
  
  return ret;  
}


void mirrored_config_restore(void){  
  if(!allowed_to_write_config_eeprom){
    return;
  }

  // copy each byte from the Flash into the EEPROM
  for (uint16_t ii = 0; ii < EEPROM_CONFIG_MEMORY_SIZE; ii++) {
    uint8_t value = flash.readByte(((uint32_t) SECOND_TO_LAST_4K_PAGE_ADDRESS) + ((uint32_t) ii));
    eeprom_write_byte((uint8_t *) (EEPROM_CRC_CHECKSUM + ii), value);
  }
}

boolean mirrored_config_restore_and_validate(void){
  boolean integrity_check_passed = false;
  
  if(mirrored_config_integrity_check()){
    mirrored_config_restore();
    integrity_check_passed = checkConfigIntegrity();
    if(integrity_check_passed){
      Serial.println(F("Info: Successfully restored to last valid configuration.")); 
    }
    else{
      Serial.println(F("Info: Restored last valid configuration, but it's still not valid."));
    }  
  }
  else{
    Serial.println(F("Error: Mirrored configuration is not valid, cannot restore to last valid configuration.")); 
  }
  
  return integrity_check_passed;
}

void mirrored_config_copy_from_eeprom(void){
  
  mirrored_config_erase();
  Serial.print(F("Info: Writing mirrored config..."));

  // copy each byte from the EEPROM into the Flash
  for (uint16_t ii = 0; ii < EEPROM_CONFIG_MEMORY_SIZE; ii++) {
    uint8_t value = eeprom_read_byte((uint8_t *) (EEPROM_CRC_CHECKSUM + ii));
    flash.writeByte(((uint32_t) SECOND_TO_LAST_4K_PAGE_ADDRESS) + ((uint32_t) ii), value);    
  }
  Serial.println(F("OK."));
}

void mirrored_config_erase(void){
  Serial.print(F("Info: Erasing mirrored config..."));  
  flash.blockErase4K(SECOND_TO_LAST_4K_PAGE_ADDRESS);
  Serial.println(F("OK."));  
}

/****** TIMESTAMPING SUPPORT FUNCTIONS ******/
time_t AQE_now(void){
  selectSlot3();
  DateTime t = rtc.now();
  return (time_t) t.unixtime();
}

void currentTimestamp(char * dst, uint16_t max_len){
  time_t n = now();
  
  snprintf(dst, max_len, "%02d/%02d/%04d %02d:%02d:%02d", 
    month(n),
    day(n),
    year(n),
    hour(n),
    minute(n),
    second(n));
}

void printCurrentTimestamp(char * append_to, uint16_t * append_to_capacity_and_update){
  char datetime[32] = {0};
  currentTimestamp(datetime, 31);
  Serial.print(datetime);
  
  appendToString(datetime, append_to, append_to_capacity_and_update);  
}

void appendToString(char * str, char * append_to, uint16_t * append_to_capacity_and_update){
  if(append_to != 0){
    uint16_t len = strlen(str);
    if(*append_to_capacity_and_update >= len){
      strcat(append_to, str);
      *append_to_capacity_and_update -= len;
    }
  }  
}

void appendToString(float val, uint8_t digits_after_decimal_point, char * append_to, uint16_t * append_to_capacity_and_update){
  char temp[32] = {0};
  safe_dtostrf(val, 0, digits_after_decimal_point, temp, 31);
  appendToString(temp, append_to, append_to_capacity_and_update);
}

void getNowFilename(char * dst, uint16_t max_len){
  time_t n = now();
  snprintf(dst, max_len, "%02d%02d%02d%02d.csv", 
    year(n) % 100,
    month(n),
    day(n),
    hour(n));
}

void rtcClearOscillatorStopFlag(void){
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write(DS3231_REG_CONTROL);
    Wire.endTransmission();

    // control registers
    Wire.requestFrom(DS3231_ADDRESS, 2);
    uint8_t creg = Wire.read(); 
    uint8_t sreg = Wire.read();   
    
    sreg &= ~_BV(7); // clear bit 7 (msbit)
    
    Wire.beginTransmission(DS3231_ADDRESS);
    Wire.write((uint8_t) DS3231_REG_STATUS_CTL);
    Wire.write((uint8_t) sreg);
    Wire.endTransmission();    
}

/****** GPS SUPPORT FUNCTIONS ******/
void updateGpsStrings(void){
//  const char * gps_lat_lng_field_mqtt_template = ",\"latitude\":%.6f,\"longitude\":%.6f";
//  const char * gps_lat_lng_alt_field_mqtt_template  = ",\"latitude\":%.6f,\"longitude\":%.6f,\"altitude\":%.2f"; 
  const char * gps_lat_lng_field_mqtt_template = ",\"__location\":{\"lat\":%.6f,\"lon\":%.6f}"; 
  const char * gps_lat_lng_alt_field_mqtt_template  = ",\"__location\":{\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.2f}"; 
  const char * gps_lat_lng_field_csv_template = ",%.6f,%.6f,---";
  const char * gps_lat_lng_alt_field_csv_template  = ",%.6f,%.6f,%.2f";   

  static boolean first = true;
  if(first){
    first = false;
    float tmp = eeprom_read_float((float *) EEPROM_USER_LATITUDE_DEG);
    // Serial.println(tmp,6);    
    if(!isnan(tmp) && (tmp >= -90.0f) && (tmp <= 90.0f)){
      user_latitude = tmp;
    }
    
    tmp = eeprom_read_float((float *) EEPROM_USER_LONGITUDE_DEG);
    // Serial.println(tmp,6);
    if(!isnan(tmp) && (tmp >= -180.0f) && (tmp <= 180.0f)){
      user_longitude = tmp;
    }

    int16_t l_tmp = (int16_t) eeprom_read_word((uint16_t *) EEPROM_ALTITUDE_METERS);
    // Serial.println(l_tmp);
    if(tmp != -1){
      user_altitude = 1.0f * l_tmp;
    }

    // Serial.println(user_latitude, 6);
    // Serial.println(user_longitude, 6);
    // Serial.println(user_altitude, 2);
  }
  
  memset(gps_mqtt_string, 0, GPS_MQTT_STRING_LENGTH);
  memset(gps_csv_string, 0, GPS_CSV_STRING_LENGTH);  
  
  if(user_location_override && (user_latitude != TinyGPS::GPS_INVALID_F_ANGLE) && (user_longitude != TinyGPS::GPS_INVALID_F_ANGLE)){    
    if(user_altitude != -1.0f){
      snprintf(gps_mqtt_string, GPS_MQTT_STRING_LENGTH-1, gps_lat_lng_alt_field_mqtt_template, user_latitude, user_longitude, user_altitude);
      snprintf(gps_csv_string, GPS_CSV_STRING_LENGTH-1, gps_lat_lng_alt_field_csv_template, user_latitude, user_longitude, user_altitude);
    }
    else{      
      snprintf(gps_mqtt_string, GPS_MQTT_STRING_LENGTH-1, gps_lat_lng_field_mqtt_template, user_latitude, user_longitude);
      snprintf(gps_csv_string, GPS_CSV_STRING_LENGTH-1, gps_lat_lng_field_csv_template, user_latitude, user_longitude);
    }       
  }  
  else if((gps_latitude != TinyGPS::GPS_INVALID_F_ANGLE) && (gps_longitude != TinyGPS::GPS_INVALID_F_ANGLE)){
    if(gps_altitude != TinyGPS::GPS_INVALID_F_ALTITUDE){
      snprintf(gps_mqtt_string, GPS_MQTT_STRING_LENGTH-1, gps_lat_lng_alt_field_mqtt_template, gps_latitude, gps_longitude, gps_altitude);
      snprintf(gps_csv_string, GPS_CSV_STRING_LENGTH-1, gps_lat_lng_alt_field_csv_template, gps_latitude, gps_longitude, gps_altitude);
    }
    else{
      snprintf(gps_mqtt_string, GPS_MQTT_STRING_LENGTH-1, gps_lat_lng_field_mqtt_template, gps_latitude, gps_longitude);
      snprintf(gps_csv_string, GPS_CSV_STRING_LENGTH-1, gps_lat_lng_field_csv_template, gps_latitude, gps_longitude);
    }    
  }
  else{    
    strcpy_P(gps_csv_string, PSTR(",---,---,---"));
  }
}

void suspendGpsProcessing(void){
  gpsSerial.end();
  gps_disabled = true;
}

void resumeGpsProcessing(void){
  gpsSerial.begin(9600);
  gps_disabled = false;
}

/****** NTP SUPPORT FUNCTIONS ******/
void getNetworkTime(void){
  const unsigned long connectTimeout  = 15L * 1000L; // Max time to wait for server connection
  const unsigned long responseTimeout = 15L * 1000L; // Max time to wait for data from server  
  char server[32] = {0};  
  eeprom_read_block(server, (void *) EEPROM_NTP_SERVER_NAME, 31);
  uint8_t       buf[48];
  unsigned long ip, startTime, t = 0L;
  
  if(esp.getHostByName(server, &ip)) {
    static const char PROGMEM
      timeReqA[] = { 227,  0,  6, 236 },
      timeReqB[] = {  49, 78, 49,  52 };
    
    Serial.print(F("Info: Getting NTP Time..."));
    startTime = millis();
    do {
      esp.connectUDP(ip, 123);
    } while((!esp.connected()) &&
            ((millis() - startTime) < connectTimeout));

    if(esp.connected()) {      
      // Assemble and issue request packet
      memset(buf, 0, sizeof(buf));
      memcpy_P( buf    , timeReqA, sizeof(timeReqA));
      memcpy_P(&buf[12], timeReqB, sizeof(timeReqB));
      esp.write(buf, sizeof(buf));
      memset(buf, 0, sizeof(buf));
      startTime = millis();
      while((esp.available() < 44) &&
            ((millis() - startTime) < responseTimeout));
      
      if(esp.available() >= 44) {
        esp.read(buf, sizeof(buf));
        t = (((unsigned long)buf[40] << 24) |
             ((unsigned long)buf[41] << 16) |
             ((unsigned long)buf[42] <<  8) |
              (unsigned long)buf[43]) - 2208988800UL;      
      }
      esp.stop();
    }
  }

  if(t){
    t += eeprom_read_float((float *) EEPROM_NTP_TZ_OFFSET_HRS) * 60UL * 60UL; // convert offset to seconds
    tmElements_t tm;
    breakTime(t, tm);
    setTime(t);   
          
    selectSlot3();     
    DateTime datetime(t);
    rtc.adjust(datetime);
    rtcClearOscillatorStopFlag();
    selectNoSlot();
    
    memset(buf, 0, 48);
    snprintf((char *) buf, 47, 
      "%d/%d/%d",
      tm.Month,
      tm.Day,
      1970 + tm.Year);
      
    clearLCD();
    updateLCD((char *) buf, 0);    
    
    Serial.print((char *) buf);
    Serial.print(" ");
    memset(buf, 0, 48);
    snprintf((char *) buf, 47, 
      "%02d:%02d:%02d",
      tm.Hour,
      tm.Minute,
      tm.Second);

    updateLCD((char *) buf, 1);
    Serial.println((char *) buf);

    
    SUCCESS_MESSAGE_DELAY(); 
    
  }
  else{
    Serial.print(F("Failed."));
  }
}

void doSoftApModeConfigBehavior(void){  
  
  clearTempBuffers();
  
  randomSeed(micros());
  char random_password[16] = {0}; 
  strcpy(random_password, "AQ"); // start with AQ, and subsequent characters randomly chosen
  uint8_t fixed_password_length = strlen(random_password);
  const uint8_t random_password_length = 8;
    
  static const char whitelist[] PROGMEM = {
    '2',  '3',  '4',  '5',  '6',  '7',  '8',  '9',  
    'A',  'B',  'C',  'D',  'E',  'F',  'G',  'H',  
    'J',  'K',  'L',  'M',  'N',  'P',  'R',  'S',
    'T',  'U',  'V',  'W',  'X',  'Y',  'Z',  'a',  
    'b',  'c',  'd',  'e',  'f',  'g',  'h',  'i',  
    'j',  'k',  'm',  'n',  'o',  'p',  'q',  'r',  
    's',  't',  'u',  'v',  'w',  'x',  'y',  'z'
  };
  uint8_t _mac_address[6] = {0};
  eeprom_read_block((void *)_mac_address, (const void *) EEPROM_MAC_ADDRESS, 6);
  char egg_ssid[16] = {0};
  sprintf(egg_ssid, "egg-%02x%02x%02x", _mac_address[3], _mac_address[4], _mac_address[5]);  

  uint8_t ii = fixed_password_length;
  while(ii < random_password_length){
    uint8_t idx = random(0, 'z' - '0' + 1);
    char c = (char) ('0' + idx); 
    
    // if it's in the white list allow it
    boolean in_whitelist = false;
    for(uint8_t jj = 0; jj < sizeof(whitelist); jj++){
      char wl_char = pgm_read_byte(&(whitelist[jj]));      
      if(wl_char == c){
        in_whitelist = true;
        break;
      }
    }

    if(in_whitelist){
      random_password[ii++] = c;     
      random_password[ii] = NULL;     
    }
  }

  clearLCD();
  updateLCD(egg_ssid, 0);   
  updateLCD(&random_password[fixed_password_length], 1);

  const uint32_t default_seconds_remaining_in_softap_mode = 5UL * 60UL; // stay in softap for 5 minutes max  
  uint32_t seconds_remaining_in_softap_mode = default_seconds_remaining_in_softap_mode;
  
  boolean explicit_exit_softap = false;
  const uint16_t softap_http_port = 80;
  char ssid[33] = {0};
  char pwd[33] = {0};
  
  if(esp.setNetworkMode(3)){ // means softAP mode
    Serial.println(F("Info: Enabled Soft AP"));    
    if(esp.configureSoftAP(egg_ssid, random_password, 5, 3)){ // channel = 5, sec = WPA      
      // open a port and listen for config data messages, for up to two minutes
      if(esp.listen(softap_http_port)){
        Serial.print(F("Info: Listening for connections on port "));
        Serial.print(softap_http_port);
        Serial.println(F("..."));
        Serial.print(F("Info: SoftAP password is \""));
        Serial.print(random_password);        
        Serial.println(F("\""));
        
        unsigned long previousMillis = 0;
        const long interval = 1000;                
        boolean got_opening_brace = false;        
        boolean got_closing_brace = false;
                
        while((seconds_remaining_in_softap_mode != 0) && (!explicit_exit_softap)){
          unsigned long currentMillis = millis();

          if (currentMillis - previousMillis >= interval) {                        
            previousMillis = currentMillis;
            if(seconds_remaining_in_softap_mode != 0){
              seconds_remaining_in_softap_mode--;
              Serial.print(".");                           
              if((seconds_remaining_in_softap_mode % 60) == 0){
                Serial.println();
              }
            }                        
            petWatchdog();            
          }

          // check backlight touch
          if(currentMillis - previous_touch_sampling_millis >= touch_sampling_interval){              
            previous_touch_sampling_millis = currentMillis;    
            collectTouch();    
            processTouchQuietly();                            
          }      

          // check to determine if we have a GPS
          while(!gps_installed && gpsSerial.available()){
            char c = gpsSerial.read();
            if(c == '$'){
              gps_installed = true;
            }
          }
          
          // pay attention to incoming traffic          
          while(esp.available()){
            char c = esp.read();
            if(got_opening_brace){
              if(c == '}'){
                got_closing_brace = true;    
                scratch[scratch_idx++] = c;
                break;
              }
              else{
                if(scratch_idx < SCRATCH_BUFFER_SIZE - 1){
                  scratch[scratch_idx++] = c;                  
                }
                else{
                  Serial.println("Warning: scratch buffer out of memory");
                }
              }
            }
            else if(c == '{'){              
              got_opening_brace = true;
              scratch[scratch_idx++] = c;
            }            
          }                     

          if(got_closing_brace){
            // Serial.println("Message Body: ");
            // Serial.println(scratch);  
            
            // send back an HTTP response 
            // Then send a few headers to identify the type of data returned and that
            // the connection will not be held open.                            
            char response_template[] PROGMEM = "HTTP/1.1 200 OK\r\n"         
              "Content-Type: application/json; charset=utf-8\r\n"
              "Connection: close\r\n"
              "Server: air quality egg\r\n"
              "Content-Length: %d\r\n"
              "Access-Control-Allow-Origin: *\r\n"
              "\r\n"
              "%s";
            
            const char response_body_template[] PROGMEM = 
              "{"
               "\"sn\":\"%s\","
               "\"model\":\"%s\","
               "\"fw_ver\":\"%d.%d.%d\","
               "\"has_gps\":%s,"
               "\"use_gps\":%s,"
               "\"temp_unit\":\"%s\","
               "\"ssid\":\"%s\","
               "\"wifi_conn\":%s,"
               "\"lat\":%s,"
               "\"lng\":%s,"
               "\"alt\":%s"
              "}";            
                                       
            char serial_number[32] = {0};
            char ssid[33] = {0};
            char userLat[16] = {0};
            char userLng[16] = {0};
            char userAlt[16] = {0};  
            
            if(parseConfigurationMessageBody(scratch)){
              explicit_exit_softap = true;              
            }
            
            eeprom_read_block(serial_number, (const void *) EEPROM_MQTT_CLIENT_ID, 31);
            eeprom_read_block(ssid, (const void *) EEPROM_SSID, 32);
            
            floatToJsString(eeprom_read_float((float *) EEPROM_ALTITUDE_METERS), userAlt, 2);
            floatToJsString(eeprom_read_float((float *) EEPROM_USER_LATITUDE_DEG), userLat, 6);
            floatToJsString(eeprom_read_float((float *) EEPROM_USER_LONGITUDE_DEG), userLng, 6);
            
            const char model_type[] = "A";
            const char true_string[] = "true";
            const char false_string[] = "false";
            
            char * hasGPS = gps_installed ? (char *) true_string : (char *) false_string;
            char * useGPS = user_location_override ? (char *) false_string : (char *) true_string;
            char * wifiConn = wifi_can_connect ? (char *) true_string : (char *) false_string;
                        
            char tempUnit[2] = {0};
            tempUnit[0] = eeprom_read_byte((const uint8_t *) EEPROM_TEMPERATURE_UNITS);          

            clearTempBuffers();   
            
            sprintf(response_body, response_body_template, 
              serial_number,
              model_type,
              AQEV2FW_MAJOR_VERSION,
              AQEV2FW_MINOR_VERSION,
              AQEV2FW_PATCH_VERSION,
              hasGPS,
              useGPS,
              tempUnit,
              ssid,
              wifiConn,
              userLat,
              userLng,
              userAlt
              );             
                                           
            sprintf(scratch, response_template, strlen(response_body), response_body);            
            
            // Serial.print("Responding With: ");
            // Serial.println(scratch);  
            esp.print(scratch);
            seconds_remaining_in_softap_mode = default_seconds_remaining_in_softap_mode;
            
            // and wait 100ms to make sure it gets back to the caller
            delay(100); 
            
            clearTempBuffers();                 
            // if the parse failed we're back to waiting for a message body
            got_closing_brace = false;
            got_opening_brace = false;            
          }          
        }

        // if we got an explicit command to exit softap mode. 
        // commit configuration to mirrored backup if there were changes.
        // this is different behavior from CLI mode, which requires
        // successful connection to the target network to commit the configuration.
        if(explicit_exit_softap){
          if(!mirrored_config_matches_eeprom_config()){
            Serial.println(F("Info: Detected configuration changes"));
            if(checkConfigIntegrity()){              
              commitConfigToMirroredConfig();
            }
            else {
              Serial.print(F("Error: Integrity check failed, discarding changes..."));
              if(mirrored_config_restore_and_validate()){
                Serial.println(F("OK"));
              }
              else{
                Serial.println(F("Failed"));
              }
            }
          }
          else{
            Serial.println("Info: No configuration changes detected");
          }
        }
        
      }
      else{
        Serial.print(F("Error: Failed to start TCP server on port "));
        Serial.println(softap_http_port);
      }

      esp.setNetworkMode(1);
    }
    else{
      Serial.println(F("Error: Failed to configure Soft AP"));
    }
  }
  else{
    Serial.println(F("Error: Failed to start Soft AP Mode"));  
  }  

  Serial.println(F("Info: Exiting SoftAP Mode"));
}

void floatToJsString(float f, char * target, uint8_t digits_after_decimal_point){
  // only allow up 0 - 9 digits after decimal point
  if(digits_after_decimal_point > 9){
    digits_after_decimal_point = 9;
  }

  if(isnan(f)){
    strcpy(target, "null");
  }
  else{
    char format_string[] = "%.0f";                  // initialize digits after decimal point to 0
    format_string[2] += digits_after_decimal_point; // update the digits after decimal point
    sprintf(target, format_string, f);     
  }
}

boolean parseConfigurationMessageBody(char * body){
  jsmn_init(&parser);
  
  boolean found_ssid = false;
  boolean found_pwd = false;
  boolean handled_ssid_pwd = false;
  boolean found_exit = false;

  int16_t r = jsmn_parse(&parser, body, strlen(body), json_tokens, sizeof(json_tokens)/sizeof(json_tokens[0]));
  if(r > 0) {
    Serial.print(F("Info: Found "));
    Serial.print(r);
    Serial.println(F(" JSON tokens"));
  }
  else{
    Serial.print(F("Info: JSON parse failed for body \""));
    Serial.print(body);
    Serial.print(F("\" response code "));
    Serial.println(r);
  }
  char key[33] = {0};
  char value[33] = {0};
  char ssid[33] = {0};
  char pwd[33] = {0};
  
  for(uint8_t ii = 1; ii < r; ii+=2){    
    memset(key, 0, 33);
    memset(value, 0, 33);
    uint16_t keylen = json_tokens[ii].end - json_tokens[ii].start;
    uint16_t valuelen = json_tokens[ii+1].end - json_tokens[ii+1].start;

    if(keylen <= 32){
      strncpy(key, body + json_tokens[ii].start, keylen);      
    }

    if(valuelen <= 32){
      strncpy(value, body + json_tokens[ii+1].start, valuelen);
    }

     Serial.print(F("Info: JSON token: "));
     Serial.print(key);
     Serial.print(" => ");
     Serial.print(value);
     Serial.println();   

    // handlers for valid JSON keys
    if(strcmp(key, "ssid") == 0){
      found_ssid = true;
      strcpy(ssid, value);
    }
    else if(strcmp(key, "pwd") == 0){
      found_pwd = true;
      strcpy(pwd, value);
    }
    else if((strcmp(key, "exit") == 0) && (strcmp(value, "true") == 0)){
      found_exit = true;
    }
    else if(strcmp(key, "use_gps") == 0){
      if(strcmp(value, "true") == 0){
        set_user_location_enable("disable");
      }
      else if(strcmp(value, "false") == 0){
        set_user_location_enable("enable");
      }
    }
    else if(strcmp(key, "lat") == 0){
      set_user_latitude(value);
    }
    else if(strcmp(key, "lng") == 0){
      set_user_longitude(value);
    }
    else if(strcmp(key, "temp_unit") == 0){
      set_temperature_units(value);
    }
    else{
      Serial.print(F("Warn: posted key \""));
      Serial.print(key);
      Serial.print(F("\" is unrecognized and will be ignored."));
      Serial.println();
    }

    if(!handled_ssid_pwd && found_ssid && found_pwd){
      Serial.print(F("Info: Trying to connect to target network"));
      if(esp.connectToNetwork((char *) ssid, (char *) pwd, 30000)){      
        Serial.print(F("Info: Successfully connected to Network \""));
        Serial.print(ssid);
        Serial.print(F("\""));
        Serial.println();
        set_ssid(ssid);
        set_network_password(pwd);     
        wifi_can_connect = true;   
      }
      else{
        Serial.print(F("Info: Unable to connect to Network \""));
        Serial.print(ssid);
        Serial.print(F("\""));
        Serial.println();        
        wifi_can_connect = false;
      }
      handled_ssid_pwd = true;
    }
  }

  return found_exit;
}

