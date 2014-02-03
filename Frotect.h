// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section
#ifndef FrostProtect_H_
#define FrostProtect_H_

#include "Arduino.h"

#include <DallasTemperature.h>

#ifdef OSX
	#define MAX_STRANDS 4
	#define MAX_SENSORS 5
#else
	#define MAX_STRANDS 4
	#define MAX_SENSORS 5
#endif

// min/max. temperature history length
#define STATS_PER_HOUR_DAY_COUNT 2
#define STATS_PER_DAY_WEEK_COUNT 2

// maximum number of weeks, per-hour stats should be recorded for
//#define MAX_WEEKS 5 // for testing
#define MAX_WEEKS 30 // more than half a year

// record min/max temp. per sensor every 6 hours (do not reduce .... eeprom write cycles!)
#define MMHIST_INTERVAL 6

// record min/max temperatures for the last 12 days
#ifdef OSX
  #define MMHIST_DAY_COUNT 11
#else
  #define MMHIST_DAY_COUNT  11
#endif
#define TEMP_HIST_LEN (24/MMHIST_INTERVAL*MMHIST_DAY_COUNT)


#if TEMP_HIST_LEN>255
#error TEMP_HIST_LEN exceeds data range 0-255
#endif

#define WITH_TEMP_RAMPING


#define MAX_START_HOURS 15
typedef struct
{
	uint8_t  idx;
	uint16_t hours[MAX_START_HOURS];
} s_start_hours; // 81 bytes

typedef struct
{
	uint8_t       used  : 1;
	uint8_t       avail : 1;
	uint8_t       bound : 1;
	DeviceAddress address;
} s_sensor_info; // 9 bytes

typedef struct s_strand_info_eeprom
{
  public:

	// Since this info goes to EEProm, values are stored as 100*value in C, e.g. 350 means 3.5C.
	// Range -32768,...,32767 thus corresponds to limits -327C,...,327C
	int16_t  temp_lower_start100; // lower limit that will cause strand to light up
	int16_t  temp_upper_start100; // upper limit that will cause power down
	// power consumption stored in 2*watts, e.g. a value of 200 means 400W
	uint8_t  watts2;
	// 1+1+1 = 2 bytes

	s_sensor_info sensor;
	// 2+10 = 12 bytes

	#ifdef WITH_TEMP_RAMPING
	// day (thus tp_optime/1440) when ramp down phase started:
	uint16_t ramp_start_day;
	// 12+4 = 16 bytes
	// 100*C the value to increase/decrease temperature per day for ramp up/down.
	// The range -128,...,127 corresponds to -1.28,...,1.27C als allows e.g. to
	// ramp up/down the thresholds by almost 13C in ten days
	int8_t   ramp_per_day_temp100;
	// 16+1 = 17 bytes

	// lower limit end   value for ramping
	int16_t  temp_ramping_limit100 ; // lower limit end value for ramping
	#endif
} s_strand_info_eeprom; // 17 bytes


// s_strand_info inherits everything from s_strand_info_eeprom but adds those members
// that does not need to go to EEProm
struct s_strand_info //: s_strand_info_eeprom
{
  public:
	// below values are the same as in s_strand_info_eeprom
	int16_t       temp_lower100;
	int16_t       temp_upper100;
	uint8_t       watts2;
	s_sensor_info sensor;
	#ifdef WITH_TEMP_RAMPING
	uint16_t      ramp_start_day;
	int8_t        ramp_per_day_temp100;
	// same as s_strand_info_eeprom till here: 17 bytes
	int16_t       temp_lower_start100; // lower limit start value for ramping
	// lower/upper limit end value for ramping down/up
	int16_t       temp_ramping_limit100 ;
	#endif

	byte  lit        : 1; // status: light strand lit or off
	byte  update     : 1; // temperature or status changes -> print update
	byte  temp_valid : 1; // whether temperature measure was valid
	byte  pin        : 5; // pin this strand is connected to (0-31)
	// 17+1 = 18 bytes

	// Temperatures are stored as 100*value in celsius, e.g. 350 means 3.5C.
	// Total range is -160C,...,160C therefore and should be sufficient for "normal" environments.
	//int16_t  curr_temp100;
	int16_t curr_temp100;
	// 18+4 = 22 bytes

	// last time stamp when status changed:
	uint32_t ts_last_change;
	// 22+4 = 26 bytes

	// total number of errors on the sensor bound to this strand
	uint8_t  errors;
	// 26+1 = 27 bytes
};
typedef struct s_strand_info s_strand_info;


typedef struct  // 1 byte
{
	// This bits marks the entry in the array of hours which corresponds to the current hour.
	// Moving to the next hour means to update 4 bytes in EEProm - the former hour and the next hour
	// and spreads write cycles equally over the array of hours rather than writing continuously to
	// the same cells if an index was used. Using this flag means that every cell in this array will
	// be modifies at most once per day and will exceed max. write cycles after 100k/265=273 years.
	uint8_t current  : 1;
	uint8_t valid    : 1;
	uint8_t mins_on  : 6; // 2^6-1=63 > 60 m/h
} s_stats_hour;


typedef struct  // 2 bytes
{
	uint16_t valid   : 1;
	uint16_t mins_on : 15; // 2^16-1=65535 > 24*60=1440 m/d
} s_stats_day;

typedef struct  // 1 byte
{
	uint8_t valid      : 1;
	uint8_t hours2_on  : 7; // 127 > 7*24/2=84 h/week
} s_stats_week;

typedef struct
{
	s_stats_hour hours[24*STATS_PER_HOUR_DAY_COUNT]; // 24*1 bytes = 24
	s_stats_day  days[7*STATS_PER_DAY_WEEK_COUNT];   // 28*2 bytes = 56
	s_stats_week weeks[MAX_WEEKS]; // 24*1 bytes = 24
	// 24+56+24 = 104
} s_statistics; // total: 104 bytes

// temperature history saved in EEProm (requires 2 bytes only)
typedef struct
{
	int8_t  min2;      // -128,...,127 -> -64C,...,63.5C
	// max := min+diff
	uint8_t diff2 : 7; // 0,...,127 -> 0,...,63C (on top of min)
	uint8_t valid : 1;
} s_min_max; // 2 bytes

// temperature min/max for current hour (requires 4 bytes)
typedef struct  // 1 byte
{
	byte    valid  : 1;
	int16_t curr_min100; // -32768,...,32767 -> -327C,...,327C
	int16_t curr_max100; // -32768,...,32767 -> -327C,...,327C
} s_curr_minmax;

typedef struct
{
	s_min_max data[TEMP_HIST_LEN];
}
s_temp_history; // (24/6)*7*2 bytes = 56 bytes

// contains everything that goes to EEprom
typedef struct
{
	uint16_t       magic;            // 2 bytes
	uint8_t        num_sensors;
	uint8_t        num_strands;
	uint32_t       optime_minutes;   // 4 bytes (sufficient for 8000 years)
	uint16_t       cost_per_kWh100;  // 100*cost in cent per kWh
	uint8_t        day_count;        // 1 byte: 0-28
	uint8_t        week_count;       // 1 byte: 0-16
	uint8_t        hist_count;       // 1 byte
	s_strand_info_eeprom  strand_infos[MAX_SENSORS];  // 4*22 =  88 bytes
	s_statistics          statistics[MAX_STRANDS];    // 4*96 = 384 bytes
	s_temp_history        temp_histories[MAX_SENSORS]; // 5*56 bytes = 280
	s_start_hours         start_hours;
} s_eeprom_data; // 998 bytes < 1024


typedef enum { SENSOR_KNOWN, SENSOR_ADDED, SENSOR_IGNORED } e_senor_add_result;

// If the compiler complains about an error here (overflow in array dimension),
// the reason is that the size of s_eeprom_data excceds the EEPROM capacity.
// Try to reduce STATS_PER_HOUR_DAY_COUNT/7*STATS_PER_DAY_WEEK_COUNT/MAX_WEEKS or MMHIST_DAY_COUNT in this case.
struct FailsIfEEPromExceeded
{
	int c[E2END-sizeof(s_eeprom_data)];
};

//#define DUMP_EESIZE_COMPILE_TIME
#ifdef DUMP_EESIZE_COMPILE_TIME
template<int s> struct total_size_is_;
#warning For total size of EEPROM data see error in the next line
total_size_is_<sizeof(s_eeprom_data)> size_s_eeprom_data; // 998
//total_size_is_<sizeof(s_sensor_info)>  size_; // 45
//total_size_is_<sizeof(strand_infos)>  size_; // 135
//total_size_is_<sizeof(s_strand_info_eeprom)> size_; // 27
//total_size_is_<sizeof(s_statistics)>  size_; // 104
#endif

//end of add your includes here
#ifdef __cplusplus
//extern "C" {
#endif

void loop();
void setup();

#ifdef __cplusplus
//} // extern "C"
#endif

//add your function definitions for the project FrostProtect here


//Do not add code below this line
#endif /* FrostProtect_H_ */
