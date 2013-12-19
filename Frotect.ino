#include "Frotect.h"

/*
pin-out of sensor connector (top view on jack mounted in housing)
              ___ ___
             /   V   \
yellow/data |  o   o  | orange/Vcc 5V
gren/gnd    |  o   o  | n.c.
             \_______/
              
NOTES:

- External 4.k7 resistor needed between Vcc and data !!!!
- RST pin and GND connected via reed relay for external reset with a magnet.
- RST pin connected back to pin 10 for self-reboot.
- Sensors connected to one-wire bus on pin 9
- Strand relays connected to pins 2, 4, 5, 6, (8 - backup for cable failure)
- External LED connected to 11 (onboard LED pin 13 does not allow PWM)

Sensors:

1.	28;89;48;C8;04;00;00;0E
2.	28;50;81;E1;04;00;00;0E
3.	28;7B;2E;D9;04;00;00;28
4.	00
5.	28;29;7F;C8;04;00;00;87	was strand 4 (former strand 2)
*/

#include <eEEPROM.h>
//#include <MemoryFree.h>

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
// (warning: only initialized variables can be placed into program memory area)
#ifdef PROGMEM
	#undef PROGMEM
	#define PROGMEM __attribute__((section(".progmem.data")))
#endif

#ifdef OSX
#include <math.h>
#endif

///////////////////////////////////////////////////////////////////////////////////
// definitions for compiled in features
#ifdef OSX
	// #define WITH_TIME_LAPSE
	//#define WITH_TIMEWARP
#endif
#define WITH_HEARTBEAT // adds 90 bytes
#define WITH_PING
#define WITH_INDICATE_UPDATE // show "UP. " messages whenthere was a measuring recently
#define WITH_DUMPING // adds   1k code
#define WITH_MIN_MAX // adds 0.8k code + EEPROM data
#define WITH_CURR_MEASURE // dump "MEAS:" info (recent temp. measurements)
#define WITH_HELP
#define WITH_INDICATE_STARTUP // adds 100 bytes code
#define WITH_REAL_SENSORS
#define WITH_SAVE_UPTIME
#define WITH_SERIAL
#define WITH_STATS // adds 4k
#define WITH_SHOW_EVENTS
#define WITH_STRAND_UPDATES
#define WITH_MY_SENSORS // adds 100 bytes
//#define WITH_AUTOCONFIG
//#define WITH_TESTMODE
//#define WITH_DEBUG
//#define WITH_MEMSIZE_DEBUG
///////////////////////////////////////////////////////////////////////////////////

// disable the following info when time lapse enabled (too many otherwise)
#ifdef WITH_TIME_LAPSE
	#undef WITH_INDICATE_UPDATE
	#undef WITH_PING
	#undef WITH_CURR_MEASURE
	#undef WITH_HEARTBEAT
#endif


#ifdef WITH_REAL_SENSORS
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif

#ifdef OSX
	#if defined constrain
		#undef constrain
		#define constrain(VAL,MIN,MAX) ((VAL)<=(MIN) ? (MIN) : (VAL)>=(MAX) ? (MAX) : (VAL))
	#endif
#endif

// default thresholds for lower/upper temperature hysteresis
#define DEFAULT_LOWER (3.5f)
#define DEFAULT_UPPER (4.5f)


//////////////////////////////////// <prototypes>
// Arduino environment does not require these, however compiler on the Mac
void update_temp_ramping();
static void println_version();
static void eeprom_init();
void reboot();
static void reset();
void save_strand_infos();
void save_sensor(uint8_t i);
void save_sensors();
static float read_cost_per_kWh();
void save_cost_per_kWh100(uint16_t cost_per_kWh100);
void save_number_of_strands(uint8_t m);
void save_number_of_sensors_expected(uint8_t n);
static void set_day_index(uint8_t index);
static void set_week_index(uint8_t index);
static uint8_t get_current_hour();
void set_current_hour(uint8_t index);
void read_hour(uint8_t index, uint8_t strand, s_stats_hour & hour);
void write_hour(uint8_t index, uint8_t strand, s_stats_hour & hour);
void read_day(uint8_t index, uint8_t strand, s_stats_day & day);
void write_day(uint8_t index, uint8_t strand, s_stats_day & day);
void read_week(uint8_t index, uint8_t strand, s_stats_week & week);
void write_week(uint8_t index, uint8_t strand, s_stats_week & week);
uint8_t get_day_index();
uint8_t get_week_index();
static void save_one_week(uint8_t day_start_index);
void save_one_day(uint8_t hour_start_index);
static void dump_stats_hours();
void dump_stats_days();
void dump_stats_weeks();
static void save_statistics();
void set_minmax_index(uint8_t index);
uint8_t get_minmax_index();
static void read_minmax_entry(uint8_t sensor, uint16_t index, s_min_max & mm);
void save_minmax();
void dump_minmax(s_min_max & mm);
void dump_minmax();
void add_ontime(int s, const char * reason);
uint8_t update_history();
void save_uptime();
static boolean check_limits();
void show_string (PGM_P s);
static void dump_help();
static char to_upper(char c);
static boolean dump_strand_infos(boolean updates_only, boolean verbose);
static void dump_strand_info(uint8_t n, s_strand_info & i, boolean single, boolean verbose); //, const char * why);
void dump_general_info();
void compute_temps();
static void update_uptime();
static void blink_pin(uint8_t n, int8_t strand);
void inputClear();
void serialEvent();
static void mydelay(long ms);
boolean sensor_addr_defined(DeviceAddress addr);
static void dump_sensor_address(uint8_t  * addr);
static void dump_sensor(s_sensor_info & sensor);
static void dump_sensors();
void sensors_swap(s_sensor_info & a, s_sensor_info & b);
#ifdef WITH_REAL_SENSORS
int sensor_compare_addr(s_sensor_info & a, s_sensor_info & b);
int sensor_compare(s_sensor_info & a, s_sensor_info & b);
void sensors_sort();
static boolean sensor_replace(s_sensor_info & sensor);
static e_senor_add_result sensor_add(s_sensor_info & sensor, int8_t & at);
#endif
static void sensor_move(uint8_t index, int8_t direct);
static uint8_t sensors_count_available();
static void sensor_unbind(uint8_t i);
static boolean sensor_bind(uint8_t i);
static void sensors_detect();
static void print_lit(uint8_t n, boolean lit);
static void print_hb(uint8_t status, uint8_t cond, int16_t temp); //, byte err, byte cmpl, byte none);
static void flash_status_led(int8_t n);
static void indicate_startup();
boolean sensors_complete();
boolean sensors_none();
void show_status(boolean working);
//int get_free_memory();
void dump_starttime();
void setup();
void loop(void);
int16_t get_hb_temp100();
static void set_status_leds(uint8_t status);
static void set_status_leds_analog(uint8_t value);

#ifdef WITH_AUTOCONFIG
void sensor_get(uint8_t i, s_sensor_info & sensor);
static void auto_config();
#endif

#ifdef WITH_TESTMODE
static void test_mode();
#endif
//////////////////////////////////// </prototypes>


#ifdef OSX
	extern uint8_t doRestart;
#endif

const static uint8_t MAX_HOURS = 24*STATS_PER_HOUR_DAY_COUNT;
const static uint8_t MAX_DAYS  =  7*STATS_PER_DAY_WEEK_COUNT;

typedef enum { V_SILENT=0, V_NORMAL=1, V_VERBOSE=2, V_DEBUG=3 } e_verbosity;
const static e_verbosity V_DEFAULT = V_NORMAL;
static e_verbosity verbosity = V_DEFAULT;

// Data wire is plugged into pin 2 on the Arduino
const static byte PIN_ONE_WIRE_BUS =  9;
const static byte PIN_ONBOARD_LED  = 13;
const static byte PIN_STATUS_LED   = 11;
const static byte PIN_RESET        = 10;

// default pins that connect to the relays controlling the heating strands
static const uint8_t RELAY_PINS[] = { 2, 6, 5, 4, 8 };

//static const s_stats_hour ZERO_HOUR = { current : 1, valid : 0, mins_on : 0 };

#define MAX(A,B) (((A)>(B)) ? (A) : (B))
#define MIN(A,B) (((A)<(B)) ? (A) : (B))

#ifdef WITH_REAL_SENSORS
	// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
	static OneWire oneWire(PIN_ONE_WIRE_BUS);

	// Pass our oneWire reference to Dallas Temperature.
	static DallasTemperature sensors(&oneWire);
#endif

static boolean eeprom_reset   = false;
static boolean severe_error   = false;
static boolean input_complete = false;

#ifdef WITH_PING
static uint32_t millis_ping     = 0;
#endif

static uint32_t ts_optime       = 0; // total operation time in minutes
static uint32_t ts_starttime    = 0; // current optime when uC started
static uint32_t ts_last         = 0; // last timestanp seen in main loop()

static uint8_t   num_sensors_available = 0;
static uint8_t   num_sensors           = MAX_SENSORS;
static uint8_t   num_strands           = MAX_STRANDS;

#ifdef WITH_SERIAL
static char      input_buffer[16]= { 0 };         // a string to hold incoming serial data
static char    * inputPos  = input_buffer;
static char    * inputEnd  = input_buffer+sizeof(input_buffer)-1;
#endif

static s_strand_info strand_infos[MAX_SENSORS];  // 5*22 = 110 bytes (in RAM ,not EEProm)

#ifdef WITH_STATS
static s_stats_hour  current_stats[MAX_STRANDS];
#endif

static s_curr_minmax current_minmax[MAX_SENSORS];

static const s_eeprom_data * EE = 0;
eEE_CHECKSIZE(*EE); 

//const  uint16_t MAGIC = 0x2347;
// Magic number to write to the beginniing of EEProm to check whether it was initialized before.
// The value will change whenever n umber of sensors or strands was changed or when
// the structure s_eeprom_data has been changed significantly (thus the position ofstrand_infos changed).
const  uint16_t MAGIC = (uint16_t)((MAX_SENSORS<<12) + (MAX_STRANDS<<8) + (long)&(EE->strand_infos));


#define BZERO(TOKEN) memset(&(TOKEN), 0, sizeof((TOKEN)))

static const int16_t MIN_TEMP = -200;
static const int16_t MAX_TEMP =  300;

static uint16_t curr_op_day()
{
	return ts_optime/(24*60);
}

static int16_t to_temp100(float f)
{
	f = constrain(f, MIN_TEMP, MAX_TEMP);
	// do NOT use round() here (adds almost 100 bytes of code)
	int16_t i = 100*f;
	return i;
}

uint8_t to_eeprom_watts(int16_t w)
{
	return constrain(w, 0, 255);
}

static void from_eeprom(s_strand_info_eeprom &ee, s_strand_info &ram)
{
	ram.temp_lower100         = ee.temp_lower_start100;
	ram.temp_upper100         = ee.temp_upper_start100;
	ram.watts                 = ee.watts;
	ram.sensor                = ee.sensor;
	#ifdef WITH_TEMP_RAMPING
	// initial call to update_temp_ramping() in setup() will update
	// temp_lower100 and temp_upper100 according to ramping settings
	ram.temp_lower_start100   = ee.temp_lower_start100;
	ram.temp_ramping_limit100 = ee.temp_ramping_limit100;
	ram.ramp_start_day        = ee.ramp_start_day;
	ram.ramp_per_day_temp100  = ee.ramp_per_day_temp100;
	#endif
}

static void to_eeprom(s_strand_info &ram, s_strand_info_eeprom &ee)
{
	ee.temp_lower_start100   = ram.temp_lower100;
	ee.temp_upper_start100   = ram.temp_upper100;
	ee.watts                 = to_eeprom_watts(ram.watts);
	ee.sensor                = ram.sensor;
	#ifdef WITH_TEMP_RAMPING
	ee.temp_lower_start100   = ram.temp_lower_start100;
	ee.temp_ramping_limit100 = ram.temp_ramping_limit100;
	ee.ramp_start_day        = ram.ramp_start_day;
	ee.ramp_per_day_temp100  = ram.ramp_per_day_temp100;
	#endif
}


#ifdef WITH_MY_SENSORS
// This is my own sensor configuration. Using WITH_MY_SENSORS will avoid
// that sensors get rescanned in a different order after substantial reprogramming of
// the Arduino (i.e. if it was necessary to erase EEProm because of major changes)
//1.	28;89;48;C8;04;00;00;0E
//2.	28;29;7F;C8;04;00;00;87	was strand 4 (former strand 2)
//3.	28;7B;2E;D9;04;00;00;28
//4.	00
//5.	28;50;81;E1;04;00;00;6E

static const DeviceAddress MY_SENSORS[]  =
{
  { 0x28, 0x89, 0x48, 0xC8, 0x04, 0x00, 0x00, 0x0E },
  { 0x28, 0x29, 0x7F, 0xC8, 0x04, 0x00, 0x00, 0x87 },
  { 0x28, 0x7B, 0x2E, 0xD9, 0x04, 0x00, 0x00, 0x28 },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x28, 0x50, 0x81, 0xE1, 0x04, 0x00, 0x00, 0x6E }
};

static void mysensors_init()
{
	//const boolean debug = false;
	//if (debug) { PPRINTLN("D.mysensors_init"); dump_sensors(); }
	for (uint8_t s=0; s<MAX_SENSORS; s++)
	{
		s_strand_info & strand = strand_infos[s];
		memcpy(strand.sensor.address, MY_SENSORS[s], sizeof(DeviceAddress));
		strand.sensor.bound = strand.sensor.used = sensor_addr_defined(strand.sensor.address);
	}
	//if (debug) { dump_sensors(); PPRINTLN("D./mysensors_init");  }
}
#endif // WITH_MY_SENSORS


static void eeprom_init()
{
	//const boolean debug = false;
	//if (debug) PPRINTLN("D.eeprom_init");

	uint16_t magic = eEE_READN(EE->magic);
	BZERO(strand_infos);

	PPRINT("MAGIC."); Serial.print(magic, HEX);
	if (MAGIC!=magic)
	{
		PPRINTLN(".inv");

		magic = MAGIC;
		update_uptime(); // this will set: ts_optime = approx. 0;
		ts_last = 0;

		#ifdef WITH_STATS
		eEE_ZERO(EE->statistics);
		#endif
		eEE_ZERO(EE->temp_histories);
		eEE_ZERO(EE->start_hours);

		save_number_of_strands(MAX_STRANDS);
		// by default expect one sensor per strand thus no extra (unbound) sensors
		save_number_of_sensors_expected(MAX_STRANDS);

		save_cost_per_kWh100(25*100); // 2500 = 25 cents/kWh
		save_uptime();
		set_day_index(0);
		set_week_index(0);
		set_minmax_index(0);
		set_current_hour(MAX_HOURS-1);

		for (uint8_t s=0; s<MAX_SENSORS; s++)
		{
			s_strand_info & strand = strand_infos[s];
			strand.temp_lower100         = 350; // DEFAULT_LOWER; // 3.0f;
			strand.temp_upper100         = 500; // DEFAULT_UPPER; // 5.0f;
			// my 2nd (s=1) strand has 16W/m and is 9 meters long, other strands are 6m*16.1W
			strand.watts                 = (1==s) ? (9*16) : 97; // (6.0f*16.1f);
			strand.pin                   = RELAY_PINS[s];
			strand.ts_last_change        = ts_optime;
			strand.update                = 1; // make sure it gets dumped initially
			#ifdef WITH_TEMP_RAMPING
			strand.temp_lower_start100   = 350;
			strand.temp_ramping_limit100 = 250;
			strand.ramp_start_day        = curr_op_day();
			strand.ramp_per_day_temp100  = -25; // decrease limits by 0.2C per day
			#endif
		}

		#ifdef WITH_MY_SENSORS
		mysensors_init();
		#endif

		save_strand_infos();

		eEE_WRITEN(EE->magic, magic);
		eeprom_reset = true;
	}
	else
	{
		PPRINTLN(".val");

		num_sensors = eEE_READN(EE->num_sensors);
		num_sensors = MIN(num_sensors, MAX_SENSORS);

		num_strands = eEE_READN(EE->num_strands);
		num_strands = MIN(num_strands, MAX_STRANDS);

		ts_optime = eEE_READN(EE->optime_minutes);
		ts_last = ts_optime;

		for (uint8_t s=0; s<MAX_SENSORS; s++)
		{
			s_strand_info & strand = strand_infos[s];

			// read into the struct that represents the strand in eeprom
			s_strand_info_eeprom ee;
			eEE_READ(EE->strand_infos[s], ee);

			// and convert to the representation in RAM
			from_eeprom(ee, strand);

			strand.pin            = RELAY_PINS[s];
			strand.ts_last_change = ts_optime;
			strand.update         = 1; // make sure it gets dumped initially
			//strand.lit          = 0;
			//strand.sensor.avail = 0;
		}
	}

	//if (debug) PPRINTLN("D./eeprom_init");
}

void reboot()
{
	#ifdef OSX
		doRestart = true;
	#else
        digitalWrite(PIN_RESET, LOW);
        asm volatile ("jmp 0");
	#endif
}

static void reset()
{
    // explicit cast necessary to uint16_t to make the templating work
	uint16_t wrong_magic = 0xffff;
	#ifdef OSX
	fprintf(stderr, "OSX: WRITING WRONG MAGIC %x\n", wrong_magic);
	#endif
	eEE_WRITEN(EE->magic, wrong_magic);
	reboot();
}

void save_strand_infos()
{
	// takes 90 * 3.3ms = 297ms to complete
	for (uint8_t i=0; i<MAX_SENSORS; i++)
	{
		s_strand_info & strand = strand_infos[i];
		s_strand_info_eeprom ee;
		to_eeprom(strand, ee);
		eEE_WRITE(ee, EE->strand_infos[i]);
	}
}

void save_sensor(uint8_t i)
{
	// take 9 * 3.3ms = 30ms to complete
	eEE_WRITE(strand_infos[i].sensor, EE->strand_infos[i].sensor);
}

void save_sensors()
{
	// take 4 * 9 * 3.3ms = 118ms to complete
	 for (uint8_t i=0; i<MAX_SENSORS; i++) save_sensor(i);
}

static float read_cost_per_kWh()
{
	uint16_t cost_per_kWh100;
	cost_per_kWh100 = eEE_READN(EE->cost_per_kWh100);
	return 0.01*cost_per_kWh100;
}

void save_cost_per_kWh100(uint16_t cost_per_kWh100)
{
	//uint16_t cost_per_kWh100 = 100*cost_per_kWh;
	eEE_WRITE(cost_per_kWh100, EE->cost_per_kWh100);
}

void save_number_of_strands(uint8_t m)
{
	eEE_WRITE(m, EE->num_strands);
	num_strands = m;
}

void save_number_of_sensors_expected(uint8_t n)
{
	eEE_WRITE(n, EE->num_sensors);
	num_sensors = n;
}

static void set_day_index(uint8_t index)
{
	eEE_WRITE(index, EE->day_count);
}

static void set_week_index(uint8_t index)
{
	eEE_WRITE(index, EE->week_count);
}

#ifdef WITH_STATS
void set_current_hour(uint8_t index)
{
	uint8_t strand = 0;
	for (uint8_t h=0; h<MAX_HOURS; h++)
	{
		s_stats_hour hour;
		eEE_READ(EE->statistics[strand].hours[h], hour);

		int expected = (h==index) ? 1 : 0;

		if (hour.current !=expected)
		{
			hour.current = expected;
			eEE_WRITE(hour, EE->statistics[strand].hours[h]);
		}
	}
}

static uint8_t get_current_hour()
{
	uint8_t strand = 0;
	for (uint8_t h=0; h<MAX_HOURS; h++)
	{
		s_stats_hour hour;
		eEE_READ(EE->statistics[strand].hours[h], hour);
		if (hour.current)
		{
			//PPRINT("get_current_hour: found current flag at index "); Serial.println(h);
			return h;
		}
	}

	//PPRINTLN("get_current_hour: no current, returning MAX_HOURS-1");
	return MAX_HOURS-1;
}

void read_hour(uint8_t index, uint8_t strand, s_stats_hour & hour)
{
	eEE_READ(EE->statistics[strand].hours[index], hour);
}

void write_hour(uint8_t index, uint8_t strand, s_stats_hour & hour)
{
	eEE_WRITE(hour, EE->statistics[strand].hours[index]);
}

void read_day(uint8_t index, uint8_t strand, s_stats_day & day)
{
	eEE_READ(EE->statistics[strand].days[index], day);
}

void write_day(uint8_t index, uint8_t strand, s_stats_day & day)
{
	eEE_WRITE(day, EE->statistics[strand].days[index]);
}

void read_week(uint8_t index, uint8_t strand, s_stats_week & week)
{
	eEE_READ(EE->statistics[strand].weeks[index], week);
}

void write_week(uint8_t index, uint8_t strand, s_stats_week & week)
{
	eEE_WRITE(week, EE->statistics[strand].weeks[index]);
}

uint8_t get_day_index()
{
	uint8_t index = eEE_READN(EE->day_count);
	index = MIN(index, MAX_DAYS-1);
	//PPRINT("get_day_index: index="); Serial.println(index);
	return index;
}

uint8_t get_week_index()
{
	uint8_t index = eEE_READN(EE->week_count);
	index = MIN(index, MAX_WEEKS-1);
	return index;
}

static void save_one_week(uint8_t day_start_index)
{
	//boolean debug = false;
	uint8_t index = get_week_index();
	//if (debug) { PPRINT("save_one_week(day_start_index="); Serial.print(day_start_index); PPRINT("), found week index="); Serial.println(index); }

	for (uint8_t strand=0; strand<MAX_STRANDS; strand++)
	{
		s_stats_day  day;
		s_stats_week week;
		//if (debug) { PPRINT("Strand "); Serial.print(strand+1); PPRINT(": days: [");  }

		long mins_on = 0;
		for (uint8_t n=0; n<7; n++)
		{
			int day_index = (day_start_index+n)%MAX_DAYS;
			read_day(day_index, strand, day);

			//if (debug) { Serial.print(day_index); PPRINT("="); Serial.print(day.mins_on); PPRINT(","); }
			mins_on += day.mins_on;
			if (day.valid) week.valid=1;
		}

		//long hours_on = constrain((mins_on+30)/60, 0, 255);
		//if (debug) { PPRINT("], sum="); Serial.print(mins_on); PPRINT("m ("); Serial.print(hours_on); PPRINTLN("h)"); }

		// divide by 120m because of limited precision (down to 2 hours)
		week.hours2_on = constrain(( ((int)mins_on) +60)/120, 0, 255);
		//week.days_index = day_start_index;

		write_week(index, strand, week);
	}

	index = (index+1) % MAX_WEEKS;
	set_week_index(index);
}

/*
void dump_day(s_stats_day & d)
{
	PPRINT("mon="); Serial.print(d.mins_on);
	//PPRINT(", hour_index="); Serial.print(d.hour_index);
	PPRINT(",v="); Serial.print(d.valid);
}
*/

void save_one_day(uint8_t hour_start_index)
{
	boolean debug = false;
	uint8_t index = get_day_index();
	//if (debug) { PPRINT("save_one_day(hour_start_index="); Serial.print(hour_start_index); PPRINT("), found day index="); Serial.println(index); }

	for (uint8_t strand=0; strand<MAX_STRANDS; strand++)
	{
		s_stats_hour hour;
		s_stats_day day;
		day.valid = 0;
		//if (debug) { PPRINT("Strand "); Serial.print(strand+1); PPRINT(": hours: [");  }

		long mins_on = 0;
		for (uint8_t n=0; n<24; n++)
		{
			int hour_index = (hour_start_index+n)%MAX_HOURS;
			read_hour(hour_index, strand, hour);

			if (debug) { Serial.print(hour_index); PPRINT("="); Serial.print(hour.mins_on); PPRINT(","); }
			mins_on += hour.mins_on;
			if (hour.valid) day.valid=1;
		}
		//if (debug) { PPRINT("], sum="); Serial.print(mins_on); PPRINTLN("m"); }

		//day.hour_index = hour_start_index;
		day.mins_on = constrain(mins_on, 0, 3000);

		//PPRINT("save_one_day: saving to index "); dump_day(day); Serial.println();
		write_day(index, strand, day);
	}

	index++;
	set_day_index(index % MAX_DAYS);
	if (0==index%7) // written to day 7 (14, 21)? -> days 0,...,6 need to be saved
	{
		int day_start_index = index%MAX_DAYS; // save the next 7 entries

		/*
		if (0)
		{
			dump_stats_days();
			dump_stats_weeks();
			PPRINTLN("WILL SAVE OLDEST 7 DAYS TO NEW WEEK");
			PPRINT("  index="); Serial.println(index);
			PPRINT("  MAX_DAYS="); Serial.println(MAX_DAYS);
			PPRINT("  index+MAX_DAYS-7="); Serial.println(index+MAX_DAYS-7);
			PPRINT("  day_start_index="); Serial.println(day_start_index);
		}
		*/

		save_one_week(day_start_index); // e.g. [0,...,7) i.e. inkl./excl.
		//dump_stats_weeks();
		//sleep(1);
	}
}


// called once per hour
static void save_statistics()
{
	//boolean debug = true;

	uint8_t index = get_current_hour();
	//PPRINT("D. save_statistics: index="); Serial.println(index);
	index = (index+1)%MAX_HOURS;
	//PPRINT("D. save_statistics: new index="); Serial.println(index);

	for (uint8_t s=0; s<MAX_STRANDS; s++)
	{
		s_strand_info  & strand = strand_infos[s];
		s_stats_hour & curr   = current_stats[s];

		//if (debug) PPRINTLN("D. save_statistics: ***************** checking current *****************");
		if (strand.lit && strand.ts_last_change<ts_optime)
		{
			add_ontime(s, "save");
		}

		s_stats_hour hour;
		hour.current = 1;
		hour.mins_on = curr.mins_on;
		hour.valid = 1;
		write_hour(index, s, hour);

		strand.ts_last_change = ts_optime;

		// mark dirty such that new min/max will be stored upon next measure
		// and also reset teh minutes per hour counter since a new hour starts now
		curr.valid   = 0;
		curr.mins_on = 0;
	}

	set_current_hour(index);
	// data for 24 hours recorded (days 0,...,23 were saved)
	// that means that the day before yesterday is representetd twice: once with per-hour precision,
	// and once with aggregated data.hours -24,...,-47 are -1d
	if (0 == (index+1)%24)
	{
		int hour_start_index = (index+1)%MAX_HOURS;
		/*
		if (0)
		{
			dump_stats_hours();
			dump_stats_days();
			PPRINTLN("D. WILL SAVE OLDEST 24 HOURS TO NEW DAY");
			PPRINT("D.  index="); Serial.println(index);
			PPRINT("D.  hour_start_index="); Serial.println(hour_start_index);
		}
		 */
		save_one_day(hour_start_index);
		//dump_stats_days();
	}
}


static void dump_stats_hours()
{
	s_stats_hour hour;
	uint8_t last = get_current_hour();
	float totalE = 0;
	float cost_per_kWh = read_cost_per_kWh();
	int hours = 0;

	for (uint8_t back=0; back<MAX_HOURS; back++)
	{
		float subE = 0;
		uint8_t index = (last+MAX_HOURS-back) % MAX_HOURS;

		read_hour(index, 0, hour);
		if (hour.valid)
		{
			hours++;
		}
		else if (verbosity<V_VERBOSE)
		{
			continue;
		}

		PPRINT("STH:t=-"); Serial.print(back, DEC); PPRINT("h,");
		PPRINT("i="); Serial.print(index);

		for  (uint8_t strand=0; strand<num_strands; strand++)
		{
			s_strand_info & info = strand_infos[strand];
			read_hour(index, strand, hour);

			float ratio=0, E=0;
			if (hour.valid)
			{
				if (0==strand)
				{
					PPRINT(",v=1,");
				}
				//p = (1.0f*hour.mins_on)/60;
				//E = p*0.5f*(info.watts2);
				ratio = (1.0f/60.0f) * hour.mins_on;
				E = ratio*info.watts;
				subE   += E;
				totalE += E;
			}
			else
			{
				if (0==strand)
				{
					PPRINT(",v=0,");
				}
			}

			PPRINT("{n="); Serial.print(strand+1);
			PPRINT(",on=");
			Serial.print(hour.mins_on, DEC);
			PPRINT("m,r="); Serial.print(ratio);
			PPRINT(",P=");  Serial.print(E,1);
			PPRINT("}");
		}
		float cost = cost_per_kWh*subE/1000;
		PPRINT(",P="); Serial.print(subE); PPRINT(",C="); Serial.println(cost);
	}
	float cost = cost_per_kWh*totalE/1000;
	float costPerHout = hours<1 ? 0 : cost/hours;
	float ePerHour    = hours<1 ? 0 : totalE/hours;
	PPRINT("STH!h="); Serial.print(hours);  PPRINT(",P="); Serial.print(totalE); PPRINT(",P/h="); Serial.print(ePerHour); PPRINT(",C="); Serial.print(cost); PPRINT(",C/h="); Serial.println(costPerHout);
}

void dump_stats_days()
{
	// get_day_index points to next entry to be written. last days is one index lower
	uint8_t curr = get_day_index();
	s_stats_day day;
	float totalE = 0;
	float cost_per_kWh = read_cost_per_kWh();
	int days = 0;

	for (uint8_t back=1; back<=MAX_DAYS; back++)
	{
		float subE = 0;
		uint8_t index = (curr+MAX_DAYS-back) % MAX_DAYS;

		read_day(index, 0, day);
		if (day.valid)
		{
			days++;
		}
		else if (verbosity<V_VERBOSE)
		{
			continue;
		}

		PPRINT("STD:t=-"); Serial.print(STATS_PER_HOUR_DAY_COUNT+back-1, DEC); PPRINT("d,"); PPRINT("i="); Serial.print(index);

		for  (uint8_t strand=0; strand<num_strands; strand++)
		{
			s_strand_info & info  = strand_infos[strand];
			read_day(index, strand, day);

			float ratio=0, E=0;

			if (day.valid)
			{
				if (0==strand)
				{
					PPRINT(",v=1,");
					//PPRINT("hour_idx=["); Serial.print(day.hour_index); PPRINT(","); Serial.print(day.hour_index+23); PPRINT("]");
				}
				ratio = (1.0f/(24.0f*60.0f)) * day.mins_on;
				E     = ratio * 24.0f        * info.watts;
				subE   += E;
				totalE += E;
			}
			else
			{
				if (0==strand)
				{
					PPRINT(",v=0,");
				}
			}

			PPRINT("{n="); Serial.print(strand+1);
			PPRINT(",on=");
			Serial.print(day.mins_on, DEC); PPRINT("m,r=");
			Serial.print(ratio);            PPRINT(",P=");
			Serial.print(E/1000);           PPRINT("k}");
		}
		float cost = cost_per_kWh*subE/1000;
		PPRINT(",P="); Serial.print(subE/1000); PPRINT("k,C="); Serial.println(cost);
	}

	float cost = cost_per_kWh*totalE/1000;
	float costPerDay = days<1 ? 0 : cost/days;
	float ePerDay    = days<1 ? 0 : totalE/days;
	PPRINT("STD!d="); Serial.print(days);  PPRINT(",P="); Serial.print(totalE/1000); PPRINT("k,P/d="); Serial.print(ePerDay/1000); PPRINT("k,C="); Serial.print(cost); PPRINT(",C/d="); Serial.println(costPerDay);
}


void dump_stats_weeks()
{
	// get_day_index pintrs to next entry to be writte. last days is one index lower
	uint8_t curr = get_week_index();
	s_stats_week week;
	float totalE = 0;
	float cost_per_kWh = read_cost_per_kWh();
	int weeks = 0;

	for (uint8_t back=1; back<=MAX_WEEKS; back++)
	{
		float subE = 0;
		uint8_t index = (curr+MAX_WEEKS-back) % MAX_WEEKS;

		read_week(index, 0, week);
		if (week.valid)
		{
			weeks++;
		}
		else if (verbosity<V_VERBOSE)
		{
			continue;
		}

		PPRINT("STW:t=-"); Serial.print(STATS_PER_DAY_WEEK_COUNT+back-1, DEC); PPRINT("w,");
		PPRINT("i="); Serial.print(index);

		for  (uint8_t strand=0; strand<num_strands; strand++)
		{
			s_strand_info & info  = strand_infos[strand];
			read_week(index, strand, week);
			float ratio=0, E=0;

			if (week.valid)
			{
				if (0==strand)
				{
					PPRINT(",v=1,");
					//PPRINT(",didx=["); Serial.print(week.days_index); PPRINT(","); Serial.print(week.days_index+6); PPRINT("]");
				}

				//p = (2.0f*week.hours2_on)/(7*24); // *2 because of limited precision (2 hour slots)
				// compiler will optimize the float number computations
				ratio = (2.0f/(7.0f*24.0f)) * week.hours2_on; // *2 because of limited precision (2 hour slots)
				E     = ratio*(7.0f*24.0f)  * info.watts;
				subE   += E;
				totalE += E;
			}
			else
			{
				if (0==strand)
				{
					PPRINT(",v=0,");
				}
			}

			PPRINT("{n="); Serial.print(strand+1);
			PPRINT(",on=");
			Serial.print(2*week.hours2_on, DEC); PPRINT("h,r=");
			Serial.print(ratio);                 PPRINT(",P=");
			Serial.print(E/1000);                PPRINT("k}");
		}
		float cost = cost_per_kWh*subE/1000;
		PPRINT(",P="); Serial.print(subE/1000); PPRINT("k,C="); Serial.println(cost);
	}
	float cost = cost_per_kWh*totalE/1000;
	float costPerWeek = weeks<1 ? 0 : cost/weeks;
	float ePerWeek    = weeks<1 ? 0 : totalE/weeks;
	PPRINT("STW!w="); Serial.print(weeks);  PPRINT(",P="); Serial.print(totalE/1000); PPRINT("kWh,P/w="); Serial.print(ePerWeek/1000); PPRINT("k,C="); Serial.print(cost); PPRINT(",C/w="); Serial.println(costPerWeek);

}

void dump_stats()
{
	dump_stats_hours();
	dump_stats_days();
	dump_stats_weeks();
	PPRINTLN("ST!");
}
#else
static void save_statistics() {}
void dump_stats() { PPRINTLN("D. stats=n.a."); }
#endif


#ifdef WITH_STATS
void dump_current_stats()
{
	for (int n=0; n<MAX_STRANDS; n++)
	{
		s_stats_hour & c = current_stats[n];
		PPRINT("CUR:n="); Serial.print(n+1); PPRINT(",on="); Serial.println(c.mins_on);
	}
	PPRINTLN("CUR!");
}
#endif

void set_minmax_index(uint8_t index)
{
	 eEE_WRITE(index, EE->hist_count);
}


#ifdef WITH_MIN_MAX
uint8_t get_minmax_index()
{
	uint8_t index = eEE_READN(EE->hist_count);
	return index;
}


static void read_minmax_entry(uint8_t sensor, uint16_t index, s_min_max & mm)
{
	eEE_READ(EE->temp_histories[sensor].data[index], mm);
}


void save_minmax()
{
	//PPRINTLN("D.save_minmax");
	uint8_t count = get_minmax_index();
	for (uint8_t i=0; i<MAX_SENSORS; i++)
	{
		s_curr_minmax & mm = current_minmax[i];
		s_min_max save;

		save.valid = mm.valid;
		// Current min/max stored as 100*value in C, thus multiply with 0.01 to convert back.
		// EEOProm values stored as 2*value in C, thus need to multiply with 2. Total: 2*0.001 = 0.002

		int16_t min  = mm.curr_min100/50;
		int16_t diff = (mm.curr_max100-mm.curr_min100)/50;
		save.min2  = constrain(min, -128, 127);
		save.diff2 = constrain(diff,   0, 255);

		eEE_WRITE(save, EE->temp_histories[i].data[count]);

		// mark dirty to record new min/max
		mm.valid = 0;
	}

	set_minmax_index((count+1) % TEMP_HIST_LEN);
	//PPRINTLN("D./save_minmax");
}

#define HIST_UPD_NONE 0
#define HIST_UPD_1ST  1
#define HIST_UPD_MIN  2
#define HIST_UPD_MAX  3

uint8_t update_history()
{
	const boolean debug  = false;
	if (debug) { PPRINTLN("D.update_history"); dump_minmax(); }

	uint8_t change = HIST_UPD_NONE;
	// update min/max for current hour along the way
	for (uint8_t s=0; s<MAX_SENSORS; s++)
	{
		//s_strand_info_eeprom & info = strand_infos[s];
		s_strand_info & strand = strand_infos[s];

		if (debug) { PPRINT("check s="); Serial.println(s); }

		if (!strand.temp_valid)
		{
			if (debug) { PPRINT("no temp s="); Serial.println(s); }
			continue;
		}

		s_curr_minmax & mm = current_minmax[s];
		int16_t t = strand.curr_temp100;

		if (!mm.valid)
		{
			mm.curr_min100 = mm.curr_max100 = t;
			mm.valid = 1;
			change = HIST_UPD_1ST;
		}
		else if (t<mm.curr_min100)
		{
			mm.curr_min100 = t;
			change = HIST_UPD_MIN;
		}
		else if (t>mm.curr_max100)
		//else if (t>mm.min+mm.diff)
		{
			mm.curr_max100 = t;
			change = HIST_UPD_MAX;
		}
	}

	if (debug) { dump_minmax(); PPRINTLN("D./update_history"); }
	return change;
}

void dump_curr_minmax(s_curr_minmax & mm)
{
	if (mm.valid)
	{
		// ,2 means two digits after the decimal delimiter(dot)
		PPRINT("lo=");  Serial.print(mm.curr_min100, DEC);
		PPRINT(",hi="); Serial.print(mm.curr_max100, DEC);
	}
	else
	{
		PPRINT("val=0");
	}
}

void dump_minmax(s_min_max & mm)
{
	if (mm.valid)
	{
		// ,1 means one digit after the decimal delimiter(dot)
		PPRINT("lo=");  Serial.print((100/2)*(mm.min2        ),  DEC);
		PPRINT(",hi="); Serial.print((100/2)*(mm.min2+mm.diff2), DEC);
	}
	else
	{
		PPRINT("val=0");
	}
}

void dump_minmax()
{
	int interval_minutes = (MMHIST_INTERVAL*60); // 360 minutes if MMHIST_INTERVAL == 6 hours

	int   minutes  = ts_optime % interval_minutes; // 0,...,359 minutes
	//float hours    = 1.0*minutes/60;
	//PPRINT("MM:N=0,c=1,t=-"); Serial.print(hours,1); PPRINT("h,i=-1,");
	PPRINT("MM:N=0,c=1,t=0,i=-1,");

	for (uint8_t s=0; s<MAX_SENSORS; s++)
	{
		PPRINT("{s="); Serial.print(s+1); PPRINT(","); dump_curr_minmax(current_minmax[s]);
		PPRINT("}");
	}
	Serial.println();

	uint8_t hists = 0;
	uint8_t start = get_minmax_index();
	for (uint8_t back=1; back<TEMP_HIST_LEN; back++)
	{
		uint8_t index = (start+TEMP_HIST_LEN-back) % TEMP_HIST_LEN;
		s_min_max hist;

		boolean valid = false;
		for (uint8_t s=0; s<MAX_SENSORS; s++)
		{
			read_minmax_entry(s, index, hist);
			if (hist.valid) valid = true;
		}
		if (!valid)
		{
			continue;
		}

		hists++;

		//int   back_minutes = (back*interval_minutes+minutes);
		int   back_minutes = ((back-1)*interval_minutes+minutes);
		float back_hours   = (1.0f/60.0f)*back_minutes;

		PPRINT("MM:N="); Serial.print(back); PPRINT(",c=0,t=-"); Serial.print(back_hours,1); PPRINT("h,i="); Serial.print(index); PPRINT(",");

		for (uint8_t s=0; s<MAX_SENSORS; s++)
		{
			read_minmax_entry(s, index, hist);
			PPRINT("{s="); Serial.print(s+1); PPRINT(","); dump_minmax(hist);
			//PPRINT(", rand="); Serial.print(hist.random);
			PPRINT("}");
		}
		Serial.println();
	}

	int16_t avg100 = get_hb_temp100();
	PPRINT("MM!h="); Serial.print(hists); PPRINT(" hbt="); Serial.println(avg100, DEC);
}
#else
void save_minmax() {}
uint8_t update_history() { return HIST_UPD_NONE; }
void dump_minmax() { PPRINT("D.history=n.a."); }
#endif


#ifdef WITH_SAVE_UPTIME
void save_uptime()
{
//	PPRINT("save_uptime");
	eEE_WRITE(ts_optime, EE->optime_minutes);
}
#else
void save_uptime() {}
#endif


static boolean check_limits()
{
	boolean change = false;

	// Find first strand with a valid temperature that will control zombie
	// strands for which sensor is missing or failed:
	s_strand_info * valid = NULL;

	for (uint8_t s=0; s<num_strands; s++)
	{
		//s_strand_info_eeprom & strand = strand_infos[s];
		s_strand_info   & strand   = strand_infos[s];
		if (strand.temp_valid)
		{
			valid = &strand;
			break;
		}
	}

	// all strands will be powered off in the loop below (power_off=true)
	severe_error = 0;
	if (NULL==valid)
	{
		severe_error = 1;
	}

	for (uint8_t s=0; s<num_strands; s++)
	{
		s_strand_info & strand = strand_infos[s];

		// Current temperatures are stored as 100*value in C, e.g. 350 means 3.5C while the limits
		// are stored as 2*C (to save space in EEProm), thus need to convert form ram to eeprom representation
		// before comparing with limits.
		//int8_t t2 = from_ram_to_eeprom_temp(strand.curr_temp100, -128, 127);
		float t = strand.curr_temp100;

		boolean power_on=false, power_off=true;
		// status of strands with a valid temperature depends on the temperature limits
		if (strand.temp_valid)
		{
			power_on   = (t<=strand.temp_lower100 && !strand.lit && strand.sensor.bound);
			power_off  = (t >strand.temp_upper100 &&  strand.lit);
		}
		// strands without a valid value are zombie controlled by the first valid strand (if any at all)
		else if (valid)
		{
			//PPRINTLN("not valid");
			power_on  = valid->lit  && ! strand.lit && strand.sensor.bound;
			power_off = strand.lit  && ! valid->lit;
		}

		if (strand.lit && !strand.sensor.bound)
		{
			//PPRINTLN("D. strand unbnd->off");
			power_off = true;
			power_on  = false;
		}

		if (power_on)
		{
			if (verbosity>V_SILENT)
			{
				PPRINT("EV.e=pwr.on,n="); Serial.println(s+1);
			}
		}

		if (power_off)
		{
			if (verbosity>V_SILENT)
			{
				PPRINT("EV.e=pwr.off,n="); Serial.println(s+1);
			}

			#ifdef WITH_STATS
			// if strand was on until now, add difference to mins_on
			if (strand.lit)
			{
				add_ontime(s, "poff");
			}
			#endif

			digitalWrite(strand.pin, LOW);
			strand.lit = 0;
			strand.ts_last_change = ts_optime;
			strand.update = 1;
			change = true;
		}
		if (power_on)
		{
			digitalWrite(strand.pin, HIGH);
			strand.lit = 1;
			strand.ts_last_change = ts_optime;
			strand.update = 1;
			change = true;
		}
	}

	// If some strands (i.e. between num_strands and MAX_STRANDS) were unbound recently
	// upon user request, switch them off and update current stats as well.
	for (uint8_t s=num_strands; s<MAX_STRANDS; s++)
	{
		s_strand_info & info = strand_infos[s];
		if (info.lit && !info.sensor.used)
		{
			#ifdef WITH_STATS
			add_ontime(s, "unbnd");
			#endif

			digitalWrite(info.pin, LOW);
			info.lit = 0;
			info.ts_last_change = ts_optime;
			info.update = 1;
			change = true;
		}
	}

	return change;
}

void add_ontime(int s, const char * reason)
{
	s_strand_info   &strand = strand_infos[s];
	s_stats_hour  &curr   = current_stats[s];

	long last    = strand.ts_last_change;
	long delta   = ts_optime-last;
	long    _old = curr.mins_on;
	uint8_t _new = constrain(_old+delta,0,63);
	curr.mins_on = _new;

	if (verbosity>V_SILENT)
	{
		PPRINT("EV.e=ontime.upd,n="); Serial.print(s+1);
		//PPRINT(",last=");   Serial.print(last);
		//PPRINT(",now=");    Serial.print(ts_optime);
		PPRINT(",ago=");    Serial.print(delta);
		PPRINT(",oldon=");  Serial.print(_old);
		PPRINT(",newon=");  Serial.print(_new);
		PPRINT(",reas=");   Serial.println(reason);
	}
}

void show_string (PGM_P s)
{
	char c;
	while ((c = pgm_read_byte(s++)) != 0)
	{
		Serial.print(c);
	}
}

/*
 *
 	 A = auto config
 	 B = boot
 	 C = cost
 	 D = dump
 	 E = estbl. conn
 	 F = forget sensos
 	 G = rebind sensor
 	 H = help
 	 I = identify strand
 	 J = identify pin
 	 K = ackn
 	 L = lower
 	 M = num strands
 	 N = num sensors
 	 O =
	 P = power
	 Q =
	 R = set ramping delta
	 S =show sensors
	 T = test mode
	 U = upper limit
	 V = verbosity (reserved)
	 W =
	 X = rescan sensors
	 Y =
	 Z = zap+reset

 */
static void dump_help()
{
#ifdef WITH_HELP
	PPRINTLN(
		"H:E1|0 est/end conn\n"
		"H:L1=300 set lower T to 3C\n"
		"H:U2=500 set upper T to 5C\n"
		#ifdef WITH_TEMP_RAMPING
		"H:R2=200 set ramp end lower T to 2C/day\n"
		"H:Q2=-10 set ramp delta to -.1C/day\n"
		#endif
		"H:P3=90.0 set pwr=90W\n"
		"H:C0.2 cost/kWh to 0.2\n"
		"H:M3 #strnds\n"
		"H:N4 #sensrs\n"
		"H:I1 ident str 1\n"
		"H:J2 ident pin 2\n"
		"H:V1 set verbosity 0-2\n"
		"H:Kxys ackn\n"
		#ifdef WITH_STATS
		"H:DT dump uptimes\n"
		"H:DH h-stats\n"
		"H:DD d-stats\n"
		"H:DW w-stats\n"
		"H:DS strnd info\n"
		#ifdef WITH_MIN_MAX
		"H:DM temp hist\n"
		"H:DC curr minmax\n"
		#endif
		"H:DG gen.info\n"
		#endif
		"H:F1 unbnd sens 1\n"
		"H:G1 bnd sens\n"
		"H:+1 mv sens dwn\n"
		"H:-2 mv sens up\n"
		#ifdef WITH_REAL_SENSORS
		"H:X rescan sensrs\n"
		"H:S sensr info\n"
		#endif
		#ifdef WITH_TESTMODE
		"H:T test mode\n"
		#endif
		#ifdef WITH_AUTOCONFIG
		"H:A enter auto config\n"
		#endif
		"H:?/H hlp\n"
		"H:B=4711 reboot uC\n"
		"H:Z=4711 zap+reset uC\n"
		"H!"
);
#endif
}

static char to_upper(char c)
{
	if ('a'<=c && c<='z') c='A'+(c-'a');
	return c;
}

static boolean dump_strand_infos(boolean updates_only, boolean verbose)
{
	boolean update = false;
	boolean single = updates_only;

	for  (uint8_t n=0; n<MAX_SENSORS; n++)
	{
		s_strand_info & strand = strand_infos[n];
		boolean show = false;
		if (updates_only)
		{
			if (strand.update)
			{
				update = show = true;
				strand.update = false; // reset
			}
		}
		else
		{
			show = true;
		}

		if (show)
		{
			dump_strand_info(n, strand, single, verbose); // all: STR:, otherwise STR.
			Serial.println();
		}
	}

	if (!single)
	{
		PPRINTLN("STR!");
	}

	return update;
}

/*
static boolean show_updates()
{
	boolean update = false;

	for (uint8_t i=0; i<MAX_SENSORS; i++)
	{
		//s_strand_info_eeprom & s = strand_infos[i];
		s_strand_info   & s = strand_infos[i];
		if (s.update)
		{
			update   = true;
			s.update = false;
			dump_strand_info(i, s, true); //, "upd");
			Serial.println();
		}
	}

	return update;
}
*/

static void dump_strand_info(uint8_t n, s_strand_info & i, boolean single, boolean verbose)
{
#ifdef WITH_DUMPING
	long delta = ts_optime-i.ts_last_change;

	if (single)
	{
		PPRINT("STR.n=");
	}
	else
	{
		PPRINT("STR:n=");
	}

	Serial.print(n+1);
	PPRINT(",v=");     Serial.print(i.temp_valid);
	PPRINT(",lit=");   Serial.print(i.lit);
	PPRINT(",upd=");   Serial.print(i.update);

	// limits are stored as 2*value in C, e.g. 7 means 3.5C
	PPRINT(",tl=");    Serial.print(i.temp_lower100, DEC);
	PPRINT(",tu=");    Serial.print(i.temp_upper100, DEC);
	#ifdef WITH_TEMP_RAMPING
	if (verbose)
	{
		PPRINT(",tls=");   Serial.print(i.temp_lower_start100,  DEC);
		PPRINT(",trl=");   Serial.print(i.temp_ramping_limit100,    DEC);
		PPRINT(",rpd=");   Serial.print(i.ramp_per_day_temp100, DEC); // rpd = ramp per day
		PPRINT(",rsd=");   Serial.print(i.ramp_start_day,       DEC); // rsd = ramp start days
	}
	#endif
	PPRINT(",P=");     Serial.print(i.watts,      DEC);
	PPRINT(",t=");
	if (i.temp_valid)
	{
		Serial.print(i.curr_temp100, DEC);
	}
	else
	{
		PPRINT("?");
	}
	PPRINT(",err=");  Serial.print(i.errors);
	PPRINT(",ago=");  Serial.print(delta);
	PPRINT(",pin=");  Serial.print(i.pin);   
	//PPRINT(",why=");  Serial.print(why);
	if (verbose)
	{
		PPRINT(",");
		dump_sensor(i.sensor);
	}
#endif
}

void dump_general_info()
{
#ifdef WITH_DUMPING
	uint32_t uptime = ts_optime-ts_starttime;
	  PPRINT("I:");           println_version();
	  PPRINT("I:OpMins=");    Serial.println(ts_optime);
	  PPRINT("I:OpDays=");    Serial.println(curr_op_day());
	  PPRINT("I:StrtMin=");   Serial.print(ts_starttime);    //PPRINTLN("m");
	PPRINT("\nI:UpMins=");    Serial.print(uptime);          //PPRINTLN("m");
	PPRINT("\nI:MxStrtTms="); Serial.print(MAX_START_HOURS); //PPRINTLN("m");
	PPRINT("\nI:Cost=");      Serial.println((int)(100*read_cost_per_kWh()));

	PPRINT("I:MxStrnds=");    Serial.println(MAX_STRANDS);
	PPRINT("I:MxSens=");      Serial.println(MAX_SENSORS);
	#ifdef WITH_MY_SENSORS
	PPRINTLN("I:MySens=1");
	#else
	PPRINTLN("I:MySens=0");
	#endif
	PPRINT("I:#Strnds=");     Serial.println(num_strands);
	PPRINT("I:#Sens=");       Serial.println(num_sensors); // expected
	PPRINT("I:AvlSens=");     Serial.println(num_sensors_available);
	PPRINT("I:MxStHrs=");     Serial.println(MAX_HOURS);
	PPRINT("I:MxStDays=");	  Serial.println(MAX_DAYS);
	PPRINT("I:MxStWks=");     Serial.println(MAX_WEEKS);
	PPRINT("I:MMDays=");      Serial.println(MMHIST_DAY_COUNT);
	PPRINT("I:MMIntrvl=");    Serial.print(MMHIST_INTERVAL); PPRINTLN("h");
	PPRINT("I:Verb=");        Serial.println(verbosity);
	PPRINTLN("I!");
#endif
}

void compute_temps()
{
	const boolean debug = false;

    #ifdef WITH_REAL_SENSORS
	 // call sensors.requestTemperatures() to issue a global temperature request to all devices on the bus
	if (debug) PPRINTLN("D.Measuring");
	sensors.requestTemperatures(); // Send the command to get temperatures
	if (debug) PPRINTLN("D.Done");
    #endif

	for (uint8_t s=0; s<MAX_SENSORS; s++)
	{
		s_strand_info & strand = strand_infos[s];

		if (!strand.sensor.used)
		{
			if (debug) { PPRINT("W.sens="); Serial.print(s+1); PPRINT(",st=unbnd\n"); }
			if (strand.temp_valid) strand.update = 1;
			strand.temp_valid = 0;
			continue;
		}
		if (!strand.sensor.avail)
		{
			PPRINT("W.sens="); Serial.print(s+1); PPRINTLN(",st=unavl");
			if (strand.temp_valid) strand.update = 1;
			strand.temp_valid = 0;
			continue;
		}

	#ifdef WITH_REAL_SENSORS

		uint8_t * addr = strand.sensor.address;
		if (!sensors.isConversionAvailable(addr))
		{
			PPRINT("W.sens="); Serial.print(s+1); PPRINTLN(",st=noconv");
			if (strand.errors>=50)
			{
				if (strand.temp_valid) strand.update = 1;
				strand.temp_valid = 0;
				continue;
			}
			// continue with last known value (if there was one at all)
			strand.errors++;
			strand.update = true;
		}

		float celsius = sensors.getTempC(addr);
		int16_t now = to_temp100(celsius);
		int16_t old = strand.temp_valid ? strand.curr_temp100 : now;

	#else

		int former = strand.valid ? strand.curr_temp : 100.0f*(DEFAULT_LOWER-3);
		#ifdef OSX
			int current = former;
			int r = 0;
			if (random()%10000>9998)
			{
				r = 30-random()%60;
				current = 100.0f*(DEFAULT_LOWER-3) + r;
			}

		#else
			int r = random(0,1000); // thus vary in the range [0,...,10C)
			int current = former + r;
		#endif
		//PPRINT("D. Generated random "); Serial.println(r);

	#endif

		int16_t avg = (now+old)/2;
		if (abs(old-avg)>0.01 || !strand.temp_valid || strand.errors>0)
		{
			strand.update = 1;
		}

		strand.curr_temp100 = avg;
		strand.temp_valid   = 1;
		strand.errors       = 0;

		// when time lapse is enabled, do not print actual measurements (too many)
		#ifdef WITH_CURR_MEASURE
		PPRINT("MEAS:n="); Serial.print(s+1);
		PPRINT(",now=");   Serial.print  (now, DEC);
		PPRINT(",old=");   Serial.print  (old, DEC);
		PPRINT(",avg=");   Serial.println(avg, DEC);
		#endif
	}

	#ifdef WITH_CURR_MEASURE
	PPRINTLN("MEAS!");
	#endif
}


#ifdef  WITH_TIME_LAPSE
	// normal
	//  #define MILLIS_PER_UPTIME (60*1000UL)

	// 2 x faster. will simulate one minute in 30 seconds
	// #define MILLIS_PER_UPTIME   (30*1000UL)

	// 10 x faster. will simulate one minute in 6 seconds
	//#define MILLIS_PER_UPTIME   6000UL

	// 60 x faster. will simulate one minute in a second, one hour in one minute, one day in 24 minutes, 1 week in approx. 2.8 hours
	 //#define MILLIS_PER_UPTIME 1000UL

	// 120 x faster. will simulate two minutes in a second, one hour in 30s, one day in 12 minutes, 1 week in approx. 1.4 hours
 	// #define MILLIS_PER_UPTIME 500UL

	// 600  x faster. will simulate one minute in 1/10s, one hour in 6 seconds,6 hours in 1 minute
	// #define MILLIS_PER_UPTIME 100UL

	// 1000 x faster. will simulate one week in approx. 10 minutes
	// #define MILLIS_PER_UPTIME 60UL

	// 2000 x faster. will simulate one week in approx. 5 minutes	
	// #define MILLIS_PER_UPTIME 30UL

	// 6000 x faster. will simulate one minute in 1/100s, one hour in 0.6 seconds, 6 hours in <1 minute
	 #define MILLIS_PER_UPTIME  10UL
#else
	// default: calculate times on a per minute base˘
	// 60s*1000ms = 1 minute
	#define MILLIS_PER_UPTIME (60*1000UL)
#endif


#ifdef WITH_TIMEWARP
static int ticks = 0;
#endif

uint32_t mins_last = 0;

static void update_uptime()
{
  #ifdef WITH_TIMEWARP
  
	//fprintf(stderr, "timewarp, return %i\n", ts_optime);
	ticks=ticks+1;
	if (1==ticks)
		return;
	ts_optime = ts_last+1;
	ticks = 0;
        return;
  
  #endif

	uint32_t mins = millis()/MILLIS_PER_UPTIME;
	//printf("update_uptime: mins: %i, mins_last: %i\n", mins, mins_last);
	// ignore millis overrun, upon next invocation things will be right
	if (mins_last<mins)
	{
		ts_optime += mins-mins_last;
		//printf("ts_optime:=%i\n", ts_optime);
	}
	mins_last=mins;
	return;
}


static void blink_pin(uint8_t pin, int8_t strand)
{
	for (uint8_t i=0; i<4; i++)
	{
		digitalWrite(pin, i%2);
		if (strand>-1) print_lit(strand, i%2);
		delay(200);
	}
	digitalWrite(pin, LOW);
}


static void blink_strand(uint8_t i)
{
	s_strand_info &s = strand_infos[i];
	blink_pin(s.pin, i);
	digitalWrite(s.pin, s.lit);
	print_lit(i, s.lit);
}

// idicate that a conenction was established or ends
void indicate_conn(boolean connected)
{
  flash_status_led(connected ? 2 : 3);
}


#ifdef WITH_SERIAL
static void check_serial()
{
	if (input_complete)
	{
		//if (verbosity>=V_VERBOSE) { PPRINT("CMD. "); Serial.println((const char*)input_buffer); }
		#ifdef OSX
		fprintf(stderr, "OSX: CMD.'%s'\n", input_buffer);
		#endif
		
		boolean valid = true;

		char first = to_upper(input_buffer[0]);
		char c = input_buffer[1];
		uint8_t  n = c-'0'; // '1' -> 1

		if ('E'==first)
		{
			indicate_conn('1'==c);
		}
		else if ('C'==first)
		{
			char * str = &input_buffer[1];
			uint16_t cost100 = atoi(str);
			PPRINT("SET.cost="); Serial.println(cost100, DEC);
			save_cost_per_kWh100(cost100);
		}
		else if ('M'==first)
		{
			if ((valid = (1<=n && n<=MAX_SENSORS)))
			{
				PPRINT("SET.nstr="); Serial.println(n);
				save_number_of_strands(n);
			}
		}
		else if ('N'==first)
		{
			if ((valid = (1<=n && n<=MAX_SENSORS)))
			{
				PPRINT("SET.nsens="); Serial.println(n);
				save_number_of_sensors_expected(n);
			}
		}
		#ifdef WITH_TEMP_RAMPING
		else if (('L'==first || 'U'==first || 'P'==first || 'Q'==first || 'R'==first)  && 0!=input_buffer[1])
		#else
		else if (('L'==first || 'U'==first || 'P'==first)  && 0!=input_buffer[1])
		#endif
		{
			char strand = input_buffer[1];
			char delim  = input_buffer[2];
			valid = false;
			if (delim=='=' && '1'<=strand && strand<='4')
			{
				uint8_t num = strand-'1'; // '1' -> 0
				if (num<MAX_STRANDS)
				{
					valid = true;
					int16_t i = atoi(&(input_buffer[3]));

					if ('L'==first)
					{
						i = constrain(i, MIN_TEMP*100, MAX_TEMP*100);
						PPRINT("SET.s="); Serial.print(num+1); PPRINT(",tls="); Serial.println(i, DEC);
						strand_infos[num].temp_lower100       = i;
						#ifdef WITH_TEMP_RAMPING
						strand_infos[num].temp_lower_start100 = i;
						strand_infos[num].ramp_start_day      = curr_op_day();
						#endif
					}
					else if ('U'==first)
					{
						i = constrain(i, MIN_TEMP*100, MAX_TEMP*100);
						PPRINT("SET.s="); Serial.print(num+1); PPRINT(",tus="); Serial.println(i, DEC);
						strand_infos[num].temp_upper100 = i;
					}
					else if ('P'==first)
					{
						i = constrain(i, 0, 250);
						PPRINT("SET.s="); Serial.print(num+1); PPRINT(",P="); Serial.println(i, DEC);
						strand_infos[num].watts = i;
					}
					#ifdef WITH_TEMP_RAMPING
					else if ('R'==first)
					{
						i = constrain(i, MIN_TEMP*100, MAX_TEMP*100);
						PPRINT("SET.s="); Serial.print(num+1); PPRINT(",trl="); Serial.println(i, DEC);
						strand_infos[num].temp_ramping_limit100 = i;
						strand_infos[num].ramp_start_day = curr_op_day();
					}
					else if ('Q'==first)
					{
						i = constrain(i, -128, 127);
						PPRINT("SET.s="); Serial.print(num+1); PPRINT(",rpd="); Serial.println(i, DEC);
						strand_infos[num].ramp_per_day_temp100 = i;
						strand_infos[num].ramp_start_day = curr_op_day();
					}
					#endif
					else
					{
						valid = false;
					}
					if (valid)
					{
						save_strand_infos();
						delay(200);
						dump_strand_info(num, strand_infos[num], true, true); //, "set");
						// dump twice to make sure android app won't miss it
//						Serial.println();
//						delay(200);
//						dump_strand_info(num, strand_infos[num], true, true); //, "set");
						Serial.println();
					}
				}
			}
		}
		else if ('I'==first)
		{
			if ((valid = (1<=n && n<=MAX_STRANDS)))
			{
				blink_strand(n-1);
			}
		}
		else if ('J'==first)
		{
			if ((valid = (2<=n && n<=13)))
			{
				blink_pin(n, -1); // strand unknown
			}
		}
		else if ('V'==first)
		{
			// commented out first part of condition to avoid compiler
			//  warning: comparison is always true due to limited range of data type
			valid = /* (V_NORMAL<=n) && */ (n<=V_DEBUG); // avoid
			if (valid)
			{
				verbosity = (e_verbosity)n;
			}
		}
#ifdef WITH_STATS
		else if ('D'==first)
		{
			char sec = to_upper(input_buffer[1]);
			if ('T'==sec)
			{
				dump_starttime();
			}
			else if ('H'==sec)
			{
				dump_stats_hours();
			}
			else if ('D'==sec)
			{
				dump_stats_days();
			}
			else if ('W'==sec)
			{
				dump_stats_weeks();
			}
			else if ('S'==sec)
			{
				// show complete information, not only updated strand and include ramping info as well
				dump_strand_infos(false, true);
			}
			#ifdef WITH_MIN_MAX
			else if ('M'==sec)
			{
				dump_minmax();
			}
			#endif
			else if ('G'==sec)
			{
				dump_general_info();
			}
			else if ('C'==sec)
			{
				dump_current_stats();
			}
			else if (0==sec)
			{
				dump_general_info();
				dump_starttime();
				dump_strand_infos(false,true);
				dump_stats();
				dump_current_stats();
				#ifdef WITH_MIN_MAX
				dump_minmax();
				#endif
			}
			else
			{
				valid = false;
			}
		}
#endif
#ifdef WITH_REAL_SENSORS
		else if ('S'==first)
		{
			dump_sensors();
		}
#endif
		else if ('F'==first)
		{
			if ((valid = (1<=n && n<=MAX_SENSORS)))
			{
				#ifdef OSX
				fprintf(stderr, "OSX: UNBINDING SENSOR %i\n", n);
				#endif
				PPRINT("BND.ac=1,n="); Serial.println(n);
				sensor_unbind(n-1);
				// no need to dump strand sensor_unbind() sets update flags on modifies strands
			}
		}
		else if ('G'==first)
		{
			if ((valid = (1<=n && n<=MAX_SENSORS)))
			{
				#ifdef OSX
				fprintf(stderr, "OSX: BINDING SENSOR %i\n", n);
				#endif
				if ((valid = sensor_bind(n-1)))
				{
					PPRINT("BND.ac=0,n="); Serial.println(n);
				}
				// no need to dump strand sensor_unbind() sets update flags on modifies strands
			}
		}
		else if ('K'==first)
		{
			PPRINT("ACK."); Serial.println(input_buffer+1);
		}
		else if ('-'==first)
		{
			if ((valid = (2<=n && n<=MAX_SENSORS)))
			{
				PPRINT("MVUP.n="); Serial.println(n);
				sensor_move(n-1, -1);
				// no need to dump strand save_sensors() sets update flags on modifies strands
				save_sensors();
			}
		}
		else if ('+'==first)
		{
			if ((valid = (1<=n && n<MAX_SENSORS)))
			{
				PPRINT("MVDWN.n="); Serial.println(n);
				sensor_move(n-1, +1);
				// no need to dump strand save_sensors() sets update flags on modifies strands
				save_sensors();
			}
		}
		else if ('X'==first)
		{
			sensors_detect();
		}
#ifdef WITH_TESTMODE
		else if ('T'==first)
		{
			test_mode();
		}
#endif
#ifdef WITH_AUTOCONFIG
		else if ('A'==first)
		{
			auto_config();
		}
#endif
		else if ('?'==first || 'H'==first)
		{
			dump_help();
		}
		else if (!strncmp(input_buffer, "B=4711", 6))
		{
			reboot();
		}
		else if (!strncmp(input_buffer, "Z=4711", 6))
		{
			reset();
		}
		else if (0==first)
		{
			// empty
		}
		else
		{
			valid = false;
		}

		if (!valid)
		{
			PPRINT("E.CMD["); Serial.print(input_buffer); PPRINTLN("]");
		}
		else
		{
			if (verbosity>=V_VERBOSE) { PPRINT("CMD!"); Serial.println(input_buffer);  }
		}

		inputClear();
	}
}

void inputClear()
{
   // clear the string:
   *(inputPos=input_buffer) = 0;
   input_complete = false;
   //fprintf(stderr, "inputClear: cleared: '%s'\n", input_buffer);
}


void serialEvent()
{
  //PPRINT("serialEvent\n");
	int a=0;
  while (Serial.available())
  {
	a=1;
	//fprintf(stderr, "serialEvent: serial available\n");
	char c = (char)Serial.read();
	//fprintf(stderr, "serialEvent: read: %i (%c)\n", c, c);

         if (0x1b==c)
         {
             reboot();
         }
         
	if (c=='\n' || c=='\r')
	{
	  *inputPos = 0; // strip CR at end for convenience
	  input_complete = true;
	  //fprintf(stderr, "complete\n");
	  return;
	}
	else if (inputPos<inputEnd)
	{
	  *(inputPos++) = c;
	  if (inputPos<inputEnd)
	  {
		  //fprintf(stderr, "serialEvent: not full: '%s'\n", input_buffer);
		  *inputPos = 0;
	  }
	}
  }
  //if (a) fprintf(stderr, "serialEvent: leaving\n");
}
#else
void check_serial() {}
#endif


static void dump_sensor_address(uint8_t  * addr)
{
	for (uint8_t i=0; i<8; i++)
	{
		if (i>0) Serial.print(';');
		if (*(addr+i)<0x10) Serial.print('0');
		Serial.print(*(addr+i), 16);
	}
}


static void dump_sensor(s_sensor_info & sensor)
{
	uint8_t * addr = &(sensor.address[0]);
	PPRINT("@=");    dump_sensor_address(addr);
	PPRINT(",usd=");  Serial.print(sensor.used);
	PPRINT(",avl="); Serial.print(sensor.avail);
	PPRINT(",bnd="); Serial.print(sensor.bound);
}


static void dump_sensors()
{
	//PPRINT("sensor_infos:\n");
	for (uint8_t i=0; i<MAX_SENSORS; i++)
	{
		PPRINT("SEN:n="); Serial.print(i+1); PPRINT(",");
		dump_sensor(strand_infos[i].sensor);
		Serial.println();
	}
	PPRINTLN("SEN!"); // indicates end of sensor info
}


static uint8_t sensors_count_available()
{
	uint8_t n=0;
	for (uint8_t s=0; s<MAX_SENSORS; s++)
	{
		s_sensor_info & i = strand_infos[s].sensor;
		if (i.used && i.avail) n++;
	}
	return n;
}

void sensors_swap(s_sensor_info & a, s_sensor_info & b)
{
	s_sensor_info t;
	memcpy(&t, &a, sizeof(a));
	memcpy(&a, &b, sizeof(a));
	memcpy(&b, &t, sizeof(a));
}

#ifdef WITH_REAL_SENSORS
int sensor_compare(s_sensor_info & a, s_sensor_info & b)
{
	if (a.bound && !b.bound)
	{
		// same for bound items
		return -1;
	}
	if (!a.bound && b.bound)
	{
		return 1;
	}

	if (a.used && !b.used)
	{
		// move used items to the beginning of the array
		return 1;
	}
	if (!a.used && b.used)
	{
		return -1;
	}

	return sensor_compare_addr(a,b);
}


int sensor_compare_addr(s_sensor_info & a, s_sensor_info & b) 
{
	return memcmp(a.address, b.address, sizeof(a.address));
}


void sensors_sort()
{
	boolean done=false;
	while (!done)
	{
		done = true;
		for (uint8_t i=0; i<MAX_SENSORS-1; i++)
		{
			if (sensor_compare(strand_infos[i].sensor, strand_infos[i+1].sensor)>0)
			{
				sensors_swap(strand_infos[i].sensor, strand_infos[i+1].sensor);
				done = false; // continue sorting
			}
		}
	}
}


static boolean sensor_replace(s_sensor_info & sensor)
{
	for (uint8_t s=0; s<MAX_SENSORS; s++)
	{
		s_sensor_info & info = strand_infos[s].sensor;
		if (info.used && 0==sensor_compare_addr(info, sensor))
		{
			info.avail = 1;
			return false; // sensor not overwritten
		}
	}
	for (uint8_t s=0; s<MAX_SENSORS; s++)
	{
		s_sensor_info & info = strand_infos[s].sensor;
		if (!info.avail)
		{
			info = sensor;
			info.avail = 1;
			return true; // sensor overwritten
		}
	}
	return false; // sensor not overwritten
}


static e_senor_add_result sensor_add(s_sensor_info & sensor, int8_t & at)
{
	int8_t first_free = -1;

	for (uint8_t s=0; s<MAX_SENSORS; s++)
	{
		s_sensor_info & info = strand_infos[s].sensor;

		if (info.used && 0==sensor_compare_addr(info, sensor))
		{
			info.avail = 1;
			at = s;
			return SENSOR_KNOWN;
		}
		if (!info.used && -1==first_free)
		{
			first_free = s;
		}
	}

	if (first_free>-1)
	{
		s_sensor_info & info = strand_infos[first_free].sensor;
		info = sensor;
		// by default assume that this sensor is bound to its strand,
		// i.e. its temperature will control the strand's lit/unlit state.
		// however do not assume this for the last one
		info.used = info.avail = 1;
		// by default assume that this sensor is bound to its strand,
		// i.e. its temperature will control the strand's lit/unlit state.
		// however do not assume this for the last one
		info.bound = first_free<4;
		at = first_free;
		return SENSOR_ADDED; // sensor added now thus known
	}

	at = -1;
	return SENSOR_IGNORED; // sensor remains unknown
}
#endif

boolean sensor_addr_defined(DeviceAddress addr)
{
	for (uint8_t i=0; i<8; i++)
	{
		if (0!=addr[i]) return true;
	}
	return false;
}

static boolean sensor_bind(uint8_t i)
{
	s_strand_info   &target   = strand_infos[i];
	s_sensor_info &sensor = target.sensor;
	DeviceAddress &addr   = sensor.address;

	if (!sensor_addr_defined(addr))
	{
		PPRINT("E.NOSENS["); Serial.print(i+1); PPRINTLN("]");
		return false;
	}

	sensor.bound = 1;
	return true;
}

static void sensor_unbind(uint8_t i)
{
	// Find last strand with no sensor assigned after the strand to be unbound .
	// Thus move unbound sensors to the end such that they might measure 
	// temperature independently.
	int8_t last = -1;
	for (int8_t j=MAX_SENSORS-1; j>i; j--)
	{
		if (j!=i)
		{
			s_strand_info & strand = strand_infos[j];
			s_sensor_info & sensor = strand.sensor;
			DeviceAddress & addr   = sensor.address;
			if (!sensor_addr_defined(addr))
			{
				last = j;
				break;
			}
		}
	}

	s_strand_info & source = strand_infos[i];

	// If a sensor-less strand was found, assign the sensor which is about to be freed.
	if (-1<last && last!=i)
	{
		s_strand_info & target = strand_infos[last];
		s_sensor_info & sensor = target.sensor;
		DeviceAddress & addr   = sensor.address;

		if (!sensor_addr_defined(addr))
		{
			target.sensor = source.sensor;
			// however do not bind the new strand to this sensor
			target.sensor.bound = 0;
			target.update = true;
		}

		#ifdef OSX
		fprintf(stderr, "OSX: Deleting sensor address of strand %i\n", (i+1));
		#endif
		BZERO(source.sensor);
	}

	// regardless of whether the address was nulled above, make sure to reset the 'bound' flag on this strand
	source.sensor.bound = 0;
	source.update       = true;

//	#ifdef OSX
//	fprintf(stderr, "OSX: Powering of pin %i\n", strand_infos[i].pin);
//	#endif

	digitalWrite(source.pin, LOW);
	source.lit = 0;
	save_sensors();
}

static void sensor_move(uint8_t index, int8_t direct)
{  
  s_strand_info & a = strand_infos[index];
  s_strand_info & b = strand_infos[index+direct];
  sensors_swap(a.sensor, b.sensor);
  a.update = true;
  b.update = true;
}

// must assert a deterministic order of sensors to make sure its predictable which sensor will control which strand.
// therefore sensor addresses are stored in eeprom and new addresses added only if old one are not visible any more.
static void sensors_detect()
{
	const boolean debug = false;
	dump_sensors();

#ifdef WITH_REAL_SENSORS
	sensors.begin();

	uint8_t found = sensors.getDeviceCount();
	if (debug) { PPRINT("sensors found: "); Serial.println(found); }

	s_sensor_info   new_sensor;
	uint8_t       * new_addr = &(new_sensor.address[0]);

	uint8_t added = 0;
	uint8_t total = 0;
	if (found>0)
	{
		for (uint8_t i=0; i<found; i++)
		{
			sensors.getAddress(new_addr, i);
			if (debug) { PPRINT("try to add: "); dump_sensor_address(new_addr); PPRINT("\n"); }

			int8_t at = -1;
			e_senor_add_result res = sensor_add(new_sensor, at);
			PPRINT("SADD:i="); Serial.print(i+1); 
			PPRINT(",n=");     Serial.print(at+1); 
			PPRINT(",@="); dump_sensor_address(new_addr);

			switch (res)
			{
			case SENSOR_ADDED:
				PPRINTLN(",a=NEW");
				added++;
				if (!eeprom_reset) save_sensor(i); // save all after sorting if eeprom was reset
				break;
			case SENSOR_KNOWN:
				PPRINTLN(",a=KNWN");
				total++;
				break;
			case SENSOR_IGNORED:
				PPRINTLN(",a=IGN");
				break;
			}
		}
	}
	PPRINTLN("SADD!");

	// Check it not all sensors were added. This might hapen if a sensor was replaced.
	// Try to overwrite unavailable sensor addresses with the new one.
	uint8_t replaced = 0;
	if (total<found && found<=MAX_SENSORS)
	{
		for (uint8_t i=0; i<found; i++)
		{
			sensors.getAddress(new_addr, i);
			PPRINT("SREP: n="); Serial.print(i+1); PPRINT(","); dump_sensor_address(new_addr);
			if (sensor_replace(new_sensor))
			{
				if (!eeprom_reset) save_sensor(i); // save all after sorting if eeprom was reset
				replaced++;
				PPRINTLN(",a=REPLD");
			}
			else
			{
				PPRINTLN(",a=KEPT");
			}
		}
	}
	PPRINTLN("SREP!");

	PPRINT("SDET:fnd="); Serial.print(found, 10);
	PPRINT(",add=");        Serial.print(added, 10);
	PPRINT(",totl=");        Serial.print(total, 10);
	PPRINT(",repl=");     Serial.println(replaced);

	// if the eeprom was reset initially, sort the sensors to assert same oder even after a reset
	if (eeprom_reset)
	{
		// Do NOT sort sensors by address if they have been hardcoded.
		#ifndef WITH_MY_SENSORS
		sensors_sort();
		#endif
		// Write cycles should not be too critical here, since changes are not very likely
		// (actually a result of broken sensors, cable problems etc.)
		// Moreover eEEPROM will not overwrite cells that already have the target value.
		save_sensors();
	}
#else

	for (uint8_t i=0; i<MAX_SENSORS; i++)
	{
		 s_sensor_info & info = strand_infos[i].sensor;
		 info.used = info.avail = 1;
		 for (int a=1; a<=8; a++) info.address[a-1] = a<<4|a;
	}

#endif

	num_sensors_available = sensors_count_available();
	dump_sensors();
	PPRINTLN("SDET!");
}


static void print_lit(uint8_t n, boolean lit)
{
	PPRINT("F:n="); Serial.print(n+1); PPRINT(",lit="); Serial.println(lit);
} 

static void print_hb(uint8_t status, uint8_t cond, int16_t temp) //, byte err, byte cmpl, byte none)
{
	if (verbosity>V_SILENT)
	{
		PPRINT("HB.l=");  Serial.print(status);
		PPRINT(",c=");    Serial.print(cond);
		PPRINT(",t=");    Serial.println(temp);
	}
}

static void flash_status_led(int8_t n)
{
	for (uint8_t i=0; i<n; i++)
	{
		set_status_leds(HIGH);
		delay(50);

		set_status_leds(LOW);
		delay(100);
	}
}

#ifdef WITH_INDICATE_STARTUP
static void indicate_startup()
{
	#ifdef OSX 
	#define multiply 1
	#else
	#define multiply 1
	#endif

	// that's the situation after a reset: all pins off. send to app
	for (uint8_t s=0; s<MAX_STRANDS; s++)
	{
		 print_lit(s, false);
	}

	// light up all strands with a short delay inbetween
	for (uint8_t s=0; s<MAX_STRANDS; s++)
	{
		int pin = strand_infos[s].pin;
		pinMode(pin, OUTPUT);
		digitalWrite(pin, HIGH);
		print_lit(s, true);
		delay(multiply*250);
	}

	if (num_sensors_available>=num_sensors)
    {
      flash_status_led(num_sensors_available);
    }

	for (uint8_t s=0; s<MAX_STRANDS; s++)
	{
		int pin = strand_infos[s].pin;
		digitalWrite(pin, LOW);
		print_lit(s, false);
		delay(multiply*100);
	}
	PPRINTLN("F!");
}
#else
static void indicate_startup()
{
	for (uint8_t s=0; s<MAX_STRANDS; s++)
	{
		int pin = strand_infos[s].pin;
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
	}
}
#endif


boolean sensors_complete()
{
	return num_sensors_available>=num_sensors;
}


boolean sensors_none()
{
	return num_sensors_available<1;
}


int16_t get_hb_temp100()
{
	// no temp? return 50C -> make heartbeat flash really fast
	float t = 50;
	boolean ok = false;

	// if there is an unbound sensor (i.e. the last one),
	// use its temperature value
	if (num_sensors_available>num_strands)
	{
		s_strand_info & strand = strand_infos[num_sensors_available-1];
		if (strand.temp_valid)
		{
			t = strand.curr_temp100;
			ok = true;
		}
	}

	if (!ok)
	{
		// otherwise compute average of all available sensors
		byte    count   =  0;
		int16_t sum     =  0;
		//int16_t coldest = 100*50; // =50C

		for (byte i=0; i<num_sensors_available; i++)
		{
			//s_strand_info_eeprom &s = strand_infos[i];
			s_strand_info & strand = strand_infos[num_sensors_available-1];
			if (strand.temp_valid)
			{
				sum += strand.curr_temp100;
				count++;
				//if (strand.curr_temp100<coldest) coldest=strand.curr_temp100;
			}
		}

		if (count>0)
		{
			t = sum/count;
		}
	}

	return t;
}

#ifdef WITH_HEARTBEAT
static struct
{
	unsigned long last_millis;
	unsigned long last_warn;
	unsigned long sequence_start; // ms
	unsigned long sequence_end; // ms
	unsigned int  light_up_len;
	unsigned int  dim_down_len;
	byte          last_status;
	int16_t       last_temp;
} hb =
{
		last_millis:     -1,
        last_warn:        0,
		sequence_start:   0,
		sequence_end:   100,
		light_up_len:    10,
		dim_down_len:    20,
		last_status:      0,
		last_temp:        0
};
#endif

void show_status(boolean working)
{
#ifdef WITH_HEARTBEAT
	unsigned long now = millis();

	if (now==hb.last_millis)
	{
		return;
	}
	hb.last_millis = now;

	byte new_status = hb.last_status;
	byte cond = 0;
	byte complete = sensors_complete();
	byte error    = severe_error;
	byte none     = sensors_none();

	if (complete && !error)
	{
		cond = 1;
		int value = 0;
//		printf("-----\n");
//		printf("D.hb.now: %li\n", now);
//		printf("D.hb.sequence_end: %li\n", hb.sequence_end);

		if (now>hb.sequence_end)
		{
			// compute new value for whole range when sequence exceeded
			hb.last_temp = get_hb_temp100()/100;
			int16_t temp = constrain(hb.last_temp, -30, 30);

			// for temp=-30C,...,30C yield f=60,...,0
			hb.sequence_start = now;

			// len: delay value: for -30C,...,30C yield 300+120=420,...,300-120=80 (beat faster when hotter)
			int16_t len = 300-4*temp;
			hb.light_up_len = 2*len;
			hb.dim_down_len = 4*len;

			// from temp=-30,...,30 yield 60,...,0 (shorter sequence when hotter)
			int16_t f = (30-temp);
			//hb.sequence_end = now+4*len+2.5*f*f;
			hb.sequence_end = now+4*len+(5*f*f)/2;
		}
		else
		{
 			//printf("D.hb.sequence_start: %li\n", hb.sequence_start);
			long delta = now-hb.sequence_start;
 			//printf("D.delta: %li\n", delta);
			if (delta<hb.light_up_len)
			{
                // heartbeat idicator starting to face => set status to 1 (inverse logic)
				value = constrain(250*delta/hb.light_up_len, 0, 250);
  			    new_status = value>180;
			}
			else if (delta<(hb.light_up_len+hb.dim_down_len))
			{
				value = 250-constrain(250*(delta-hb.light_up_len)/hb.dim_down_len, 0, 250);
				new_status = value>180;
			}
		}

		set_status_leds_analog(5+value);
	}
	else if (none || error)
	{
		cond = 3;
		new_status = (now%200)<100 ? LOW : HIGH;
		set_status_leds(new_status);
	}
	else // some sensors missing but no severe error
	{
		cond = 2;
		unsigned long seq = now%4000; // 0,...,6999

		unsigned int missing = num_sensors-num_sensors_available;
		int value = 255;
		if (seq<511*missing)
		{
			new_status = 1;
			// will yield "missing" times a sequence 0,...,255
			//analogWrite(PIN_STATUS_LED, (seq%512)/2);
			int value = (seq%511); // 0,..,510
			value = value-255;  // -255,...,255
			value = abs(value); // 255,...,0,...255
		}
		set_status_leds_analog(value);
	}

	if (new_status != hb.last_status)
	{
		print_hb(new_status, cond, hb.last_temp); //, error, complete, none);
		hb.last_status = new_status;
	}

	if (cond!=1 && millis()-hb.last_warn>5000)
	{
	  PPRINT("W.cnd=");  Serial.print(cond);
//	  PPRINT(",compl="); Serial.print(complete);
//	  PPRINT(",none=");  Serial.print(none);
//	  PPRINT(",err=");   Serial.print(error);
	  PPRINT(",exp=");   Serial.print(num_sensors);
	  PPRINT(",avl=");   Serial.println(num_sensors_available);
	  hb.last_warn=millis();
	}
#else
  //PPRINTLN("D. No LED ind.");
#endif
}

void dump_starttime()
{
	uint16_t start_hour;
	uint16_t curr_hour = ts_optime/60;

	uint8_t idx = eEE_READN(EE->start_hours.idx);
	idx = constrain(idx, 0, MAX_START_HOURS-1);

	// start reading one after the current index (this is the oldest entry)
	// and move on to younger ones with every iteration.
	uint8_t total = 0;
	for (uint8_t n=1; n<=MAX_START_HOURS; n++)
	{
		uint8_t i = (idx+n)%MAX_START_HOURS;
		start_hour = eEE_READN(EE->start_hours.hours[i]);
		if (start_hour>0)
		{
			total++;
			PPRINT("T:n="); Serial.print(total); PPRINT(",i="); Serial.print(i); PPRINT(",t="); Serial.print(start_hour); PPRINT("h,ago="); Serial.print(curr_hour-start_hour); PPRINTLN("h");
		}
	}
	PPRINT("T:n="); Serial.print(total+1); PPRINT(",i=-1,t="); Serial.print(ts_starttime); PPRINT("m,ago="); Serial.print(ts_optime-ts_starttime); PPRINTLN("m");
	PPRINTLN("T!");
}

void setup_starttime()
{	
	ts_starttime = ts_optime;
	uint8_t idx = eEE_READN(EE->start_hours.idx);
	idx = constrain(idx, 0, MAX_START_HOURS-1);

	uint16_t hour_curr = (ts_optime+30)/60;
	uint16_t hour_last = eEE_READN(EE->start_hours.hours[idx]);
	if (hour_curr>hour_last)
	{
		idx = (idx+1)%MAX_START_HOURS;
		eEE_WRITE(hour_curr, EE->start_hours.hours[idx]);
		eEE_WRITE(idx, EE->start_hours.idx);
	}
}

static void mydelay(long ms)
{
  uint32_t now, end = millis()+ms;
  do
  {
    now   = millis();

    #ifdef WITH_PING
    uint32_t delta = now-millis_ping;
    if (delta>1000)
    {
    	if (verbosity>V_SILENT)
    	{
    		PPRINT("P.s="); Serial.println(0.001f*now);
    	}
    	millis_ping = now;
    }
    #endif

    delay(1);
    serialEvent();
    check_serial();
    show_status(true);
  }
  while (now<end);
}

static void println_version()
{
	PPRINT("Frotect=v0.8."); Serial.print(MAGIC, HEX);
	PPRINT(",d="); PPRINT(__DATE__);
	Serial.println();
}

static void set_status_leds(uint8_t status)
{
	digitalWrite(PIN_STATUS_LED,  status);
	digitalWrite(PIN_ONBOARD_LED, status);

}

static void set_status_leds_analog(uint8_t value)
{
	analogWrite(PIN_STATUS_LED,  value);
	analogWrite(PIN_ONBOARD_LED, value);
}

#ifdef WITH_TEMP_RAMPING
void update_temp_ramping()
{
	const boolean debug = false;
	if (debug) PPRINTLN("D.update_temp_ramping");

	boolean change = false;

	// current day
	uint16_t day_now = curr_op_day();
	if (debug) { PPRINT("D.day_now="); Serial.println(day_now); }

	for (uint8_t i=0; i<MAX_STRANDS; i++)
	{
		s_strand_info & s = strand_infos[i];

		// day when ramping is supposed to start
		uint16_t day_start = s.ramp_start_day;
		int8_t   ramp100   = s.ramp_per_day_temp100;
		if (debug) { PPRINT("D.n="); Serial.print(i); PPRINT(",day_start="); Serial.println(day_start); }

		// 0==ramp100 means ramping was disabled for this strand
		if (0==ramp100) continue;
		if (day_start>=day_now)

		if (day_start<day_now)
		{
			uint16_t days    = day_now-day_start;

			if (debug) { PPRINT("D.n="); Serial.print(i); PPRINT(",days="); Serial.print(days); PPRINT(",ramp100="); Serial.println(ramp100); }

			int16_t prev_lower100 = s.temp_lower100;
			int16_t prev_upper100 = s.temp_upper100;
			int16_t prev_diff     = prev_upper100-prev_lower100;
			if (debug) { PPRINT("D.prev_lower100="); Serial.print(prev_lower100); PPRINT(",prev_upper100="); Serial.print(prev_upper100); PPRINT(", prev_diff="); Serial.println(prev_diff); }

			int16_t next_lower100 = s.temp_lower_start100+days*ramp100;

			// check for lower limit if ramping down
			if (ramp100<0 && next_lower100<s.temp_ramping_limit100)
			{
				next_lower100 = s.temp_ramping_limit100;
				if (debug) { PPRINT("D.****** lower ramping limit reached: "); Serial.println(next_lower100); }
			}
			// otherwise (ramping up), check if upper limit was exceeded
			else if (ramp100>0 && next_lower100>s.temp_ramping_limit100)
			{
				next_lower100 = s.temp_ramping_limit100;
				if (debug) { PPRINT("D.****** upper ramping limit reached: "); Serial.println(next_lower100); }
			}

			int16_t next_upper100 = next_lower100+prev_diff;

			if (prev_lower100!=next_lower100)
			{
				s.temp_lower100 = next_lower100;
				s.temp_upper100 = next_upper100;
				if (debug) { PPRINT("D.next_lower100:="); Serial.print(next_lower100); PPRINT(",next_upper100:="); Serial.println(next_upper100); }
				change = true;
			}
		}
	}

	if (change)
	{
		if (verbosity>V_SILENT)
		{
			PPRINTLN("EV.e=limits.chg");
			// dump strands verbosely to make sure the android app updates its limits
			dump_strand_infos(false, true);
		}
	}

	if (debug) PPRINTLN("D./update_temp_ramping");
}
#endif

void setup()
{
	// start serial port at the same baud rate avrdude uses to flash the pro mini
    // this way we'll be able to flash over bluetooth (pressing the reset button manually)
	Serial.begin(57600);
	PPRINT("I. "); println_version();
  
	digitalWrite(PIN_RESET, HIGH);
	pinMode(PIN_RESET,       OUTPUT);
	pinMode(PIN_STATUS_LED,  OUTPUT);
	pinMode(PIN_ONBOARD_LED, OUTPUT);

	#ifdef WITH_MEMSIZE_DEBUG
	PPRINT("D.freeMemory()=");      Serial.println(freeMemory());
	PPRINT("D.get_free_memory()="); Serial.println(get_free_memory());
	#endif

	eeprom_init();
	setup_starttime();
	#ifdef WITH_TEMP_RAMPING
	update_temp_ramping();
	#endif

	//BZERO(hb);

	for (uint8_t i=10; i>0; i--)
	{
		sensors_detect();
		if (sensors_complete())
		{
			break;
		}
		else
		{
			PPRINT("EV.e=sens.retry,r="); Serial.println(i);
			delay(350);
			set_status_leds(LOW);
			print_hb(0, 0, 0); //, severe_error, sensors_complete(), sensors_none());

			delay(110);
			set_status_leds(HIGH);
			print_hb(0, 0, 0); //, severe_error, sensors_complete(), sensors_none());
		}
	}

	#ifdef OSX
	//	#warning setting eeprom_reset to false in setup to force missing sensor condition
	//	eeprom_reset = false;
	#endif

	if (eeprom_reset)
	{
		num_sensors = MAX(1, num_sensors_available);
		PPRINT("EV.e=sens.init,exp="); Serial.print(num_sensors); PPRINT(",avl="); Serial.println(num_sensors_available);
		save_number_of_sensors_expected(num_sensors);
	}

	dump_general_info();
	dump_strand_infos(false, true);

	// indicate everything is okay: flicker status LED for 1 second and let strands
	// flash up for a second (after eeprom was read and actual pins are known)
	indicate_startup();
	compute_temps();
	update_history();

	#ifdef WITH_STATS
	/*
	int hour_index = get_current_hour();
	int day_index  = get_day_index();
	int week_index = get_week_index();
	PPRINT("D. INDEXES: hour="); Serial.print(hour_index); PPRINT(", day="); Serial.print(day_index); PPRINT(", week="); Serial.println(week_index);
	*/
	dump_stats();
	#endif

	dump_starttime();

	boolean change=check_limits();
	#ifdef WITH_INDICATE_UPDATE
	PPRINT("UP.chng="); Serial.print(change); PPRINT(",t="); Serial.println(0.001f*millis(), 1);
	#endif

}


void loop()
{
	//PPRINT("ts_optime: "); Serial.println(ts_optime);
	show_status(true);

	// Check update flags on strands, dump status if so and reset the flag.
	// This will assert continuous status updates to the mobile app.
	#ifdef WITH_STRAND_UPDATES
	if (dump_strand_infos(true, false))
	{
		//PPRINTLN("EV. e=strands.chg");
	}
	#else
	#warning STRAND UPDATES NOT SHOWN !!!!!!
	#endif


	// check the temperature limits when the current ts changed (thus every full minute)
	if (ts_last==ts_optime) // minute did not change
	{
		long pause_len = MILLIS_PER_UPTIME/3;

		// if we're missing sensors, re-check as often as possible
		if (!sensors_complete())
		{
			if (verbosity>V_SILENT)
			{
				PPRINTLN("EV.e=sens.redetect");
			}
			sensors_detect();
			pause_len = 10000; // retry in 10s
		}

		// start measuring the temepratures only if heartbeat is currently not beating
		// (i.e. showing steady state 0=LED lit) otherwise the animation will become a little bit bumpy
		boolean do_compute_temps = true;
		#ifdef WITH_HEARTBEAT
		do_compute_temps = (hb.last_status==0);
		#endif
		if (do_compute_temps)
		{
			compute_temps();
		}

		if (sensors_none() || severe_error)
		{
			pause_len = 500;
		}

		show_status(false);

		// wait a tiny amount of time if there was no measure
		//PPRINT("D. mydelay "); Serial.println(pause_len);
		mydelay(pause_len);
	}
	else // minute changed
	{
		ts_last = ts_optime;

		if (0==(millis()/10000))
		{
			if (severe_error)
			{
				PPRINTLN("E.NODATA");
			}
		}

		// if we're missing sensors, re-check every 15 minutes
		if (!sensors_complete() && 0==ts_optime%5)
		{
			sensors_detect();
		}

		boolean change = check_limits();
		// "UP." stand for (possibly) update (of the strand states)
		#ifdef WITH_INDICATE_UPDATE
		PPRINT("UP.chng="); Serial.print(change); PPRINT(",t="); Serial.println(0.001f*millis());
		#endif
		if (change)
		{
			dump_current_stats();
		}

		uint8_t rc = update_history();
		#ifdef WITH_SHOW_EVENTS
		if (HIST_UPD_NONE!=rc)
		{
			if (verbosity>V_SILENT)
			{
				PPRINT("EV.e=hist.updtd,reas=");
				switch (rc)
				{
				case HIST_UPD_1ST : PPRINTLN("init"); break;
				case HIST_UPD_MIN : PPRINTLN("min"); break;
				case HIST_UPD_MAX : // fall through
				default:            PPRINTLN("max"); break;
				}
			}
		}
		#endif

		// update stats every full hour. since actual write cycles are distributed equally
		// over MAX_HOURS (24) entries (see: s_stats_hour), there is only 1 write cycle per day
		// thus max. write cycles now exceeded before 270 years
		//if (0==ts_optime%60)
		if (0==ts_optime%60)
		{
			save_statistics();
			#ifdef WITH_SHOW_EVENTS
			if (verbosity>V_SILENT)
			{
				PPRINTLN("EV.e=stats.savd");
			}
			#endif

			#ifdef OSX
			usleep(30000);
			#endif
		}

		// save history every MMHIST_INTERVAL (6) hours means 4 writes a day to the index (hist_count)
		// and will not exceed max. write cycles before approx. 68 years (4*365*68 = 99348)
		//printf("t till next save_minmax: %i\n", MMHIST_INTERVAL*60-ts_optime%(MMHIST_INTERVAL*60));
		if (0==ts_optime%(MMHIST_INTERVAL*60))
		{
			#ifdef WITH_TEMP_RAMPING
			update_temp_ramping();
			#endif

			save_uptime();
			save_minmax();
			#ifdef WITH_SHOW_EVENTS
			if (verbosity>V_SILENT)
			{
				PPRINTLN("EV.e=hist.savd");
			}
			#endif
		}

		show_status(false);
	}

	check_serial();
	update_uptime();
}


//////////// unused stuff


#ifdef WITH_MEMSIZE_DEBUG
extern int __bss_end;
extern void *__brkval;

int get_free_memory()
{
  int free_memory;

  if((int)__brkval == 0)
	free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
	free_memory = ((int)&free_memory) - ((int)__brkval);

  return free_memory;
}
#endif

#ifdef WITH_AUTOCONFIG
void sensor_get(uint8_t i, s_sensor_info & sensor)
{
	sensor = strand_infos[i].sensor;
}


static void auto_config()
{
	compute_temps();

	int8_t T[num_sensors_available];
	int8_t result[num_strands];

	for (uint8_t j=0; j<num_sensors_available; j++)
	{
		T[j] = strand_infos[j].curr_temp;
	}

	for (uint8_t i=0; i<num_strands; i++)
	{
		result[i] = -1;

		PPRINT("AUTOCONF: n=");  Serial.println(i+1);

		s_strand_info_eeprom & s = strand_infos[i];
		pinMode(s.pin, OUTPUT);
		digitalWrite(s.pin, HIGH);

		uint32_t ts_timeout = millis()+30*1000;
		int8_t found = -1;
		while (found<0 && millis()<ts_timeout);
		{
			delay(1000);
			compute_temps();

			for (int j=0; j<num_sensors_available; j++)
			{
				// check if temp. increased by 1 degrees
				if (strand_infos[j].curr_temp>T[j]+2*1)
				{
					found = j;
				}
			}
		}

		if (found<0)
		{
			PPRINT("AUTORES. n="); Serial.print(i+1); PPRINTLN(", sens=none");
		}
		else
		{
			PPRINT("AUTORES. n="); Serial.print(i); PPRINT(", sens="); Serial.println(found);
			T[found] -= 10;
			result[i] = found;
		}
	}

	s_sensor_info sensor_infos_new[MAX_SENSORS];
	for (uint8_t i=0; i<num_strands; i++)
	{
		int j = result[i];
		if (j>-1)
		{
			s_sensor_info s;
			sensor_get(j, s);
			sensor_infos_new[i] = s;
		}
	}

	PPRINTLN("AUTOCONF!");
}
#endif


#ifdef WITH_TESTMODE

boolean isTestModeActive = false;

static void test_mode()
{
	if (isTestModeActive)
	{
		isTestModeActive = 0;
		return;
	}

	isTestModeActive = 1;

	PPRINTLN("TM:for 5 minutes\n");
	PPRINTLN("TM:Heat up by 1C to lights strand");

	compute_temps();
	uint32_t ts_end = millis()+2*60*1000L; // "now" plus 120seconds

	float T[num_sensors_available];
	for (uint8_t i=0; i<num_sensors_available; i++)
	{
		T[i] = strand_infos[i].curr_temp;
	}

	while (isTestModeActive && millis()<ts_end)
	{
		mydelay(1000); // will handle serial command end possible reset "isTestmodeActive"
		compute_temps();

		for (uint8_t i=0; i<num_sensors_available; i++)
		{
			int8_t t = strand_infos[i].curr_temp;
			PPRINT("TMRES.n="); Serial.print(i+1); PPRINT(", t="); Serial.print(0.5*t); PPRINT(" (init: "); Serial.print(0.5*T[i]); PPRINT(") ");

                        int pin   = strand_infos[i].pin;
			int level = (t>T[i]+2.0) ? HIGH : LOW;

                        pinMode(pin, OUTPUT);
			digitalWrite(pin, level);
                        PPRINT(" pin "); Serial.print(pin); PPRINT(" -> "); Serial.println(level);

                        if (HIGH==level) ts_end = millis()+2*60*1000L;
                        PPRINT("TM: ms left: "); Serial.println(ts_end-millis());
		}
	}

	PPRINTLN("TM!");
	isTestModeActive = false;
}
#endif

