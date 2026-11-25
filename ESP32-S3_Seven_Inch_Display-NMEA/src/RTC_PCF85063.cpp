#include "RTC_PCF85063.h"

datetime_t datetime= {0};

static uint8_t decToBcd(int val);
static int bcdToDec(uint8_t val);

const unsigned char MonthStr[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov","Dec"};


void RTC_Loop(void *arg);
/******************************************************************************
function:	PCF85063 initialized
parameter:
            
Info:Initiate Normal Mode, RTC Run, NO reset, No correction , 24hr format, Internal load capacitane 12.5pf
******************************************************************************/
// The Waveshare ESP32-S3-Touch-LCD-7 has no RTC. Without this check every
// one-second read below would fail and log, and leave the time unset.
static bool s_rtc_present = false;
bool PCF85063_Present(void) { return s_rtc_present; }

// Software clock used when there's no RTC: set from Signal K / NMEA 2000 time
// (via PCF85063_Set_All) and advanced with millis(). Lost at power-off, like
// any clock without a battery; it's set again by the next time message.
static bool     s_soft_set = false;
static int64_t  s_soft_base_s = 0;      // seconds since 1970-01-01 at s_soft_base_ms
static uint32_t s_soft_base_ms = 0;

// Days since 1970-01-01 for a proleptic Gregorian date (Howard Hinnant).
static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
	y -= m <= 2;
	const int64_t era = (y >= 0 ? y : y - 399) / 400;
	const unsigned yoe = (unsigned)(y - era * 400);
	const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
	const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
	return era * 146097 + (int64_t)doe - 719468;
}

static void civil_from_days(int64_t z, int *y, unsigned *m, unsigned *d)
{
	z += 719468;
	const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
	const unsigned doe = (unsigned)(z - era * 146097);
	const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
	const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	const unsigned mp = (5 * doy + 2) / 153;
	*d = doy - (153 * mp + 2) / 5 + 1;
	*m = mp < 10 ? mp + 3 : mp - 9;
	*y = (int)(yoe + era * 400) + (*m <= 2);
}

static void soft_clock_set(const datetime_t &t)
{
	if (t.month < 1 || t.month > 12 || t.day < 1 || t.day > 31) return;
	s_soft_base_s = days_from_civil(t.year, t.month, t.day) * 86400LL
	              + t.hour * 3600LL + t.minute * 60LL + t.second;
	s_soft_base_ms = millis();
	s_soft_set = true;
}

static void soft_clock_read(datetime_t *t)
{
	*t = datetime_t{};
	if (!s_soft_set) return;                       // year 0: callers treat as "no time"
	int64_t now = s_soft_base_s + (int64_t)((uint32_t)(millis() - s_soft_base_ms) / 1000);
	int64_t days = now / 86400;
	int64_t secs = now % 86400;
	int y; unsigned m, d;
	civil_from_days(days, &y, &m, &d);
	t->year = (uint16_t)y; t->month = (uint8_t)m; t->day = (uint8_t)d;
	t->hour = (uint8_t)(secs / 3600); t->minute = (uint8_t)((secs / 60) % 60); t->second = (uint8_t)(secs % 60);
	t->dotw = (uint8_t)((days + 4) % 7);           // 1970-01-01 was a Thursday
}

void PCF85063_Init()
{
	Wire.beginTransmission(PCF85063_ADDRESS);
	s_rtc_present = (Wire.endTransmission() == 0);
	if (!s_rtc_present) {
		printf("PCF85063 not fitted - using a software clock set from Signal K / NMEA 2000\r\n");
		return;
	}
	uint8_t Value = RTC_CTRL_1_DEFAULT|RTC_CTRL_1_CAP_SEL;

	I2C_Write(PCF85063_ADDRESS, RTC_CTRL_1_ADDR, &Value, 1);
  I2C_Read(PCF85063_ADDRESS, RTC_CTRL_1_ADDR,  &Value, 1);
	if(Value & RTC_CTRL_1_STOP)
		printf("PCF85063 failed to be initialized.state :%d\r\n",Value);
	else
		printf("PCF85063 is running,state :%d\r\n",Value);

}

void RTC_Loop(void)
{
  PCF85063_Read_Time(&datetime);
}
/******************************************************************************
function:	Reset PCF85063
parameter:
Info:		
******************************************************************************/
void PCF85063_Reset()
{
	uint8_t Value = RTC_CTRL_1_DEFAULT|RTC_CTRL_1_CAP_SEL|RTC_CTRL_1_SR;
	esp_err_t ret = I2C_Write(PCF85063_ADDRESS, RTC_CTRL_1_ADDR, &Value, 1);
	if(ret != ESP_OK)
		printf("PCF85063 : Reset failure\r\n");
}
/******************************************************************************
function:	Set Time 
parameter:
Info:		
******************************************************************************/
void PCF85063_Set_Time(datetime_t time)
{
	uint8_t buf[3] = {decToBcd(time.second),
					  decToBcd(time.minute),
					  decToBcd(time.hour)};
	esp_err_t ret = I2C_Write(PCF85063_ADDRESS, RTC_SECOND_ADDR, buf, sizeof(buf));
	if(ret != ESP_OK)
		printf("PCF85063 : Time setting failure\r\n");
}

/******************************************************************************
function:	Set Date
parameter:
Info:		
******************************************************************************/
void PCF85063_Set_Date(datetime_t date)
{
	uint8_t buf[4] = {decToBcd(date.day),
					  decToBcd(date.dotw),
					  decToBcd(date.month),
					  decToBcd(date.year - YEAR_OFFSET)};
	esp_err_t ret = I2C_Write(PCF85063_ADDRESS, RTC_DAY_ADDR, buf, sizeof(buf));
	if(ret != ESP_OK)
		printf("PCF85063 : Date setting failed\r\n");
}

/******************************************************************************
function:	Set Time And Date
parameter:
Info:		
******************************************************************************/
void PCF85063_Set_All(datetime_t time)
{
	if (!s_rtc_present) { soft_clock_set(time); return; }
	uint8_t buf[7] = {decToBcd(time.second),
					  decToBcd(time.minute),
					  decToBcd(time.hour),
					  decToBcd(time.day),
					  decToBcd(time.dotw),
					  decToBcd(time.month),
					  decToBcd(time.year - YEAR_OFFSET)};
	esp_err_t ret = I2C_Write(PCF85063_ADDRESS, RTC_SECOND_ADDR, buf, sizeof(buf));
	if(ret != ESP_OK)
		printf("PCF85063 : Failed to set the date and time\r\n");
}

/******************************************************************************
function:	Read Time And Date
parameter:
Info:		
******************************************************************************/
void PCF85063_Read_Time(datetime_t *time)
{
	if (!s_rtc_present) { soft_clock_read(time); return; }
	uint8_t buf[7] = {0};
	esp_err_t ret = I2C_Read(PCF85063_ADDRESS, RTC_SECOND_ADDR, buf, sizeof(buf));
	if(ret != ESP_OK)
		printf("PCF85063 : Time read failure\r\n");
	else{
		time->second = bcdToDec(buf[0] & 0x7F);
		time->minute = bcdToDec(buf[1] & 0x7F);
		time->hour = bcdToDec(buf[2] & 0x3F);
		time->day = bcdToDec(buf[3] & 0x3F);
		time->dotw = bcdToDec(buf[4] & 0x07);
		time->month = bcdToDec(buf[5] & 0x1F);
		time->year = bcdToDec(buf[6]) + YEAR_OFFSET;
	}
}

/******************************************************************************
function:	Enable Alarm and Clear Alarm flag
parameter:			
Info:		
******************************************************************************/
void PCF85063_Enable_Alarm()
{
	uint8_t Value = RTC_CTRL_2_DEFAULT | RTC_CTRL_2_AIE;
	Value &= ~RTC_CTRL_2_AF;
	esp_err_t ret = I2C_Write(PCF85063_ADDRESS, RTC_CTRL_2_ADDR, &Value, 1);
	if(ret != ESP_OK)
		printf("PCF85063 : Failed to enable Alarm Flag and Clear Alarm Flag \r\n");
}

/******************************************************************************
function:	Get Alarm flag
parameter:			
Info:		
******************************************************************************/
uint8_t PCF85063_Get_Alarm_Flag()
{
	uint8_t Value = 0;
	esp_err_t ret = I2C_Read(PCF85063_ADDRESS, RTC_CTRL_2_ADDR, &Value, 1);
	if(ret != ESP_OK)
		printf("PCF85063 : Failed to obtain a warning flag.\r\n");
	else
		Value &= RTC_CTRL_2_AF | RTC_CTRL_2_AIE;
	//printf("Value = 0x%x",Value);
	return Value;
}

/******************************************************************************
function:	Set Alarm
parameter:			
Info:		
******************************************************************************/
void PCF85063_Set_Alarm(datetime_t time)
{

	uint8_t buf[5] ={
		(uint8_t)(decToBcd(time.second)&(~RTC_ALARM)),
		(uint8_t)(decToBcd(time.minute)&(~RTC_ALARM)),
		(uint8_t)(decToBcd(time.hour)&(~RTC_ALARM)),
		//decToBcd(time.day)&(~RTC_ALARM),
		//decToBcd(time.dotw)&(~RTC_ALARM)
		RTC_ALARM, 	//disalbe day
		RTC_ALARM	//disalbe weekday
	};
	esp_err_t ret = I2C_Write(PCF85063_ADDRESS, RTC_SECOND_ALARM, buf, sizeof(buf));
	if(ret != ESP_OK)
		printf("PCF85063 : Failed to set alarm flag\r\n");
}

/******************************************************************************
function:	Read Alarm
parameter:			
Info:		
******************************************************************************/
void PCF85063_Read_Alarm(datetime_t *time)
{
	uint8_t buf[5] = {0};
	esp_err_t ret = I2C_Read(PCF85063_ADDRESS, RTC_SECOND_ALARM, buf, sizeof(buf));
	if(ret != ESP_OK)
		printf("PCF85063 : Failed to read the alarm sign\r\n");
	else{
		time->second = bcdToDec(buf[0] & 0x7F);
		time->minute = bcdToDec(buf[1] & 0x7F);
		time->hour = bcdToDec(buf[2] & 0x3F);
		time->day = bcdToDec(buf[3] & 0x3F);
		time->dotw = bcdToDec(buf[4] & 0x07);
	}
}


/******************************************************************************
function:	Convert normal decimal numbers to binary coded decimal
parameter:			
Info:		
******************************************************************************/
static uint8_t decToBcd(int val)
{
	return (uint8_t)((val / 10 * 16) + (val % 10));
}

/******************************************************************************
function:	Convert binary coded decimal to normal decimal numbers
parameter:			
Info:		
******************************************************************************/
static int bcdToDec(uint8_t val)
{
	return (int)((val / 16 * 10) + (val % 16));
}

/******************************************************************************
function:	
parameter:	
Info:		
******************************************************************************/
void datetime_to_str(char *datetime_str,datetime_t time)
{
	sprintf(datetime_str, " %d.%d.%d  %d %d:%d:%d ", time.year, time.month, 
			time.day, time.dotw, time.hour, time.minute, time.second);
} 