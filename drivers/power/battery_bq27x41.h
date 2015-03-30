#ifndef __BATTERY_BQ27x41_H__
#define __BATTERY_BQ27x41_H__

#define GET_BAT_VOLTAGE		1
#define GET_BAT_TEMPERATURE	2
#define GET_BAT_CURRENT		3

int32_t get_battery_parameters(int prop_type);

#endif

