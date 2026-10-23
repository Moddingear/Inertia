#pragma once

#include <HardwareSerial.h>

#define PACKED __attribute__((packed))

class ydlidar
{
protected:
	bool scanning_wanted = false, scanning_inferred = false;
	int set_scan_frequency_dihz = 60;
	HardwareSerial &interface;
	uint8_t rx_buffer[1<<10];
	size_t rx_write_index = 0;

    static const uint8_t PACKAGE_SAMPLE_MAX_LENGTH = 40;

	uint32_t sn_number = 0;

	int ct_index = 0;

	enum class command_code :uint8_t {
		SCAN = 0x60,
		STOP = 0x65,
		GET_DEVICE_INFORMATION = 0x90,
		GET_DEVICE_HEALTH = 0x92,
		FREQ_ADD_01HZ = 0x09,
		FREQ_REM_01HZ = 0x0A,
		FREQ_ADD_1HZ = 0x0B,
		FREQ_REM_1HZ = 0x0C,
		FREQ_GET = 0x0D,
		RESTART = 0x40,
	};

	enum class answer_code : uint8_t {
		INFORMATION = 0x04,
		HEALTH = 0x06,
	};

	static const char command_sync_byte = 0xA5;
	static const char response_sync_byte1 = 0xA5, response_sync_byte2 = 0x5A;
	static const char data_sync_byte1 = 0xAA, data_sync_byte2 = 0x55;

    
    struct PACKED device_info {
		uint8_t		model;
		uint16_t	firmware_version;
		uint8_t		hardware_version;
		uint8_t		serialnum[16];
	};
    
    struct PACKED device_health {
		union{
			struct
			{
				bool	sensor_status:1;
				bool	encoder_status:1;
				bool	wireless_power_status:1;
				bool	photodiode_status:1;
				bool	laser_status:1;
				bool	data_status:1;
				uint8_t :2;
			};
			uint8_t status;
		};
		uint16_t	error_code;
    };
    
    struct PACKED command_packet {
		uint8_t			syncByte = command_sync_byte;
		command_code	cmd_flag;
		//uint8_t			size;
		//uint8_t			data;
    } ;
    
    struct PACKED answer_header {
		uint8_t		syncByte1 = response_sync_byte1;
		uint8_t		syncByte2 = response_sync_byte2;
		uint32_t	size:30;
		uint8_t		mode:2;
		answer_code	type;
    };
    
public:
	struct PACKED distance_measurement {
		uint8_t		intensity;
		uint8_t		flag:2;
		uint16_t	distance:14;
    };

protected:
    struct PACKED data_header {
		uint16_t	package_Head;
		bool		packet_zero:1;
		uint8_t		packet_CT:7;
		uint8_t		nowPackageNum; // distance sample count
		uint16_t	packageFirstSampleAngle;
		uint16_t	packageLastSampleAngle;
		uint16_t	checkSum;
    };

	void send_command(command_code code);

	int consume_answer(int read_index);
	int consume_data(int read_index);

public:

	struct distance_data
	{
		float start_angle, end_angle;
		bool checksum_ok;
		int num_points;
		distance_measurement* points;
	};
	
	typedef void(*data_callback)(const distance_data &data);

private:
	data_callback OnData = nullptr;

public:
	ydlidar(HardwareSerial &in_interface);
	~ydlidar();

	void soft_restart();

	void get_device_information();
	void get_device_health();

	void set_scanning(bool scanning);
	bool is_scanning();
	void set_scan_frequency(float frequency);
	float get_scan_frequency();

	void loop();

	void set_data_callback(data_callback callback) {OnData = callback;};
};

#undef PACKED

