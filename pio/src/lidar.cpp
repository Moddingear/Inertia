#include <lidar.hpp>

#include <wifi_protobuf.hpp>
#include <ydlidar.hpp>
ydlidar lidar1(Serial1);

#if 0
bool lidar_point_encoder(pb_ostream_t *stream, const pb_field_iter_t *field, void * const *arg)
{
	auto data = reinterpret_cast<const ydlidar::distance_data*>(*arg);
	printf("Encoding %d points from %p\n", data->num_points, data->points);
	for (size_t i = 0; i < data->num_points; i++)
	{
		if (!pb_encode_tag_for_field(stream, field))
		{
			return false;
		}
		LidarData_LidarPoint point;
		point.distance = data->points[i].distance;
		point.luminosity = data->points[i].intensity;
		if (!pb_encode_submessage(stream, LidarData_LidarPoint_fields, &point))
		{
			return false;
		}
	}
	return true;
}
#endif

void OnLidarData(const ydlidar::distance_data &data)
{
	gd_esp_packet packet;
	packet.which_data = gd_esp_packet_lidar_data_tag;
	auto &ldata = packet.data.lidar_data;
	ldata.start_angle = data.start_angle;
	ldata.end_angle = data.end_angle;
	strcpy(ldata.lidar_source, "T-mini+|I8F2D14");
	#if 1
	int num_invalid = 0;
	for (size_t i = 0; i < data.num_points; i++)
	{
		if (data.points[i].distance <= 10)
		{
			num_invalid++;
		}
	}
	#endif
	
	ldata.raw.size = sizeof(data.points[0])*data.num_points;
	memcpy(ldata.raw.bytes, data.points, ldata.raw.size);
	if (num_invalid)
	{
		printf("Sending lidar packet with %d invalid points\n", num_invalid);
	}
	
	wifi_if::broadcast(packet);
}

void lidar::setup()
{
    Serial1.setRxBufferSize(1<<10);
	Serial1.begin(230400, SERIAL_8N1, 18, 17);

	lidar1.soft_restart();
	delay(100);
	lidar1.get_device_information();
	lidar1.set_data_callback(OnLidarData);
	lidar1.set_scan_frequency(12);
}

void lidar::loop()
{
	lidar1.loop();
}

void lidar::start()
{
    lidar1.set_scanning(true);
}

void lidar::stop()
{
    lidar1.set_scanning(false);
}