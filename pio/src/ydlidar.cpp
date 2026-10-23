#include <ydlidar.hpp>
#include <algorithm>

ydlidar::ydlidar(HardwareSerial &in_interface)
	:interface(in_interface)
{
}

ydlidar::~ydlidar()
{
}

void ydlidar::send_command(command_code code)
{
	command_packet packet;
	packet.cmd_flag = code;
	interface.write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
}

int ydlidar::consume_answer(int read_index)
{
	int bytes_after = rx_write_index - read_index;
	if (bytes_after < sizeof(answer_header))
	{
		return 0;
	}
	const auto header = reinterpret_cast<answer_header*>(&rx_buffer[read_index]);
	int full_packet_size = sizeof(answer_header) + header->size;
	switch (header->mode)
	{
	case 0: //Single response
		scanning_inferred = false;
		if (bytes_after < full_packet_size) //check the whole packet is in the buffer
		{
			printf("Not enough bytes after single packet (%d required, %d left), doing it later...\n", full_packet_size, bytes_after);
			return 0;
		}
		break;
	case 1: //Continous response (scan)
		scanning_inferred = true;
		return sizeof(answer_header);
	default:
		printf("Got invalid mode ! %d\n", header->mode);
		return 0;
	}
	
	switch (header->type)
	{
	case answer_code::INFORMATION:
		if (header->size == sizeof(device_info))
		{
			//Device information
			const auto information = reinterpret_cast<const device_info*>(&header[1]);
			printf("Lidar model %d, firmware %d, hardware %d, serial number 0x", 
				information->model, information->firmware_version, information->hardware_version);
			for (size_t i = 0; i < sizeof(information->serialnum); i++)
			{
				printf("%02x ", information->serialnum[i]);
			}
			printf("\n");
			
		}
		else if (header->size == sizeof(uint32_t))
		{
			//Frequency information
			const auto freq = *reinterpret_cast<const uint32_t*>(&header[1]);
			set_scan_frequency_dihz = (freq+4)/10;
			printf("Lidar currently running at %d.%2d Hz (%ddiHz)\n", freq/100, freq%100, set_scan_frequency_dihz);
		}
		else
		{
			printf("Huh?! %d bytes of information\n", header->size);
		}
		break;
	case answer_code::HEALTH:
		break;
	
	default:
		printf("Got unknown answer code %d\n", header->type);
		break;
	}
	
	return full_packet_size;
}

int ydlidar::consume_data(int read_index)
{
	scanning_inferred = true;
	int bytes_after = rx_write_index - read_index;
	if (bytes_after < sizeof(data_header))
	{
		return 0;
	}
	const auto header = reinterpret_cast<data_header*>(&rx_buffer[read_index]);
	int full_packet_size = sizeof(data_header) + header->nowPackageNum * sizeof(distance_measurement);
	if (bytes_after < full_packet_size)
	{
		return 0;
	}
	const auto checksum_u16_ptr = reinterpret_cast<uint16_t*>(header);
	uint16_t checksum_computed = 0;
	for (size_t i = 0; i < 4; i++)
	{
		checksum_computed ^= checksum_u16_ptr[i];
	}
	auto node_ptr = reinterpret_cast<distance_measurement*>(&header[1]);
	for (size_t i = 0; i < header->nowPackageNum; i++)
	{
		uint8_t *cptr = reinterpret_cast<uint8_t*>(&node_ptr[i]);
		checksum_computed ^= cptr[0];
		checksum_computed ^= *reinterpret_cast<uint16_t*>(&cptr[1]);
	}
	if (checksum_computed != header->checkSum)
	{
		printf("Checksum doesn't match! %#x vs %#x\n", checksum_computed, header->checkSum);
	}
	if (header->package_Head != 0x55AA)
	{
		printf("Data header wrong! %x instead of %x\n", header->package_Head, 0x55AA);
	}
	

	if (header->nowPackageNum == 1)
	{
		ct_index = 0;
	}
	else
	{
		ct_index++;
	}
	switch (ct_index)
	{
	case 0:
		//printf("Current scan frequency : %d.%dHz\n", header->packet_CT/10, header->packet_CT%10);
		break;
	case 3:
		if (header->packet_CT)
		{
			printf("Warning Health: %x\n", header->packet_CT);
		}
		break;
	#if 0
	case 9:
		printf("Serial number year %d\n", (header->packet_CT>>2) + 2020);
		sn_number &= ~(0x3<<19);
		sn_number |= (header->packet_CT & 0x3) << 19;
		break;
	case 10:
		printf("Serial number month %d\n", (header->packet_CT>>3));
		sn_number &= ~(0x7<<16);
		sn_number |= (header->packet_CT & 0x7) << 16;
		break;
	case 11:
		printf("Serial number day %d\n", (header->packet_CT>>2));
		sn_number &= ~(0x3<<14);
		sn_number |= (header->packet_CT & 0x3) << 14;
		break;
	case 12:
		sn_number &= ~(0x7f<<7);
		sn_number |= (header->packet_CT & 0x7f) << 7;
		break;
	case 13:
		sn_number &= ~(0x7f<<0);
		sn_number |= (header->packet_CT & 0x7f) << 0;
		printf("Serial number %#x\n", sn_number);
		break;
	#endif
	
	default:
		break;
	}

	if (OnData)
	{
		distance_data converted;
		converted.start_angle = (header->packageFirstSampleAngle >> 1) / 64.f;
		converted.end_angle = (header->packageLastSampleAngle >> 1) / 64.f;
		converted.checksum_ok = checksum_computed == header->checkSum;
		converted.num_points = header->nowPackageNum;
		converted.points = node_ptr;
		OnData(converted);
	}
	return full_packet_size;
}

void ydlidar::soft_restart()
{
	send_command(command_code::RESTART);
}

void ydlidar::get_device_information()
{
	send_command(command_code::GET_DEVICE_INFORMATION);
}

void ydlidar::get_device_health()
{
	send_command(command_code::GET_DEVICE_HEALTH);
}

void ydlidar::set_scanning(bool in_scanning)
{
	if (in_scanning == scanning_wanted)
	{
		return;
	}
	if (in_scanning)
	{
		printf("Sending START command\n");
		send_command(command_code::SCAN);
	}
	else
	{
		printf("Sending STOP command\n");
		send_command(command_code::STOP);
	}
	scanning_wanted = in_scanning;
}

bool ydlidar::is_scanning()
{
	return scanning_inferred;
}

void ydlidar::set_scan_frequency(float frequency)
{
	int target_dihz = roundf(frequency*10);
	int diff_dihz = target_dihz - set_scan_frequency_dihz;
	int diff_hz = (diff_dihz +4) / 10;
	while (diff_hz != 0)
	{
		if (diff_hz > 0)
		{
			send_command(command_code::FREQ_ADD_1HZ);
			diff_hz--;
			diff_dihz -= 10;
		}
		if (diff_hz < 0)
		{
			send_command(command_code::FREQ_REM_1HZ);
			diff_hz++;
			diff_dihz += 10;
		}
	}
	while (diff_dihz != 0)
	{
		if (diff_dihz > 0)
		{
			send_command(command_code::FREQ_ADD_01HZ);
			diff_dihz--;
		}
		if (diff_dihz < 0)
		{
			send_command(command_code::FREQ_REM_01HZ);
			diff_dihz++;
		}
	}
}

float ydlidar::get_scan_frequency()
{
	return set_scan_frequency_dihz /10.f;
}

void ydlidar::loop()
{
	auto num_available = interface.available();
	if (num_available)
	{
		auto num_read = interface.readBytes(&rx_buffer[rx_write_index], std::min<int>(sizeof(rx_buffer) - rx_write_index, num_available));
		rx_write_index += num_read;
		//printf("Lidar gathered %d/%d bytes\n", num_read, num_available);
		int num_consumed = 0;
		for (size_t i = 1; i < rx_write_index; i++)
		{
			int consumed_here = 0;
			if (rx_buffer[i] == response_sync_byte2 && rx_buffer[i-1] == response_sync_byte1)
			{
				consumed_here = consume_answer(i-1);
				//printf("Found answer at offset %d, consumed %d bytes (%d bytes in buffer)\n", i-1, consumed_here, rx_write_index);
			}
			else if (rx_buffer[i] == data_sync_byte2 && rx_buffer[i-1] == data_sync_byte1)
			{
				consumed_here = consume_data(i-1);
				//printf("Found data at offset %d, consumed %d bytes (%d bytes in buffer)\n", i-1, consumed_here, rx_write_index);
			}
			if (consumed_here)
			{
				num_consumed = i-1+consumed_here;
				i = num_consumed-1;
			}
		}
		if (num_consumed > 0)
		{
			memmove(rx_buffer, &rx_buffer[num_consumed], rx_write_index - num_consumed);
			printf("Shrinking from %d to %d\n", rx_write_index, rx_write_index-num_consumed);
			rx_write_index -= num_consumed;
		}
	}
}
