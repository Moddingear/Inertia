#!/usr/bin/env python3
"""
Gamepad input capture and UDP transmission to ESP32
Requires: pip install pygame
"""

import pygame
import socket
import struct
import time
import sys
from typing import Dict, Any

class GamepadClient:
	def __init__(self, esp32_ip, esp32_port):
		self.esp32_ip = esp32_ip
		self.esp32_port = esp32_port
		self.running = False
		self.joystick = None
		self.socket = None
		
		# Initialize pygame
		pygame.init()
		pygame.joystick.init()
		
	def setup_socket(self) -> bool:
		try:
			self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.socket.settimeout(0.1)  # Non-blocking with short timeout
			return True
		except Exception as e:
			print(f"Failed to create socket: {e}")
			return False
		
	def connect_gamepad(self) -> bool:
		if self.joystick is not None:
			return True
		
		pygame.event.pump()
		joystick_count = pygame.joystick.get_count()
		
		if joystick_count == 0:
			print("No gamepad detected!")
			return False
			
		self.joystick = pygame.joystick.Joystick(0)
		self.joystick.init()
		
		print(f"Connected to: {self.joystick.get_name()}")
		print(f"Axes: {self.joystick.get_numaxes()}")
		print(f"Buttons: {self.joystick.get_numbuttons()}")
		print(f"Hats: {self.joystick.get_numhats()}")
		
		return True
	
	def get_gamepad_state(self) -> Dict[str, Any]:

		if self.joystick is None and not self.connect_gamepad():
			return {}
			
		# Update pygame events
		#pygame.event.pump()

		events = pygame.event.get(pygame.JOYDEVICEREMOVED)

		for event in events:
			if event.type == pygame.JOYDEVICEREMOVED and self.joystick.get_instance_id() == event.instance_id:
				self.joystick.quit()
				self.joystick = None
				print("Joystick disconnection event")
				return {}
				
		state = {
			"axes": [],
			"buttons": [],
			"hats": []
		}
		
		# Read all axes (analog sticks, triggers)
		for i in range(self.joystick.get_numaxes()):
			axis_value = self.joystick.get_axis(i)
			state["axes"].append(axis_value)
		
		# Read all buttons
		for i in range(self.joystick.get_numbuttons()):
			button_state = self.joystick.get_button(i)
			state["buttons"].append(button_state)
		
		# Read all hat switches (D-pad)
		for i in range(self.joystick.get_numhats()):
			hat_state = self.joystick.get_hat(i)
			state["hats"].append(hat_state)
		
		return state
	
	def pack_gamepad_data(self, state: Dict[str, Any]) -> bytes:
		# Header: 4 bytes - num_axes, num_buttons, num_hats, reserved
		header = struct.pack('BBBB', 
						   len(state.get('axes', [])), 
						   len(state.get('buttons', [])),
						   len(state.get('hats', [])),
						   0)  # reserved
		
		# Axes: 4 bytes each (float)
		axes_data = b''
		for axis in state.get('axes', []):
			axes_data += struct.pack('f', axis)
		
		# Buttons: 1 bit each, packed into bytes
		buttons = state.get('buttons', [])
		button_bytes = []
		for i in range(0, len(buttons), 8):
			byte_val = 0
			for j in range(8):
				if i + j < len(buttons) and buttons[i + j]:
					byte_val |= (1 << j)
			button_bytes.append(byte_val)
		buttons_data = struct.pack(f'{len(button_bytes)}B', *button_bytes)
		
		# Hats: 2 bytes each (2 signed chars)
		hats_data = b''
		for hat in state.get('hats', []):
			hats_data += struct.pack('bb', hat[0], hat[1])
		
		return header + axes_data + buttons_data + hats_data
	
	def send_to_esp32(self, data: Dict[str, Any]) -> bool:
		"""Send gamepad data to ESP32 via UDP"""
		if not self.socket:
			return False
			
		try:
			packed_data = self.pack_gamepad_data(data)
			self.socket.sendto(packed_data, (self.esp32_ip, self.esp32_port))
			return True
		except Exception as e:
			return False
	
	def test_esp32_connection(self) -> bool:
		"""Test connection to ESP32 with ping packet"""
		if not self.socket:
			return False
			
		try:
			# Send a simple ping packet (just the header with zeros)
			ping_packet = struct.pack('BBBB', 0, 0, 0, 255)  # 255 = ping marker
			self.socket.sendto(ping_packet, (self.esp32_ip, self.esp32_port))
			
			# Try to receive response (optional - UDP is fire-and-forget)
			try:
				response, addr = self.socket.recvfrom(64)
				return len(response) > 0
			except socket.timeout:
				return True  # No response needed for UDP, just assume it worked
		except Exception as e:
			return False
	
	def run(self):
		if not self.setup_socket():
			return
		
		self.running = True
		send_counter = 0
		error_count = 0
		
		try:
			while self.running:
				current_state = self.get_gamepad_state()
				if len(current_state) == 0:
					time.sleep(0.1)
					continue
				
				success = self.send_to_esp32(current_state)
				
				if not success:
					error_count += 1
				
				if send_counter % 10 == 0:
					status = "✓" if error_count < 1 else "✗"
					print(f"{status} Axes:{current_state['axes']} Buttons: {current_state['buttons']} Hats: {current_state['hats']} Errors: {error_count}")
					error_count = 0
				
				send_counter += 1
				time.sleep(0.01)
				
		except KeyboardInterrupt:
			print("\nStopping gamepad client...")
		finally:
			self.running = False
			if self.socket:
				self.socket.close()
			if self.joystick:
				self.joystick.quit()
			pygame.quit()

def main():
	default_ip = "192.168.0.6"
	if len(sys.argv) > 1:
		esp32_ip = sys.argv[1]
	else:
		esp32_ip = input(f"Enter ESP32 IP address (default: {default_ip}): ").strip()
		if not esp32_ip:
			esp32_ip = default_ip
	
	# Use UDP port instead of HTTP port
	client = GamepadClient(esp32_ip, 12345)
	client.run()

if __name__ == "__main__":
	main()