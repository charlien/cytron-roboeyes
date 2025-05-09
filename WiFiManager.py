import board # type: ignore
import busio # type: ignore
import asyncio # type: ignore
import json
import binascii
import hashlib
import base64

class WiFiManager:
    def __init__(self, tx_pin, rx_pin, baud=115200, port=8080):
        """Initialize WiFi Manager with ESP8266 over UART"""
        # Use a slower baud rate initially for more reliable communication
        self.uart = busio.UART(tx_pin, rx_pin, 
                              baudrate=baud,  # Start with 115200 for stability
                              timeout=1,
                              receiver_buffer_size=2048)
        self.port = port
        self.is_active = False
        self.buffer = ""
        self.websocket_clients = {}  # Store WebSocket connections

    async def send_at_command(self, command, wait_time=1):
        """Send AT command to ESP8266 and wait for response"""
        print(f"Sending command: {command}")
        self.uart.reset_input_buffer()  # Clear any pending input
        self.uart.write(f"{command}\r\n".encode())
        await asyncio.sleep(wait_time)  # Give ESP time to respond
        
        response = ""
        while self.uart.in_waiting:
            chunk = self.uart.read(1)
            if chunk:
                try:
                    response += chunk.decode()
                except UnicodeError:
                    print("Decode error on byte:", chunk)
        print(f"Response: {response}")
        return response

    async def initialize_esp(self):
        """Initialize ESP with basic AT commands"""
        print("Starting ESP initialization...")
        
        # Try multiple times to establish communication
        for attempt in range(3):
            print(f"Initialization attempt {attempt + 1}")
            
            # Test AT communication
            response = await self.send_at_command("AT", 1)
            if "OK" in response:
                print("ESP responding to AT commands")
                break
            print("No response, retrying...")
            await asyncio.sleep(1)
        else:
            print("Failed to communicate with ESP")
            return False

        # Reset module
        print("Resetting ESP...")
        await self.send_at_command("AT+RST", 2)
        await asyncio.sleep(2)  # Give it time to restart

        # Test communication again after reset
        response = await self.send_at_command("AT", 1)
        if "OK" not in response:
            print("ESP not responding after reset")
            return False

        # Disable echo
        await self.send_at_command("ATE0")
        
        return True

    async def start_ap(self, ssid="RoboEyes"):
        """Start ESP8266 in Access Point mode"""
        try:
            # First initialize the ESP
            if not await self.initialize_esp():
                print("Failed to initialize ESP")
                return False

            print("Setting up Access Point...")
            
            # Set to AP mode
            response = await self.send_at_command("AT+CWMODE=2", 2)
            if "OK" not in response:
                print("Failed to set AP mode")
                return False

            # Configure AP
            cmd = f'AT+CWSAP="{ssid}","robotics123",1,3'
            response = await self.send_at_command(cmd, 5)
            if "OK" not in response:
                print("Failed to configure AP")
                return False

            # Enable multiple connections
            response = await self.send_at_command("AT+CIPMUX=1", 1)
            if "OK" not in response:
                print("Failed to enable multiple connections")
                return False

            # Start TCP server
            response = await self.send_at_command(f"AT+CIPSERVER=1,{self.port}", 1)
            if "OK" not in response:
                print("Failed to start TCP server")
                return False

            # Get AP IP address
            response = await self.send_at_command("AT+CIFSR", 1)
            print(f"AP Info: {response}")

            self.is_active = True
            print("WiFi AP and TCP server started successfully")
            return True

        except Exception as e:
            print(f"Error during AP setup: {e}")
            return False

    async def process_uart_data(self):
        """Process incoming UART data from ESP8266"""
        while self.is_active:
            if self.uart.in_waiting:
                try:
                    raw_data = self.uart.read()
                    if raw_data:
                        try:
                            data = raw_data.decode()
                            self.buffer += data
                            print(f"Buffer: {self.buffer}")
                            # Process complete messages
                            while '\n' in self.buffer:
                                try:
                                    line, self.buffer = self.buffer.split('\n', 1)
                                    await self.handle_esp_message(line.strip())
                                except Exception as e:
                                    print(f"Error processing message line: {e}")
                                    # Reset buffer if it becomes corrupted
                                    if len(self.buffer) > 1024:
                                        print("Buffer overflow, resetting")
                                        self.buffer = ""
                                except ValueError:
                                    pass
                        except UnicodeDecodeError as e:
                            print(f"Decode error: {e}, skipping bytes")
                            # Skip corrupted data
                except Exception as e:
                    print(f"UART read error: {e}")
                    await asyncio.sleep(0.1)  # Give UART time to recover
            
            await asyncio.sleep(0.01)

    async def handle_websocket_handshake(self, conn_id, headers):
        """Handle WebSocket handshake"""
        try:
            # Extract WebSocket key from headers
            key = None
            for line in headers.split('\r\n'):
                if 'Sec-WebSocket-Key:' in line:
                    key = line.split(': ')[1].strip()
                    break
            
            if not key:
                print("No WebSocket key found")
                return False

            # Generate WebSocket accept key
            GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
            accept_key = base64.b64encode(
                hashlib.sha1((key + GUID).encode()).digest()
            ).decode()

            # Create handshake response
            response = (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept_key}\r\n"
                "\r\n"
            )

            # Send handshake response
            length = len(response)
            await self.send_at_command(f"AT+CIPSEND={conn_id},{length}")
            self.uart.write(response.encode())

            # Store client connection
            self.websocket_clients[conn_id] = {'connected': True}
            print(f"WebSocket client {conn_id} connected")
            return True

        except Exception as e:
            print(f"Handshake error: {e}")
            return False

    def decode_websocket_frame(self, data):
        """Decode WebSocket frame"""
        try:
            if len(data) < 2:
                return None

            # First byte contains FIN bit and opcode
            first_byte = data[0]
            fin = (first_byte & 0b10000000) != 0
            opcode = first_byte & 0b00001111

            # Second byte contains MASK bit and payload length
            second_byte = data[1]
            mask = (second_byte & 0b10000000) != 0
            payload_length = second_byte & 0b01111111

            # Get index where payload data starts
            data_start = 2
            if payload_length == 126:
                payload_length = int.from_bytes(data[2:4], 'big')
                data_start = 4
            elif payload_length == 127:
                payload_length = int.from_bytes(data[2:10], 'big')
                data_start = 10

            # Get masking key if present
            if mask:
                mask_key = data[data_start:data_start + 4]
                data_start += 4
            else:
                mask_key = None

            # Get payload data
            payload = data[data_start:data_start + payload_length]

            # Unmask data if necessary
            if mask and mask_key:
                unmasked = bytearray(payload_length)
                for i in range(payload_length):
                    unmasked[i] = payload[i] ^ mask_key[i % 4]
                payload = unmasked

            return {
                'fin': fin,
                'opcode': opcode,
                'payload': payload
            }

        except Exception as e:
            print(f"Frame decode error: {e}")
            return None

    def encode_websocket_frame(self, payload, opcode=0x01):
        """Encode data into WebSocket frame"""
        try:
            payload_length = len(payload)
            
            # Create first byte (FIN bit + opcode)
            first_byte = 0b10000000 | opcode  # FIN bit set + text frame

            # Create second byte (payload length)
            if payload_length <= 125:
                frame = bytearray([first_byte, payload_length])
            elif payload_length <= 65535:
                frame = bytearray([first_byte, 126]) + payload_length.to_bytes(2, 'big')
            else:
                frame = bytearray([first_byte, 127]) + payload_length.to_bytes(8, 'big')

            # Add payload
            frame.extend(payload)
            return frame

        except Exception as e:
            print(f"Frame encode error: {e}")
            return None

    async def handle_esp_message(self, message):
        """Handle messages from ESP8266"""
        print(f"Raw ESP message received: {message}")  # Debug raw message
        
        if "+IPD" not in message:
            return

        try:
            # Parse IPD message
            ipd_parts = message.split(',', 2)
            if len(ipd_parts) < 3:
                print(f"Invalid IPD format: not enough parts in {message}")
                return

            conn_id = ipd_parts[1].strip()
            length_and_data = ipd_parts[2].split(':', 1)
            if len(length_and_data) < 2:
                print(f"Invalid IPD format: no colon in {message}")
                return

            data = length_and_data[1]
            print(f"Parsed data: {data}")  # Debug parsed data

            # Check if this is a WebSocket handshake request
            if "GET" in data and "Upgrade: websocket" in data:
                success = await self.handle_websocket_handshake(conn_id, data)
                print(f"Handshake {'successful' if success else 'failed'}")
                return

            # Handle WebSocket frame for existing connections
            if conn_id in self.websocket_clients:
                print(f"Processing WebSocket frame for client {conn_id}")  # Debug
                frame = self.decode_websocket_frame(data.encode())
                if frame:
                    print(f"Decoded frame: {frame}")  # Debug frame contents
                    if frame['opcode'] == 0x08:  # Close frame
                        self.websocket_clients.pop(conn_id)
                        print(f"WebSocket client {conn_id} disconnected")
                    elif frame['opcode'] == 0x01:  # Text frame
                        try:
                            json_data = json.loads(frame['payload'])
                            print(f"Received JSON: {json_data}")  # Debug JSON
                            await self.process_json_command(conn_id, json_data)
                        except json.JSONDecodeError as e:
                            print(f"JSON parse error: {e}")
                            print(f"Raw payload: {frame['payload']}")
                else:
                    print(f"Failed to decode WebSocket frame from: {data}")

        except Exception as e:
            print(f"Error handling message: {e}")
            import traceback
            traceback.print_exc()

    async def process_json_command(self, conn_id, data):
        """Process received JSON commands"""
        try:
            print(f"Processing command for client {conn_id}: {data}")  # Debug
            message_type = data.get('type')
            if message_type == 'command':
                command = data.get('command')
                if command == 'move_eye':
                    x = data.get('x')
                    y = data.get('y')
                    print(f"Move eye command: x={x}, y={y}")
                    # Send response back to client
                    print(f"Sending response to client {conn_id}")  # Debug
                    await self.send_response(conn_id, {"status": "ok", "message": "Eye moved"})
                elif command == 'blink':
                    print("Blink command received")
                    print(f"Sending response to client {conn_id}")  # Debug
                    await self.send_response(conn_id, {"status": "ok", "message": "Blink executed"})
            else:
                print(f"Unknown message type: {message_type}")
                await self.send_response(conn_id, {"status": "error", "message": "Unknown command type"})
                
        except Exception as e:
            print(f"Error processing command: {e}")
            import traceback
            traceback.print_exc()
            await self.send_response(conn_id, {"status": "error", "message": str(e)})

    async def send_response(self, conn_id, data):
        """Send JSON response through WebSocket"""
        try:
            if conn_id not in self.websocket_clients:
                print(f"Client {conn_id} not found in websocket_clients")
                return

            # Convert data to JSON and encode as WebSocket frame
            json_str = json.dumps(data)
            print(f"Sending response to {conn_id}: {json_str}")  # Debug
            frame = self.encode_websocket_frame(json_str.encode())
            
            if frame:
                length = len(frame)
                print(f"Sending frame of length {length}")  # Debug
                await self.send_at_command(f"AT+CIPSEND={conn_id},{length}")
                await asyncio.sleep(0.1)  # Small delay to ensure ESP is ready
                self.uart.write(frame)
                print("Response sent")  # Debug
            else:
                print("Failed to encode WebSocket frame")

        except Exception as e:
            print(f"Error sending response: {e}")
            import traceback
            traceback.print_exc()

    def stop(self):
        """Stop the WiFi AP"""
        if self.is_active:
            self.uart.write("AT+CIPSERVER=0\r\n".encode())  # Stop TCP server
            self.uart.write("AT+CWMODE=0\r\n".encode())     # Disable WiFi
            self.is_active = False
            print("WiFi AP stopped")
