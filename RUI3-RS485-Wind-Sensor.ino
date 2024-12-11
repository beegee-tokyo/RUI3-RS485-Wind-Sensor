/**
 * @file main.cpp
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief Modbus Master reading data from environment sensors
 * @version 0.1
 * @date 2024-11-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "app.h"

/** Data array for modbus 9 registers */
union coils_n_regs_u coils_n_regs = {0, 0, 0, 0, 0, 0, 0, 0, 0};

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI
 *               or any pin number > 1 for RS-485
 */
Modbus master(0, Serial1, 0); // this is master and RS-232 or USB-FTDI

/** This is an structure which contains a query to an slave device */
modbus_t telegram;

/** This is the structure which contains a write to set/reset coils */
struct coil_s
{
	int8_t dev_addr = 1;
	int8_t coils[16];
	int8_t num_coils = 0;
};

/** This is the structure to write to specific registers in the ModBus slave */
struct register_s
{
	int8_t dev_addr = 1;
	int8_t registers[16];
	int8_t num_registers = 0;
	int16_t register_start_address = 0;
};

/** Coils structure */
coil_s coil_data;

/** Register structure */
register_s register_data;

/** Flag if write is for coils or for registers */
bool is_registers = false;

/** Packet is confirmed/unconfirmed (Set with AT commands) */
bool g_confirmed_mode = false;
/** If confirmed packet, number or retries (Set with AT commands) */
uint8_t g_confirmed_retry = 0;
/** Data rate  (Set with AT commands) */
uint8_t g_data_rate = 3;

/** fPort to send packages */
uint8_t set_fPort = 2;

/** Payload buffer */
WisCayenne g_solution_data(255);

/**
 * @brief Callback after join request cycle
 *
 * @param status Join result
 */
void joinCallback(int32_t status)
{
	if (status != 0)
	{
		MYLOG("JOIN-CB", "LoRaWan OTAA - join fail! \r\n");
		// To be checked if this makes sense
		// api.lorawan.join();
	}
	else
	{
		MYLOG("JOIN-CB", "LoRaWan OTAA - joined! \r\n");
		digitalWrite(LED_BLUE, LOW);
	}
}

/**
 * @brief LoRaWAN callback after packet was received
 *
 * @param data pointer to structure with the received data
 */
void receiveCallback(SERVICE_LORA_RECEIVE_T *data)
{
	MYLOG("RX-CB", "RX, port %d, DR %d, RSSI %d, SNR %d", data->Port, data->RxDatarate, data->Rssi, data->Snr);
	for (int i = 0; i < data->BufferSize; i++)
	{
		Serial.printf("%02X", data->Buffer[i]);
	}
	Serial.print("\r\n");
}

/**
 * @brief LoRaWAN callback after TX is finished
 *
 * @param status TX status
 */
void sendCallback(int32_t status)
{
	MYLOG("TX-CB", "TX status %d", status);
	digitalWrite(LED_BLUE, LOW);
}

/**
 * @brief LoRa P2P callback if a packet was received
 *
 * @param data pointer to the data with the received data
 */
void recv_cb(rui_lora_p2p_recv_t data)
{
	MYLOG("RX-P2P-CB", "P2P RX, RSSI %d, SNR %d", data.Rssi, data.Snr);
	for (int i = 0; i < data.BufferSize; i++)
	{
		Serial.printf("%02X", data.Buffer[i]);
	}
	Serial.print("\r\n");
}

/**
 * @brief LoRa P2P callback if a packet was sent
 *
 */
void send_cb(void)
{
	MYLOG("TX-P2P-CB", "P2P TX finished");
	digitalWrite(LED_BLUE, LOW);
}

/**
 * @brief LoRa P2P callback for CAD result
 *
 * @param result true if activity was detected, false if no activity was detected
 */
void cad_cb(bool result)
{
	MYLOG("CAD-P2P-CB", "P2P CAD reports %s", result ? "activity" : "no activity");
}

/**
 * @brief Arduino setup, called once after reboot/power-up
 *
 */
void setup()
{
	// Setup for LoRaWAN
	if (api.lorawan.nwm.get() == 1)
	{
		g_confirmed_mode = api.lorawan.cfm.get();

		g_confirmed_retry = api.lorawan.rety.get();

		g_data_rate = api.lorawan.dr.get();

		// Setup the callbacks for joined and send finished
		api.lorawan.registerRecvCallback(receiveCallback);
		api.lorawan.registerSendCallback(sendCallback);
		api.lorawan.registerJoinCallback(joinCallback);
	}
	else // Setup for LoRa P2P
	{
		api.lora.registerPRecvCallback(recv_cb);
		api.lora.registerPSendCallback(send_cb);
		api.lora.registerPSendCADCallback(cad_cb);
	}

	pinMode(LED_GREEN, OUTPUT);
	digitalWrite(LED_GREEN, HIGH);
	pinMode(LED_BLUE, OUTPUT);
	digitalWrite(LED_BLUE, HIGH);

	Serial.begin(115200);

	// Delay for 5 seconds to give the chance for AT+BOOT
	delay(5000);
	api.system.firmwareVersion.set("RUI3-Soil-Sensor-V1.0.0");
	Serial.println("RAKwireless RUI3 Soil Sensor");
	Serial.println("------------------------------------------------------");
	Serial.println("Setup the device with WisToolBox or AT commands before using it");
	Serial.printf("Ver %s\n", api.system.firmwareVersion.get().c_str());
	Serial.println("------------------------------------------------------");

	// Register the custom AT command to get device status
	if (!init_status_at())
	{
		MYLOG("SETUP", "Add custom AT command STATUS fail");
	}

	// Register the custom AT command to set the send interval
	if (!init_interval_at())
	{
		MYLOG("SETUP", "Add custom AT command Send Interval fail");
	}

	// Get saved sending interval from flash
	get_at_setting();

	digitalWrite(LED_GREEN, LOW);

	// Initialize the Modbus interface on Serial1 (connected to RAK5802 RS485 module)
	pinMode(WB_IO2, OUTPUT);
	digitalWrite(WB_IO2, HIGH);
	Serial1.begin(4800); // baud-rate at 4800
	master.start();
	master.setTimeOut(2000); // if there is no answer in 2000 ms, roll over

	// Create a timer for interval reading of sensor from Modbus slave.
	api.system.timer.create(RAK_TIMER_0, modbus_read_register, RAK_TIMER_PERIODIC);
	if (custom_parameters.send_interval != 0)
	{
		// Start a timer.
		api.system.timer.start(RAK_TIMER_0, custom_parameters.send_interval, NULL);
	}

	// Check if it is LoRa P2P
	if (api.lorawan.nwm.get() == 0)
	{
		digitalWrite(LED_BLUE, LOW);
		MYLOG("SETUP", "P2P mode, start a reading");
		modbus_read_register(NULL);
	}
	else
	{
		// // Shut down 12V supply and RS485
		// digitalWrite(WB_IO2, LOW);
		// Serial1.end();
		// udrv_serial_deinit(SERIAL_UART1);
	}

	if (api.lorawan.nwm.get() == 1)
	{
		if (g_confirmed_mode)
		{
			MYLOG("SETUP", "Confirmed enabled");
		}
		else
		{
			MYLOG("SETUP", "Confirmed disabled");
		}

		MYLOG("SETUP", "Retry = %d", g_confirmed_retry);

		MYLOG("SETUP", "DR = %d", g_data_rate);
	}

	// Enable low power mode
	api.system.lpm.set(1);

	// If available, enable BLE advertising for 30 seconds and open the BLE UART channel
#if defined(_VARIANT_RAK3172_) || defined(_VARIANT_RAK3172_SIP_)
// No BLE
#else
	Serial6.begin(115200, RAK_AT_MODE);
	api.ble.advertise.start(30);
#endif
}

/**
 * @brief Read ModBus registers
 * 		Reads first 9 registers with the sensor data
 * 		If sensor data could be retrieved, the sensor data is sent over LoRa/LoRaWAN
 *
 */
void modbus_read_register(void *)
{
	time_t start_poll;
	bool data_ready;

	Serial1.begin(9600);
	delay(500);
	MYLOG("MODR", "Send read request over ModBus");
	// Clear data structure
	coils_n_regs.data[0] = coils_n_regs.data[1] = coils_n_regs.data[2] = coils_n_regs.data[3] = coils_n_regs.data[4] = 0xFFFF;
	coils_n_regs.data[5] = coils_n_regs.data[6] = coils_n_regs.data[7] = coils_n_regs.data[8] = 0xFFFF;
	// Setup read command
	telegram.u8id = 1;					   // slave address
	telegram.u8fct = MB_FC_READ_REGISTERS; // function code (this one is registers read)
	telegram.u16RegAdd = 0;				   // start address in slave
	telegram.u16CoilsNo = 5;			   // number of elements (coils or registers) to read
	telegram.au16reg = coils_n_regs.data;  // pointer to a memory array in the Arduino

	// Send query (only once)
	master.query(telegram);

	start_poll = millis();
	data_ready = false;
	// Wait for slave response for 5 seconds
	while ((millis() - start_poll) < 5000)
	{
		// Check incoming messages
		master.poll();
		// Status idle, either data was received or the slave response timed out
		if (master.getState() == COM_IDLE)
		{
			// Check if register structure has changed
			if ((coils_n_regs.data[0] == 0xFFFF) && (coils_n_regs.data[1] == 0xFFFF) && (coils_n_regs.data[2] == 0xFFFF) && (coils_n_regs.data[3] == 0xFFFF) && (coils_n_regs.data[4] == 0xFFFF))
			{
				MYLOG("MODR", "No data received");
				MYLOG("MODR", "%04X %04X %04X %04X %04X",
					  coils_n_regs.data[0], coils_n_regs.data[1], coils_n_regs.data[2], coils_n_regs.data[3],
					  coils_n_regs.data[4]);
				break;
			}
			else
			{
				MYLOG("MODR", "Windspeed = %.2f", (uint16_t)(coils_n_regs.sensor_data.reg_1) / 100); // Windspeed
				MYLOG("MODR", "Wind direction = %d", (uint16_t)(coils_n_regs.sensor_data.reg_2)); // Wind direction
				MYLOG("MODR", "Temperature = %.2f", (uint16_t)(coils_n_regs.sensor_data.reg_3) / 10.0);  // Temperature
				MYLOG("MODR", "Humidity = %.2f", (uint16_t)(coils_n_regs.sensor_data.reg_4) / 10.0); // Humidity
				MYLOG("MODR", "Barometer = %.2f", (uint16_t)(coils_n_regs.sensor_data.reg_5) / 10.0); // Barometric Pressure

				data_ready = true;

				// Clear payload
				g_solution_data.reset();

				// Add temperature level to payload
				g_solution_data.addTemperature(LPP_CHANNEL_TEMP, coils_n_regs.sensor_data.reg_3 / 10.0);

				// Add humidity level to payload
				g_solution_data.addRelativeHumidity(LPP_CHANNEL_HUMID, (uint16_t)(coils_n_regs.sensor_data.reg_4) / 10.0);

				// Add barometric pressure value to payload
				g_solution_data.addBarometricPressure(LPP_CHANNEL_BARO, (uint16_t)(coils_n_regs.sensor_data.reg_5) / 10.0);

				// Add wind speed value to payload
				g_solution_data.addAnalogOutput(LPP_CHANNEL_WINDS, (uint16_t)(coils_n_regs.sensor_data.reg_1) / 100);

				// Add wind direction to payload
				g_solution_data.addGenericSensor(LPP_CHANNEL_WINDD, (uint16_t)(coils_n_regs.sensor_data.reg_2));

				float battery_reading = 0.0;
				// Add battery voltage
				for (int i = 0; i < 10; i++)
				{
					battery_reading += api.system.bat.get(); // get battery voltage
				}

				battery_reading = battery_reading / 10;

				g_solution_data.addVoltage(LPP_CHANNEL_BATT, battery_reading);

				break;
			}
		}
	}

	if (data_ready)
	{
		// Send the packet if data was received
		send_packet();
	}
	// // Shut down sensors and communication for lowest power consumption
	// digitalWrite(WB_IO2, LOW);
	// Serial1.end();
	// udrv_serial_deinit(SERIAL_UART1);
	digitalWrite(LED_BLUE, LOW);
}

/**
 * @brief This example is complete timer driven.
 * The loop() does nothing than sleep.
 *
 */
void loop(void)
{
	api.system.sleep.all();
}

/**
 * @brief Send the data packet that was prepared in
 * Cayenne LPP format by the different sensor functions
 *
 */
void send_packet(void)
{
	// Check if it is LoRaWAN
	if (api.lorawan.nwm.get() == 1)
	{
		MYLOG("UPLINK", "Sending packet over LoRaWAN with size %d", g_solution_data.getSize());
		uint8_t proposed_dr = get_min_dr(api.lorawan.band.get(), g_solution_data.getSize());
		MYLOG("UPLINK", "Check if datarate allows payload size, proposed is DR %d, current DR is %d", proposed_dr, api.lorawan.dr.get());

		if (proposed_dr == 16)
		{
			MYLOG("UPLINK", "No matching DR found");
		}
		else
		{
			if (proposed_dr < api.lorawan.dr.get())
			{
				MYLOG("UPLINK", "Proposed DR is lower than current selected, switching to lower DR");
				api.lorawan.dr.set(proposed_dr);
			}

			if (proposed_dr > api.lorawan.dr.get())
			{
				MYLOG("UPLINK", "Proposed DR is higher than current selected, switching to higher DR");
				api.lorawan.dr.set(proposed_dr);
			}
		}

		// Send the packet
		if (api.lorawan.send(g_solution_data.getSize(), g_solution_data.getBuffer(), set_fPort, g_confirmed_mode, g_confirmed_retry))
		{
			MYLOG("UPLINK", "Packet enqueued, size %d", g_solution_data.getSize());
		}
		else
		{
			MYLOG("UPLINK", "Send failed");
		}
	}
	// It is P2P
	else
	{
		MYLOG("UPLINK", "Send packet with size %d over P2P", g_solution_data.getSize());

		digitalWrite(LED_BLUE, LOW);

		if (api.lora.psend(g_solution_data.getSize(), g_solution_data.getBuffer(), true))
		{
			MYLOG("UPLINK", "Packet enqueued");
		}
		else
		{
			MYLOG("UPLINK", "Send failed");
		}
	}
}
