#include <Arduino.h>

#include "nRF8001.h"

#include <stdint.h>

#include <assert.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

/*
#ifdef PROGMEM
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))
#endif
*/

nRFDeviceState nRF8001::getDeviceState()
{
	return deviceState;
}

nRFCmd nRF8001::setup(const uint8_t nb_setup_msgs)
{
	hal_aci_data_t cmd;

	int previousMessageSent = -1;
#if DEBUG_LEVEL
	nrf_debugln(F("Waiting for Standby state"));
#endif
	for (;;) {
#if DEBUG_LEVEL
		nrf_debugln(F("Calling transmitReceive..."));
#endif
		transmitReceive(0, 0);
		if (deviceState == nRFSetupState || deviceState == PreSetup) {
			// Start the setup process
			nextSetupMessage = 0;
			break;
		}
		else {
#if DEBUG_LEVEL
			nrf_debugln(F("Received message in setup but device not ready for setup"));
#endif
		}
	}

	for (;;) {
		if (nextSetupMessage >= 0
			&& nextSetupMessage < nb_setup_msgs
			&& nextSetupMessage > previousMessageSent) {
#if DEBUG_LEVEL
			nrf_debugln(F("sending setup message number "));
			nrf_debugln(nextSetupMessage);
#endif
			cmd = setup_msgs(nextSetupMessage);
			transmitReceive((nRFCommand *)(cmd.buffer), 0);
			previousMessageSent = nextSetupMessage;
		}
		else if (nextSetupMessage >= 0
			&& nextSetupMessage > previousMessageSent) {
#if DEBUG_LEVEL
			nrf_debugln(F("we ran out of setup messages!"));
#endif
			deviceState = Invalid;
			return cmdSetupError;
		}
		else if (nextSetupMessage == -1) {
#if DEBUG_LEVEL
			nrf_debugln(F("setup is complete!"));
#endif
		}

		if (deviceState == Standby) {
#if DEBUG_LEVEL
			nrf_debugln(F("device in Standby state, returning"));
#endif
			return cmdSuccess;
		}

		transmitReceive(0, 0);
	}

}

nRF8001::nRF8001(uint8_t reset_pin_arg,
	uint8_t reqn_pin_arg,
	uint8_t rdyn_pin_arg)
{
#if DEBUG_LEVEL
	nrf_debugln(F("Initializing"));
#endif
	// Initialize data structures
	reset_pin = reset_pin_arg;
	reqn_pin = reqn_pin_arg;
	rdyn_pin = rdyn_pin_arg;

	deviceState = Initial;
	credits = 0;
	nextSetupMessage = -2;
	connectionStatus = Disconnected;

	// Zero all the handlers
	listener = 0;
	commandResponseHandler = 0;
	temperatureHandler = 0;
	batteryLevelHandler = 0;
	deviceVersionHandler = 0;
	deviceAddressHandler = 0;
	dynamicDataHandler = 0;
	connectedHandler = 0;
	disconnectedHandler = 0;
	bondStatusHandler = 0;
	keyRequestHandler = 0;
	pipeErrorHandler = 0;
	dataReceivedHandler = 0;
	dataAckHandler = 0;

	// Prepare pins and start SPI
	pinMode(reqn_pin, OUTPUT);
	pinMode(rdyn_pin, INPUT);
	digitalWrite(rdyn_pin, HIGH);
	digitalWrite(reqn_pin, HIGH);

	// Only do this if reset_pin is not -1
	if (reset_pin != -1) {
		pinMode(reset_pin, OUTPUT);
		digitalWrite(reset_pin, LOW);
		delayMicroseconds(1);
		digitalWrite(reset_pin, HIGH);
	}

	// inialize SPI
	pinMode(SCK, OUTPUT);
	pinMode(MOSI, OUTPUT);
	pinMode(SS, OUTPUT);

	digitalWrite(SCK, LOW);
	digitalWrite(MOSI, LOW);
	digitalWrite(SS, HIGH);

	// SPI mode 0; /16 clock divider
	SPI.setDataMode(SPI_MODE0);
#ifndef tinySPI_h
	SPI.setBitOrder(LSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV16);
#endif
	SPI.begin();

	// Load up the first setup message and start interrupts
}

void nRF8001::addressToString(char *str, uint8_t *address)
{
	for (int i = NRF_ADDRESS_LENGTH - 1; i >= 0; i--) {
		uint8_t c = address[i];
		sprintf(str + (NRF_ADDRESS_LENGTH - 1 - i) * 2, "%02x", c);
	}
}

void nRF8001::debugAddress(uint8_t *address)
{
#if DEBUG_LEVEL
	nrf_debugln(F("0x"));
	// Addresses are NRF_ADDRESS_LENGTH long, MSB to LSB
	char buf[NRF_ADDRESS_LENGTH * 2 + 1];
	addressToString(buf, address);
	nrf_debugln(buf);
#endif
}

hal_aci_data_t nRF8001::setup_msgs(const uint8_t i)
{
	hal_aci_data_t res;
	res.status_byte = pgm_read_byte_near(SETUP_MESSAGES_CONTENTS[i].status_byte);
	for (uint8_t k = 0; k < 32; ++k)
	{
		res.buffer[k] = pgm_read_byte_near(SETUP_MESSAGES_CONTENTS[i].buffer + k);
	}
	return res;
}

void nRF8001::debugEvent(nRFEvent *event)
{
#if DEBUG_LEVEL
	nrf_debugln(F("EVENT debug="));
	nrf_debugln(event->debug);
	nrf_debugln(F(" length="));
	nrf_debugln(event->length);
	nrf_debugln(F(" event="));

	switch (event->event) {
	case NRF_DEVICESTARTEDEVENT:
		nrf_debugln(F("DeviceStartedEvent"));

		nrf_debugln(F("Operating mode: "));
		switch (event->msg.deviceStarted.operatingMode) {
		case 0x01:
			nrf_debugln(F("Test"));
			break;
		case 0x02:
			nrf_debugln(F("Setup"));
			break;
		case 0x03:
			nrf_debugln(F("Standby"));
			break;
		default:
			nrf_debugln(event->msg.deviceStarted.operatingMode, HEX);
			break;
		}

		nrf_debugln(F("Hardware error: "));
		switch (event->msg.deviceStarted.hwError) {
		case 0x00:
			nrf_debugln(F("No error"));
			break;
		case 0x01:
			nrf_debugln(F("Fatal error"));
			break;
		default:
			nrf_debugln(event->msg.deviceStarted.hwError, HEX);
			break;
		}

		nrf_debugln(F("DataCreditAvailable: "));
		nrf_debugln(event->msg.deviceStarted.dataCreditAvailable);
		break;
	case NRF_ECHOEVENT:
		nrf_debugln(F("EchoEvent"));
		break;
	case NRF_HARDWAREERROREVENT:
		nrf_debugln(F("HardwareErrorEvent!  Line: "));
		nrf_debugln(event->msg.hardwareError.lineNo);
		nrf_debugln(F(", File: "));
		nrf_debugln((char *)event->msg.hardwareError.fileName);
		break;
	case NRF_COMMANDRESPONSEEVENT:
		nrf_debugln(F("CommandResponseEvent"));

		nrf_debugln(F("Status: "));
		switch (event->msg.commandResponse.status) {
		case NRF_STATUS_SUCCESS:
			nrf_debugln(F("Success"));
			break;
		case NRF_STATUS_TRANSACTION_CONTINUE:
			nrf_debugln(F("Transaction continuation status"));
			break;
		case NRF_STATUS_TRANSACTION_COMPLETE:
			nrf_debugln(F("Transaction completed"));
			break;
		case NRF_STATUS_EXTENDED:
			nrf_debugln(F("Extended status, further checks needed"));
			break;
		case NRF_STATUS_ERROR_UNKNOWN:
			nrf_debugln(F("Unknown error"));
			break;
		case NRF_STATUS_ERROR_INTERNAL:
			nrf_debugln(F("Internal error"));
			break;
		case NRF_STATUS_ERROR_CMD_UNKNOWN:
			nrf_debugln(F("Unknown command"));
			break;
		case NRF_STATUS_ERROR_DEVICE_STATE_INVALID:
			nrf_debugln(F("Command invalid in the current device state"));
			break;
		case NRF_STATUS_ERROR_INVALID_LENGTH:
			nrf_debugln(F("Invalid length"));
			break;
		case NRF_STATUS_ERROR_INVALID_PARAMETER:
			nrf_debugln(F("Invalid input parameters"));
			break;
		case NRF_STATUS_ERROR_BUSY:
			nrf_debugln(F("Busy"));
			break;
		case NRF_STATUS_ERROR_INVALID_DATA:
			nrf_debugln(F("Invalid data format or contents"));
			break;
		case NRF_STATUS_ERROR_CRC_MISMATCH:
			nrf_debugln(F("CRC mismatch"));
			break;
		case NRF_STATUS_ERROR_UNSUPPORTED_SETUP_FORMAT:
			nrf_debugln(F("Unsupported setup format"));
			break;
		case NRF_STATUS_ERROR_INVALID_SEQ_NO:
			nrf_debugln(F("Invalid sequence number during a write dynamic data sequence"));
			break;
		case NRF_STATUS_ERROR_SETUP_LOCKED:
			nrf_debugln(F("Setup data is locked and cannot be modified"));
			break;
		case NRF_STATUS_ERROR_LOCK_FAILED:
			nrf_debugln(F("Setup error due to lock verification failure"));
			break;
		case NRF_STATUS_ERROR_BOND_REQUIRED:
			nrf_debugln(F("Bond required: Local service pipes need bonded/trusted peer"));
			break;
		case NRF_STATUS_ERROR_REJECTED:
			nrf_debugln(F("Command rejected as a transaction is still pending"));
			break;
		case NRF_STATUS_ERROR_DATA_SIZE:
			nrf_debugln(F("Pipe Error Event: Data size exceeds size specified for pipe, Transmit failed"));
			break;
		case NRF_STATUS_ERROR_PIPE_INVALID:
			nrf_debugln(F("Pipe Error Event: Transmit failed, Invalid or unavailable Pipe number or unknown pipe type"));
			break;
		case NRF_STATUS_ERROR_CREDIT_NOT_AVAILABLE:
			nrf_debugln(F("Pipe Error Event: Credit not available"));
			break;
		case NRF_STATUS_ERROR_PEER_ATT_ERROR:
			nrf_debugln(F("Pipe Error Event: Peer device has sent an error on an pipe operation on the remote characteristic"));
			break;
		case NRF_STATUS_ERROR_ADVT_TIMEOUT:
			nrf_debugln(F("Connection was not established before the BTLE advertising was stopped"));
			break;
		case NRF_STATUS_ERROR_PEER_SMP_ERROR:
			nrf_debugln(F("Remote device triggered a Security Manager Protocol error"));
				break;
		case NRF_STATUS_ERROR_PIPE_TYPE_INVALID:
			nrf_debugln(F("Pipe Error Event: Pipe type invalid for the selected operation"));
			break;
		case NRF_STATUS_ERROR_PIPE_STATE_INVALID:
			nrf_debugln(F("Pipe Error Event: Pipe state invalid for the selected operation"));
			break;
		case NRF_STATUS_ERROR_INVALID_KEY_SIZE:
			nrf_debugln(F("Invalid key size provided"));
			break;
		case NRF_STATUS_ERROR_INVALID_KEY_DATA:
			nrf_debugln(F("Invalid key data provided"));
			break;
		default:
			nrf_debugln(event->msg.commandResponse.status, HEX);
			break;
		}

		nrf_debugln(F("Op: "));
		switch (event->msg.commandResponse.opcode) {
		case NRF_TEST_OP:
			nrf_debugln(F("Test"));
			break;
		case NRF_GETDEVICEVERSION_OP:
			nrf_debugln(F("GetDeviceVersion"));
			nrf_debugln(F("ConfigurationID="));
			nrf_debugln(event->msg.commandResponse.data
				.getDeviceVersion.configurationId);
			nrf_debugln(F(" ACIVersion="));
			nrf_debugln(event->msg.commandResponse.data
				.getDeviceVersion.aciVersion);
			nrf_debugln(F(" SetupFormat="));
			nrf_debugln(event->msg.commandResponse.data
				.getDeviceVersion.setupFormat);
			nrf_debugln(F(" SetupID="));
			nrf_debugln(event->msg.commandResponse.data
				.getDeviceVersion.setupId);
			nrf_debugln(F(" ConfigurationStatus="));
			if (event->msg.commandResponse.data.getDeviceVersion
				.configurationStatus == 1) {
				nrf_debugln(F("SetupLocked"));
			}
			else {
				nrf_debugln(F("SetupOpen"));
			}
			break;
		case NRF_WAKEUP_OP:
			nrf_debugln(F("Wakeup"));
			break;
		case NRF_GETBATTERYLEVEL_OP:
			nrf_debugln(F("GetBatteryLevel"));
			nrf_debugln(event->msg.commandResponse
				.data.voltage*3.52, 2);
			nrf_debugln(F("mV"));
			break;
		case NRF_GETTEMPERATURE_OP:
			nrf_debugln(F("GetTemperature"));
			nrf_debugln(event->msg.commandResponse
				.data.temperature / 4.0, 2);
			nrf_debugln(F(" C"));
			break;
		case NRF_SETUP_OP:
			nrf_debugln(F("Setup"));
			break;
		case NRF_SETTXPOWER_OP:
			nrf_debugln(F("SetTxPower"));
			break;
		case NRF_GETDEVICEADDRESS_OP:
			nrf_debugln(F("GetDeviceAddress"));
			nrf_debugln(F("Address: "));
			debugAddress(event->msg.commandResponse.data
				.getDeviceAddress.deviceAddress);
			nrf_debugln(F("\n"));
			break;
		case NRF_CONNECT_OP:
			nrf_debugln(F("Connect"));
			break;
		case NRF_RADIORESET_OP:
			nrf_debugln(F("RadioReset"));
			break;
		case NRF_BOND_OP:
			nrf_debugln(F("Bond"));
			break;
		case NRF_DISCONNECT_OP:
			nrf_debugln(F("Disconnect"));
			break;
		case NRF_CHANGETIMINGREQUEST_OP:
			nrf_debugln(F("ChangeTimingRequest"));
			break;
		case NRF_OPENREMOTEPIPE_OP:
			nrf_debugln(F("OpenRemotePipe"));
			break;
		case NRF_CLOSEREMOTEPIPE_OP:
			nrf_debugln(F("CloseRemotePipe"));
			break;
		case NRF_DTMCOMMAND_OP:
			nrf_debugln(F("DtmCommand"));
			nrf_debugln(F("DTM data: "));
			nrf_debugln(event->msg.commandResponse.data.dtmEvent,
				HEX);
			break;
		case NRF_READDYNAMICDATA_OP:
			nrf_debugln(F("ReadDynamicData"));
			nrf_debugln(F("Sequence no="));
			nrf_debugln(event->msg.commandResponse.data
				.readDynamicData.sequenceNo);
			nrf_debugln(F("TODO: data here"));
			break;
		case NRF_WRITEDYNAMICDATA_OP:
			nrf_debugln(F("WriteDynamicData"));
			break;
		case NRF_SETAPPLICATIONLATENCY_OP:
			nrf_debugln(F("SetApplLatency"));
			break;
		case NRF_SETKEY_OP:
			nrf_debugln(F("SetKey"));
			break;
		case NRF_OPENADVPIPE_OP:
			nrf_debugln(F("OpenAdvPipe"));
			break;
		case NRF_BROADCAST_OP:
			nrf_debugln(F("Broadcast"));
			break;
		case NRF_BONDSECREQUEST_OP:
			nrf_debugln(F("BondSecurityRequest"));
			break;
		case NRF_DIRECTEDCONNECT_OP:
			nrf_debugln(F("DirectedConnect"));
			break;
		case NRF_SETLOCALDATA_OP:
			nrf_debugln(F("SetLocalData"));
			break;
		default:
			nrf_debugln(event->msg.commandResponse.opcode, HEX);
			break;
		}
		break;
	case NRF_CONNECTEDEVENT:
		nrf_debugln(F("ConnectedEvent"));
		nrf_debugln(F("Address type: "));
		switch (event->msg.connected.addressType) {
		case 0x01:
			nrf_debugln(F("Public address"));
			break;
		case 0x02:
			nrf_debugln(F("Random static address"));
			break;
		case 0x03:
			nrf_debugln(F("Random private address (resolvable)"));
			break;
		case 0x04:
			nrf_debugln(F("Random private address (non-resolvable)"));
			break;
		default:
			nrf_debugln(event->msg.connected.addressType, HEX);
			break;
		}

		nrf_debugln(F("Peer address: "));
		debugAddress(event->msg.connected.peerAddress);
		nrf_debugln(F("\n"));

		nrf_debugln(F("Connection interval: "));
		nrf_debugln(event->msg.connected.connectionInterval / 1.25, 2);
		nrf_debugln(F("ms"));

		nrf_debugln(F("Slave latency: "));
		nrf_debugln(event->msg.connected.slaveLatency);

		nrf_debugln(F("Supervision timeout: "));
		nrf_debugln(event->msg.connected.supervisionTimeout);

		nrf_debugln(F("Master clock accuracy: "));
		switch (event->msg.connected.masterClockAccuracy) {
		case 0x00:
			nrf_debugln(F("500 ppm"));
			break;
		case 0x01:
			nrf_debugln(F("250 ppm"));
			break;
		case 0x02:
			nrf_debugln(F("150 ppm"));
			break;
		case 0x03:
			nrf_debugln(F("100 ppm"));
			break;
		case 0x04:
			nrf_debugln(F("75 ppm"));
			break;
		case 0x05:
			nrf_debugln(F("50 ppm"));
			break;
		case 0x06:
			nrf_debugln(F("30 ppm"));
			break;
		case 0x07:
			nrf_debugln(F("20 ppm"));
			break;
		default:
			nrf_debugln(event->msg.connected.masterClockAccuracy,
				HEX);
			break;
		}
		break;
	case NRF_DISCONNECTEDEVENT: {
		nrf_debugln(F("DisconnectedEvent"));

		nrf_debugln(F("ACI status: "));
		switch (event->msg.disconnected.aciStatus) {
		case 0x03:
			nrf_debugln(
				F("Check the Bluetooth low energy status code"));
			break;
		case 0x93:
			nrf_debugln(F(
				"Timeout while advertising, "
				"unable to establish connection"));
			break;
		case 0x8d:
			nrf_debugln(F(
				"Bond required to proceed with connection"));
			break;
		default:
			nrf_debugln(event->msg.disconnected.aciStatus, HEX);
			break;
		}

		nrf_debugln(F("BtLeStatus: 0x"));
		uint8_t btLeStatus = event->msg.disconnected.btLeStatus;
		if (btLeStatus < 0x10) {
			nrf_debugln(F("0"));
		}
		nrf_debugln(btLeStatus, HEX);
		break;
	}
	case NRF_BONDSTATUSEVENT:
		nrf_debugln(F("BondStatusEvent"));
		nrf_debugln(F("TODO: bond status data"));
		break;
	case NRF_PIPESTATUSEVENT:
		nrf_debugln(F("PipeStatusEvent"));

		nrf_debugln(F("Open: "));
		for (int i = 0; i < 64; i++) {
			if (event->msg.pipeStatus.pipesOpen & ((uint64_t)1) << i) {
				nrf_debugln(F(" "));
				nrf_debugln(i, DEC);
			}
		}
		nrf_debugln(F("\n"));

		nrf_debugln(F("Closed: "));
		for (int i = 0; i < 64; i++) {
			if (event->msg.pipeStatus.pipesClosed & ((uint64_t)1) << i) {
				nrf_debugln(F(" "));
				nrf_debugln(i, DEC);
			}
		}
		nrf_debugln(F("\n"));
		break;
	case NRF_TIMINGEVENT:
		nrf_debugln(F("TimingEvent"));

		nrf_debugln(F("Connection interval: "));
		nrf_debugln(event->msg.timing.connectionInterval / 1.25, 2);

		nrf_debugln(F("Slave latency: "));
		nrf_debugln(event->msg.timing.slaveLatency, 2);

		nrf_debugln(F("Supervision timeout: "));
		nrf_debugln(event->msg.timing.supervisionTimeout*10.0, 2);
		nrf_debugln(F("ms"));
		break;
	case NRF_DISPLAYKEYEVENT:
		nrf_debugln(F("DisplayKeyEvent"));
		nrf_debugln(F("Passkey: "));
		for (int i = 0; i < NRF_PASSKEY_LENGTH; i++) {
			nrf_debugln(event->msg.passkey[i]);
		}
		nrf_debugln(F("\n"));
		break;
	case NRF_KEYREQUESTEVENT:
		nrf_debugln(F("KeyRequestEvent"));
		break;
	case NRF_DATACREDITEVENT:
		nrf_debugln(F("DataCreditEvent credits="));
		nrf_debugln(event->msg.dataCredits);
		break;
	case NRF_PIPEERROREVENT:
		nrf_debugln(F("PipeErrorEvent"));
		nrf_debugln(F("Pipe: "));
		nrf_debugln(event->msg.pipeError.servicePipeNo);

		nrf_debugln(F("Error code: 0x"));
		nrf_debugln(event->msg.pipeError.errorCode, HEX);
		break;
	case NRF_DATARECEIVEDEVENT:
		nrf_debugln(F("DataReceivedEvent"));
		nrf_debugln(F("Pipe: "));
		nrf_debugln(event->msg.dataReceived.servicePipeNo);
		break;
	case NRF_DATAACKEVENT:
		nrf_debugln(F("DataAckEvent"));
		nrf_debugln(F("Pipe: "));
		nrf_debugln(event->msg.servicePipeNo);
		break;
	default:
		nrf_debugln(F("Unknown "));
		nrf_debugln(event->event);
		break;
	}
#endif
}

// Transmit a command, and simultaneously receive a message from nRF8001 if
// there is one. To just receive without transmitting anything, call this
// function with a NULL argument.
nRFTxStatus nRF8001::transmitReceive(nRFCommand *txCmd, uint16_t timeout)
{
	// Buffer that we will receive into
	uint8_t rxBuffer[sizeof(nRFEvent)];
	nRFEvent *rxEvent = (nRFEvent *)rxBuffer;
	memset(&rxBuffer, 0, sizeof(nRFEvent));

	uint8_t *txBuffer = (uint8_t *)txCmd;

	// Transmit length
	uint8_t txLength, txCommand;
	if (txCmd != NULL) {
		txLength = txCmd->length;
		txCommand = txCmd->command;
	}
	else {
		txLength = 0;
		txCommand = 0;
	}

	assert(txLength <= NRF_MAX_PACKET_LENGTH);

	// Enough credits?
	if (txLength &&
		(txCmd->command == NRF_SENDDATA_OP
			|| txCmd->command == NRF_REQUESTDATA_OP
			//|| txCmd->command == NRF_SETLOCALDATA_OP
			|| txCmd->command == NRF_SENDDATAACK_OP
			|| txCmd->command == NRF_SENDDATANACK_OP)) {
		if (credits < 1) {
#if DEBUG_LEVEL
			nrf_debugln(F("transmitReceive fail, not enough credits"));
#endif
			return InsufficientCredits;
		}

		// Use a credit
		credits--;
	}

	// Bring REQN low
	if (txLength > 0) {
		digitalWrite(reqn_pin, LOW);

#if DEBUG_LEVEL > 1
		nrf_debugln(F("transmitReceive: called with transmission, command "));
		nrf_debugln(txCommand);
#endif

	}
	else {
#if DEBUG_LEVEL > 1
		nrf_debugln(F("transmitReceive: called in polling mode"));
#endif

	}

	// TODO: Timeout

	if (txLength > 0 || timeout == 0) {
		// Wait for RDYN low indefinitely
#if DEBUG_LEVEL > 1
		nrf_debugln(F("waiting for RDYN low indefinitely"));
#endif

		while (digitalRead(rdyn_pin) == HIGH);
	}
	else {
#if DEBUG_LEVEL > 1
		nrf_debugln(F("polling for "));
		nrf_debugln(timeout);
		nrf_debugln(F("ms"));
#endif

		uint8_t rdy = 0;
		// Wait for RDYN low for 1 ms at a time
		for (uint16_t waitPeriods = 0; waitPeriods < timeout; waitPeriods++) {
			if (digitalRead(rdyn_pin) == LOW) {
				rdy = 1;
				break;
			}
			else {
				delay(1);
			}
		}

		if (!rdy) {
			return Timeout;
		}
	}

	if (txLength == 0) {
		digitalWrite(reqn_pin, LOW);
	}

#if DEBUG_LEVEL > 1
	nrf_debugln(F("Ready to transmit & receive full duplex!"));
#endif


	// Send length and command bytes,
	// receive debug and length bytes
	rxEvent->debug = SPI.transfer(txLength);
	rxEvent->length = SPI.transfer(txCommand);

	assert(rxEvent->length <= NRF_MAX_PACKET_LENGTH);

	// nextByte points to the next byte to be transferred in
	// txBuffer, or the next byte to be received in rxBuffer
	// For TX, packets are data + 1 byte (length)
	// For RX, packets are data + 2 bytes (debug and length)
	for (uint8_t nextByte = 2;
	nextByte < txLength + 1 || nextByte < rxEvent->length + 2;
		nextByte++) {
		uint8_t c;
		if (nextByte < txLength + 1) {
			c = SPI.transfer(txBuffer[nextByte]); // transmit
		}
		else {
			c = SPI.transfer(0); // receive only
		}

		if (nextByte < rxEvent->length + 2) { // receive
			rxBuffer[nextByte] = c;
		}
	}

	// Bring REQN high
	digitalWrite(reqn_pin, HIGH);
	// Wait for RDYN high
	while (digitalRead(rdyn_pin) == LOW);

	// Return immediately if we didn't receive anything
	if (!rxEvent->length) {
		return Success;
	}

	// Handle response
	switch (rxEvent->event) {
	case NRF_DEVICESTARTEDEVENT:
		credits = rxEvent->msg.deviceStarted.dataCreditAvailable;

		switch (rxEvent->msg.deviceStarted.operatingMode) {
		case 0x01:
			deviceState = Test;
			break;
		case 0x02:
			deviceState = nRFSetupState;
			break;
		case 0x03:
			if (deviceState == Initial) {
				// Before we are setup, Nordic calls it "Standby",
				// but that's different from post-setup Standby
				deviceState = PreSetup;
			}
			else {
				deviceState = Standby;
			}
		}
		break;
	case NRF_COMMANDRESPONSEEVENT: {
		if (rxEvent->msg.commandResponse.status != 0x00) {
#if DEBUG_LEVEL
			nrf_debugln(F("non-success command response event: 0x"));
			nrf_debugln(rxEvent->msg.commandResponse.status);
#endif
			if (commandResponseHandler != 0) {
				commandResponseHandler(
					rxEvent->msg.commandResponse.opcode,
					rxEvent->msg.commandResponse.status);
			}

			if (rxEvent->msg.commandResponse.opcode == NRF_SETUP_OP &&
				rxEvent->msg.commandResponse.status ==
				NRF_STATUS_TRANSACTION_CONTINUE) {
				nextSetupMessage++;
#if DEBUG_LEVEL
				nrf_debugln(F("ready for next setup message"));
#endif
			}
			else if (rxEvent->msg.commandResponse.opcode == NRF_SETUP_OP &&
				rxEvent->msg.commandResponse.status ==
				NRF_STATUS_TRANSACTION_COMPLETE) {
#if DEBUG_LEVEL
				nrf_debugln(F("setup complete"));
#endif
				nextSetupMessage = -1;
			}
			break;
		}

		switch (rxEvent->msg.commandResponse.opcode) {
			// We only do handling of some of these messages
		case NRF_SETUP_OP:
			break;
		case NRF_GETTEMPERATURE_OP:
			if (temperatureHandler != 0) {
				temperatureHandler(rxEvent->msg.commandResponse
					.data.temperature / 4.0);
			}
			break;
		case NRF_GETBATTERYLEVEL_OP:
			if (batteryLevelHandler != 0) {
				batteryLevelHandler(rxEvent->msg.commandResponse
					.data.voltage * 0.00352);
			}
			break;
		case NRF_GETDEVICEVERSION_OP:
			if (deviceVersionHandler != 0) {
				deviceVersionHandler(
					rxEvent->msg.commandResponse
					.data.getDeviceVersion.configurationId,
					rxEvent->msg.commandResponse
					.data.getDeviceVersion.aciVersion,
					rxEvent->msg.commandResponse
					.data.getDeviceVersion.setupFormat,
					rxEvent->msg.commandResponse
					.data.getDeviceVersion.configurationStatus);
			}
			break;
		case NRF_GETDEVICEADDRESS_OP:
			if (deviceAddressHandler != 0) {
				deviceAddressHandler(
					rxEvent->msg.commandResponse
					.data.getDeviceAddress.deviceAddress,
					rxEvent->msg.commandResponse
					.data.getDeviceAddress.addressType);
			}
			break;
		case NRF_READDYNAMICDATA_OP:
			if (dynamicDataHandler != 0) {
				dynamicDataHandler(
					rxEvent->msg.commandResponse
					.data.readDynamicData.sequenceNo,
					rxEvent->msg.commandResponse
					.data.readDynamicData.dynamicData);
			}
			break;
		default:
			if (commandResponseHandler != 0) {
				commandResponseHandler(
					rxEvent->msg.commandResponse.opcode,
					rxEvent->msg.commandResponse.status);
			}
			break;
		}

		// Dispatch event
		break;
	}
	case NRF_CONNECTEDEVENT:
		connectionStatus = Connected;
		if (connectedHandler != 0) {
			connectedHandler(
				rxEvent->msg.connected.addressType,
				rxEvent->msg.connected.peerAddress,
				0);
			// TODO: put in other data
		}
		break;
	case NRF_DISCONNECTEDEVENT:
		connectionStatus = Disconnected;
		pipesOpen = 0;
		if (disconnectedHandler != 0) {
			disconnectedHandler(
				rxEvent->msg.disconnected.aciStatus,
				rxEvent->msg.disconnected.btLeStatus);
		}
		break;
	case NRF_DATACREDITEVENT:
		credits += rxEvent->msg.dataCredits;
		break;
	case NRF_PIPESTATUSEVENT:
		pipesOpen = rxEvent->msg.pipeStatus.pipesOpen;
		break;
	case NRF_BONDSTATUSEVENT:
		if (bondStatusHandler != 0) {
			bondStatusHandler(0);
		}
		// TODO: put in bond status data
		break;
	case NRF_KEYREQUESTEVENT:
		if (keyRequestHandler != 0) {
			keyRequestHandler(rxEvent->msg.keyType);
		}
		break;
	case NRF_PIPEERROREVENT:
		if (pipeErrorHandler != 0) {
			pipeErrorHandler(
				rxEvent->msg.pipeError.servicePipeNo,
				rxEvent->msg.pipeError.errorCode,
				rxEvent->msg.pipeError.rawData);
		}
		break;
	case NRF_DATARECEIVEDEVENT:
		if (dataReceivedHandler != 0) {
			dataReceivedHandler(
				rxEvent->msg.dataReceived.servicePipeNo,
				rxEvent->msg.dataReceived.data);
		}
		break;
	case NRF_DATAACKEVENT:
		if (dataAckHandler != 0) {
			dataAckHandler(rxEvent->msg.servicePipeNo);
		}
		break;
	case NRF_HARDWAREERROREVENT:
		break;
	default: {
		break;
	}
	}

	// Dispatch event
	if (listener != 0) {
		// first to generic handler
		listener(rxEvent);
	}

	return Success;
}

// Informational functions

uint8_t nRF8001::creditsAvailable()
{
	return credits;
}

uint8_t nRF8001::isConnected() {
	return connectionStatus == Connected;
}

nRFConnectionStatus nRF8001::getConnectionStatus()
{
	return connectionStatus;
}

// Receive functions

nRFTxStatus nRF8001::poll(const uint16_t timeout)
{
	return transmitReceive(0, timeout);
}

uint8_t nRF8001::isPipeOpen(nRFPipe servicePipeNo)
{
	return (pipesOpen & ((uint64_t)1) << servicePipeNo) != 0;
}

// Transmit functions

nRFTxStatus nRF8001::transmitCommand(uint8_t command)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

#if DEBUG_LEVEL
	nrf_debugln(F("calling transmitCommand"));
#endif

	nRFCommand cmd;
	cmd.length = 1;
	cmd.command = command;
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::transmitPipeCommand(uint8_t command, nRFPipe pipe)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

#if DEBUG_LEVEL
	nrf_debugln(F("calling transmitPipeCommand"));
#endif

	nRFCommand cmd;
	cmd.length = 2;
	cmd.command = command;
	cmd.content.servicePipeNo = pipe;
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::test(uint8_t feature)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

#if DEBUG_LEVEL
	nrf_debugln(F("calling test"));
#endif

	nRFCommand cmd;
	cmd.length = 2;
	cmd.command = NRF_TEST_OP;
	cmd.content.testFeature = feature;
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::sleep()
{
	return transmitCommand(NRF_SLEEP_OP);
}

nRFTxStatus nRF8001::echo(nRFLen dataLength, uint8_t *data)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	if (dataLength > NRF_MAX_ECHO_MESSAGE_LENGTH) {
#if DEBUG_LEVEL
		nrf_debugln(F("data too long"));
#endif
		return DataTooLong;
	}

#if DEBUG_LEVEL
	nrf_debugln(F("calling echo"));
#endif

	nRFCommand cmd;
	cmd.length = dataLength + 1;
	cmd.command = NRF_ECHO_OP;
	memcpy(cmd.content.echoData, data, dataLength);
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::wakeup()
{
	return transmitCommand(NRF_WAKEUP_OP);
}

nRFTxStatus nRF8001::getBatteryLevel()
{
	return transmitCommand(NRF_GETBATTERYLEVEL_OP);
}

nRFTxStatus nRF8001::getTemperature()
{
	return transmitCommand(NRF_GETTEMPERATURE_OP);
}

nRFTxStatus nRF8001::setTxPower(uint8_t powerLevel)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	nRFCommand cmd;
	cmd.length = 2;
	cmd.command = NRF_SETTXPOWER_OP;
	cmd.content.radioTxPowerLevel = powerLevel;
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::getDeviceAddress()
{
	return transmitCommand(NRF_GETDEVICEADDRESS_OP);
}

nRFTxStatus nRF8001::getDeviceVersion()
{
	return transmitCommand(NRF_GETDEVICEVERSION_OP);
}

nRFTxStatus nRF8001::connect(uint16_t timeout, uint16_t advInterval)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	connectionStatus = Connecting;

#if DEBUG_LEVEL
	nrf_debugln(F("connecting"));
#endif

	nRFCommand cmd;
	cmd.length = 5;
	cmd.command = NRF_CONNECT_OP;
	cmd.content.connect.timeout = timeout;
	cmd.content.connect.advInterval = advInterval;
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::radioReset()
{
#if DEBUG_LEVEL
	nrf_debugln(F("sending radio reset"));
#endif
	nRFCommand cmd;
	cmd.length = 1;
	cmd.command = NRF_RADIORESET_OP;
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::bond(uint16_t timeout, uint16_t advInterval)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	nRFCommand cmd;
	cmd.length = 5;
	cmd.command = NRF_BOND_OP;
	cmd.content.bond.timeout = timeout;
	cmd.content.bond.advInterval = advInterval;
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::disconnect(uint8_t reason)
{
	if (deviceState != Standby || connectionStatus != Connected) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in standby and connected state"));
#endif
		return InvalidState;
	}

	nRFCommand cmd;
	cmd.length = 2;
	cmd.command = NRF_DISCONNECT_OP;
	cmd.content.disconnectReason = reason;
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::changeTimingRequest(uint16_t intervalMin,
	uint16_t intervalMax,
	uint16_t slaveLatency,
	uint16_t timeout)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	nRFCommand cmd;
	cmd.command = NRF_CHANGETIMINGREQUEST_OP;

	if (intervalMin > 0 || intervalMax > 0
		|| slaveLatency > 0 || timeout >> 0) {
		cmd.length = 9;
		cmd.content.changeTimingRequest.intervalMin = intervalMin;
		cmd.content.changeTimingRequest.intervalMax = intervalMax;
		cmd.content.changeTimingRequest.slaveLatency = slaveLatency;
		cmd.content.changeTimingRequest.timeout = timeout;
	}
	else {
		cmd.length = 1;
	}

	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::openRemotePipe(nRFPipe servicePipeNo)
{
	return transmitPipeCommand(NRF_OPENREMOTEPIPE_OP, servicePipeNo);
}

nRFTxStatus nRF8001::closeRemotePipe(nRFPipe servicePipeNo)
{
	return transmitPipeCommand(NRF_CLOSEREMOTEPIPE_OP, servicePipeNo);
}

nRFTxStatus nRF8001::dtmCommand(uint16_t dtmCmd)
{
	nRFCommand cmd;
	cmd.length = 3;
	cmd.command = NRF_DTMCOMMAND_OP;
	cmd.content.dtmCommand = dtmCmd;
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::readDynamicData()
{
	return transmitCommand(NRF_READDYNAMICDATA_OP);
}

nRFTxStatus nRF8001::writeDynamicData(uint8_t seqNo, nRFLen dataLength,
	uint8_t *data)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	nRFCommand cmd;
	cmd.length = dataLength + 2;
	cmd.command = NRF_WRITEDYNAMICDATA_OP;
	cmd.content.writeDynamicData.sequenceNo = seqNo;
	memcpy(cmd.content.writeDynamicData.dynamicData, data, dataLength);
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::setApplLatency(uint8_t applLatencyMode, uint16_t latency)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	nRFCommand cmd;
	cmd.length = 4;
	cmd.command = NRF_SETAPPLICATIONLATENCY_OP;
	cmd.content.setApplLatency.applLatencyMode = applLatencyMode;
	cmd.content.setApplLatency.latency = latency;
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::setKey(uint8_t keyType, uint8_t *key)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	nRFCommand cmd;
	cmd.command = NRF_SETKEY_OP;
	cmd.content.setKey.keyType = keyType;

	if (keyType == 0) {
		cmd.length = 2;
	}
	else if (keyType == 1) {
		cmd.length = 2 + NRF_PASSKEY_LENGTH;
		memcpy(cmd.content.setKey.key, key, NRF_PASSKEY_LENGTH);
	}
	else {
		return InvalidParameter;
	}

	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::openAdvPipe(uint64_t advServiceDataPipes)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	nRFCommand cmd;
	cmd.length = 9;
	cmd.command = NRF_OPENADVPIPE_OP;
	cmd.content.advServiceDataPipes = advServiceDataPipes;

	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::broadcast(uint16_t timeout, uint16_t advInterval)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	nRFCommand cmd;
	cmd.length = 5;
	cmd.command = NRF_BROADCAST_OP;
	cmd.content.broadcast.timeout = timeout;
	cmd.content.broadcast.advInterval = advInterval;

	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::bondSecurityRequest()
{
	return transmitCommand(NRF_BONDSECREQUEST_OP);
}

nRFTxStatus nRF8001::directedConnect()
{
	return transmitCommand(NRF_DIRECTEDCONNECT_OP);
}

nRFTxStatus nRF8001::sendData(nRFPipe servicePipeNo,
	nRFLen dataLength, uint8_t *data)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	if (connectionStatus != Connected) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not connected"));
#endif
		return NotConnected;
	}

	if (!(pipesOpen & ((uint64_t)1) << servicePipeNo)) {
#if DEBUG_LEVEL
		nrf_debugln(F("pipe not open"));
#endif
		return PipeNotOpen;
	}

	if (credits <= 0) {
#if DEBUG_LEVEL
		nrf_debugln(F("not enough credits"));
#endif
		return InsufficientCredits;
	}

	if (dataLength > NRF_DATA_LENGTH) {
#if DEBUG_LEVEL
		nrf_debugln(F("data too long"));
#endif
		return DataTooLong;
	}

	nRFCommand cmd;
	cmd.command = NRF_SENDDATA_OP;
	cmd.length = dataLength + 2;
	cmd.content.data.servicePipeNo = servicePipeNo;
	memcpy(cmd.content.data.data, data, dataLength);
	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::requestData(nRFPipe servicePipeNo)
{
	return transmitPipeCommand(NRF_REQUESTDATA_OP, servicePipeNo);
}

nRFTxStatus nRF8001::setLocalData(nRFPipe servicePipeNo, nRFLen dataLength,
	uint8_t *data)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	nRFCommand cmd;
	cmd.length = 2 + dataLength;
	cmd.command = NRF_SETLOCALDATA_OP;
	cmd.content.data.servicePipeNo = servicePipeNo;

	memcpy(cmd.content.data.data, data, dataLength);

	return transmitReceive(&cmd, 0);
}

nRFTxStatus nRF8001::sendDataAck(nRFPipe servicePipeNo)
{
	return transmitPipeCommand(NRF_SENDDATAACK_OP, servicePipeNo);
}

nRFTxStatus nRF8001::sendDataNack(nRFPipe servicePipeNo, uint8_t errorCode)
{
	if (deviceState != Standby) {
#if DEBUG_LEVEL
		nrf_debugln(F("device not in Standby state"));
#endif
		return InvalidState;
	}

	nRFCommand cmd;
	cmd.length = 3;
	cmd.command = NRF_SENDDATANACK_OP;
	cmd.content.sendDataNack.servicePipeNo = servicePipeNo;
	cmd.content.sendDataNack.errorCode = errorCode;

	return transmitReceive(&cmd, 0);
}

// Event handler registration

void nRF8001::setEventHandler(nRFEventHandler handler)
{
	listener = handler;
}

void nRF8001::setCommandResponseHandler(nRFCommandResponseHandler handler)
{
	commandResponseHandler = handler;
}

void nRF8001::setTemperatureHandler(nRFTemperatureHandler handler)
{
	temperatureHandler = handler;
}

void nRF8001::setBatteryLevelHandler(nRFBatteryLevelHandler handler)
{
	batteryLevelHandler = handler;
}

void nRF8001::setDeviceVersionHandler(nRFDeviceVersionHandler handler)
{
	deviceVersionHandler = handler;
}

void nRF8001::setDeviceAddressHandler(nRFDeviceAddressHandler handler)
{
	deviceAddressHandler = handler;
}

void nRF8001::setDynamicDataHandler(nRFDynamicDataHandler handler)
{
	dynamicDataHandler = handler;
}

void nRF8001::setConnectedHandler(nRFConnectedHandler handler)
{
	connectedHandler = handler;
}

void nRF8001::setDisconnectedHandler(nRFDisconnectedHandler handler)
{
	disconnectedHandler = handler;
}

void nRF8001::setBondStatusHandler(nRFBondStatusHandler handler)
{
	bondStatusHandler = handler;
}

void nRF8001::setKeyRequestHandler(nRFKeyRequestHandler handler)
{
	keyRequestHandler = handler;
}

void nRF8001::setPipeErrorHandler(nRFPipeErrorHandler handler)
{
	pipeErrorHandler = handler;
}

void nRF8001::setDataReceivedHandler(nRFDataReceivedHandler handler)
{
	dataReceivedHandler = handler;
}

void nRF8001::setDataAckHandler(nRFDataAckHandler handler)
{
	dataAckHandler = handler;
}
