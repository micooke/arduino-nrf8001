#ifndef _NRF8001_H
#define _NRF8001_H

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 2
#endif

#if defined (__AVR_ATtiny85__)
#ifndef SS
#define SS 3
#define MOSI 0
#define MISO 1
#define SCK 2
#endif
#include <tinySPI.h>
#else
#include <SPI.h>
#endif

typedef uint8_t nRFLen;
typedef uint8_t nRFPipe;

#include "constants.h"
#include "data.h" // data structures for requests and responses
#include "services.h"

#define NRF_RX_BUFFERS 5

#if DEBUG_LEVEL
#ifndef nrf_debug
	#ifndef uart
		#if defined (__AVR_ATtiny85__)
			#include <SoftwareSerialWithHalfDuplex.h>
			SoftwareSerialWithHalfDuplex ss(5,5);
			#define uart ss
		#else
			#define uart Serial
		#endif
	#endif
	#define nrf_debug uart.print
	#define nrf_debugln uart.println
	#else
	#define nrf_debug 
	#define nrf_debugln 
#endif
#endif

// event handlers
typedef void(*nRFEventHandler) (nRFEvent *);
typedef void(*nRFCommandResponseHandler) (uint8_t opcode, uint8_t status);
typedef void(*nRFTemperatureHandler) (float tempC);
typedef void(*nRFBatteryLevelHandler) (float voltage);
typedef void(*nRFDeviceVersionHandler) (uint16_t configId,
	uint8_t aciVersion, uint8_t setupFormat, uint8_t configStatus);
typedef void(*nRFDeviceAddressHandler) (uint8_t *address,
	uint8_t addressType);
typedef void(*nRFDynamicDataHandler) (uint8_t seqNo, uint8_t *data);
typedef void(*nRFConnectedHandler) (uint8_t addressType, uint8_t *peerAddress,
	void *connectionData);
typedef void(*nRFDisconnectedHandler) (uint8_t aciStatus, uint8_t btLeStatus);
typedef void(*nRFBondStatusHandler) (void *bondStatusData);
typedef void(*nRFKeyRequestHandler) (uint8_t keyType);
typedef void(*nRFPipeErrorHandler) (nRFPipe servicePipeNo,
	uint8_t errorCode, uint8_t *errorData);
typedef void(*nRFDataReceivedHandler) (nRFPipe servicePipeNo,
	uint8_t *data);
typedef void(*nRFDataAckHandler) (nRFPipe servicePipeNo);

class nRF8001
{
private:
	uint64_t pipesOpen;
	uint8_t reset_pin;
	uint8_t reqn_pin;
	uint8_t rdyn_pin;
	nRFEventHandler listener;
	uint8_t credits;
	nRFDeviceState deviceState;
	int8_t nextSetupMessage;
	nRFConnectionStatus connectionStatus;

	nRFTxStatus transmitReceive(nRFCommand *txCmd, uint16_t timeout);
	nRFTxStatus transmitCommand(uint8_t command);
	nRFTxStatus transmitPipeCommand(uint8_t command, nRFPipe pipe);

	nRFCommandResponseHandler commandResponseHandler;
	nRFTemperatureHandler temperatureHandler;
	nRFBatteryLevelHandler batteryLevelHandler;
	nRFDeviceVersionHandler deviceVersionHandler;
	nRFDeviceAddressHandler deviceAddressHandler;
	nRFDynamicDataHandler dynamicDataHandler;
	nRFConnectedHandler connectedHandler;
	nRFDisconnectedHandler disconnectedHandler;
	nRFBondStatusHandler bondStatusHandler;
	nRFKeyRequestHandler keyRequestHandler;
	nRFPipeErrorHandler pipeErrorHandler;
	nRFDataReceivedHandler dataReceivedHandler;
	nRFDataAckHandler dataAckHandler;

	hal_aci_data_t setup_msgs(const uint8_t i);

public:
	void debugEvent(nRFEvent *event);//USED
	void debugAddress(uint8_t *address);
	void addressToString(char *str, uint8_t *address);

	nRFTxStatus poll(const uint16_t timeout = 0);//USED
	nRFDeviceState getDeviceState();
	nRFCmd setup(const uint8_t nb_setup_msgs);//USED

	nRF8001(uint8_t reset_pin,
		uint8_t reqn_pin,
		uint8_t rdyn_pin);//USED

	uint8_t creditsAvailable();
	uint8_t isConnected();
	nRFConnectionStatus getConnectionStatus();//USED

	uint8_t isPipeOpen(nRFPipe servicePipeNo);//USED

	nRFTxStatus test(uint8_t feature);
	nRFTxStatus sleep();
	nRFTxStatus getDeviceVersion();
	nRFTxStatus echo(nRFLen dataLength, uint8_t *data);
	nRFTxStatus wakeup();
	nRFTxStatus getBatteryLevel();
	nRFTxStatus getTemperature();//USED
	nRFTxStatus setTxPower(uint8_t powerLevel);
	nRFTxStatus getDeviceAddress();//USED
	nRFTxStatus connect(uint16_t timeout, uint16_t advInterval);//USED - UINT8_T
	nRFTxStatus radioReset();
	nRFTxStatus bond(uint16_t timeout, uint16_t advInterval);
	nRFTxStatus disconnect(uint8_t reason);
	nRFTxStatus changeTimingRequest(uint16_t intervalMin,
		uint16_t intervalMax,
		uint16_t slaveLatency,
		uint16_t timeout);
	nRFTxStatus openRemotePipe(nRFPipe servicePipeNo);
	nRFTxStatus closeRemotePipe(nRFPipe servicePipeNo);
	nRFTxStatus dtmCommand(uint16_t dtmCmd);
	nRFTxStatus readDynamicData();
	nRFTxStatus writeDynamicData(uint8_t seqNo,
		nRFLen dataLength,
		uint8_t *data);
	nRFTxStatus setApplLatency(uint8_t applLatencyMode,
		uint16_t latency);
	nRFTxStatus setKey(uint8_t keyType, uint8_t *key);
	nRFTxStatus openAdvPipe(uint64_t advServiceDataPipes);
	nRFTxStatus broadcast(uint16_t timeout, uint16_t advInterval);
	nRFTxStatus bondSecurityRequest();
	nRFTxStatus directedConnect();
	nRFTxStatus sendData(nRFPipe servicePipeNo,
		nRFLen dataLength,
		uint8_t *data);//USED
	nRFTxStatus requestData(nRFPipe servicePipeNo);
	nRFTxStatus setLocalData(nRFPipe servicePipeNo,
		nRFLen dataLength,
		uint8_t *data);
	nRFTxStatus sendDataAck(nRFPipe servicePipeNo);
	nRFTxStatus sendDataNack(nRFPipe servicePipeNo,
		uint8_t errorCode);

	void setEventHandler(nRFEventHandler handler);//USED
	void setCommandResponseHandler(nRFCommandResponseHandler handler);
	void setTemperatureHandler(nRFTemperatureHandler handler);//USED
	void setBatteryLevelHandler(nRFBatteryLevelHandler handler);
	void setDeviceVersionHandler(nRFDeviceVersionHandler handler);
	void setDeviceAddressHandler(nRFDeviceAddressHandler handler);
	void setDynamicDataHandler(nRFDynamicDataHandler handler);
	void setConnectedHandler(nRFConnectedHandler handler);
	void setDisconnectedHandler(nRFDisconnectedHandler handler);
	void setBondStatusHandler(nRFBondStatusHandler handler);
	void setKeyRequestHandler(nRFKeyRequestHandler handler);
	void setPipeErrorHandler(nRFPipeErrorHandler handler);
	void setDataReceivedHandler(nRFDataReceivedHandler handler);
	void setDataAckHandler(nRFDataAckHandler handler);
};

#endif /* _NRF8001_H */
