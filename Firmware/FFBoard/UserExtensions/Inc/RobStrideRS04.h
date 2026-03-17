/*
 * RobStrideRS04.h
 *
 *  Created on: Mar 7, 2026
 *      Author: Gemini (Industrial & Software Expert)
 */

#ifndef USEREXTENSIONS_INC_ROBSTRIDERS04_H_
#define USEREXTENSIONS_INC_ROBSTRIDERS04_H_

#include "MotorDriver.h"
#include "cpp_target_config.h"
#include "CAN.h"
#include "Encoder.h"
#include "thread.hpp"
#include "CanHandler.h"
#include "CommandHandler.h"
#include "PersistentStorage.h"

#define RS04_THREAD_MEM 512
#define RS04_THREAD_PRIO 28

enum class RS04Protocol : uint8_t { MIT = 0, PRIVATE = 1 };

enum class RS04Commands : uint32_t {
	canid, protocol, maxtorque, connected, errors, temp, rawcan, lasterr,
	readparam, writeparam, savemotor, version, faultbits, enable, stop, setzero=100
};

class RobStrideRS04 : public MotorDriver, public PersistentStorage, public Encoder, public CanHandler, public CommandHandler, cpp_freertos::Thread {
public:
	RobStrideRS04(uint8_t instance);
	virtual ~RobStrideRS04();

	const ClassIdentifier getInfo() = 0;

	// MotorDriver interface
	void turn(int16_t power) override;
	void stopMotor() override;
	void startMotor() override;
	bool motorReady() override;
	Encoder* getEncoder() override { return this; }
	bool hasIntegratedEncoder() override { return true; }

	// Encoder interface
	int32_t getPos() override;
	void setPos(int32_t pos) override;
	float getPos_f() override;
	uint32_t getCpr() override { return 65536; } // 16-bit mapped
	EncoderType getEncoderType() override { return EncoderType::absolute; }

	// CAN Callbacks
	void canRxPendCallback(CANPort* port, CAN_rx_msg& msg) override;
	
	// Thread
	void Run() override;

	// Commands & Storage
	CommandStatus command(const ParsedCommand& cmd, std::vector<CommandReply>& replies) override;
	void saveFlash() override;
	void restoreFlash() override;

	void setCanFilter();

protected:
	void sendTorqueMIT(float torque);
	void sendTorquePrivate(float torque);
	void sendEnablePrivate();
	void sendStopPrivate();
	void sendEnableActiveReporting();
	void sendReadParam(uint16_t index);
	void sendWriteParam(uint16_t index, float value);
	void sendSaveMotor();
	void sendRequestVersion();
	void enterMITMode();
	void exitMITMode();
	
	// Data conversion helpers
	uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
	float uint_to_float(int x_int, float x_min, float x_max, int bits);

	CANPort* canPort = &canport;
	uint8_t motorId = 1;
	uint8_t masterId = 0xFD;
	uint8_t instanceId = 0;
	int32_t filterId = 0;
	RS04Protocol protocol = RS04Protocol::PRIVATE;	
	float maxTorque = 5.0f; // Default 5Nm for safety
	float lastPos = 0;
	float lastVelocity = 0;
	float lastTorqueFeedback = 0;
	float lastTemp = 0;
	
	uint32_t lastMessageTick = 0;
	bool isConnected = false;
	bool isActive = false;
	bool modeSynced = false;

	uint32_t lastRawId = 0;
	uint8_t lastRawData[8] = {0};
	uint32_t lastError = 0; // 0: OK, 1: Wrong ID, 2: Wrong Type (Private), 3: Wrong MasterID (MIT), 4: Wrong MotorID (MIT)
	uint32_t faultBits = 0; // Fault bits from motor
	uint16_t lastReadParamIndex = 0;
	float lastReadParamValue = 0;
	char versionString[16] = "Unknown";

	void registerCommands();
};

class RobStrideRS04_1 : public RobStrideRS04 {
public:
	RobStrideRS04_1() : RobStrideRS04(0) { inUse = true; }
	~RobStrideRS04_1() { inUse = false; }
	const ClassIdentifier getInfo() override { return info; }
	static ClassIdentifier info;
	static bool inUse;
	static bool isCreatable() { return !inUse; }
};

class RobStrideRS04_2 : public RobStrideRS04 {
public:
	RobStrideRS04_2() : RobStrideRS04(1) { inUse = true; }
	~RobStrideRS04_2() { inUse = false; }
	const ClassIdentifier getInfo() override { return info; }
	static ClassIdentifier info;
	static bool inUse;
	static bool isCreatable() { return !inUse; }
};

#endif /* USEREXTENSIONS_INC_ROBSTRIDERS04_H_ */
