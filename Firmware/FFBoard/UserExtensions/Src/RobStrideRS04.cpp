/*
 * RobStrideRS04.cpp
 *
 *  Created on: Mar 7, 2026
 *      Author: Gemini (Industrial & Software Expert)
 */

#include "RobStrideRS04.h"
#include "ClassIDs.h"
#include <algorithm>
#include <cstring>

// --- Static Info ---
bool RobStrideRS04_1::inUse = false;
ClassIdentifier RobStrideRS04_1::info = {
	.name = "RobStride RS04 (Ax1)",
	.id = CLSID_MOT_RS04_1,
};

bool RobStrideRS04_2::inUse = false;
ClassIdentifier RobStrideRS04_2::info = {
	.name = "RobStride RS04 (Ax2)",
	.id = CLSID_MOT_RS04_2,
};

// --- Constructor ---
RobStrideRS04::RobStrideRS04(uint8_t instance) 
	: CommandHandler("rs04", CLSID_MOT_RS04_1, instance),
	  Thread("RS04", RS04_THREAD_MEM, RS04_THREAD_PRIO),
	  instanceId(instance) {
	
	motorId = instance + 1; // Default ID 1 and 2
	restoreFlash();
	
	setCanFilter();
	
	this->registerCommands();
	this->canPort->takePort();
	this->Start();
}

RobStrideRS04::~RobStrideRS04() {
	stopMotor();
	canPort->removeCanFilter(filterId);
	this->canPort->freePort();
}

void RobStrideRS04::setCanFilter() {
	CAN_filter filter;
	// RS04 MIT response is standard frame (11-bit) with ID = motorId
	// RS04 Private response is extended frame (29-bit)
	filter.filter_id = 0; 
	filter.filter_mask = 0; // Receive all to handle both protocols easily
	filter.buffer = instanceId % 2;
	this->filterId = this->canPort->addCanFilter(filter);
}

void RobStrideRS04::registerCommands() {
	CommandHandler::registerCommands();
	registerCommand("canid", (uint32_t)RS04Commands::canid, "Motor CAN ID", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("protocol", (uint32_t)RS04Commands::protocol, "0:MIT, 1:Private", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("maxtorque", (uint32_t)RS04Commands::maxtorque, "Max torque scale (Nm)", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("connected", (uint32_t)RS04Commands::connected, "Connection state", CMDFLAG_GET);
}

// --- Motor Control ---
void RobStrideRS04::turn(int16_t power) {
	if (!isActive) return;
	
	float torque = ((float)power / 32767.0f) * maxTorque;
	
	if (protocol == RS04Protocol::MIT) {
		sendTorqueMIT(torque);
	} else {
		sendTorquePrivate(torque);
	}
}

void RobStrideRS04::startMotor() {
	isActive = true;
	if (protocol == RS04Protocol::MIT) {
		enterMITMode();
	}
}

void RobStrideRS04::stopMotor() {
	isActive = false;
	if (protocol == RS04Protocol::MIT) {
		sendTorqueMIT(0.0f);
		exitMITMode();
	} else {
		sendTorquePrivate(0.0f);
	}
}

bool RobStrideRS04::motorReady() {
	return isConnected;
}

// --- Protocol Implementations ---

void RobStrideRS04::enterMITMode() {
	CAN_tx_msg msg;
	msg.header.id = motorId;
	msg.header.length = 8;
	msg.header.rtr = false;
	msg.header.extId = false;
	memset(msg.data, 0xFF, 7);
	msg.data[7] = 0xFC; // Enable MIT Mode
	canPort->sendMessage(msg);
}

void RobStrideRS04::exitMITMode() {
	CAN_tx_msg msg;
	msg.header.id = motorId;
	msg.header.length = 8;
	msg.header.rtr = false;
	msg.header.extId = false;
	memset(msg.data, 0xFF, 7);
	msg.data[7] = 0xFD; // Exit MIT Mode
	canPort->sendMessage(msg);
}

void RobStrideRS04::sendTorqueMIT(float torque) {
	CAN_tx_msg msg;
	msg.header.id = motorId;
	msg.header.length = 8;
	msg.header.extId = false;

	// MIT Frame: P(16), V(12), KP(12), KD(12), T(12)
	// For FFB, we mainly use T and set P, V, KP, KD to 0 or neutral
	uint16_t p_int = float_to_uint(0, -12.5f, 12.5f, 16);
	uint16_t v_int = float_to_uint(0, -45.0f, 45.0f, 12);
	uint16_t kp_int = float_to_uint(0, 0, 500.0f, 12);
	uint16_t kd_int = float_to_uint(0, 0, 5.0f, 12);
	uint16_t t_int = float_to_uint(torque, -12.0f, 12.0f, 12);

	msg.data[0] = p_int >> 8;
	msg.data[1] = p_int & 0xFF;
	msg.data[2] = v_int >> 4;
	msg.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
	msg.data[4] = kp_int & 0xFF;
	msg.data[5] = kd_int >> 4;
	msg.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
	msg.data[7] = t_int & 0xFF;

	canPort->sendMessage(msg);
}

void RobStrideRS04::sendTorquePrivate(float torque) {
	CAN_tx_msg msg;
	// RS04 Private Protocol: 0x10050100 + ID (Extended)
	// Command for Current Control
	msg.header.id = 0x10050100 | motorId; 
	msg.header.extId = true;
	msg.header.length = 4;
	
	int32_t current_ma = (int32_t)(torque * 1000.0f); // Simplistic torque to current mapping
	memcpy(msg.data, &current_ma, 4);
	
	canPort->sendMessage(msg);
}

// --- Helpers ---
uint16_t RobStrideRS04::float_to_uint(float x, float x_min, float x_max, int bits) {
	// Clamping input to prevent bit wrapping (Crucial for safety)
	if (x > x_max) x = x_max;
	if (x < x_min) x = x_min;
	float span = x_max - x_min;
	float offset = x_min;
	return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float RobStrideRS04::uint_to_float(int x_int, float x_min, float x_max, int bits) {
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// --- Data Feedback ---
void RobStrideRS04::canRxPendCallback(CANPort* port, CAN_rx_msg& msg) {
	lastMessageTick = HAL_GetTick();
	isConnected = true;

	if (!msg.header.extId) { // MIT Response
		if (msg.header.id != motorId) return;
		
		// Parse MIT feedback
		uint16_t p_int = (msg.data[1] << 8) | msg.data[2];
		uint16_t v_int = (msg.data[3] << 4) | (msg.data[4] >> 4);
		uint16_t t_int = ((msg.data[4] & 0xF) << 8) | msg.data[5];
		
		lastPos = uint_to_float(p_int, -12.5f, 12.5f, 16);
		lastVelocity = uint_to_float(v_int, -45.0f, 45.0f, 12);
		lastTorqueFeedback = uint_to_float(t_int, -12.0f, 12.0f, 12);
		lastTemp = msg.data[6];
	} else { // Private Protocol Response
		if ((msg.header.id & 0xFF) != motorId) return;
		// Handle private status frames here
	}
}

void RobStrideRS04::Run() {
	while (true) {
		if (HAL_GetTick() - lastMessageTick > 500) {
			isConnected = false;
		}
		Delay(100);
	}
}

// --- Encoder & Storage Implementation ---
int32_t RobStrideRS04::getPos() {
	return (int32_t)(lastPos * 10430.378f); // rad to counts (65536 / 2pi)
}

float RobStrideRS04::getPos_f() {
	return lastPos;
}

void RobStrideRS04::setPos(int32_t pos) {
	// RS04 supports zeroing via command
}

void RobStrideRS04::restoreFlash() {
	uint16_t val = 0;
	if (Flash_Read(ADR_AXIS1_CONFIG + 20, &val)) { // Use offset from known address
		protocol = (RS04Protocol)(val & 0x1);
		motorId = (val >> 8) & 0xFF;
	}
	if (Flash_Read(ADR_AXIS1_CONFIG + 21, &val)) {
		maxTorque = (float)val / 100.0f;
		if (maxTorque > 12.0f) maxTorque = 12.0f; // Limit to MIT protocol max physical range
	}
}

void RobStrideRS04::saveFlash() {
	uint16_t val = ((uint16_t)motorId << 8) | (uint8_t)protocol;
	Flash_Write(ADR_AXIS1_CONFIG + 20, val);
	Flash_Write(ADR_AXIS1_CONFIG + 21, (uint16_t)(maxTorque * 100.0f));
}

CommandStatus RobStrideRS04::command(const ParsedCommand& cmd, std::vector<CommandReply>& replies) {
	switch ((RS04Commands)cmd.cmdId) {
	case RS04Commands::canid:
		handleGetSet(cmd, replies, motorId);
		break;
	case RS04Commands::protocol:
		if (cmd.type == CMDtype::set) protocol = (RS04Protocol)cmd.val;
		else replies.emplace_back((uint8_t)protocol);
		break;
	case RS04Commands::maxtorque:
		if (cmd.type == CMDtype::set) {
			maxTorque = (float)cmd.val / 100.0f;
			if (maxTorque > 12.0f) maxTorque = 12.0f; // Max clamp for safety
		} else {
			replies.emplace_back((uint32_t)(maxTorque * 100.0f));
		}
		break;
	case RS04Commands::connected:
		replies.emplace_back(isConnected ? 1 : 0);
		break;
	default:
		return CommandStatus::NOT_FOUND;
	}
	return CommandStatus::OK;
}
