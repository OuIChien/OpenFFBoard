/*
 * RobStrideRS04.cpp
 *
 *  Created on: Mar 7, 2026
 *      Author: Gemini (Industrial & Software Expert)
 */

#include "RobStrideRS04.h"
#include "ClassIDs.h"
#include "eeprom_addresses.h"
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
        : CommandHandler("rs04", instance == 0 ? CLSID_MOT_RS04_1 : CLSID_MOT_RS04_2, instance),
          Thread("RS04", 2048, RS04_THREAD_PRIO),
          instanceId(instance) {

        motorId = instance + 1; 
        masterId = 0xFD; 
        protocol = RS04Protocol::PRIVATE; // Default to Private
        restoreFlash();

        setCanFilter();
        this->registerCommands();

        this->Start();
}

RobStrideRS04::~RobStrideRS04() {
        stopMotor();
        canPort->removeCanFilter(filterId);
        this->canPort->freePort();
}

void RobStrideRS04::setCanFilter() {
        CAN_filter filter;
        filter.buffer = instanceId % 2;
        
        // Disable filtering to see all packets for debugging
        filter.filter_id = 0; 
        filter.filter_mask = 0; 
        filter.extid = true; 
        
        this->filterId = this->canPort->addCanFilter(filter);
}

void RobStrideRS04::registerCommands() {
        CommandHandler::registerCommands();
        registerCommand("canid", (uint32_t)RS04Commands::canid, "Motor CAN ID", CMDFLAG_GET | CMDFLAG_SET);
        registerCommand("protocol", (uint32_t)RS04Commands::protocol, "0:MIT, 1:Private", CMDFLAG_GET | CMDFLAG_SET);
        registerCommand("maxtorque", (uint32_t)RS04Commands::maxtorque, "Max torque scale (Nm)", CMDFLAG_GET | CMDFLAG_SET);
        registerCommand("connected", (uint32_t)RS04Commands::connected, "Connection state", CMDFLAG_GET);
        registerCommand("temp", (uint32_t)RS04Commands::temp, "Motor temperature", CMDFLAG_GET);
        registerCommand("rawcan", (uint32_t)RS04Commands::rawcan, "Last CAN ID received", CMDFLAG_GET);
        registerCommand("lasterr", (uint32_t)RS04Commands::lasterr, "Last rejection reason", CMDFLAG_GET);
        registerCommand("readparam", (uint32_t)RS04Commands::readparam, "Read param by index", CMDFLAG_GET | CMDFLAG_SET);
        registerCommand("writeparam", (uint32_t)RS04Commands::writeparam, "Write param (index:val)", CMDFLAG_SET);
        registerCommand("savemotor", (uint32_t)RS04Commands::savemotor, "Save to motor EEPROM", CMDFLAG_SET);
        registerCommand("version", (uint32_t)RS04Commands::version, "Motor version", CMDFLAG_GET);
        registerCommand("faultbits", (uint32_t)RS04Commands::faultbits, "Detailed fault bits", CMDFLAG_GET);
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
        } else {
                sendEnablePrivate();
                sendEnableActiveReporting();
        }
}

void RobStrideRS04::stopMotor() {
        isActive = false;
        if (protocol == RS04Protocol::MIT) {
                sendTorqueMIT(0.0f);
                exitMITMode();
        } else {
                sendStopPrivate();
        }
}

bool RobStrideRS04::motorReady() {
        return isConnected;
}

// --- Protocol Implementations ---

void RobStrideRS04::sendEnablePrivate() {
        CAN_tx_msg msg;
        msg.header.id = (3 << 24) | (uint32_t)masterId << 8 | (uint32_t)motorId;
        msg.header.extId = true;
        msg.header.length = 8;
        memset(msg.data, 0, 8);
        canPort->sendMessage(msg);
}

void RobStrideRS04::sendStopPrivate() {
        CAN_tx_msg msg;
        msg.header.id = (4 << 24) | (uint32_t)masterId << 8 | (uint32_t)motorId;
        msg.header.extId = true;
        msg.header.length = 8;
        memset(msg.data, 0, 8);
        canPort->sendMessage(msg);
}

void RobStrideRS04::sendEnableActiveReporting() {
        if (protocol != RS04Protocol::PRIVATE) return;
        CAN_tx_msg msg;
        msg.header.id = (24 << 24) | (uint32_t)masterId << 8 | (uint32_t)motorId;
        msg.header.extId = true;
        msg.header.length = 8;
        memset(msg.data, 0, 8);
        // Manual Page 38 indicates F_CMD is at index 6 (7th byte) for Type 24
        msg.data[6] = 0x01; 
        canPort->sendMessage(msg);
}

void RobStrideRS04::sendReadParam(uint16_t index) {
        CAN_tx_msg msg;
        msg.header.id = (17 << 24) | (uint32_t)masterId << 8 | (uint32_t)motorId; 
        msg.header.extId = true;
        msg.header.length = 8;
        memset(msg.data, 0, 8);
        msg.data[0] = index & 0xFF;
        msg.data[1] = (index >> 8) & 0xFF;
        canPort->sendMessage(msg);
}

void RobStrideRS04::sendWriteParam(uint16_t index, float value) {
        CAN_tx_msg msg;
        msg.header.id = (18 << 24) | (uint32_t)masterId << 8 | (uint32_t)motorId;
        msg.header.extId = true;
        msg.header.length = 8;
        memset(msg.data, 0, 8);
        msg.data[0] = index & 0xFF;
        msg.data[1] = (index >> 8) & 0xFF;
        
        // Data format depends on parameter. Most 0x7000 are float IEEE-754
        memcpy(&msg.data[4], &value, 4);
        canPort->sendMessage(msg);
}

void RobStrideRS04::sendSaveMotor() {
        CAN_tx_msg msg;
        msg.header.id = (22 << 24) | (uint32_t)masterId << 8 | (uint32_t)motorId;
        msg.header.extId = true;
        msg.header.length = 8;
        for(int i=0; i<8; i++) msg.data[i] = i+1; // 01 02 ... 08
        canPort->sendMessage(msg);
}

void RobStrideRS04::sendRequestVersion() {
        CAN_tx_msg msg;
        msg.header.id = (26 << 24) | (uint32_t)masterId << 8 | (uint32_t)motorId;
        msg.header.extId = true;
        msg.header.length = 8;
        memset(msg.data, 0, 8);
        canPort->sendMessage(msg);
}

void RobStrideRS04::sendTorquePrivate(float torque) {
        CAN_tx_msg msg;
        uint16_t t_int = float_to_uint(torque, -120.0f, 120.0f, 16);
        // Type 1: bits 8-23 are torque, bits 0-7 are motor ID.
        msg.header.id = (1 << 24) | (uint32_t)t_int << 8 | motorId;
        msg.header.extId = true;
        msg.header.length = 8;

        uint16_t p_int = float_to_uint(0, -12.57f, 12.57f, 16);
        uint16_t v_int = float_to_uint(0, -15.0f, 15.0f, 16);
        uint16_t kp_int = float_to_uint(0, 0, 5000.0f, 16);
        uint16_t kd_int = float_to_uint(0, 0, 100.0f, 16);

        msg.data[0] = p_int >> 8;
        msg.data[1] = p_int & 0xFF;
        msg.data[2] = v_int >> 8;
        msg.data[3] = v_int & 0xFF;
        msg.data[4] = kp_int >> 8;
        msg.data[5] = kp_int & 0xFF;
        msg.data[6] = kd_int >> 8;
        msg.data[7] = kd_int & 0xFF;

        canPort->sendMessage(msg);
}

void RobStrideRS04::enterMITMode() {
        CAN_tx_msg msg;
        msg.header.id = motorId;
        msg.header.length = 8;
        msg.header.extId = false;
        memset(msg.data, 0xFF, 7);
        msg.data[7] = 0xFC; 
        canPort->sendMessage(msg);
}

void RobStrideRS04::exitMITMode() {
        CAN_tx_msg msg;
        msg.header.id = motorId;
        msg.header.length = 8;
        msg.header.extId = false;
        memset(msg.data, 0xFF, 7);
        msg.data[7] = 0xFD; 
        canPort->sendMessage(msg);
}

void RobStrideRS04::sendTorqueMIT(float torque) {
        CAN_tx_msg msg;
        msg.header.id = motorId;
        msg.header.length = 8;
        msg.header.extId = false;

        uint16_t p_int = float_to_uint(0, -12.5f, 12.5f, 16);
        uint16_t v_int = float_to_uint(0, -45.0f, 45.0f, 12);
        uint16_t kp_int = float_to_uint(0, 0, 500.0f, 12);
        uint16_t kd_int = float_to_uint(0, 0, 5.0f, 12);
        uint16_t t_int = float_to_uint(torque, -120.0f, 120.0f, 12);

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

// --- Helpers ---
uint16_t RobStrideRS04::float_to_uint(float x, float x_min, float x_max, int bits) {
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
        lastRawId = msg.header.id;
        if (!msg.header.extId) { 
                // MIT Mode feedback: ID is MasterID (usually 0xFD), Data[0] is Motor ID
                if (msg.header.id != masterId) {
                        lastError = 3; // Wrong MasterID (MIT)
                        return;
                }
                if (msg.data[0] != motorId) {
                        lastError = 4; // Wrong MotorID (MIT)
                        return;
                }
                lastMessageTick = HAL_GetTick();
                isConnected = true;
                lastError = 0;

                uint16_t p_int = (msg.data[1] << 8) | msg.data[2];
                uint16_t v_int = (msg.data[3] << 4) | (msg.data[4] >> 4);
                uint16_t t_int = ((msg.data[4] & 0xF) << 8) | msg.data[5];

                lastPos = uint_to_float(p_int, -12.5f, 12.5f, 16);
                lastVelocity = uint_to_float(v_int, -45.0f, 45.0f, 12);
                lastTorqueFeedback = uint_to_float(t_int, -12.0f, 12.0f, 12);
                
                // Temp * 10 is in bytes 6-7
                int16_t temp_int = (int16_t)(msg.data[6] << 8 | msg.data[7]);
                lastTemp = (float)temp_int / 10.0f;
        } else { 
                uint8_t type = (msg.header.id >> 24) & 0x1F;
                uint8_t senderMotorId = (msg.header.id >> 8) & 0xFF;

                if (senderMotorId == motorId) {
                        lastMessageTick = HAL_GetTick();
                        isConnected = true;
                        lastError = 0;

                        if (type == 1 || type == 2) { 
                                uint16_t p_int = (msg.data[0] << 8) | msg.data[1];
                                uint16_t v_int = (msg.data[2] << 8) | msg.data[3];
                                uint16_t t_int = (msg.data[4] << 8) | msg.data[5];

                                lastPos = uint_to_float(p_int, -12.57f, 12.57f, 16);
                                lastVelocity = uint_to_float(v_int, -15.0f, 15.0f, 16);
                                lastTorqueFeedback = uint_to_float(t_int, -120.0f, 120.0f, 16);
                                
                                int16_t temp_int = (int16_t)(msg.data[6] << 8 | msg.data[7]);
                                lastTemp = (float)temp_int / 10.0f;

                                // Type 2 ID Bits 21~16 are fault info according to Page 35.
                                faultBits = (msg.header.id >> 16) & 0x3F; 
                        } else if (type == 17) { // Read Response
                                lastReadParamIndex = (msg.data[1] << 8) | msg.data[0];
                                memcpy(&lastReadParamValue, &msg.data[4], 4);
                        } else if (type == 21) { // Fault frame
                                faultBits = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];
                        } else if (type == 26) { // Version Response
                                snprintf(versionString, 16, "%d.%d.%d.%d", msg.data[2], msg.data[3], msg.data[4], msg.data[5]);
                        } else if (type == 0) {
                                // Just heartbeat response
                        }
                } else {
                        lastError = 1; // Wrong MotorID (Private)
                }
        }
}

void RobStrideRS04::Run() {
        if(this->canPort->getSpeedPreset() != 5){
                this->canPort->setSpeedPreset(5); 
        }
        this->canPort->takePort();

        uint32_t lastHeartbeatTick = 0;
        uint32_t lastActiveReportingTick = 0;

        while (true) {
                uint32_t now = HAL_GetTick();
                if (now - lastMessageTick > 500) {
                        isConnected = false;
                }

                if (protocol == RS04Protocol::PRIVATE) {
                        if (!isConnected) {
                                if (now - lastHeartbeatTick > 500) {
                                        CAN_tx_msg msg;
                                        // Heartbeat Type 0: bits 0-7 MotorID, 8-15 MasterID
                                        msg.header.id = (0 << 24) | (uint32_t)masterId << 8 | (uint32_t)motorId;
                                        msg.header.extId = true;
                                        msg.header.length = 0;
                                        canPort->sendMessage(msg);
                                        lastHeartbeatTick = now;
                                }
                        } else {
                                // Periodically re-enable active reporting every 1s to keep high frequency even when idle
                                if (now - lastActiveReportingTick > 1000) {
                                        sendEnableActiveReporting();
                                        lastActiveReportingTick = now;
                                }
                        }
                }

                Delay(20);
        }
}
// --- Encoder & Storage Implementation ---
int32_t RobStrideRS04::getPos() {
        return (int32_t)(lastPos * 10430.378f); 
}

float RobStrideRS04::getPos_f() {
        return lastPos;
}

void RobStrideRS04::setPos(int32_t pos) {
        CAN_tx_msg msg;
        msg.header.id = (6 << 24) | (uint32_t)masterId << 8 | (uint32_t)motorId;
        msg.header.extId = true;
        msg.header.length = 8;
        memset(msg.data, 0, 8);
        msg.data[0] = 0x01; 
        canPort->sendMessage(msg);
}
void RobStrideRS04::restoreFlash() {
        uint16_t val = 0;
        uint16_t offset = instanceId * 0x40;
        if (Flash_Read(ADR_AXIS1_CONFIG + offset + 20, &val)) {
                protocol = (RS04Protocol)(val & 0x1);
                motorId = (val >> 8) & 0xFF;
        }
        if (Flash_Read(ADR_AXIS1_CONFIG + offset + 21, &val)) {
                maxTorque = (float)val / 100.0f;
        }
}

void RobStrideRS04::saveFlash() {
        uint16_t val = ((uint16_t)motorId << 8) | (uint8_t)protocol;
        uint16_t offset = instanceId * 0x40;
        Flash_Write(ADR_AXIS1_CONFIG + offset + 20, val);
        Flash_Write(ADR_AXIS1_CONFIG + offset + 21, (uint16_t)(maxTorque * 100.0f));
}

CommandStatus RobStrideRS04::command(const ParsedCommand& cmd, std::vector<CommandReply>& replies) {
        CommandStatus status = CommandHandler::internalCommand(cmd, replies);
        if (status != CommandStatus::NOT_FOUND) return status;

        switch ((RS04Commands)cmd.cmdId) {
        case RS04Commands::canid:
                handleGetSet(cmd, replies, motorId);
                if (cmd.type == CMDtype::set) setCanFilter(); 
                break;
        case RS04Commands::protocol:
                if (cmd.type == CMDtype::set) {
                        protocol = (RS04Protocol)cmd.val;
                        setCanFilter();
                } else {
                        replies.emplace_back((uint8_t)protocol);
                }
                break;
        case RS04Commands::maxtorque:
                if (cmd.type == CMDtype::set) {
                        float requestedTorque = (float)cmd.val / 100.0f;
                        // Safety threshold: 0.5Nm min to 15.0Nm max (between Entry and Pro levels)
                        maxTorque = std::max(0.5f, std::min(requestedTorque, 15.0f));
                } else {
                        replies.emplace_back((uint32_t)(maxTorque * 100.0f));
                }
                break;
        case RS04Commands::connected:
                replies.emplace_back(isConnected ? 1 : 0);
                break;
        case RS04Commands::temp:
                replies.emplace_back((int32_t)(lastTemp * 100));
                break;
        case RS04Commands::rawcan:
                replies.emplace_back((uint32_t)lastRawId);
                break;
        case RS04Commands::lasterr:
                replies.emplace_back((uint32_t)lastError);
                break;
        case RS04Commands::readparam:
                if (cmd.type == CMDtype::set) {
                        sendReadParam(cmd.val);
                } else {
                        replies.emplace_back(lastReadParamValue);
                }
                break;
        case RS04Commands::writeparam:
                if (cmd.type == CMDtype::set) {
                        sendWriteParam(cmd.adr, (float)cmd.val);
                }
                break;
        case RS04Commands::savemotor:
                sendSaveMotor();
                break;
        case RS04Commands::version:
                sendRequestVersion();
                replies.emplace_back(versionString);
                break;
        case RS04Commands::faultbits:
                replies.emplace_back((uint32_t)faultBits);
                break;
        default:
                return CommandStatus::NOT_FOUND;
        }
        return CommandStatus::OK;
}
