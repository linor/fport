#include <Arduino.h>
#include "fport.h"
#include "frsky_crc.h"

#define FPORT_TIME_NEEDED_PER_FRAME_US 3000
#define FPORT_MAX_TELEMETRY_RESPONSE_DELAY_US 2000
#define FPORT_MIN_TELEMETRY_RESPONSE_DELAY_US 500
#define FPORT_MAX_TELEMETRY_AGE_MS 500

#define FPORT_TELEMETRY_MAX_CONSECUTIVE_TELEMETRY_FRAMES 2


#define FPORT_FRAME_MARKER 0x7E

#define FPORT_ESCAPE_CHAR 0x7D
#define FPORT_ESCAPE_MASK 0x20

#define FPORT_BAUDRATE 115200

enum
{
    FSSP_START_STOP = 0x7E,

    FSSP_DLE        = 0x7D,
    FSSP_DLE_XOR    = 0x20,

    FSSP_DATA_FRAME = 0x10,
};

enum {
    FPORT_FRAME_TYPE_CONTROL = 0x00,
    FPORT_FRAME_TYPE_TELEMETRY_REQUEST = 0x01,
    FPORT_FRAME_TYPE_TELEMETRY_RESPONSE = 0x81,

};

enum {
    FPORT_FRAME_ID_NULL = 0x00,     // (master/slave)
    FPORT_FRAME_ID_DATA = 0x10,     // (master/slave)
    FPORT_FRAME_ID_READ = 0x30,     // (master)
    FPORT_FRAME_ID_WRITE = 0x31,    // (master)
    FPORT_FRAME_ID_RESPONSE = 0x32, // (slave)
};

#define SBUS_FLAG_CHANNEL_17        (1 << 0)
#define SBUS_FLAG_CHANNEL_18        (1 << 1)

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

void FPort::begin(HardwareSerial *port) {
    this->_port = port;

    _port->begin(FPORT_BAUDRATE);
}

void FPort::receiveData(uint8_t val) {
    const unsigned long currentTimeUs = micros(); 

    _clearToSend = false;

    if (_framePosition > 1 && (currentTimeUs - _frameStartAt) > FPORT_TIME_NEEDED_PER_FRAME_US + 500) {
        // TODO: reportFrameError(DEBUG_FPORT_ERROR_TIMEOUT);
        Serial.println("Timeout!");
        _framePosition = 0;
    }

    if (val == FPORT_FRAME_MARKER) {
        if (_framePosition > 1) {
            const uint8_t nextWriteIndex = (_rxBufferWriteIndex + 1) % rxBuffers;
            if (nextWriteIndex != _rxBufferReadIndex) {
                _rxBuffer[_rxBufferWriteIndex].length = _framePosition - 1;
                _rxBufferWriteIndex = nextWriteIndex;
            }

            if (_telemetryFrame) {
                _clearToSend = true;
                _lastTelemetryFrameReceivedUs = currentTimeUs;
                _telemetryFrame = false;
            }

            _lastFrameReceivedUs = currentTimeUs;
            _escapedCharacter = false;
        }

        _frameStartAt = currentTimeUs;
        _framePosition = 1;

        _rxBuffer[_rxBufferWriteIndex].frameStartTimeUs = currentTimeUs;
    } else if (_framePosition > 0) {
        if (_framePosition >= FPORT_BUFFER_SIZE + 1) {
            _framePosition = 0;

            // TODO:    reportFrameError(DEBUG_FPORT_ERROR_OVERSIZE);
            Serial.println("Error oversize!");
        } else {
            if (_escapedCharacter) {
                val = val ^ FPORT_ESCAPE_MASK;
                _escapedCharacter = false;
            } else if (val == FPORT_ESCAPE_CHAR) {
                _escapedCharacter = true;
                return;
            }

            if (_framePosition == 2 && val == FPORT_FRAME_TYPE_TELEMETRY_REQUEST) {
                _telemetryFrame = true;
            }

            _rxBuffer[_rxBufferWriteIndex].data[_framePosition - 1] = val;
            _framePosition = _framePosition + 1;
        }
    }
}

uint8_t FPort::decodeSBusChannels(const sbusChannels_t *sbusChannels) {
    channels[0] = sbusChannels->chan0;
    channels[1] = sbusChannels->chan1;
    channels[2] = sbusChannels->chan2;
    channels[3] = sbusChannels->chan3;
    channels[4] = sbusChannels->chan4;
    channels[5] = sbusChannels->chan5;
    channels[6] = sbusChannels->chan6;
    channels[7] = sbusChannels->chan7;
    channels[8] = sbusChannels->chan8;
    channels[9] = sbusChannels->chan9;
    channels[10] = sbusChannels->chan10;
    channels[11] = sbusChannels->chan11;
    channels[12] = sbusChannels->chan12;
    channels[13] = sbusChannels->chan13;
    channels[14] = sbusChannels->chan14;
    channels[15] = sbusChannels->chan15;

    if (sbusChannels->flags & SBUS_FLAG_CHANNEL_17) {
        channels[16] = SBUS_DIGITAL_CHANNEL_MAX;
    } else {
        channels[16] = SBUS_DIGITAL_CHANNEL_MIN;
    }

    if (sbusChannels->flags & SBUS_FLAG_CHANNEL_18) {
        channels[17] = SBUS_DIGITAL_CHANNEL_MAX;
    } else {
        channels[17] = SBUS_DIGITAL_CHANNEL_MIN;
    }

    bool hasValues = false;
    for(int i = 0; i < 16; i++) if (channels[i]) hasValues = true;
    if (!sbusChannels->flags && !hasValues) {
        return RX_FRAME_COMPLETE | RX_NOT_INITIALIZED;
    }


    if (sbusChannels->flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
        // internal failsafe enabled and rx failsafe flag set
        // RX *should* still be sending valid channel data (repeated), so use it.
        return RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
    }

    if (sbusChannels->flags & SBUS_FLAG_SIGNAL_LOSS) {
        // The received data is a repeat of the last valid data so can be considered complete.
        return RX_FRAME_COMPLETE | RX_FRAME_DROPPED;
    }

    return RX_FRAME_COMPLETE;
}

uint8_t FPort::parsePendingData() {
    uint8_t result = RX_FRAME_PENDING;

    if (_rxBufferReadIndex != _rxBufferWriteIndex) {
        uint8_t bufferLength = _rxBuffer[_rxBufferReadIndex].length;
        uint8_t frameLength = _rxBuffer[_rxBufferReadIndex].data[0];
        if (frameLength != bufferLength - 2) {
            // reportFrameError(DEBUG_FPORT_ERROR_SIZE);
            // TODO
            Serial.println("Fport frame length error");
        } else {
            if (!frskyCheckSumIsGood(&_rxBuffer[_rxBufferReadIndex].data[0], bufferLength)) {
                // TODO: reportFrameError(DEBUG_FPORT_ERROR_CHECKSUM);
                Serial.println("checksum error");
            } else {
                fportFrame_t *frame = (fportFrame_t *)&_rxBuffer[_rxBufferReadIndex].data[1];

                switch (frame->type) {
                    case FPORT_FRAME_TYPE_CONTROL:
                        if (frameLength != FPORT_FRAME_PAYLOAD_LENGTH_CONTROL) {
                            // TODO reportFrameError(DEBUG_FPORT_ERROR_TYPE_SIZE);
                            Serial.println("Error Type Size");
                        } else {
                            result = decodeSBusChannels(&frame->data.controlData.channels);
                            _lastRcFrameReceivedMs = millis();

                            if (!(result & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED | RX_NOT_INITIALIZED))) {
                                _lastRcFrameTimeUs = _rxBuffer[_rxBufferReadIndex].frameStartTimeUs;
                            }
                        }
                        break;
                    case FPORT_FRAME_TYPE_TELEMETRY_REQUEST:
                        if (frameLength != FPORT_FRAME_PAYLOAD_LENGTH_TELEMETRY_REQUEST) {
                            // TODO reportFrameError(DEBUG_FPORT_ERROR_TYPE_SIZE);
                            Serial.println("Error size");
                        } else {
                            switch(frame->data.telemetryData.frameId) {
                                case FPORT_FRAME_ID_DATA:
                                    if (!_rxDrivenFrameRate) {
                                        _rxDrivenFrameRate = true;
                                    }

                                    _hasTelemetryRequest = true;
                                    break;
                                case FPORT_FRAME_ID_NULL:
                                    if (!_rxDrivenFrameRate) {
                                        if (_consecutiveTelemetryFrameCount >= FPORT_TELEMETRY_MAX_CONSECUTIVE_TELEMETRY_FRAMES) {
                                            _consecutiveTelemetryFrameCount = 0;
                                        } else {
                                            _hasTelemetryRequest = true;
                                            _consecutiveTelemetryFrameCount++;
                                        }
                                    }

                                    break;
                                case FPORT_FRAME_ID_READ:
                                case FPORT_FRAME_ID_WRITE: // never used
                                /*
                                    memcpy(&payloadBuffer, &frame->data.telemetryData, sizeof(smartPortPayload_t));
                                    mspPayload = &payloadBuffer;
*/
                                    break;
                                default:
                                    break;
                            }
                        }

                        break;
                    default:
                        // TODO reportFrameError(DEBUG_FPORT_ERROR_TYPE);
                        break;
                }
            }
        }

        _rxBufferReadIndex = (_rxBufferReadIndex + 1) % rxBuffers;
    }

    if (_hasTelemetryRequest && (micros() - _lastTelemetryFrameReceivedUs) >= FPORT_MIN_TELEMETRY_RESPONSE_DELAY_US) {
        _hasTelemetryRequest = false;

        result = (result & ~RX_FRAME_PENDING) | RX_FRAME_PROCESSING_REQUIRED;
    }

    if (_lastRcFrameReceivedMs && ((millis() - _lastRcFrameReceivedMs) > FPORT_MAX_TELEMETRY_AGE_MS)) {
        _lastRcFrameReceivedMs = 0;
    }

    return result;
}

void FPort::addSensor(sensorType sensorId, sensorDataFnPtr data, unsigned long intervalMs) {
    if (_sensorCount >= maxSensors) return;
    _sensors[_sensorCount].type = sensorId;
    _sensors[_sensorCount].interval = intervalMs;
    _sensors[_sensorCount].fn = data;
    _sensorCount++;
}

void smartPortSendByte(uint8_t c, uint16_t *checksum, HardwareSerial *port)
{
    // smart port escape sequence
    if (c == FSSP_DLE || c == FSSP_START_STOP) {
        port->write(FSSP_DLE);
        port->write(c ^ FSSP_DLE_XOR);
    } else {
        port->write(c);
    }

    if (checksum != NULL) {
        frskyCheckSumStep(checksum, c);
    }
}

void smartPortWriteFrameFport(HardwareSerial *port, const smartPortPayload_t *payload)
{
    uint16_t checksum = 0;
    smartPortSendByte(FPORT_RESPONSE_FRAME_LENGTH, &checksum, port);
    smartPortSendByte(FPORT_FRAME_TYPE_TELEMETRY_RESPONSE, &checksum, port);

    uint8_t *data = (uint8_t *)payload;
    for (unsigned i = 0; i < sizeof(smartPortPayload_t); i++) {
        smartPortSendByte(*data++, &checksum, port);
    }
    
    frskyCheckSumFini(&checksum);
    smartPortSendByte(checksum, NULL, port);
}

void FPort::processTelemetry() {
    unsigned long currentTimeUs = micros();
    if ((currentTimeUs - _lastTelemetryFrameReceivedUs) > FPORT_MAX_TELEMETRY_RESPONSE_DELAY_US) {
        _clearToSend = false;
    }

    if (_clearToSend) {
        unsigned long timeout = _lastTelemetryFrameReceivedUs + FPORT_MAX_TELEMETRY_RESPONSE_DELAY_US;
        while(_clearToSend & ((timeout - micros()) > 0)) {
            unsigned long currentTimeMs = millis();

            for(int sensor = 0; sensor < _sensorCount; sensor++) {
                if (currentTimeMs > _sensors[sensor].nextUpdate) {
                    smartPortPayload_t payload;
                    payload.frameId = FSSP_DATA_FRAME;
                    payload.valueId = _sensors[sensor].type;
                    payload.data = _sensors[sensor].fn(_sensors[sensor].type);

                    smartPortWriteFrameFport(_port, &payload);
                    _sensors[sensor].nextUpdate = currentTimeMs + _sensors[sensor].interval;
                    _clearToSend = false;
                    break;
                }
            }
        }

        if (_clearToSend) {
//            smartPortWriteFrameFport(this->_port, &emptySmartPortFrame);
            _clearToSend = false;
        }

        _lastTelemetryFrameSentUs = currentTimeUs;
    }
}

uint8_t FPort::loop() {
    if (_port->available()) {
        char c = _port->read();
        receiveData(c);
    }

    uint8_t status = parsePendingData();
    if (status & RX_FRAME_PROCESSING_REQUIRED) {
        processTelemetry();
    }

    if ((micros() - _lastFrameReceivedUs) > (2 * FPORT_TIME_NEEDED_PER_FRAME_US)) {
        status |= RX_TIMEOUT;
    }

    return status;
}