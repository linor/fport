#pragma once

typedef enum
{
    FSSP_DATAID_SPEED      = 0x0830 ,
    FSSP_DATAID_VFAS       = 0x0210 ,
    FSSP_DATAID_VFAS1      = 0x0211 ,
    FSSP_DATAID_VFAS2      = 0x0212 ,
    FSSP_DATAID_VFAS3      = 0x0213 ,
    FSSP_DATAID_VFAS4      = 0x0214 ,
    FSSP_DATAID_VFAS5      = 0x0215 ,
    FSSP_DATAID_VFAS6      = 0x0216 ,
    FSSP_DATAID_VFAS7      = 0x0217 ,
    FSSP_DATAID_VFAS8      = 0x0218 ,
    FSSP_DATAID_CURRENT    = 0x0200 ,
    FSSP_DATAID_CURRENT1   = 0x0201 ,
    FSSP_DATAID_CURRENT2   = 0x0202 ,
    FSSP_DATAID_CURRENT3   = 0x0203 ,
    FSSP_DATAID_CURRENT4   = 0x0204 ,
    FSSP_DATAID_CURRENT5   = 0x0205 ,
    FSSP_DATAID_CURRENT6   = 0x0206 ,
    FSSP_DATAID_CURRENT7   = 0x0207 ,
    FSSP_DATAID_CURRENT8   = 0x0208 ,
    FSSP_DATAID_RPM        = 0x0500 ,
    FSSP_DATAID_RPM1       = 0x0501 ,
    FSSP_DATAID_RPM2       = 0x0502 ,
    FSSP_DATAID_RPM3       = 0x0503 ,
    FSSP_DATAID_RPM4       = 0x0504 ,
    FSSP_DATAID_RPM5       = 0x0505 ,
    FSSP_DATAID_RPM6       = 0x0506 ,
    FSSP_DATAID_RPM7       = 0x0507 ,
    FSSP_DATAID_RPM8       = 0x0508 ,
    FSSP_DATAID_ALTITUDE   = 0x0100 ,
    FSSP_DATAID_FUEL       = 0x0600 ,
    FSSP_DATAID_ADC1       = 0xF102 ,
    FSSP_DATAID_ADC2       = 0xF103 ,
    FSSP_DATAID_LATLONG    = 0x0800 ,
    FSSP_DATAID_VARIO      = 0x0110 ,
    FSSP_DATAID_CELLS      = 0x0300 ,
    FSSP_DATAID_CELLS_LAST = 0x030F ,
    FSSP_DATAID_HEADING    = 0x0840 ,
// DIY range 0x5100 to 0x52FF
    FSSP_DATAID_CAP_USED   = 0x5250 ,
#if defined(USE_ACC)
    FSSP_DATAID_PITCH      = 0x5230 , // custom
    FSSP_DATAID_ROLL       = 0x5240 , // custom
    FSSP_DATAID_ACCX       = 0x0700 ,
    FSSP_DATAID_ACCY       = 0x0710 ,
    FSSP_DATAID_ACCZ       = 0x0720 ,
#endif
    FSSP_DATAID_T1         = 0x0400 ,
    FSSP_DATAID_T11        = 0x0401 ,
    FSSP_DATAID_T2         = 0x0410 ,
    FSSP_DATAID_HOME_DIST  = 0x0420 ,
    FSSP_DATAID_GPS_ALT    = 0x0820 ,
    FSSP_DATAID_ASPD       = 0x0A00 ,
    FSSP_DATAID_TEMP       = 0x0B70 ,
    FSSP_DATAID_TEMP1      = 0x0B71 ,
    FSSP_DATAID_TEMP2      = 0x0B72 ,
    FSSP_DATAID_TEMP3      = 0x0B73 ,
    FSSP_DATAID_TEMP4      = 0x0B74 ,
    FSSP_DATAID_TEMP5      = 0x0B75 ,
    FSSP_DATAID_TEMP6      = 0x0B76 ,
    FSSP_DATAID_TEMP7      = 0x0B77 ,
    FSSP_DATAID_TEMP8      = 0x0B78 ,
    FSSP_DATAID_A3         = 0x0900 ,
    FSSP_DATAID_A4         = 0x0910
} sensorType;

typedef enum {
    RX_FRAME_PENDING = 0,
    RX_FRAME_COMPLETE = (1 << 0),
    RX_FRAME_FAILSAFE = (1 << 1),
    RX_FRAME_PROCESSING_REQUIRED = (1 << 2),
    RX_FRAME_DROPPED = (1 << 3),
    RX_TIMEOUT = (1 << 4),
    RX_NOT_INITIALIZED = (1 << 5)
} frameState;

typedef struct  {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
    uint8_t flags;
} __attribute__((__packed__)) sbusChannels_t;

typedef struct {
    uint8_t  frameId;
    uint16_t valueId;
    uint32_t data;
} __attribute__((packed)) smartPortPayload_t;

typedef struct fportControlData_s {
    sbusChannels_t channels;
    uint8_t rssi;
} fportControlData_t;

typedef union fportData_s {
    fportControlData_t controlData;
    smartPortPayload_t telemetryData;
} fportData_t;

typedef struct fportFrame_s {
    uint8_t type;
    fportData_t data;
} fportFrame_t;

typedef uint32_t (*sensorDataFnPtr)(const sensorType sensorId);

typedef struct {
    sensorType type;
    sensorDataFnPtr fn;
    unsigned long interval;
    unsigned long nextUpdate;
} sensorData_t;

static const smartPortPayload_t emptySmartPortFrame = { .frameId = 0, .valueId = 0, .data = 0 };

#define FPORT_REQUEST_FRAME_LENGTH sizeof(fportFrame_t)
#define FPORT_RESPONSE_FRAME_LENGTH (sizeof(uint8_t) + sizeof(smartPortPayload_t))

#define FPORT_FRAME_PAYLOAD_LENGTH_CONTROL (sizeof(uint8_t) + sizeof(fportControlData_t))
#define FPORT_FRAME_PAYLOAD_LENGTH_TELEMETRY_REQUEST (sizeof(uint8_t) + sizeof(smartPortPayload_t))

#define FPORT_BUFFER_SIZE (FPORT_REQUEST_FRAME_LENGTH + 2 * sizeof(uint8_t))

class HardwareSerial;

class FPort {
    public:
        void begin(HardwareSerial *port);
        void addSensor(sensorType sensorId, sensorDataFnPtr data, unsigned long intervalMs = 1000);
        uint8_t loop();

        uint16_t channels[18];
    private:
        static const int rxBuffers = 3;
        static const int maxSensors = 20;

        typedef struct {
            uint8_t data[FPORT_BUFFER_SIZE];
            uint8_t length;
            unsigned long frameStartTimeUs;
        } buffer;

        HardwareSerial *_port;

        sensorData_t _sensors[maxSensors];
        uint8_t _sensorCount = 0;
        uint8_t _currentSensor;

        unsigned long _frameStartAt = 0;
        unsigned long _lastFrameReceivedUs = 0;
        unsigned long _lastTelemetryFrameReceivedUs = 0;
        unsigned long _lastTelemetryFrameSentUs = 0;
        unsigned long _lastRcFrameReceivedMs = 0;
        unsigned long _lastRcFrameTimeUs = 0;

        bool _clearToSend = false;
        bool _telemetryFrame = false;

        buffer _rxBuffer[rxBuffers];
        uint8_t _rxBufferWriteIndex = 0;
        uint8_t _rxBufferReadIndex = 0;
        uint8_t _framePosition = 0;
        bool _escapedCharacter = false;

        bool _hasTelemetryRequest = false;
        bool _rxDrivenFrameRate = false;
        uint8_t _consecutiveTelemetryFrameCount = 0;   

        void receiveData(uint8_t c);
        uint8_t parsePendingData();
        uint8_t decodeSBusChannels(const sbusChannels_t *channels);
        void processTelemetry();
};

