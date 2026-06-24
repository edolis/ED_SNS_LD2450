#ifndef ED_LD2450_H
#define ED_LD2450_H

#include "driver/uart.h"
#include "esp_err.h"
#include <string>
#include <cmath>

#define LD2450_MAX_TARGETS       3
#define LD2450_TARGET_DATA_LEN   8
#define MAX_MAPPING_POINTS       8   // maximum number of reference points

struct LD2450_Target {
    int16_t x;          // mm (radar coordinates)
    int16_t y;          // mm
    int16_t speed;      // cm/s
    uint16_t resolution; // mm
};

class ED_LD2450 {
public:
    ED_LD2450();
    ~ED_LD2450();

    // Initialize UART
    esp_err_t begin(uart_port_t uart_num, int tx_pin, int rx_pin, uint32_t baud_rate = 256000);

    // Call regularly to process incoming radar frames
    void update();

    // Returns true if a new radar frame has been parsed since last call
    bool available();

    // Get number of active targets (0..3)
    uint8_t getTargetCount();

    // Get target data. Returns false if index invalid or target inactive.
    bool getTarget(uint8_t index, LD2450_Target &target);

    // ----- Configuration commands -----
    bool enableConfig();
    bool endConfig();
    bool setSingleTarget();
    bool setMultiTarget();
    bool queryTrackingMode(uint16_t &mode);
    bool readFirmwareVersion(std::string &version);
    bool setBaudRate(uint16_t index);
    bool factoryReset();
    bool restartModule();
    bool setBluetooth(bool enable);
    bool getMacAddress(std::string &mac);
    bool queryZoneFilter(uint8_t &type, int16_t zoneCoords[3][4]);
    bool setZoneFilter(uint8_t type, int16_t zoneCoords[3][4]);

    // ----- Flexible coordinate mapping (radar → room) -----
    // Add or update a mapping point by index (0..MAX_MAPPING_POINTS-1).
    // If the point was previously disabled, it becomes enabled (unless enable=false).
    bool setMappingPoint(uint8_t index, int16_t radarX, int16_t radarY,
                         int16_t roomX, int16_t roomY, bool enable = true);

    // Enable or disable a mapping point (it remains in the list but is ignored)
    bool enableMappingPoint(uint8_t index, bool enable);

    // Clear all mapping points (disables all)
    bool clearMappingPoints();

    // Recompute the affine transform from the currently enabled points.
    // Automatically called after any modification; you only need to call this
    // if you modify points externally.
    bool computeMapping();

    // Get a formatted string listing all mapping points with their status.
    std::string dumpMappingPoints();

    // Backward‑compatible: set exactly 4 points (clears all others)
    bool setMappingPoints(const int16_t radarX[4], const int16_t radarY[4],
                          const int16_t roomX[4], const int16_t roomY[4]);

    // Transform a radar coordinate to room coordinate. Returns false if mapping invalid.
    bool transformPoint(int16_t rx, int16_t ry, float &roomX, float &roomY);

    // Check if mapping is currently valid (at least 3 enabled points)
    bool isMappingValid() const { return _mappingValid; }

private:
    uart_port_t _uart;
    uint8_t _rxBuffer[256];
    uint16_t _rxIndex;
    bool _frameReady;
    LD2450_Target _targets[LD2450_MAX_TARGETS];
    uint8_t _targetCount;
    uint32_t _lastFrameTime;

    // ---- Mapping storage ----
    struct MappingPoint {
        int16_t radarX;
        int16_t radarY;
        int16_t roomX;
        int16_t roomY;
        bool enabled;
    };
    MappingPoint _mapPoints[MAX_MAPPING_POINTS];
    uint8_t _enabledCount;          // number of currently enabled points

    // Affine coefficients (computed from enabled points)
    float _a, _b, _c;   // roomX = a*rx + b*ry + c
    float _d, _e, _f;   // roomY = d*rx + e*ry + f
    bool _mappingValid;

    // Low‑level helpers
    bool sendCommand(uint16_t cmdWord, const uint8_t *cmdValue, uint8_t valueLen,
                     uint8_t *ackData, uint16_t &ackLen, uint32_t timeout_ms = 100);
    void parseRadarFrame(const uint8_t *frame);
    int16_t decodeSigned15(uint16_t raw);
    uint16_t getLittleEndian16(const uint8_t *p);
    uint32_t getMillis();

    // Linear solver for 3x3 system (used by computeMapping)
    bool solve3x3(const float A[3][3], const float B[3], float X[3]);

    // Internal method to rebuild affine from enabled points
    bool recomputeAffine();
};

#endif