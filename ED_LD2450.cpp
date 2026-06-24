#include "ED_LD2450.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cstdio>
#include <cmath>

static const char *TAG = "ED_LD2450";

// Frame constants
#define CMD_ENABLE_CONFIG     0x00FF
#define CMD_END_CONFIG        0x00FE
#define CMD_SINGLE_TARGET     0x0080
#define CMD_MULTI_TARGET      0x0090
#define CMD_QUERY_MODE        0x0091
#define CMD_READ_FIRMWARE     0x00A0
#define CMD_SET_BAUDRATE      0x00A1
#define CMD_FACTORY_RESET     0x00A2
#define CMD_RESTART           0x00A3
#define CMD_BLUETOOTH         0x00A4
#define CMD_GET_MAC           0x00A5
#define CMD_QUERY_ZONE        0x00C1
#define CMD_SET_ZONE          0x00C2

ED_LD2450::ED_LD2450() : _uart(UART_NUM_0), _rxIndex(0), _frameReady(false),
                         _targetCount(0), _lastFrameTime(0),
                         _enabledCount(0), _mappingValid(false) {
    memset(_targets, 0, sizeof(_targets));
    memset(_mapPoints, 0, sizeof(_mapPoints));
    // Identity transform as fallback
    _a = 1.0f; _b = 0.0f; _c = 0.0f;
    _d = 0.0f; _e = 1.0f; _f = 0.0f;
}

ED_LD2450::~ED_LD2450() {
    uart_driver_delete(_uart);
}

esp_err_t ED_LD2450::begin(uart_port_t uart_num, int tx_pin, int rx_pin, uint32_t baud_rate) {
    _uart = uart_num;
    uart_config_t uart_config = {
        .baud_rate = (int)baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = 0
    };
    esp_err_t err = uart_param_config(_uart, &uart_config);
    if (err != ESP_OK) return err;
    err = uart_set_pin(_uart, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    err = uart_driver_install(_uart, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    _rxIndex = 0;
    _frameReady = false;
    _mappingValid = false;
    return ESP_OK;
}

uint32_t ED_LD2450::getMillis() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void ED_LD2450::update() {
    int len = uart_read_bytes(_uart, _rxBuffer + _rxIndex, sizeof(_rxBuffer) - _rxIndex, 0);
    if (len > 0) {
        _rxIndex += len;
    }

    const uint8_t radarHeader[] = {0xAA, 0xFF, 0x03, 0x00};
    const uint8_t radarFooter[] = {0x55, 0xCC};
    const int frameLen = 4 + LD2450_MAX_TARGETS * LD2450_TARGET_DATA_LEN + 2;

    while (_rxIndex >= frameLen) {
        int headerPos = -1;
        for (int i = 0; i <= _rxIndex - frameLen; i++) {
            if (memcmp(_rxBuffer + i, radarHeader, 4) == 0) {
                if (memcmp(_rxBuffer + i + frameLen - 2, radarFooter, 2) == 0) {
                    headerPos = i;
                    break;
                }
            }
        }
        if (headerPos == -1) {
            if (_rxIndex > 100) {
                memmove(_rxBuffer, _rxBuffer + _rxIndex - 50, 50);
                _rxIndex = 50;
            }
            break;
        }

        parseRadarFrame(_rxBuffer + headerPos);
        _frameReady = true;
        _lastFrameTime = getMillis();

        int consumed = headerPos + frameLen;
        if (consumed < _rxIndex) {
            memmove(_rxBuffer, _rxBuffer + consumed, _rxIndex - consumed);
            _rxIndex -= consumed;
        } else {
            _rxIndex = 0;
        }
    }
}

bool ED_LD2450::available() {
    bool ret = _frameReady;
    if (ret) _frameReady = false;
    return ret;
}

uint8_t ED_LD2450::getTargetCount() {
    return _targetCount;
}

bool ED_LD2450::getTarget(uint8_t index, LD2450_Target &target) {
    if (index >= LD2450_MAX_TARGETS) return false;
    if (_targets[index].x == 0 && _targets[index].y == 0 &&
        _targets[index].speed == 0 && _targets[index].resolution == 0) {
        return false;
    }
    target = _targets[index];
    return true;
}

// ------------------- Private helpers -------------------

int16_t ED_LD2450::decodeSigned15(uint16_t raw) {
    if (raw & 0x8000) {
        return (int16_t)(raw & 0x7FFF);
    } else {
        return -(int16_t)(raw & 0x7FFF);
    }
}

uint16_t ED_LD2450::getLittleEndian16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

void ED_LD2450::parseRadarFrame(const uint8_t *frame) {
    const uint8_t *ptr = frame + 4;
    _targetCount = 0;
    for (int i = 0; i < LD2450_MAX_TARGETS; i++) {
        uint16_t rawX = getLittleEndian16(ptr); ptr += 2;
        uint16_t rawY = getLittleEndian16(ptr); ptr += 2;
        uint16_t rawSpd = getLittleEndian16(ptr); ptr += 2;
        uint16_t rawRes = getLittleEndian16(ptr); ptr += 2;

        if (rawX == 0 && rawY == 0 && rawSpd == 0 && rawRes == 0) {
            _targets[i].x = 0; _targets[i].y = 0; _targets[i].speed = 0; _targets[i].resolution = 0;
        } else {
            _targets[i].x = decodeSigned15(rawX);
            _targets[i].y = decodeSigned15(rawY);
            _targets[i].speed = decodeSigned15(rawSpd);
            _targets[i].resolution = rawRes;
            _targetCount++;
        }
    }
}

// ---------- Low‑level command sending ----------

bool ED_LD2450::sendCommand(uint16_t cmdWord, const uint8_t *cmdValue, uint8_t valueLen,
                            uint8_t *ackData, uint16_t &ackLen, uint32_t timeout_ms) {
    uint8_t txFrame[32];
    uint8_t idx = 0;
    txFrame[idx++] = 0xFD; txFrame[idx++] = 0xFC; txFrame[idx++] = 0xFB; txFrame[idx++] = 0xFA;
    uint16_t dataLen = 2 + valueLen;
    txFrame[idx++] = dataLen & 0xFF;
    txFrame[idx++] = (dataLen >> 8) & 0xFF;
    txFrame[idx++] = cmdWord & 0xFF;
    txFrame[idx++] = (cmdWord >> 8) & 0xFF;
    if (cmdValue && valueLen) {
        memcpy(&txFrame[idx], cmdValue, valueLen);
        idx += valueLen;
    }
    txFrame[idx++] = 0x04; txFrame[idx++] = 0x03; txFrame[idx++] = 0x02; txFrame[idx++] = 0x01;

    uart_write_bytes(_uart, (const char*)txFrame, idx);

    uint8_t rxBuf[64];
    uint16_t rxLen = 0;
    uint32_t start = getMillis();
    while ((getMillis() - start) < timeout_ms) {
        int len = uart_read_bytes(_uart, rxBuf + rxLen, sizeof(rxBuf) - rxLen, 0);
        if (len > 0) rxLen += len;
        if (rxLen >= 10) {
            for (int i = 0; i <= rxLen - 10; i++) {
                if (rxBuf[i] == 0xFD && rxBuf[i+1] == 0xFC && rxBuf[i+2] == 0xFB && rxBuf[i+3] == 0xFA) {
                    uint16_t ackDataLen = getLittleEndian16(&rxBuf[i+4]);
                    uint16_t totalAckLen = 4 + 2 + ackDataLen + 4;
                    if (rxLen >= i + totalAckLen) {
                        if (rxBuf[i+totalAckLen-4] == 0x04 && rxBuf[i+totalAckLen-3] == 0x03 &&
                            rxBuf[i+totalAckLen-2] == 0x02 && rxBuf[i+totalAckLen-1] == 0x01) {
                            uint16_t ackCmd = getLittleEndian16(&rxBuf[i+6]);
                            if (ackCmd == (cmdWord | 0x0100)) {
                                uint16_t retLen = ackDataLen - 2;
                                if (retLen > 0 && ackData) {
                                    if (retLen > ackLen) retLen = ackLen;
                                    memcpy(ackData, &rxBuf[i+8], retLen);
                                    ackLen = retLen;
                                } else {
                                    ackLen = 0;
                                }
                                return true;
                            }
                        }
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

// ---------- Public command implementations ----------

bool ED_LD2450::enableConfig() {
    uint16_t val = 0x0001;
    uint8_t ack[8];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_ENABLE_CONFIG, (uint8_t*)&val, 2, ack, ackLen)) {
        if (ackLen >= 2 && ack[0] == 0x00 && ack[1] == 0x00) return true;
    }
    return false;
}

bool ED_LD2450::endConfig() {
    uint8_t ack[8];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_END_CONFIG, nullptr, 0, ack, ackLen)) {
        if (ackLen >= 2 && ack[0] == 0x00 && ack[1] == 0x00) return true;
    }
    return false;
}

bool ED_LD2450::setSingleTarget() {
    uint8_t ack[8];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_SINGLE_TARGET, nullptr, 0, ack, ackLen)) {
        if (ackLen >= 2 && ack[0] == 0x00 && ack[1] == 0x00) return true;
    }
    return false;
}

bool ED_LD2450::setMultiTarget() {
    uint8_t ack[8];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_MULTI_TARGET, nullptr, 0, ack, ackLen)) {
        if (ackLen >= 2 && ack[0] == 0x00 && ack[1] == 0x00) return true;
    }
    return false;
}

bool ED_LD2450::queryTrackingMode(uint16_t &mode) {
    uint8_t ack[8];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_QUERY_MODE, nullptr, 0, ack, ackLen)) {
        if (ackLen >= 4 && ack[0] == 0x00 && ack[1] == 0x00) {
            mode = getLittleEndian16(&ack[2]);
            return true;
        }
    }
    return false;
}

bool ED_LD2450::readFirmwareVersion(std::string &version) {
    uint8_t ack[16];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_READ_FIRMWARE, nullptr, 0, ack, ackLen)) {
        if (ackLen >= 10 && ack[0] == 0x00 && ack[1] == 0x00) {
            uint16_t major = getLittleEndian16(&ack[4]);
            uint32_t minor = getLittleEndian16(&ack[6]) | ((uint32_t)getLittleEndian16(&ack[8]) << 16);
            char buf[32];
            snprintf(buf, sizeof(buf), "V%d.%02d.%08lu",
                     (major >> 8) & 0xFF,
                     major & 0xFF,
                     (unsigned long)minor);
            version = std::string(buf);
            return true;
        }
    }
    return false;
}

bool ED_LD2450::setBaudRate(uint16_t index) {
    uint8_t val[2] = {(uint8_t)index, (uint8_t)(index >> 8)};
    uint8_t ack[8];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_SET_BAUDRATE, val, 2, ack, ackLen)) {
        if (ackLen >= 2 && ack[0] == 0x00 && ack[1] == 0x00) return true;
    }
    return false;
}

bool ED_LD2450::factoryReset() {
    uint8_t ack[8];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_FACTORY_RESET, nullptr, 0, ack, ackLen)) {
        if (ackLen >= 2 && ack[0] == 0x00 && ack[1] == 0x00) return true;
    }
    return false;
}

bool ED_LD2450::restartModule() {
    uint8_t ack[8];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_RESTART, nullptr, 0, ack, ackLen)) {
        if (ackLen >= 2 && ack[0] == 0x00 && ack[1] == 0x00) return true;
    }
    return false;
}

bool ED_LD2450::setBluetooth(bool enable) {
    uint16_t val = enable ? 0x0100 : 0x0000;
    uint8_t ack[8];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_BLUETOOTH, (uint8_t*)&val, 2, ack, ackLen)) {
        if (ackLen >= 2 && ack[0] == 0x00 && ack[1] == 0x00) return true;
    }
    return false;
}

bool ED_LD2450::getMacAddress(std::string &mac) {
    uint8_t val[2] = {0x01, 0x00};
    uint8_t ack[16];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_GET_MAC, val, 2, ack, ackLen)) {
        if (ackLen >= 8 && ack[0] == 0x00 && ack[1] == 0x00) {
            char buf[18];
            snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                     ack[3], ack[4], ack[5], ack[6], ack[7], ack[8]);
            mac = std::string(buf);
            return true;
        }
    }
    return false;
}

bool ED_LD2450::queryZoneFilter(uint8_t &type, int16_t zoneCoords[3][4]) {
    uint8_t ack[32];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_QUERY_ZONE, nullptr, 0, ack, ackLen)) {
        if (ackLen >= 28 && ack[0] == 0x00 && ack[1] == 0x00) {
            type = getLittleEndian16(&ack[2]); // 2 bytes but only low byte used
            uint16_t *coords = (uint16_t*)&ack[4];
            for (int z=0; z<3; z++) {
                for (int c=0; c<4; c++) {
                    zoneCoords[z][c] = (int16_t)getLittleEndian16((uint8_t*)&coords[z*4 + c]);
                }
            }
            return true;
        }
    }
    return false;
}

bool ED_LD2450::setZoneFilter(uint8_t type, int16_t zoneCoords[3][4]) {
    uint8_t data[26];
    uint16_t t = type;
    data[0] = t & 0xFF;
    data[1] = (t >> 8) & 0xFF;
    uint16_t *coords = (uint16_t*)&data[2];
    for (int z=0; z<3; z++) {
        for (int c=0; c<4; c++) {
            coords[z*4 + c] = (uint16_t)zoneCoords[z][c];
        }
    }
    uint8_t ack[8];
    uint16_t ackLen = sizeof(ack);
    if (sendCommand(CMD_SET_ZONE, data, 26, ack, ackLen)) {
        if (ackLen >= 2 && ack[0] == 0x00 && ack[1] == 0x00) return true;
    }
    return false;
}

// ---------- Coordinate mapping ----------

// Solves a 3x3 linear system: A * X = B, returns true if unique solution.
bool ED_LD2450::solve3x3(const float A[3][3], const float B[3], float X[3]) {
    float a[3][3];
    float b[3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) a[i][j] = A[i][j];
        b[i] = B[i];
    }

    for (int col = 0; col < 3; col++) {
        int pivot = col;
        float maxVal = fabs(a[col][col]);
        for (int row = col + 1; row < 3; row++) {
            if (fabs(a[row][col]) > maxVal) {
                maxVal = fabs(a[row][col]);
                pivot = row;
            }
        }
        if (maxVal < 1e-9) return false;

        if (pivot != col) {
            for (int j = 0; j < 3; j++) {
                float tmp = a[col][j];
                a[col][j] = a[pivot][j];
                a[pivot][j] = tmp;
            }
            float tmp = b[col];
            b[col] = b[pivot];
            b[pivot] = tmp;
        }

        for (int row = col + 1; row < 3; row++) {
            float factor = a[row][col] / a[col][col];
            for (int j = col; j < 3; j++) {
                a[row][j] -= factor * a[col][j];
            }
            b[row] -= factor * b[col];
        }
    }

    for (int i = 2; i >= 0; i--) {
        float sum = b[i];
        for (int j = i + 1; j < 3; j++) {
            sum -= a[i][j] * X[j];
        }
        X[i] = sum / a[i][i];
    }
    return true;
}

// Internal: recompute affine from currently enabled mapping points
bool ED_LD2450::recomputeAffine() {
    _enabledCount = 0;
    // Count enabled points
    for (int i = 0; i < MAX_MAPPING_POINTS; i++) {
        if (_mapPoints[i].enabled) _enabledCount++;
    }

    if (_enabledCount < 3) {
        _mappingValid = false;
        return false;
    }

    // Build least‑squares system: for each enabled point, row = [x, y, 1]
    float AtA[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    float AtX[3] = {0,0,0};
    float AtY[3] = {0,0,0};

    for (int i = 0; i < MAX_MAPPING_POINTS; i++) {
        if (!_mapPoints[i].enabled) continue;
        float x = (float)_mapPoints[i].radarX;
        float y = (float)_mapPoints[i].radarY;
        float Xr = (float)_mapPoints[i].roomX;
        float Yr = (float)_mapPoints[i].roomY;

        AtA[0][0] += x * x;
        AtA[0][1] += x * y;
        AtA[0][2] += x;
        AtA[1][0] += y * x;
        AtA[1][1] += y * y;
        AtA[1][2] += y;
        AtA[2][0] += x;
        AtA[2][1] += y;
        AtA[2][2] += 1.0f;

        AtX[0] += x * Xr;
        AtX[1] += y * Xr;
        AtX[2] += Xr;

        AtY[0] += x * Yr;
        AtY[1] += y * Yr;
        AtY[2] += Yr;
    }

    float coeffX[3], coeffY[3];
    if (!solve3x3(AtA, AtX, coeffX)) return false;
    if (!solve3x3(AtA, AtY, coeffY)) return false;

    _a = coeffX[0];
    _b = coeffX[1];
    _c = coeffX[2];
    _d = coeffY[0];
    _e = coeffY[1];
    _f = coeffY[2];

    _mappingValid = true;
    return true;
}

// ---- Public mapping API ----

bool ED_LD2450::setMappingPoint(uint8_t index, int16_t radarX, int16_t radarY,
                                int16_t roomX, int16_t roomY, bool enable) {
    if (index >= MAX_MAPPING_POINTS) return false;
    _mapPoints[index].radarX = radarX;
    _mapPoints[index].radarY = radarY;
    _mapPoints[index].roomX = roomX;
    _mapPoints[index].roomY = roomY;
    _mapPoints[index].enabled = enable;
    return recomputeAffine();
}

bool ED_LD2450::enableMappingPoint(uint8_t index, bool enable) {
    if (index >= MAX_MAPPING_POINTS) return false;
    _mapPoints[index].enabled = enable;
    return recomputeAffine();
}

bool ED_LD2450::clearMappingPoints() {
    for (int i = 0; i < MAX_MAPPING_POINTS; i++) {
        _mapPoints[i].enabled = false;
        _mapPoints[i].radarX = 0;
        _mapPoints[i].radarY = 0;
        _mapPoints[i].roomX = 0;
        _mapPoints[i].roomY = 0;
    }
    _enabledCount = 0;
    _mappingValid = false;
    return true;
}

bool ED_LD2450::computeMapping() {
    return recomputeAffine();
}

std::string ED_LD2450::dumpMappingPoints() {
    char buf[256];
    std::string result = "Index  RadarX RadarY RoomX RoomY Status\n";
    for (int i = 0; i < MAX_MAPPING_POINTS; i++) {
        snprintf(buf, sizeof(buf), "%3d   %6d %6d %6d %6d   %s\n",
                 i,
                 _mapPoints[i].radarX,
                 _mapPoints[i].radarY,
                 _mapPoints[i].roomX,
                 _mapPoints[i].roomY,
                 _mapPoints[i].enabled ? "ENABLED" : "DISABLED");
        result += buf;
    }
    return result;
}

// Backward‑compatible: set exactly 4 points, clear all others
bool ED_LD2450::setMappingPoints(const int16_t radarX[4], const int16_t radarY[4],
                                 const int16_t roomX[4], const int16_t roomY[4]) {
    clearMappingPoints();
    for (int i = 0; i < 4; i++) {
        if (i < MAX_MAPPING_POINTS) {
            _mapPoints[i].radarX = radarX[i];
            _mapPoints[i].radarY = radarY[i];
            _mapPoints[i].roomX = roomX[i];
            _mapPoints[i].roomY = roomY[i];
            _mapPoints[i].enabled = true;
        }
    }
    return recomputeAffine();
}

bool ED_LD2450::transformPoint(int16_t rx, int16_t ry, float &roomX, float &roomY) {
    if (!_mappingValid) return false;
    roomX = _a * rx + _b * ry + _c;
    roomY = _d * rx + _e * ry + _f;
    return true;
}