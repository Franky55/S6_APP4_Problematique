#ifndef PILOTE_H
#define PILOTE_H

#include <Arduino.h>
#include "driver/rmt.h"

#define HALF_BIT_US   50
#define RMT_TOLERANCE 0.30f
#define RMT_RX_BUFFER_SIZE 4096

class Pilote
{
public:
    Pilote(int pinTx, int pinRx,
           rmt_channel_t chTx = RMT_CHANNEL_0,
           rmt_channel_t chRx = RMT_CHANNEL_1);
    ~Pilote();

    void sendItems(const rmt_item32_t *items, int count, bool waitDone = true);
    void waitTxDone();

    void startRx();
    void stopRx();
    int readItems(rmt_item32_t *buf, int maxItems, uint32_t timeoutMs = 200);

    static constexpr uint16_t halfBit() { return HALF_BIT_US; }

private:
    int      _pinTx, _pinRx;
    rmt_channel_t _chTx, _chRx;
    bool     _txInit, _rxInit;
    RingbufHandle_t _rxBuf;

    void initTx();
    void initRx();
};

#endif // PILOTE_H
