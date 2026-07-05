#include "Pilote.h"

// ============================================================
//  Pilote — Implémentation
// ============================================================

Pilote::Pilote(int pinTx, int pinRx, rmt_channel_t chTx, rmt_channel_t chRx)
    : _pinTx(pinTx), _pinRx(pinRx),
      _chTx(chTx),   _chRx(chRx),
      _txInit(false), _rxInit(false),
      _rxBuf(nullptr)
{
    if (_pinTx >= 0) initTx();
    if (_pinRx >= 0) initRx();
}

Pilote::~Pilote()
{
    if (_txInit) rmt_driver_uninstall(_chTx);
    if (_rxInit) rmt_driver_uninstall(_chRx);
}

// ------------------------------------------------------------
//  TX
// ------------------------------------------------------------

void Pilote::initTx()
{
    rmt_config_t cfg = {};
    cfg.rmt_mode      = RMT_MODE_TX;
    cfg.channel       = _chTx;
    cfg.gpio_num      = (gpio_num_t)_pinTx;

    // CORRECTIF bug #3 : mem_block_num = 1 (64 items par canal).
    // Avec 4, le canal 0 occupe les blocs 0-3 et chevauche le canal 2.
    // Le driver rmt_write_items() gère automatiquement les trames plus
    // longues que 64 items en mode bloquant (waitDone = true).
    cfg.mem_block_num = 1;

    // APB 80 MHz / 80 = 1 MHz → 1 tick = 1 µs
    cfg.clk_div = 80;

    cfg.tx_config.carrier_en     = false;
    cfg.tx_config.loop_en        = false;
    cfg.tx_config.idle_output_en = true;
    cfg.tx_config.idle_level     = RMT_IDLE_LEVEL_LOW;

    ESP_ERROR_CHECK(rmt_config(&cfg));
    ESP_ERROR_CHECK(rmt_driver_install(_chTx, 0, 0));
    _txInit = true;
}

void Pilote::sendItems(const rmt_item32_t *items, int count, bool waitDone)
{
    if (!_txInit) return;
    rmt_write_items(_chTx, items, count, waitDone);
}

void Pilote::waitTxDone()
{
    if (!_txInit) return;
    rmt_wait_tx_done(_chTx, portMAX_DELAY);
}

// ------------------------------------------------------------
//  RX
// ------------------------------------------------------------

void Pilote::initRx()
{
    rmt_config_t cfg = {};
    cfg.rmt_mode      = RMT_MODE_RX;
    cfg.channel       = _chRx;
    cfg.gpio_num      = (gpio_num_t)_pinRx;

    // CORRECTIF bug #3 : mem_block_num = 1 (cohérent avec TX)
    cfg.mem_block_num = 7;

    // 1 tick = 1 µs, même référence que TX
    cfg.clk_div = 80;

    // Filtre de bruit : impulsions < 10 µs ignorées
    cfg.rx_config.filter_en           = true;
    cfg.rx_config.filter_ticks_thresh = 10;

    // CORRECTIF bug #4 : idle_threshold à 20× halfBit au lieu de 8×.
    // 8× = 1 ms à 125 µs/demi-bit : trop court, le RMT peut couper la
    // capture en pleine trame si le CPU tarde entre deux items.
    // 20× = 2500 µs laisse une marge confortable.
    cfg.rx_config.idle_threshold = (uint16_t)(HALF_BIT_US * 20);

    ESP_ERROR_CHECK(rmt_config(&cfg));
    ESP_ERROR_CHECK(rmt_driver_install(_chRx, RMT_RX_BUFFER_SIZE, 0));
    ESP_ERROR_CHECK(rmt_get_ringbuf_handle(_chRx, &_rxBuf));
    _rxInit = true;
}

void Pilote::startRx()
{
    if (!_rxInit) return;
    rmt_rx_start(_chRx, true);  // true = efface le buffer avant de démarrer
}

void Pilote::stopRx()
{
    if (!_rxInit) return;
    rmt_rx_stop(_chRx);
}

int Pilote::readItems(rmt_item32_t *buf, int maxItems, uint32_t timeoutMs)
{
    if (!_rxBuf) return 0;
    int totalCount = 0;
    int chunkNum = 0;
    TickType_t deadline = pdMS_TO_TICKS(timeoutMs);

    while (totalCount < maxItems) {
        size_t rxSize = 0;
        rmt_item32_t *rxData = (rmt_item32_t *)xRingbufferReceive(
            _rxBuf, &rxSize, deadline
        );
        if (!rxData) break;

        int count = (int)(rxSize / sizeof(rmt_item32_t));
        if (totalCount + count > maxItems) count = maxItems - totalCount;
        memcpy(buf + totalCount, rxData, count * sizeof(rmt_item32_t));
        vRingbufferReturnItem(_rxBuf, rxData);
        totalCount += count;
        chunkNum++;
        deadline = pdMS_TO_TICKS(1);
    }

    // Print APRÈS la réception, plus de risque de désync
    Serial.printf("[DBG] %d chunks, %d items total\n", chunkNum, totalCount);

    return totalCount;
}
