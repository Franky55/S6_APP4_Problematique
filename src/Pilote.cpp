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
    cfg.rmt_mode             = RMT_MODE_TX;
    cfg.channel              = _chTx;
    cfg.gpio_num             = (gpio_num_t)_pinTx;
    cfg.mem_block_num        = 4;           // 1 bloc = 64 items → suffisant par octet
    cfg.clk_div              = 80;          // APB 80 MHz / 80 = 1 MHz → 1 tick = 1 µs
    cfg.tx_config.carrier_en         = false;
    cfg.tx_config.loop_en            = false;
    cfg.tx_config.idle_output_en     = true;
    cfg.tx_config.idle_level         = RMT_IDLE_LEVEL_LOW;

    ESP_ERROR_CHECK(rmt_config(&cfg));
    ESP_ERROR_CHECK(rmt_driver_install(_chTx, 0, 0));
    _txInit = true;
}

void Pilote::sendItems(const rmt_item32_t *items, int count, bool waitDone)
{
    if (!_txInit) return;
    // waitDone = true  → bloque jusqu'à la fin de l'émission (pratique pour les trames)
    // waitDone = false → retour immédiat, appeler waitTxDone() plus tard
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
    cfg.mem_block_num = 4;
    cfg.clk_div       = 80;                 // 1 tick = 1 µs, même référence que TX

    // Filtre de bruit : impulsions < 10 µs ignorées
    cfg.rx_config.filter_en           = true;
    cfg.rx_config.filter_ticks_thresh = 10;

    cfg.rx_config.idle_threshold = (uint16_t)(HALF_BIT_US * 8);

    ESP_ERROR_CHECK(rmt_config(&cfg));
    // Ring-buffer dédié RX
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
    TickType_t deadline = pdMS_TO_TICKS(timeoutMs);

    while (totalCount < maxItems) {
        size_t rxSize = 0;
        rmt_item32_t *rxData = (rmt_item32_t *)xRingbufferReceive(
            _rxBuf,
            &rxSize,
            deadline
        );

        if (!rxData) break;

        int count = (int)(rxSize / sizeof(rmt_item32_t));
        if (totalCount + count > maxItems) count = maxItems - totalCount;

        memcpy(buf + totalCount, rxData, count * sizeof(rmt_item32_t));
        vRingbufferReturnItem(_rxBuf, rxData);
        totalCount += count;

        // Après le premier bloc, timeout court pour les suivants
        deadline = pdMS_TO_TICKS(10);
    }

    return totalCount;
}