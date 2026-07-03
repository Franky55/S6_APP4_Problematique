#ifndef PILOTE_H
#define PILOTE_H

#include <Arduino.h>
#include "driver/rmt.h"

// ============================================================
//  Pilote — abstraction matérielle RMT pour Manchester
//
//  TX : Le périphérique RMT émet les items directement,
//       sans bloquer le CPU (rmt_write_items en non-bloquant).
//
//  RX : Le périphérique RMT mesure les durées de chaque
//       front entrant et les dépose dans un ring-buffer.
//       Le CPU lit ce buffer pour reconstruire les bits.
// ============================================================

// Durée d'un demi-bit en ticks RMT (1 tick = 1 µs par défaut)
// Changer HALF_BIT_US pour ajuster la vitesse.
// Exemples :  125 µs → ~4 kbps (8 demi-bits par bit Manchester)
//              62 µs → ~8 kbps
//              31 µs → ~16 kbps  (limite pratique du GPIO bit-bang évitée ici)
#define HALF_BIT_US   125

// Tolérance de ±30 % sur les durées RX pour l'auto-synchronisation
#define RMT_TOLERANCE 0.30f

// Taille du ring-buffer RX en octets (multiple de 4, min 64)
#define RMT_RX_BUFFER_SIZE 4096   // était 512 — trop petit pour une trame complète

class Pilote
{
public:
    // ----- Constructeur -----
    // pinTx  : GPIO de sortie (-1 si inutilisé)
    // pinRx  : GPIO d'entrée  (-1 si inutilisé)
    // chTx   : canal RMT TX (0–7)
    // chRx   : canal RMT RX (0–7, doit être différent de chTx)
    Pilote(int pinTx, int pinRx,
           rmt_channel_t chTx = RMT_CHANNEL_0,
           rmt_channel_t chRx = RMT_CHANNEL_1);
    ~Pilote();

    // ----- TX -----
    // Émet un tableau d'items RMT. Non-bloquant si waitDone = false.
    void sendItems(const rmt_item32_t *items, int count, bool waitDone = true);

    // Attend la fin de la dernière émission (utile si waitDone = false)
    void waitTxDone();

    // ----- RX -----
    // Démarre l'écoute RX (appeler avant ReceiveFrame)
    void startRx();

    // Arrête l'écoute et vide le buffer
    void stopRx();

    // Lit les items capturés depuis le ring-buffer.
    // Retourne le nombre d'items écrits dans buf, 0 si timeout.
    // timeoutMs : 0 = non-bloquant
    int readItems(rmt_item32_t *buf, int maxItems, uint32_t timeoutMs = 200);

    // ----- Utilitaires -----
    // Durée d'un demi-bit en ticks (pour construire les items TX)
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
