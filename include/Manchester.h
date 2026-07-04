#ifndef MANCHESTER_H
#define MANCHESTER_H

#include <Arduino.h>
#include "driver/rmt.h"
#include "Pilote.h"

// ============================================================
//  Manchester — Couche encodage/décodage Manchester (IEEE 802.3)
//
//  Convention (appliquée uniformément dans tout le code) :
//    bit 1  → front descendant au milieu du bit  (HIGH → LOW)
//    bit 0  → front montant au milieu du bit      (LOW  → HIGH)
//
//  En termes d'items RMT :
//    bit 1 : level0=1 (HIGH, durée=halfBit), level1=0 (LOW,  durée=halfBit)
//    bit 0 : level0=0 (LOW,  durée=halfBit), level1=1 (HIGH, durée=halfBit)
//
//  Structure d'une trame :
//    [PRÉAMBULE 0x55][FLAG 0x7E][TYPE][SEQ][LEN][VOL][PAYLOAD…][CRC_H][CRC_L][FLAG 0x7E]
//
//  Types de trames :
//    TYPE_START (0x01) : annonce du transfert (vol = nb total de paquets)
//    TYPE_DATA  (0x02) : paquet de données    (seq = numéro 1-based)
//    TYPE_END   (0x03) : fin de transfert
// ============================================================

#define TYPE_START 0x01
#define TYPE_DATA  0x02
#define TYPE_END   0x03

// Nombre maximal d'octets de payload par trame DATA
#define MAX_PAYLOAD 80

// Taille maximale du buffer d'items RMT pour une trame complète.
// Worst case : 2 sync + 4 entête + 80 payload + 2 CRC + 1 flag fin = 89 octets × 8 = 712 items
#define RMT_ITEMS_MAX 800

class Manchester
{
public:
    Manchester(int pinTx, int pinRx,
               rmt_channel_t chTx = RMT_CHANNEL_0,
               rmt_channel_t chRx = RMT_CHANNEL_1);
    ~Manchester();

    // Transmet un message complet (fragmentation automatique en trames DATA)
    void TransmitMessage(uint8_t *message, size_t length);

    // Reçoit une trame.
    // Retourne : nombre d'octets de payload (>= 0 : succès)
    //            code d'erreur négatif      (<  0 : échec)
    //  -1 : timeout silence
    //  -2 : données insuffisantes / auto-baud impossible
    //  -3 : flag 0x7E introuvable
    //  -4 : erreur lecture entête
    //  -5 : longueur invalide (> MAX_PAYLOAD)
    //  -6 : erreur lecture payload
    //  -7 : erreur lecture CRC
    //  -8 : CRC invalide
    int ReceiveFrame(uint8_t *payload, uint8_t &type, uint8_t &seq, uint8_t &vol);

private:
    Pilote *_pilote;

    // TX helpers
    void encodeBit(rmt_item32_t *items, int &idx, bool bit);
    void encodeByte(rmt_item32_t *items, int &idx, uint8_t byte);
    void TransmitFrame(uint8_t type, uint8_t seq, uint8_t vol,
                       uint8_t *payload, uint8_t payloadLen);

    // RX helpers
    int  decodeItem(const rmt_item32_t &item, uint16_t halfBit);
    bool decodeByte(const rmt_item32_t *items, int &idx, int total,
                    uint8_t &byteOut, uint16_t halfBit);

    // CRC-16/CCITT
    uint16_t CalculateCRC(uint8_t *data, size_t length);

    static constexpr uint8_t PREAMBLE = 0x55;
    static constexpr uint8_t FLAG     = 0x7E;
};

#endif // MANCHESTER_H
