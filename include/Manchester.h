#ifndef MANCHESTER_H
#define MANCHESTER_H

#include <Arduino.h>
#include "driver/rmt.h"
#include "Pilote.h"

// ============================================================
//  Manchester — Couche encodage/décodage Manchester II (IEEE 802.3)
//
//  Convention :
//    bit 1  → front montant au milieu du bit  (LOW→HIGH)
//    bit 0  → front descendant au milieu      (HIGH→LOW)
//
//  Structure d'une trame :
//    [PRÉAMBULE 0x55][FLAG 0x7E][TYPE][SEQ][LEN][VOL][PAYLOAD…][CRC_H][CRC_L][FLAG 0x7E]
//
//  Couche transport :
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
// Worst case : préambule(8) + flag(8) + 84 octets × 8 bits × 2 items = 1376 + marge
#define RMT_ITEMS_MAX 1400

class Manchester
{
public:
    // pinTx : GPIO de sortie  (-1 si ce nœud ne fait que recevoir)
    // pinRx : GPIO d'entrée   (-1 si ce nœud ne fait qu'émettre)
    // chTx  : canal RMT TX   (doit être unique sur l'ESP32)
    // chRx  : canal RMT RX   (doit être unique sur l'ESP32)
    Manchester(int pinTx, int pinRx,
               rmt_channel_t chTx = RMT_CHANNEL_0,
               rmt_channel_t chRx = RMT_CHANNEL_1);
    ~Manchester();

    // ----- API publique -----

    // Transmet un message complet (fragmentation automatique en trames DATA)
    void TransmitMessage(uint8_t *message, size_t length);

    // Reçoit une trame.
    // Retourne : nombre d'octets de payload  (>= 0 : succès)
    //            code d'erreur négatif        (< 0  : échec)
    //  -1 : timeout silence
    //  -2 : timeout pendant préambule
    //  -3 : flag 0x7E introuvable
    //  -4 : erreur lecture entête
    //  -5 : longueur invalide (> MAX_PAYLOAD)
    //  -6 : erreur lecture payload
    //  -7 : erreur lecture CRC
    //  -8 : CRC invalide
    int ReceiveFrame(uint8_t *payload, uint8_t &type, uint8_t &seq, uint8_t &vol);

private:
    Pilote *_pilote;

    // ----- TX helpers -----
    // Ajoute les 2 items RMT correspondant à un bit Manchester dans items[idx]
    void encodeBit(rmt_item32_t *items, int &idx, bool bit);

    // Remplit items[] avec l'encodage Manchester d'un octet (8 bits MSB first)
    void encodeByte(rmt_item32_t *items, int &idx, uint8_t byte);

    // Construit et émet une trame complète via Pilote
    void TransmitFrame(uint8_t type, uint8_t seq, uint8_t vol,
                       uint8_t *payload, uint8_t payloadLen);

    // ----- RX helpers -----
    // Décode un seul item RMT en bit Manchester.
    // Retourne 0 ou 1, -1 si durée incohérente.
    int  decodeItem(const rmt_item32_t &item, uint16_t halfBit);

    // Décode 8 items consécutifs en un octet. Retourne false si erreur.
    bool decodeByte(const rmt_item32_t *items, int &idx, int total,
                    uint8_t &byteOut, uint16_t halfBit);

    // ----- CRC-16/CCITT -----
    uint16_t CalculateCRC(uint8_t *data, size_t length);

    // Octets de synchronisation
    static constexpr uint8_t PREAMBLE    = 0x55;
    static constexpr uint8_t FLAG        = 0x7E;
};

#endif // MANCHESTER_H
