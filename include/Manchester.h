#ifndef MANCHESTER_H
#define MANCHESTER_H
#include <Arduino.h>
#include "driver/rmt.h"
#include "Pilote.h"

#define TYPE_START 0x01
#define TYPE_DATA  0x02
#define TYPE_END   0x03
#define TYPE_OUT_OF_SYNC 0x04
#define MAX_PAYLOAD 30
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
void TransmitOutOfSyncMessage(uint8_t seq);
void TransmitNACKResendMessage(uint8_t *message, size_t length, uint8_t wantedSeq);

    // ------------------------------------------------------------
    // TEST UNIQUEMENT :
    // Arme la corruption de la PROCHAINE trame émise (TransmitFrame),
    // quel que soit son type/seq. La corruption est appliquée sur le
    // CRC APRÈS son calcul sur les données réelles : le CRC transmis
    // ne correspondra donc plus aux données transmises, ce qui
    // déclenche une vraie erreur CRC (-8) côté ReceiveFrame.
    // ------------------------------------------------------------
    void DebugCorruptNextFrame();

    // ------------------------------------------------------------
    // TEST UNIQUEMENT :
    // Arme la corruption du paquet DATA dont le numéro de séquence
    // vaut exactement `seq` (1-based, celui utilisé par
    // TransmitMessage). Peu importe qu'il ait déjà commencé à être
    // envoyé ou non : dès que TransmitFrame(TYPE_DATA, seq, ...)
    // correspondant est appelé, son CRC est corrompu. Se désarme
    // automatiquement après usage.
    // ------------------------------------------------------------
    void DebugCorruptFrame(uint8_t seq);

private:
Pilote *_pilote;
    bool    _debugCorruptNext     = false; // TEST : DebugCorruptNextFrame()
    bool    _debugCorruptSeqArmed = false; // TEST : DebugCorruptFrame(seq)
    uint8_t _debugCorruptSeq      = 0;     // TEST : seq ciblé par DebugCorruptFrame()

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
