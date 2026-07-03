#include "Manchester.h"

// ============================================================
//  Manchester — Implémentation (RMT via Pilote)
// ============================================================

// ------------------------------------------------------------
//  Constructeur / Destructeur
// ------------------------------------------------------------

Manchester::Manchester(int pinTx, int pinRx, rmt_channel_t chTx, rmt_channel_t chRx)
{
    _pilote = new Pilote(pinTx, pinRx, chTx, chRx);

    // Démarre l'écoute RX immédiatement si un pin RX est configuré
    if (pinRx >= 0) {
        _pilote->startRx();
    }
}

Manchester::~Manchester()
{
    delete _pilote;
}

// ============================================================
//  TX
// ============================================================

// ------------------------------------------------------------
//  encodeBit
//  Convention Manchester II (IEEE 802.3) :
//    1 → LOW  (durée = halfBit) puis HIGH (durée = halfBit)  → front montant
//    0 → HIGH (durée = halfBit) puis LOW  (durée = halfBit)  → front descendant
// ------------------------------------------------------------
void Manchester::encodeBit(rmt_item32_t *items, int &idx, bool bit)
{
    uint16_t h = Pilote::halfBit();

    if (bit) {
        // Bit 1 : LOW → HIGH
        items[idx].level0    = 0;   // LOW
        items[idx].duration0 = h;
        items[idx].level1    = 1;   // HIGH
        items[idx].duration1 = h;
    } else {
        // Bit 0 : HIGH → LOW
        items[idx].level0    = 1;   // HIGH
        items[idx].duration0 = h;
        items[idx].level1    = 0;   // LOW
        items[idx].duration1 = h;
    }
    idx++;
}

// ------------------------------------------------------------
//  encodeByte — encode un octet MSB first (8 items RMT)
// ------------------------------------------------------------
void Manchester::encodeByte(rmt_item32_t *items, int &idx, uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        encodeBit(items, idx, (byte >> i) & 0x01);
    }
}

// ------------------------------------------------------------
//  CalculateCRC — CRC-16/CCITT (polynôme 0x1021)
// ------------------------------------------------------------
uint16_t Manchester::CalculateCRC(uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

// ------------------------------------------------------------
//  TransmitFrame — construit et émet une trame complète
//
//  Format sur le fil :
//    [PREAMBLE 0x55][FLAG 0x7E][TYPE][SEQ][LEN][VOL][PAYLOAD…][CRC_H][CRC_L][FLAG 0x7E]
//
//  Chaque octet → 8 items RMT Manchester.
//  Total worst-case (80 octets payload) :
//    2 (sync) + 4 (entête) + 80 (payload) + 2 (CRC) + 1 (flag fin) = 89 octets × 8 = 712 items
// ------------------------------------------------------------
void Manchester::TransmitFrame(uint8_t type, uint8_t seq, uint8_t vol,
                                uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen > MAX_PAYLOAD) payloadLen = MAX_PAYLOAD;

    // --- Calcul du CRC sur [TYPE, SEQ, LEN, VOL, PAYLOAD…] ---
    uint8_t crcBuf[4 + MAX_PAYLOAD];
    crcBuf[0] = type;
    crcBuf[1] = seq;
    crcBuf[2] = payloadLen;
    crcBuf[3] = vol;
    if (payload && payloadLen > 0) {
        memcpy(crcBuf + 4, payload, payloadLen);
    }
    uint16_t crc = CalculateCRC(crcBuf, 4 + payloadLen);

    // --- Construction des items RMT ---
    // Taille = (2 sync + 4 entête + payloadLen + 2 CRC + 1 flag fin) octets × 8 items
    int totalBytes = 2 + 4 + payloadLen + 2 + 1;
    int totalItems = totalBytes * 8;

    rmt_item32_t *items = (rmt_item32_t *)malloc(totalItems * sizeof(rmt_item32_t));
    if (!items) return;  // allocation échouée

    int idx = 0;

    // Préambule (synchronisation auto-baud côté récepteur)
    encodeByte(items, idx, PREAMBLE);

    // Flag de début
    encodeByte(items, idx, FLAG);

    // Entête
    encodeByte(items, idx, type);
    encodeByte(items, idx, seq);
    encodeByte(items, idx, payloadLen);
    encodeByte(items, idx, vol);

    // Payload
    for (int i = 0; i < payloadLen; i++) {
        encodeByte(items, idx, payload[i]);
    }

    // CRC
    encodeByte(items, idx, (crc >> 8) & 0xFF);
    encodeByte(items, idx, crc & 0xFF);

    // Flag de fin
    encodeByte(items, idx, FLAG);

    // --- Émission (bloquante : on attend que le RMT ait tout envoyé) ---
    _pilote->sendItems(items, idx, true);

    free(items);
}

// ------------------------------------------------------------
//  TransmitMessage — fragmentation et envoi d'un message complet
// ------------------------------------------------------------
void Manchester::TransmitMessage(uint8_t *message, size_t length)
{
    uint8_t totalPackets = (uint8_t)((length + MAX_PAYLOAD - 1) / MAX_PAYLOAD);
    if (totalPackets == 0) totalPackets = 1;

    // Trame START
    TransmitFrame(TYPE_START, 0, totalPackets, nullptr, 0);
    vTaskDelay(5 / portTICK_PERIOD_MS);   // laisser le RX se préparer

    // Trames DATA
    size_t offset = 0;
    uint8_t seq = 1;
    while (offset < length) {
        uint8_t chunkLen = (uint8_t)((length - offset > MAX_PAYLOAD)
                            ? MAX_PAYLOAD
                            : (length - offset));
        TransmitFrame(TYPE_DATA, seq, 0, message + offset, chunkLen);
        offset  += chunkLen;
        seq++;
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    // Trame END
    TransmitFrame(TYPE_END, seq, 0, nullptr, 0);
}

// ============================================================
//  RX
// ============================================================

// ------------------------------------------------------------
//  decodeItem — interprète un item RMT en bit Manchester
//
//  Un item RMT contient deux demi-périodes (level0/duration0,
//  level1/duration1).  En Manchester II :
//    level0=LOW,  level1=HIGH → bit 1
//    level0=HIGH, level1=LOW  → bit 0
//
//  On vérifie que les deux durées sont proches de halfBit
//  (±TOLERANCE) pour rejeter les items bruités.
// ------------------------------------------------------------
int Manchester::decodeItem(const rmt_item32_t &item, uint16_t halfBit)
{
    float tol = (float)halfBit * RMT_TOLERANCE;
    float lo  = halfBit - tol;
    float hi  = halfBit + tol;

    bool d0h = item.duration0 >= lo && item.duration0 <= hi;
    bool d1h = item.duration1 >= lo && item.duration1 <= hi;

    if (!d0h || !d1h) return -1;

    // Convention corrigée : HIGH→LOW = bit 1, LOW→HIGH = bit 0
    if (item.level0 == 1 && item.level1 == 0) return 1;
    if (item.level0 == 0 && item.level1 == 1) return 0;

    return -1;
}

// ------------------------------------------------------------
//  decodeByte — convertit 8 items RMT consécutifs en un octet
// ------------------------------------------------------------
bool Manchester::decodeByte(const rmt_item32_t *items, int &idx, int total,
                             uint8_t &byteOut, uint16_t halfBit)
{
    byteOut = 0;
    float tol = (float)halfBit * RMT_TOLERANCE;
    float lo  = halfBit - tol;
    float hi  = halfBit + tol;
    float lo2 = halfBit * 2 - tol * 2;
    float hi2 = halfBit * 2 + tol * 2;

    for (int b = 7; b >= 0; b--) {
        if (idx >= total) return false;

        uint16_t d0 = items[idx].duration0;
        uint16_t d1 = items[idx].duration1;
        uint8_t  l0 = items[idx].level0;
        uint8_t  l1 = items[idx].level1;

        bool d0h = d0 >= lo  && d0 <= hi;
        bool d1h = d1 >= lo  && d1 <= hi;
        bool d0d = d0 >= lo2 && d0 <= hi2;
        bool d1d = d1 >= lo2 && d1 <= hi2;

        if (d0h && d1h) {
            // Item normal : un front propre au milieu du bit
            // HIGH→LOW = bit 1, LOW→HIGH = bit 0
            if      (l0 == 1 && l1 == 0) byteOut |= (1 << b);
            else if (l0 == 0 && l1 == 1) { /* bit 0 */ }
            else return false;
            idx++;
        }
        else if (d0d && d1h) {
            // d0 est double : deux demi-bits du même niveau fusionnés par le RMT.
            // Le front de milieu de bit est dans la transition vers d1.
            // l1 = état APRÈS le front = état de la 2e moitié du bit.
            // HIGH→LOW : l0=1, après fusion l1=0 → bit 1
            // LOW→HIGH : l0=0, après fusion l1=1 → bit 0
            if      (l0 == 1 && l1 == 0) byteOut |= (1 << b);
            else if (l0 == 0 && l1 == 1) { /* bit 0 */ }
            else return false;
            idx++;
        }
        else if (d0h && d1d) {
            // d1 est double : le bit actuel est dans d0, le bit suivant
            // commence dans d1 (même niveau). On consomme cet item
            // et on laisse le prochain item traiter la suite.
            if      (l0 == 1 && l1 == 0) byteOut |= (1 << b);
            else if (l0 == 0 && l1 == 1) { /* bit 0 */ }
            else return false;
            idx++;
        }
        else {
            return false;
        }
    }
    return true;
}

// ------------------------------------------------------------
//  ReceiveFrame — lit une trame complète depuis le ring-buffer RMT
//
//  Algorithme :
//    1. Lecture du buffer RMT (items bruts, durées en µs)
//    2. Auto-baud sur le préambule 0x55 : mesure des durées
//       pour calculer halfBit réel (robustesse aux dérives)
//    3. Balayage bit à bit pour détecter le FLAG 0x7E
//    4. Lecture séquentielle de l'entête, du payload et du CRC
//    5. Validation CRC
// ------------------------------------------------------------
int Manchester::ReceiveFrame(uint8_t *payload, uint8_t &type, uint8_t &seq, uint8_t &vol)
{
    static rmt_item32_t items[RMT_ITEMS_MAX];

    int count = _pilote->readItems(items, RMT_ITEMS_MAX, 500);
if (count <= 0) return -1;
Serial.printf("[DEBUG] count=%d\n", count);

    if (count <= 0) return -1;

    // -------------------------------------------------------
    // Le RMT RX produit des items où chaque item = une paire
    // de demi-périodes consécutives. En pratique sur ESP32 :
    //   level0 = niveau du premier demi-bit
    //   level1 = niveau du deuxième demi-bit
    // On décode en expandant chaque item en demi-bits individuels,
    // puis on regroupe par paires pour reconstruire les bits Manchester.
    // -------------------------------------------------------

    // Étape 1 : expand tous les items en demi-bits (niveau + durée)
    // Max demi-bits = RMT_ITEMS_MAX * 2
    struct HalfBitEntry { uint8_t level; uint16_t dur; };
    static HalfBitEntry hb[RMT_ITEMS_MAX * 2];
    int hbCount = 0;

    for (int i = 0; i < count; i++) {
        if (items[i].duration0 > 0) {
            hb[hbCount++] = { items[i].level0, items[i].duration0 };
        }
        if (items[i].duration1 > 0) {
            hb[hbCount++] = { items[i].level1, items[i].duration1 };
        }
    }

    if (hbCount < 4) return -2;

    // Étape 2 : auto-baud sur les 16 premières durées non nulles
    uint32_t sum = 0; int n = 0;
    for (int i = 0; i < hbCount && n < 16; i++) {
        sum += hb[i].dur; n++;
    }
    uint16_t halfBit = (uint16_t)(sum / n);
    if (halfBit < 10 || halfBit > 5000) return -2;

    float tol = halfBit * 0.40f;

    // Étape 3 : subdivise les demi-bits doubles en deux demi-bits simples
    // Un demi-bit double = durée ≈ halfBit*2 → deux demi-bits consécutifs du même niveau
    static HalfBitEntry hb2[RMT_ITEMS_MAX * 4];
    int hb2Count = 0;

    for (int i = 0; i < hbCount; i++) {
        float dur = hb[i].dur;
        if (dur >= halfBit * 2 - tol * 2 && dur <= halfBit * 2 + tol * 2) {
            // Durée double → deux demi-bits identiques
            hb2[hb2Count++] = { hb[i].level, (uint16_t)halfBit };
            hb2[hb2Count++] = { hb[i].level, (uint16_t)halfBit };
        } else {
            hb2[hb2Count++] = hb[i];
        }
    }

    // Étape 4 : convertit les paires de demi-bits en bits Manchester
    // Chaque bit = deux demi-bits consécutifs de niveaux opposés
    // Convention IEEE 802.3 : LOW→HIGH = bit 1, HIGH→LOW = bit 0
    static uint8_t bitStream[RMT_ITEMS_MAX * 2];
    int bitCount = 0;

    int i = 0;
    while (i + 1 < hb2Count) {
        uint8_t l0 = hb2[i].level;
        uint8_t l1 = hb2[i+1].level;

        if (l0 == 0 && l1 == 1) {
            bitStream[bitCount++] = 1;  // LOW→HIGH = bit 1
            i += 2;
        } else if (l0 == 1 && l1 == 0) {
            bitStream[bitCount++] = 0;  // HIGH→LOW = bit 0
            i += 2;
        } else {
            // Niveaux identiques consécutifs = désynchronisation, avance d'un demi-bit
            i++;
        }
    }

    if (bitCount < 8) return -3;

    // Étape 5 : cherche le FLAG 0x7E dans le bitStream
    int flagEnd = -1;
    uint8_t shiftReg = 0;
    for (int b = 0; b < bitCount; b++) {
        shiftReg = (shiftReg << 1) | bitStream[b];
        if (shiftReg == FLAG) {
            flagEnd = b + 1;  // index du premier bit APRÈS le FLAG
            break;
        }
    }

    if (flagEnd < 0) return -3;

    // Étape 6 : lecture de l'entête et du payload depuis le bitStream
    // Fonction lambda locale pour lire un octet depuis le bitStream
    auto readByte = [&](int &pos, uint8_t &out) -> bool {
        if (pos + 8 > bitCount) return false;
        out = 0;
        for (int b = 7; b >= 0; b--) {
            out |= (bitStream[pos++] << b);
        }
        return true;
    };

    int pos = flagEnd;

    if (!readByte(pos, type)) return -4;
    if (!readByte(pos, seq))  return -4;

    uint8_t len;
    if (!readByte(pos, len))   return -4;
    if (len > MAX_PAYLOAD)     return -5;

    if (!readByte(pos, vol))   return -4;

    uint8_t crcBuf[4 + MAX_PAYLOAD];
    crcBuf[0] = type; crcBuf[1] = seq; crcBuf[2] = len; crcBuf[3] = vol;

    for (int k = 0; k < len; k++) {
    if (!readByte(pos, payload[k])) {
        // DEBUG
        Serial.printf("[DEBUG-6] len=%d k=%d pos=%d bitCount=%d manquant=%d bits\n",
            len, k, pos, bitCount, (len - k) * 8 - (bitCount - pos));
        return -6;
    }
    crcBuf[4 + k] = payload[k];
    }

    for (int k = 0; k < len; k++) {
        if (!readByte(pos, payload[k])) return -6;
        crcBuf[4 + k] = payload[k];
    }

    uint8_t crcHigh, crcLow;
    if (!readByte(pos, crcHigh)) return -7;
    if (!readByte(pos, crcLow))  return -7;

    uint16_t receivedCRC = ((uint16_t)crcHigh << 8) | crcLow;
    uint16_t computedCRC = CalculateCRC(crcBuf, 4 + len);

    if (receivedCRC != computedCRC) return -8;

    return (int)len;
}