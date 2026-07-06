#include "Manchester.h"

// ============================================================
//  Manchester — Implémentation (RMT via Pilote)
//
//  Convention unique dans tout ce fichier :
//    bit 1 → HIGH puis LOW  (level0=1, level1=0)
//    bit 0 → LOW  puis HIGH (level0=0, level1=1)
// ============================================================

// ------------------------------------------------------------
//  Constructeur / Destructeur
// ------------------------------------------------------------

Manchester::Manchester(int pinTx, int pinRx, rmt_channel_t chTx, rmt_channel_t chRx)
{
    _pilote = new Pilote(pinTx, pinRx, chTx, chRx);

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
//
//  Convention : bit 1 → HIGH→LOW  (front descendant au milieu)
//               bit 0 → LOW→HIGH  (front montant au milieu)
//
//  CORRECTIF bug #1 : la version originale encodait bit 1 en
//  LOW→HIGH, mais decodeItem interprétait HIGH→LOW comme bit 1.
//  Les deux extrémités étaient inversées l'une par rapport à
//  l'autre. Ici TX et RX utilisent la même convention.
// ------------------------------------------------------------
void Manchester::encodeBit(rmt_item32_t *items, int &idx, bool bit)
{
    uint16_t h = Pilote::halfBit();

    if (bit) {
        // Bit 1 : HIGH → LOW
        items[idx].level0    = 1;   // HIGH
        items[idx].duration0 = h;
        items[idx].level1    = 0;   // LOW
        items[idx].duration1 = h;
    } else {
        // Bit 0 : LOW → HIGH
        items[idx].level0    = 0;   // LOW
        items[idx].duration0 = h;
        items[idx].level1    = 1;   // HIGH
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
//  DebugCorruptNextFrame — TEST UNIQUEMENT
//  Arme la corruption de la prochaine trame émise.
// ------------------------------------------------------------
void Manchester::DebugCorruptNextFrame()
{
    _debugCorruptNext = true;
}

// ------------------------------------------------------------
//  TransmitFrame — construit et émet une trame complète
//
//  Format sur le fil :
//    [PREAMBLE 0x55][FLAG 0x7E][TYPE][SEQ][LEN][VOL][PAYLOAD…][CRC_H][CRC_L][FLAG 0x7E]
// ------------------------------------------------------------
void Manchester::TransmitFrame(uint8_t type, uint8_t seq, uint8_t vol,
                                uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen > MAX_PAYLOAD) payloadLen = MAX_PAYLOAD;

    // Calcul du CRC sur [TYPE, SEQ, LEN, VOL, PAYLOAD…]
    uint8_t crcBuf[4 + MAX_PAYLOAD];
    crcBuf[0] = type;
    crcBuf[1] = seq;
    crcBuf[2] = payloadLen;
    crcBuf[3] = vol;
    if (payload && payloadLen > 0) {
        memcpy(crcBuf + 4, payload, payloadLen);
    }
    uint16_t crc = CalculateCRC(crcBuf, 4 + payloadLen);

    // ----------------------------------------------------------
    // TEST UNIQUEMENT : hook de corruption.
    // On modifie le CRC ICI, APRÈS qu'il ait été calculé sur les
    // vraies données. Le CRC transmis ne correspondra donc plus
    // aux données reçues : ReceiveFrame() retournera -8 (CRC
    // invalide) côté RX, ce qui est une vraie erreur de détection,
    // pas juste un contenu faux avec un CRC qui "matche quand même".
    // ----------------------------------------------------------
    if (_debugCorruptNext) {
        _debugCorruptNext = false;
        crc ^= 0x0001;
    }

    // Construction des items RMT
    int totalBytes = 2 + 4 + payloadLen + 2 + 1;  // sync+hdr+payload+crc+flag_fin
    int totalItems = totalBytes * 8;

    rmt_item32_t *items = (rmt_item32_t *)malloc(totalItems * sizeof(rmt_item32_t));
    if (!items) return;

    int idx = 0;

    encodeByte(items, idx, PREAMBLE);   // Préambule 0x55

    encodeByte(items, idx, FLAG);        // Flag de début 0x7E

    encodeByte(items, idx, type);        // Entête
    encodeByte(items, idx, seq);
    encodeByte(items, idx, payloadLen);
    encodeByte(items, idx, vol);

    for (int i = 0; i < payloadLen; i++) {  // Payload
        encodeByte(items, idx, payload[i]);
    }

    encodeByte(items, idx, (crc >> 8) & 0xFF);  // CRC
    encodeByte(items, idx, crc & 0xFF);

    encodeByte(items, idx, FLAG);        // Flag de fin 0x7E

    // Émission bloquante : on attend que le RMT ait tout envoyé
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

    // Le délai inter-trames doit dépasser idle_threshold du RMT RX
    // (défini dans Pilote.cpp : HALF_BIT_US * 20 = 2500 µs à 125 µs/demi-bit).
    // On utilise 6 ms pour avoir une marge confortable quelle que soit
    // la résolution du tick FreeRTOS (1 tick = 1 ms par défaut).
    const TickType_t interFrameDelay = 5 / portTICK_PERIOD_MS;

    TransmitFrame(TYPE_START, 0, totalPackets, nullptr, 0);
    vTaskDelay(interFrameDelay);

    size_t offset = 0;
    uint8_t seq = 1;
    while (offset < length) {
        uint8_t chunkLen = (uint8_t)((length - offset > MAX_PAYLOAD)
                            ? MAX_PAYLOAD
                            : (length - offset));
        TransmitFrame(TYPE_DATA, seq, 0, message + offset, chunkLen);
        offset += chunkLen;
        seq++;
        vTaskDelay(interFrameDelay);
    }

    TransmitFrame(TYPE_END, seq, 0, nullptr, 0);
}

// ============================================================
//  RX
// ============================================================

// ------------------------------------------------------------
//  decodeItem — interprète un item RMT en bit Manchester
//
//  CORRECTIF bug #1 : la version originale avait la convention
//  INVERSE de encodeBit.  Ici on applique la même règle :
//    HIGH→LOW (level0=1, level1=0) → bit 1
//    LOW→HIGH (level0=0, level1=1) → bit 0
//
//  Retourne 0 ou 1, -1 si durée incohérente.
// ------------------------------------------------------------
int Manchester::decodeItem(const rmt_item32_t &item, uint16_t halfBit)
{
    float tol = (float)halfBit * RMT_TOLERANCE;
    float lo  = halfBit - tol;
    float hi  = halfBit + tol;

    bool d0ok = item.duration0 >= lo && item.duration0 <= hi;
    bool d1ok = item.duration1 >= lo && item.duration1 <= hi;

    if (!d0ok || !d1ok) return -1;

    // Convention : HIGH→LOW = bit 1, LOW→HIGH = bit 0
    if (item.level0 == 1 && item.level1 == 0) return 1;
    if (item.level0 == 0 && item.level1 == 1) return 0;

    return -1;
}

// ------------------------------------------------------------
//  decodeByte — convertit 8 items RMT consécutifs en un octet
//
//  CORRECTIF bug #1 : toutes les comparaisons de niveaux sont
//  alignées sur HIGH→LOW = bit 1, LOW→HIGH = bit 0.
//
//  Gère les items "doubles" produits par le RMT quand deux
//  demi-bits consécutifs de même niveau sont fusionnés :
//    d0 double : deux demi-bits du même niveau dans duration0
//    d1 double : le bit suivant commence dans duration1
// ------------------------------------------------------------
bool Manchester::decodeByte(const rmt_item32_t *items, int &idx, int total,
                             uint8_t &byteOut, uint16_t halfBit)
{
    byteOut = 0;
    float tol = (float)halfBit * RMT_TOLERANCE;
    float lo  = halfBit     - tol;
    float hi  = halfBit     + tol;
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
            if      (l0 == 1 && l1 == 0) byteOut |= (1 << b);  // bit 1
            else if (l0 == 0 && l1 == 1) { /* bit 0, rien à faire */ }
            else return false;
            idx++;
        }
        else if (d0d && d1h) {
            // d0 est double : deux demi-bits du même niveau fusionnés dans d0.
            // Le front de milieu de bit est dans la transition vers d1.
            // l1 = niveau APRÈS le front = second demi-bit.
            //   l0=1 (HIGH long) puis l1=0 (LOW court) → bit 1
            //   l0=0 (LOW long)  puis l1=1 (HIGH court) → bit 0
            if      (l0 == 1 && l1 == 0) byteOut |= (1 << b);  // bit 1
            else if (l0 == 0 && l1 == 1) { /* bit 0 */ }
            else return false;
            idx++;
        }
        else if (d0h && d1d) {
            // d1 est double : le bit actuel se termine dans d0,
            // et d1 contient déjà le premier demi-bit du bit suivant.
            //   l0=1 (HIGH court) puis l1=0 (LOW long) → bit 1
            //   l0=0 (LOW court)  puis l1=1 (HIGH long) → bit 0
            if      (l0 == 1 && l1 == 0) byteOut |= (1 << b);  // bit 1
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
//    1. Lecture du ring-buffer RMT (items bruts)
//    2. Expansion en demi-bits individuels (niveau + durée)
//    3. Auto-baud sur les premières durées
//    4. Subdivision des demi-bits doubles
//    5. Reconstruction du bitstream (paires de demi-bits → bit)
//    6. Recherche du FLAG 0x7E
//    7. Lecture séquentielle entête / payload / CRC
//    8. Validation CRC
// ------------------------------------------------------------
int Manchester::ReceiveFrame(uint8_t *payload, uint8_t &type, uint8_t &seq, uint8_t &vol)
{
    static rmt_item32_t items[RMT_ITEMS_MAX];

    int count = _pilote->readItems(items, RMT_ITEMS_MAX, 500);
    if (count <= 0) return -1;

    // -----------------------------------------------------------
    // Étape 1 : expansion des items en demi-bits individuels
    // Chaque item RMT = une paire (level0/duration0, level1/duration1).
    // On les sépare pour manipuler chaque demi-bit indépendamment.
    // -----------------------------------------------------------
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

    // -----------------------------------------------------------
    // Étape 2 : auto-baud sur les 16 premières durées
    // Mesure la durée moyenne des premiers demi-bits pour s'adapter
    // aux dérives d'horloge entre l'émetteur et le récepteur.
    // -----------------------------------------------------------
    uint32_t sum = 0;
    int n = 0;
    for (int i = 0; i < hbCount && n < 16; i++) {
        sum += hb[i].dur;
        n++;
    }
    uint16_t halfBit = (uint16_t)(sum / n);
    if (halfBit < 10 || halfBit > 5000) return -2;

    float tol = halfBit * 0.40f;

    // -----------------------------------------------------------
    // Étape 3 : subdivision des demi-bits doubles
    // Quand deux demi-bits consécutifs ont le même niveau, le RMT
    // les fusionne en un seul item de durée ≈ 2×halfBit.
    // On les redécoupe ici pour obtenir un flux uniforme.
    // -----------------------------------------------------------
    static HalfBitEntry hb2[RMT_ITEMS_MAX * 4];
    int hb2Count = 0;

    for (int i = 0; i < hbCount; i++) {
        float dur = hb[i].dur;
        if (dur >= halfBit * 2 - tol * 2 && dur <= halfBit * 2 + tol * 2) {
            hb2[hb2Count++] = { hb[i].level, (uint16_t)halfBit };
            hb2[hb2Count++] = { hb[i].level, (uint16_t)halfBit };
        } else {
            hb2[hb2Count++] = hb[i];
        }
    }

    // -----------------------------------------------------------
    // Étape 4 : reconstruction du bitstream
    // Chaque bit Manchester = deux demi-bits consécutifs de niveaux opposés.
    //
    // CORRECTIF bug #1 : convention alignée sur encodeBit :
    //   HIGH→LOW (1→0) = bit 1
    //   LOW→HIGH (0→1) = bit 0
    //
    // Si deux demi-bits consécutifs ont le même niveau, on est
    // désynchronisé : on avance d'un demi-bit pour se recaler.
    // -----------------------------------------------------------
    static uint8_t bitStream[RMT_ITEMS_MAX * 2];
    int bitCount = 0;

    int i = 0;
    while (i + 1 < hb2Count) {
        uint8_t l0 = hb2[i].level;
        uint8_t l1 = hb2[i+1].level;

        if (l0 == 1 && l1 == 0) {
            bitStream[bitCount++] = 1;  // HIGH→LOW = bit 1
            i += 2;
        } else if (l0 == 0 && l1 == 1) {
            bitStream[bitCount++] = 0;  // LOW→HIGH = bit 0
            i += 2;
        } else {
            // Niveaux identiques : désynchronisation, avance d'un demi-bit
            i++;
        }
    }

    if (bitCount < 8) return -3;

    // -----------------------------------------------------------
    // Étape 5 : recherche du FLAG 0x7E dans le bitstream
    // On fait glisser un registre à décalage de 8 bits jusqu'à
    // trouver la valeur 0x7E (01111110).
    // -----------------------------------------------------------
    int flagEnd = -1;
    uint8_t shiftReg = 0;
    for (int b = 0; b < bitCount; b++) {
        shiftReg = (shiftReg << 1) | bitStream[b];
        if (shiftReg == FLAG) {
            flagEnd = b + 1;  // premier bit après le FLAG
            break;
        }
    }

    if (flagEnd < 0) return -3;

    // -----------------------------------------------------------
    // Étape 6 : lecture séquentielle depuis le bitstream
    // Lambda local pour lire un octet 8 bits MSB first.
    // MSB first cohérent avec encodeByte (boucle i=7 downto 0).
    // -----------------------------------------------------------
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
    if (!readByte(pos, len))  return -4;
    if (len > MAX_PAYLOAD)    return -5;

    if (!readByte(pos, vol))  return -4;

    // -----------------------------------------------------------
    // Étape 7 : lecture du payload
    //
    // CORRECTIF bug #2 : la version originale avait DEUX boucles
    // identiques qui lisaient le payload. La deuxième tentait de
    // lire len octets supplémentaires là où il ne restait que le
    // CRC, ce qui retournait systématiquement -6. Supprimée.
    // -----------------------------------------------------------
    uint8_t crcBuf[4 + MAX_PAYLOAD];
    crcBuf[0] = type;
    crcBuf[1] = seq;
    crcBuf[2] = len;
    crcBuf[3] = vol;

    for (int k = 0; k < len; k++) {
        if (!readByte(pos, payload[k])) return -6;
        crcBuf[4 + k] = payload[k];
    }

    // Lecture et vérification du CRC
    uint8_t crcHigh, crcLow;
    if (!readByte(pos, crcHigh)) return -7;
    if (!readByte(pos, crcLow))  return -7;

    uint16_t receivedCRC = ((uint16_t)crcHigh << 8) | crcLow;
    uint16_t computedCRC = CalculateCRC(crcBuf, 4 + len);

    if (receivedCRC != computedCRC) return -8;

    // Réinitialise le récepteur RMT explicitement pour la prochaine trame.
    // Sans cela, des résidus dans le ring buffer peuvent perturber le
    // décodage si l'émetteur enchaîne rapidement.
    _pilote->startRx();

    return (int)len;
}

void Manchester::TransmitOutOfSyncMessage(uint8_t seq) {
    TransmitFrame(TYPE_OUT_OF_SYNC, 0, seq, NULL, 0);
}

void Manchester::TransmitNACKResendMessage(uint8_t *message, size_t length, uint8_t wantedSeq) {

    uint8_t totalPackets = (uint8_t)((length + MAX_PAYLOAD - 1) / MAX_PAYLOAD);
    if (totalPackets == 0) totalPackets = 1;

    // Le délai inter-trames doit dépasser idle_threshold du RMT RX
    // (défini dans Pilote.cpp : HALF_BIT_US * 20 = 2500 µs à 125 µs/demi-bit).
    // On utilise 6 ms pour avoir une marge confortable quelle que soit
    // la résolution du tick FreeRTOS (1 tick = 1 ms par défaut).
    const TickType_t interFrameDelay = 5 / portTICK_PERIOD_MS;

    TransmitFrame(TYPE_START, 0, totalPackets, nullptr, 0);
    vTaskDelay(interFrameDelay);

    size_t offset = 0;
    uint8_t seq = 1;
    while (offset < length) {
        uint8_t chunkLen;
        if(wantedSeq == seq)
        {
            chunkLen = (uint8_t)((length - offset > MAX_PAYLOAD)
                            ? MAX_PAYLOAD
                            : (length - offset));
            TransmitFrame(TYPE_DATA, seq, 0, message + offset, chunkLen);
            vTaskDelay(interFrameDelay);
            break;
        }
        offset += chunkLen;
        seq++;
    }

    TransmitFrame(TYPE_END, seq, 0, nullptr, 0);
}
