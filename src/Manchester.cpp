#include "Manchester.h"

#define TYPE_START 0x01
#define TYPE_DATA 0x02
#define TYPE_END 0x03
#define TYPE_OUT_OF_SYNC 0x04

Manchester::Manchester(int _pinNum, bool _isReceiver)
{
    pinNum = _pinNum;
    isReceiver = _isReceiver;
    if(isReceiver) {
        pinMode(pinNum, INPUT);
    } else {
        pinMode(pinNum, OUTPUT);
        digitalWrite(pinNum, LOW);
    }
}

Manchester::~Manchester() {}

void Manchester::TransmitByte(uint8_t byte)
{
    for(int i = 0; i < 8; i++) {
        bool bit = (byte >> (7 - i)) & 0x01;
        if(bit) {
            digitalWrite(pinNum, HIGH);
            delayMicroseconds(bitDuration / 2); 
            digitalWrite(pinNum, LOW);
            delayMicroseconds(bitDuration / 2); 
        } else {
            digitalWrite(pinNum, LOW);
            delayMicroseconds(bitDuration / 2); 
            digitalWrite(pinNum, HIGH);
            delayMicroseconds(bitDuration / 2); 
        }
    }
}

uint16_t Manchester::CalculateCRC(uint8_t *data, size_t length) 
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

void Manchester::TransmitFrame(uint8_t type, uint8_t seq, uint8_t vol, uint8_t *payload, uint8_t payloadLen) 
{
    digitalWrite(pinNum, LOW);
    delayMicroseconds(bitDuration * 2); 

    if(payloadLen > 80) payloadLen = 80; 

    uint8_t crcData[84]; 
    crcData[0] = type; crcData[1] = seq; crcData[2] = payloadLen; crcData[3] = vol;
    for(int i = 0; i < payloadLen; i++) crcData[4+i] = payload[i];
    
    uint16_t crc = CalculateCRC(crcData, 4 + payloadLen);

    TransmitByte(startByte);
    TransmitByte(startEndByte);
    for(int i = 0; i < 4 + payloadLen; i++) TransmitByte(crcData[i]);
    TransmitByte((crc >> 8) & 0xFF);
    TransmitByte(crc & 0xFF);
    TransmitByte(startEndByte);
}

int Manchester::ReceiveBit(uint32_t timeoutUs) 
{
    uint32_t timeout = micros();
    bool lastState = digitalRead(pinNum);
    
    while(digitalRead(pinNum) == lastState) {
        if (micros() - timeout > timeoutUs) return -1; 
    }
    
    bool newState = digitalRead(pinNum);
    int bit = (newState == LOW) ? 1 : 0; 
    
    uint32_t waitStart = micros();
    // Utilisation de la variable bitDuration qui a été calculée dynamiquement !
    while(micros() - waitStart < (bitDuration * 0.75)) {
        // Attente active
    }
    
    return bit;
}

bool Manchester::ReceiveByte(uint8_t &byteOut, uint32_t timeoutUs) 
{
    uint8_t currentByte = 0;
    for(int b = 0; b < 8; b++) {
        int bit = ReceiveBit(timeoutUs);
        if (bit < 0) return false;
        
        currentByte |= (bit << (7 - b));
        timeoutUs = bitDuration * 1.5; 
    }
    byteOut = currentByte;
    return true;
}

int Manchester::ReceiveFrame(uint8_t *payload, uint8_t &type, uint8_t &seq, uint8_t &vol) 
{
    uint32_t t[5];
    bool syncedT = false;
    int bitDurationReceive;
    
    // ---------------------------------------------------------
    // 1. AUTO-BAUD : Mesure dynamique de la durée d'un bit
    // ---------------------------------------------------------
    while (!syncedT) {
        uint32_t timeout = micros();
        bool lastState = digitalRead(pinNum);
        
        // Attente du tout premier front (réveil de la ligne)
        while(digitalRead(pinNum) == lastState) {
            if (micros() - timeout > 3000000) return -1; // Silence
        }
        t[0] = micros();
        lastState = digitalRead(pinNum);
        
        bool good = true;
        // Capture de 4 fronts consécutifs du préambule 0x55
        for(int i = 1; i < 5; i++) {
            timeout = micros();
            while(digitalRead(pinNum) == lastState) {
                // Timeout généreux (20ms) pour supporter des vitesses très lentes
                if (micros() - timeout > 20000) { good = false; break; } 
            }
            if (!good) break;
            t[i] = micros();
            lastState = digitalRead(pinNum);
        }
        
        if (good) {
            // Calcul des temps entre les fronts
            uint32_t d1 = t[1] - t[0];
            uint32_t d2 = t[2] - t[1];
            uint32_t d3 = t[3] - t[2];
            uint32_t d4 = t[4] - t[3];
            
            // Si les durées sont constantes (tolérance de 20%), on a trouvé la vitesse !
            uint32_t avg = (d1 + d2 + d3 + d4) / 4;
            uint32_t maxDiff = avg / 5; 
            
            if (abs((int)(d1 - avg)) < maxDiff &&
                abs((int)(d2 - avg)) < maxDiff &&
                abs((int)(d3 - avg)) < maxDiff &&
                abs((int)(d4 - avg)) < maxDiff) 
            {
                bitDurationReceive = avg; // <-- LA MAGIE EST ICI : On écrase la vitesse
                syncedT = true;
            }
        }
    }

    // À ce stade, nous sommes pile poil après une transition de milieu de bit.
    // On doit faire notre saut de 75% pour se synchroniser avec la boucle ReceiveBit().
    uint32_t waitStart = micros();
    while(micros() - waitStart < (bitDuration * 0.75)) { }


    // ---------------------------------------------------------
    // 2. CHASSE AU START (0x7E)
    // ---------------------------------------------------------
    uint8_t syncReg = 0;
    bool startFound = false;
    
    // On a "mangé" environ 5 bits du préambule pour chronométrer.
    // On continue de lire bit par bit jusqu'à trouver le fameux 0x7E.
    for(int attempts = 0; attempts < 20; attempts++) {
        int bit = ReceiveBit(bitDurationReceive * 2);
        if (bit < 0) return -2; 
        
        syncReg = (syncReg << 1) | bit;
        if ((syncReg & 0xFF) == startEndByte) {
            startFound = true;
            break;
        }
    }
    
    if (!startFound) return -3; // 0x7E introuvable après le préambule

    // ---------------------------------------------------------
    // 3. LECTURE CLASSIQUE (Entête, Payload, CRC)
    // ---------------------------------------------------------
    if(!ReceiveByte(type, bitDurationReceive * 1.5)) return -4;
    if(!ReceiveByte(seq, bitDurationReceive * 1.5)) return -4;
    
    uint8_t len;
    if(!ReceiveByte(len, bitDurationReceive * 1.5) || len > 80) return -5;
    
    if(!ReceiveByte(vol, bitDurationReceive * 1.5)) return -4; 

    uint8_t crcData[84];
    crcData[0] = type; crcData[1] = seq; crcData[2] = len; crcData[3] = vol;
    
    for(int i = 0; i < len; i++) {
        if(!ReceiveByte(payload[i], bitDurationReceive * 1.5)) return -6; 
        crcData[4+i] = payload[i];
    }

    uint8_t crcHigh, crcLow;
    if(!ReceiveByte(crcHigh, bitDurationReceive * 1.5) || !ReceiveByte(crcLow, bitDurationReceive * 1.5)) return -7;
    
    uint16_t receivedCRC = (crcHigh << 8) | crcLow;
    if(CalculateCRC(crcData, 4 + len) != receivedCRC) return len; 

    uint8_t endByte;
    if(!ReceiveByte(endByte, bitDurationReceive * 1.5) || endByte != startEndByte) return len; 

    return len; 
}

void Manchester::TransmitMessage(uint8_t *message, size_t length) 
{
    uint8_t totalPackets = (length + 79) / 80;
    if (totalPackets == 0) totalPackets = 1; 

    TransmitFrame(TYPE_START, 0, totalPackets, NULL, 0);
    delay(10); 

    size_t offset = 0;
    uint8_t seq = 1;

    while (offset < length) {
        uint8_t chunkLen = (length - offset > 80) ? 80 : (length - offset);
        TransmitFrame(TYPE_DATA, seq, 0, message + offset, chunkLen);
        offset += chunkLen;
        seq++;
        delay(10); 
    }

    TransmitFrame(TYPE_END, seq, 0, NULL, 0);
}

void Manchester::TransmitOutOfSyncMessage(uint8_t seq) {
    TransmitFrame(TYPE_OUT_OF_SYNC, 0, seq, NULL, 0);
}