#include "Manchester.h"

#define TYPE_START 0x01
#define TYPE_DATA 0x02
#define TYPE_END 0x03
#define TYPE_NACK 0x04

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
    while(micros() - waitStart < (bitDurationReceive * 0.75)) {
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
        timeoutUs = bitDurationReceive * 1.5; 
    }
    byteOut = currentByte;
    return true;
}

uint32_t Manchester::MeasureBitDuration() {
    uint32_t t[8]; 
    uint32_t startWait = micros();
    bool lastState = digitalRead(pinNum);
    
    // Attente initiale (Timeout défini par une constante, par exemple 3s)
    while(digitalRead(pinNum) == lastState) {
        if (micros() - startWait > 3000000UL) return 0; 
    }
    
    // Capture des 8 transitions du PREAMBLE_BYTE
    for(int i = 0; i < 8; i++) {
        uint32_t timeout = micros();
        while(digitalRead(pinNum) == lastState) {
            if (micros() - timeout > 10000UL) return 0; 
        }
        t[i] = micros();
        lastState = digitalRead(pinNum);
    }
    
    uint32_t totalTime = t[7] - t[0]; // Durée totale pour 7 demi-bits
    return (totalTime / 7) * 2;       // Conversion en durée de bit complet
}

int Manchester::ReceiveFrame(uint8_t *payload, uint8_t &type, uint8_t &seq, uint8_t &vol) 
{
    // 1. AUTO-BAUD : Mesure dynamique
    uint32_t measured = MeasureBitDuration();
    if (measured == 0) return -1; // Échec sync
    
    bitDurationReceive = measured;
    // Serial.printf("Sync réussie ! BitDuration détectée: %d us\n", bitDurationReceive);

    // 2. Chasse au START (0x7E)
    // On doit faire un petit délai car on est au milieu d'un bit du préambule
    delayMicroseconds(bitDurationReceive * 0.75); 

    uint8_t syncReg = 0;
    bool startFound = false;
    
    for(int attempts = 0; attempts < 32; attempts++) {
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
    crcData[0] = type;
    crcData[1] = seq; 
    crcData[2] = len; 
    crcData[3] = vol;
    
    for(int i = 0; i < len; i++) {
        if(!ReceiveByte(payload[i], bitDurationReceive * 1.5)) return -6; 
        crcData[4+i] = payload[i];
    }

    uint8_t crcHigh, crcLow;
    if(!ReceiveByte(crcHigh, bitDurationReceive * 1.5) || !ReceiveByte(crcLow, bitDurationReceive * 1.5)) return -7;
    
    uint16_t receivedCRC = (crcHigh << 8) | crcLow;
    if(CalculateCRC(crcData, 4 + len) != receivedCRC) return -8; 

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