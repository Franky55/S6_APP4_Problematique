#include "Manchester.h"

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
    // Nettoyage de la ligne pour éviter les transitions fantômes
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

// --- NOUVELLE LOGIQUE DE RÉCEPTION ---

int Manchester::ReceiveBit(uint32_t timeoutUs) 
{
    uint32_t timeout = micros();
    bool lastState = digitalRead(pinNum);
    
    // Attente de la prochaine transition
    while(digitalRead(pinNum) == lastState) {
        if (micros() - timeout > timeoutUs) return -1; // Timeout
    }
    
    // Lecture de l'état APRÈS la transition
    bool newState = digitalRead(pinNum);
    int bit = (newState == LOW) ? 1 : 0; // H->L = 1, L->H = 0
    
    //Maybe remove this, because it just adds more delay when the code will do clocks, so no need of more delays
    // On avance de 75% d'un bit pour esquiver les rebonds et les transitions de frontières
    // On atterrira parfaitement dans la zone d'attente du prochain milieu de bit
    uint32_t waitStart = micros();
    while(micros() - waitStart < (bitDuration * 0.75)) {
        // Attente active très précise
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
        timeoutUs = bitDuration * 1.5; // On resserre le timeout une fois lancé
    }
    byteOut = currentByte;
    return true;
}

int Manchester::ReceiveFrame(uint8_t *payload, uint8_t &type, uint8_t &seq, uint8_t &vol) 
{
    uint16_t syncReg = 0;
    bool synced = false;
    
    // 1. Chasse au Préambule et au Start en continu (Bit par Bit)
    // On cherche la combinaison exacte 0x55 (Préambule) + 0x7E (Start) = 0x557E
    for(int attempts = 0; attempts < 150; attempts++) {
        int bit = ReceiveBit(attempts == 0 ? 3000000 : bitDuration * 2);
        if (bit < 0) return -1; // Silence
        
        // On pousse le nouveau bit dans le registre de 16 bits
        syncReg = (syncReg << 1) | bit;
        
        if (syncReg == 0x557E) {
            synced = true;
            break;
        }
    }
    
    if (!synced) return -2; // Bruit continu, impossible de trouver 0x557E

    // 2. Lecture de l'entête (On est maintenant parfaitement aligné sur les octets)
    if(!ReceiveByte(type, bitDuration * 1.5)) return -4;
    if(!ReceiveByte(seq, bitDuration * 1.5)) return -4;
    
    uint8_t len;
    if(!ReceiveByte(len, bitDuration * 1.5) || len > 80) return -5;
    
    if(!ReceiveByte(vol, bitDuration * 1.5)) return -4; 

    // 3. Extraction de la charge utile
    uint8_t crcData[84];
    crcData[0] = type; crcData[1] = seq; crcData[2] = len; crcData[3] = vol;
    
    for(int i = 0; i < len; i++) {
        if(!ReceiveByte(payload[i], bitDuration * 1.5)) return -6; 
        crcData[4+i] = payload[i];
    }

    // 4. Validation du CRC
    uint8_t crcHigh, crcLow;
    if(!ReceiveByte(crcHigh, bitDuration * 1.5) || !ReceiveByte(crcLow, bitDuration * 1.5)) return -7;
    
    uint16_t receivedCRC = (crcHigh << 8) | crcLow;
    if(CalculateCRC(crcData, 4 + len) != receivedCRC) return -8; 

    // 5. Validation du End (0x7E)
    uint8_t endByte;
    if(!ReceiveByte(endByte, bitDuration * 1.5) || endByte != startEndByte) return len; 

    return len; // Succès complet !
}

void Manchester::TransmitMessage(uint8_t *message, size_t length) 
{
    // Calcul du nombre total de paquets nécessaires (division entière arrondie vers le haut)
    uint8_t totalPackets = (length + 79) / 80;
    if (totalPackets == 0) totalPackets = 1; 

    // 1. Envoi de la trame de Début (Type 0x01, Séquence 0, Volume = Total de paquets, Pas de charge utile)
    TransmitFrame(0x01, 0, totalPackets, NULL, 0);
    delay(10); // Micro-pause inter-trame pour laisser le récepteur traiter l'information

    // 2. Envoi des trames de Données (Type 0x02, par blocs de 80 octets max)
    size_t offset = 0;
    uint8_t seq = 1;

    while (offset < length) {
        uint8_t chunkLen = (length - offset > 80) ? 80 : (length - offset);
        
        TransmitFrame(0x02, seq, 0, message + offset, chunkLen);
        
        offset += chunkLen;
        seq++;
        delay(10); // Micro-pause inter-trame
    }

    // 3. Envoi de la trame de Fin (Type 0x03, Séquence finale, Volume 0, Pas de charge utile)
    TransmitFrame(0x03, seq, 0, NULL, 0);
}