#ifndef MANCHESTER_H
#define MANCHESTER_H

#include <Arduino.h>

#define TYPE_START 0x01
#define TYPE_DATA 0x02
#define TYPE_END 0x03
#define TYPE_NACK 0x04

class Manchester
{
private:
    int pinNum;
    bool isReceiver;
    int bitDuration = 500; // Durée d'un bit en microsecondes
    uint8_t startByte = 0x55; // Octet de démarrage pour la synchronisation
    uint8_t startEndByte = 0x7E; // 

    void TransmitByte(uint8_t byte);
    int ReceiveBit(uint32_t timeoutUs);
    bool ReceiveByte(uint8_t &byteOut, uint32_t timeoutUs);
    uint16_t CalculateCRC(uint8_t *data, size_t length);

public:
    Manchester(int _pinNum, bool _isReceiver);
    ~Manchester();

    void TransmitFrame(uint8_t type, uint8_t seq, uint8_t vol, uint8_t *payload, uint8_t payloadLen);
    void TransmitMessage(uint8_t *message, size_t length);
    int ReceiveFrame(uint8_t *payload, uint8_t &type, uint8_t &seq, uint8_t &vol);
};

#endif