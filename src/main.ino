#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "Manchester.h"

// ----- Pins -----
#define GPIO_TX 26
#define GPIO_RX 27
#define MAX_MESSAGES_SENDING 5
#define VAL_NEXT_SEND 5000

// ----- Instance unique Manchester -----
Manchester node(GPIO_TX, GPIO_RX, RMT_CHANNEL_1, RMT_CHANNEL_0);

static SemaphoreHandle_t serialMutex;

volatile bool NACKReceived = false;
volatile int NACKResend = 0;
volatile bool outOfSync = false;
volatile int expected = 0;
volatile bool keepReading = true;
volatile int nextSend = 0;

// ----- Flag de test (volatile car modifié par la task Console) -----
// -3 = désactivé | -2 = "le prochain paquet, peu importe son numéro" | >=1 = numéro de paquet précis à ignorer
volatile int testForceDropSeq = -3;

// ============================================================
//  Task TX — Core 1
// ============================================================
void taskTX(void *pvParameters) {
    uint8_t longMessage[MAX_MESSAGES_SENDING][80] = {
                            {"Premier message de test pour la transmission longue."},
                            {"Deuxieme message qui contient des characteres speciaux : !@#$%^&*()_+"},
                            {"Troisieme message avec des chiffres : 1234567890 42069"},
                            {"Quatrieme message il se peut que je n'ai pas d'inspiration pour ce message"},
                            {"Cinquieme message, le dernier de la liste, mais pas le moins important."}
                        };
    int i = 0;

    while(true) {
        if (NACKReceived)
        {
            vTaskDelay(5 / portTICK_PERIOD_MS);
            i = i - 1;
            if (i < 0)
                i = MAX_MESSAGES_SENDING - 1;
            size_t messageLen = sizeof(longMessage[i]) - 1;
            Serial.printf(">>> NACK reçu. Réémission du message...%d\n", NACKResend);
            NACKReceived = false;
            node.TransmitNACKResendMessage(longMessage[i], messageLen, NACKResend);
            i++;
        } 
        else if (outOfSync)
        {
            vTaskDelay(5 / portTICK_PERIOD_MS);
            Serial.printf(">>> Out of sync...%d\n", expected);
            node.TransmitOutOfSyncMessage(expected);
            outOfSync = false;
        }
        else if(nextSend > VAL_NEXT_SEND)// Envoi message normal
        {
            nextSend = 0;
            if (i >= MAX_MESSAGES_SENDING)
                i = 0;

            size_t messageLen = sizeof(longMessage[i]) - 1;

            Serial.println(">>> Début de l'envoi du grand message...");
            node.TransmitMessage(longMessage[i], messageLen);
            Serial.println(">>> Envoi complet terminé.");
            i++;
            
        }
        nextSend++;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// ============================================================
//  Task RX — Core 0
// ============================================================
void taskRX(void *pvParameters)
{
    uint8_t frameBuffer[MAX_PAYLOAD];
    uint8_t type, seq, vol;

    static uint8_t assemblyBuffer[512];
    size_t  assemblyOffset       = 0;
    uint8_t expectedSeq          = 1;
    uint8_t totalPacketsExpected = 0;

    for (;;)
    {
        int bytesRead = node.ReceiveFrame(frameBuffer, type, seq, vol);

        if (bytesRead >= 0)
        {
            switch (type)
            {
                case TYPE_START:
                    if(keepReading)
                    {
                        assemblyOffset       = 0;
                        expectedSeq          = 1;
                        totalPacketsExpected = vol;
                    }
                    keepReading = true;
                    Serial.printf("[RX] Connexion etablie — %d paquet(s) attendu(s).\n",
                                 totalPacketsExpected);
                    break;

                case TYPE_DATA:

                    // ----- Injection de test : perte volontaire d'un paquet -----
                    if (testForceDropSeq == (int)seq || testForceDropSeq == -2)
                    {
                        Serial.printf("[TEST] Paquet %d volontairement ignoré (simulation de perte).\n", seq);
                        Serial.println("[TEST] Le paquet suivant devrait etre vu 'hors sequence' -> NACK.");
                        testForceDropSeq = -3;
                        break; // on n'incrémente PAS expectedSeq : le paquet suivant sera en avance
                    }

                    if (seq == expectedSeq)
                    {
                        if (assemblyOffset + bytesRead < sizeof(assemblyBuffer) && keepReading)
                        {
                            memcpy(assemblyBuffer + assemblyOffset, frameBuffer, bytesRead);
                            assemblyOffset += bytesRead;
                            expectedSeq++;
                            Serial.printf("[RX] Paquet %d/%d recu (%d octets)\n",
                                         seq, totalPacketsExpected, bytesRead);
                        }
                    } else {
                        Serial.printf("[Rx] ERREUR : Paquet hors séquence ! Reçu %d, attendu %d\n", seq, expectedSeq);
                        if(keepReading)
                        {
                            outOfSync = true;
                            expected = expectedSeq;
                            keepReading = false;
                        }
                    }
                    break;

                case TYPE_END:
                    if(keepReading)
                    {
                        assemblyBuffer[assemblyOffset] = '\0';
                        Serial.printf("[RX] ====== MESSAGE RECONSTITUE ======\n");
                        Serial.printf("[RX] %d octets : %s\n", (int)assemblyOffset, (char *)assemblyBuffer);
                        Serial.printf("[RX] ====================================\n");
                        assemblyOffset = 0;
                        expectedSeq    = 1;
                    }
                    break;

                default:
                    Serial.printf("[RX] Type inconnu : 0x%02X\n", type);
                    break;
                
                case TYPE_OUT_OF_SYNC:
                    Serial.printf("[Rx] NACK reçu pour le paquet %d. Demande de retransmission.\n", seq);
                    if(keepReading)
                    {
                        NACKReceived = true;
                        NACKResend = vol;
                        keepReading = false;
                    }
                    break;
            }
        } 
        else if (bytesRead != -1) { 
            Serial.printf("Erreur de décodage trame. Code : %d\n", bytesRead);
            outOfSync = true;
            expected = expectedSeq;
            keepReading = false;
        }
        else if (bytesRead == -1)
        {
            // Timeout normal
        }

        taskYIELD();
    }
}

// ============================================================
//  Task Console — Core 1, priorité basse
//  Lit le port série ligne par ligne pour déclencher les tests.
//  Format des commandes : <lettre>[numéro]  ex: "n", "n3", "c", "c3"
// ============================================================
void processTestCommand(const char *cmd)
{
    if (cmd[0] == '\0') return;

    char letter = cmd[0];
    bool hasNumber = false;
    int number = -1;

    if (strlen(cmd) > 1) {
        number = atoi(cmd + 1);
        hasNumber = (number >= 1); // atoi renvoie 0 si pas un nombre valide
    }

    switch (letter)
    {
        case 'n':
        case 'N':
            if (hasNumber) {
                testForceDropSeq = number;
                Serial.printf("[TEST] Le paquet DATA #%d sera ignore cote RX (test NACK arme).\n", number);
            } else {
                testForceDropSeq = -2;
                Serial.println("[TEST] Le PROCHAIN paquet DATA sera ignore cote RX (test NACK arme).");
            }
            break;

        case 'c':
        case 'C':
            if (hasNumber) {
                node.DebugCorruptFrame((uint8_t)number);
                Serial.printf("[TEST] Le paquet DATA #%d aura un CRC corrompu a l'emission.\n", number);
            } else {
                node.DebugCorruptNextFrame();
                Serial.println("[TEST] La PROCHAINE trame emise aura un CRC corrompu.");
            }
            break;

        default:
            Serial.println("[TEST] Commande inconnue. Utilise 'n[num]' ou 'c[num]', ex: n3 / c2.");
            break;
    }
}

void taskConsole(void *pvParameters)
{
    static char lineBuf[32];
    static int  lineLen = 0;

    for (;;)
    {
        while (Serial.available())
        {
            char c = Serial.read();
            if (c == '\n' || c == '\r')
            {
                if (lineLen > 0)
                {
                    lineBuf[lineLen] = '\0';
                    processTestCommand(lineBuf);
                    lineLen = 0;
                }
            }
            else if (lineLen < (int)sizeof(lineBuf) - 1)
            {
                lineBuf[lineLen++] = c;
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// ============================================================
//  setup / loop
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    serialMutex = xSemaphoreCreateMutex();

    Serial.println("=== APP4 — Manchester RMT full-duplex (mode test) ===");
    Serial.printf("GPIO TX=%d  RX=%d\n", GPIO_TX, GPIO_RX);
    Serial.printf("Demi-bit = %d us  (~%.0f bps)\n",
                  Pilote::halfBit(),
                  1e6f / (2.0f * Pilote::halfBit()));
    Serial.println("Commandes (avec Entree) : n / n<num> = test NACK | c / c<num> = test CRC");
    Serial.println("IMPORTANT : regle le moniteur serie sur fin de ligne 'Newline' pour que les commandes soient reconnues.");

    xTaskCreate(taskRX,      "RX",      8192, NULL, 3, NULL);
    xTaskCreate(taskTX,      "TX",      4096, NULL, 2, NULL);
    xTaskCreate(taskConsole, "Console", 2048, NULL, 1, NULL);
}

void loop()
{
    vTaskDelete(NULL);
}


