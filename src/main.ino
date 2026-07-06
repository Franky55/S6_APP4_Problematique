#include <Arduino.h>
#include "Manchester.h"

// ----- Pins -----
#define GPIO_TX 26
#define GPIO_RX 27
#define MAX_MESSAGES_SENDING 5

// ----- Instance unique Manchester -----
Manchester node(GPIO_TX, GPIO_RX, RMT_CHANNEL_1, RMT_CHANNEL_0);

static SemaphoreHandle_t serialMutex;

bool NACKReceived = false;
int NACKResend = 0;
bool outOfSync = false;
int expected = 0;
bool keepReading = true;

// ----- Flag de test (volatile car modifié par la task Console) -----
volatile bool testForceDrop = false; // simule une perte de paquet DATA (déclenche NACK)

// ============================================================
//  Task TX — Core 1
// ============================================================
void taskTX(void *pvParameters) {
    uint8_t longMessage[MAX_MESSAGES_SENDING][80] = {
                            {"Lorem ipsum dolor sit amet, consectetur adipiscing elit. "},
                            {"sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. "},
                            {"Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris"},
                            {"Duis aute irure dolor in reprehenderit in voluptate velit esse. "},
                            {"Excepteur sint occaecat cupidatat non proident,"}
                        };
    int i = 0;

    while(true) {
        if (NACKReceived)
        {
            i = i - 1;
            if (i < 0)
                i = MAX_MESSAGES_SENDING - 1;
            size_t messageLen = sizeof(longMessage[i]) - 1;
            Serial.println(">>> NACK reçu. Réémission du message...");
            NACKReceived = false;
            node.TransmitNACKResendMessage(longMessage[i], messageLen, NACKResend);
            i++;
        } 
        else if (outOfSync)
        {
            node.TransmitOutOfSyncMessage(expected);
            outOfSync = false;
        }
        else // Envoi message normal
        {
            if (i >= MAX_MESSAGES_SENDING)
                i = 0;

            size_t messageLen = sizeof(longMessage[i]) - 1;

            Serial.println(">>> Début de l'envoi du grand message...");
            node.TransmitMessage(longMessage[i], messageLen);
            Serial.println(">>> Envoi complet terminé.");
            i++;
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
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
                    if (testForceDrop)
                    {
                        testForceDrop = false;
                        Serial.printf("[TEST] Paquet %d volontairement ignoré (simulation de perte).\n", seq);
                        Serial.println("[TEST] Le paquet suivant devrait etre vu 'hors sequence' -> NACK.");
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
                        keepReading = false;
                        outOfSync = true;
                        expected = expectedSeq;
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
                    NACKReceived = true;
                    NACKResend = vol;
                    break;
            }
        } 
        else if (bytesRead != -1) { 
            Serial.printf("Erreur de décodage trame. Code : %d\n", bytesRead);
            outOfSync = true;
            expected = expectedSeq;
        }
        else if (bytesRead == -1)
        {
            // Timeout normal
        }
        else
        {
            Serial.printf("[RX] Erreur decodage : code %d\n", bytesRead);
        }

        taskYIELD();
    }
}

// ============================================================
//  Task Console — Core 1, priorité basse
//  Lit le port série pour déclencher les tests.
// ============================================================
void taskConsole(void *pvParameters)
{
    for (;;)
    {
        if (Serial.available())
        {
            char c = Serial.read();
            switch (c)
            {
                case 'n':
                case 'N':
                    testForceDrop = true;
                    Serial.println("[TEST] Prochain paquet DATA sera ignoré cote RX (test NACK arme).");
                    break;

                case 'c':
                case 'C':
                    node.DebugCorruptNextFrame();
                    Serial.println("[TEST] Prochaine trame emise aura un CRC corrompu (vraie erreur CRC cote RX).");
                    break;

                default:
                    // ignore les autres caractères (retours ligne, etc.)
                    break;
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
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
    Serial.println("Commandes: 'n' = test NACK (perte simulee) | 'c' = test corruption");

    xTaskCreate(taskRX,      "RX",      8192, NULL, 3, NULL);
    xTaskCreate(taskTX,      "TX",      4096, NULL, 2, NULL);
    xTaskCreate(taskConsole, "Console", 2048, NULL, 1, NULL);
}

void loop()
{
    vTaskDelete(NULL);
}
