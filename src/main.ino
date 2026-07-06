/*
  APP4 — Communication Manchester via RMT — Full-duplex, 2 ESP32

  Chaque ESP32 est identique (même firmware).
  Câblage entre les deux cartes :
    ESP32_A GPIO 12 (TX) ────────── ESP32_B GPIO 27 (RX)
    ESP32_A GPIO 27 (RX) ────────── ESP32_B GPIO 12 (TX)
    ESP32_A GND          ────────── ESP32_B GND

  Architecture FreeRTOS :
    Task TX  (Core 1, priorité 2) : envoie un message toutes les 5 s
    Task RX  (Core 0, priorité 3) : bloque sur ReceiveFrame() en permanence

  Une seule instance Manchester par ESP32 :
    TX → RMT_CHANNEL_0
    RX → RMT_CHANNEL_1
*/

#include <Arduino.h>
#include "Manchester.h"

// ----- Pins -----
#define GPIO_TX 26
#define GPIO_RX 27
#define MAX_MESSAGES_SENDING 5

// ----- Instance unique Manchester -----
// Les deux canaux RMT sont distincts et non-chevauchants (mem_block_num=1).
Manchester node(GPIO_TX, GPIO_RX, RMT_CHANNEL_1, RMT_CHANNEL_0);

// ----- Queue de communication TX→RX (optionnel, pour debug croisé) -----
// Protège Serial contre l'accès concurrent des deux tasks.
static SemaphoreHandle_t serialMutex;

bool NACKReceived = false;
int NACKResend = 0;
bool outOfSync = false;
int expected = 0;
bool keepReading = true;


// Tâche d'émission (Core 1)
void taskTX(void *pvParameters) {
    // Message de 145 caractères (> 80) pour tester la fragmentation
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
            i = i - 1; // Ajuste l'index pour retransmettre le paquet NACKé
            if (i < 0)
                i = MAX_MESSAGES_SENDING - 1;
            size_t messageLen = sizeof(longMessage[i]) - 1;
            Serial.println(">>> NACK reçu. Réémission du message...");
            NACKReceived = false;
            node.TransmitNACKResendMessage(longMessage[i], messageLen, NACKResend);// a checker s il faut retransmettre le message complet ou juste le volume
            i++;
        } 
        else if (outOfSync)
        {
            node.TransmitOutOfSyncMessage(expected); // Envoie le numéro de séquence attendu
            outOfSync = false;
        }
        else // Envoie message normal
        {
            if (i >= MAX_MESSAGES_SENDING)
                i = 0;
            size_t messageLen = sizeof(longMessage[i]) - 1;
            Serial.println(">>> Début de l'envoi du grand message...");
            node.TransmitMessage(longMessage[i], messageLen);
            Serial.println(">>> Envoi complet terminé.");
            i++;
            vTaskDelay(5000 / portTICK_PERIOD_MS); // Attend 5 secondes avant de recommencer
        }
    }
}

// ============================================================
//  Task RX — Core 0
//  Bloque sur ReceiveFrame() en permanence.
//  Reconstruit le message complet à partir des trames DATA.
// ============================================================
void taskRX(void *pvParameters)
{
    uint8_t frameBuffer[MAX_PAYLOAD];
    uint8_t type, seq, vol;

    // Buffer de reconstitution du message complet
    static uint8_t assemblyBuffer[512];
    size_t  assemblyOffset       = 0;
    uint8_t expectedSeq          = 1;
    uint8_t totalPacketsExpected = 0;

    for (;;)
    {
        // ReceiveFrame bloque jusqu'à 500 ms (timeout interne).
        // Si rien n'arrive, elle retourne -1 et on reboucle.
        int bytesRead = node.ReceiveFrame(frameBuffer, type, seq, vol);

        if (bytesRead >= 0)
        {
            switch (type)
            {
                // ---- Trame START ----
                // Initialise l'assemblage d'un nouveau message.
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

                // ---- Trame DATA ----
                // Copie le payload dans le buffer d'assemblage.
                case TYPE_DATA:
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
                        expected = expectedSeq; // Stocke le numéro de séquence attendu pour la retransmission
                        // Note : Étant sur un lien filaire direct unidirectionnel (GPIO 12 -> 14), 
                        // le récepteur ne peut pas physiquement retransmettre un paquet NACK à l'émetteur.
                    }
                    break;

                // ---- Trame END ----
                // Affiche le message reconstitué.
                case TYPE_END:
                    
                    if(keepReading)
                    {
                        assemblyBuffer[assemblyOffset] = '\0';
                        Serial.printf("[RX] ====== MESSAGE RECONSTITUE ======\n");
                        Serial.printf("[RX] %d octets : %s\n", (int)assemblyOffset, (char *)assemblyBuffer);
                        Serial.printf("[RX] ====================================\n");
                        // Remet à zéro pour le prochain message
                        assemblyOffset = 0;
                        expectedSeq    = 1;
                    }
                    break;

                default:
                    Serial.printf("[RX] Type inconnu : 0x%02X\n", type);
                    break;
                
                case TYPE_OUT_OF_SYNC: // Trame de NACK
                    Serial.printf("[Rx] NACK reçu pour le paquet %d. Demande de retransmission.\n", seq);
                    NACKReceived = true;
                    NACKResend = vol; // Stocke le numéro de séquence pour la retransmission
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
            // Timeout normal : rien reçu pendant 500 ms, on reboucle silencieusement
        }
        else
        {
            // Vraie erreur de décodage
            Serial.printf("[RX] Erreur decodage : code %d\n", bytesRead);
        }

        // Cède brièvement le CPU (la task RX a priorité 3, TX a priorité 2,
        // ce yield permet quand même à TX de s'exécuter entre deux frames)
        taskYIELD();
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

    Serial.println("=== APP4 — Manchester RMT full-duplex ===");
    Serial.printf("GPIO TX=%d  RX=%d\n", GPIO_TX, GPIO_RX);
    Serial.printf("Demi-bit = %d us  (~%.0f bps)\n",
                  Pilote::halfBit(),
                  1e6f / (2.0f * Pilote::halfBit()));

    xTaskCreate(taskRX, "RX", 8192, NULL, 3, NULL);
    xTaskCreate(taskTX, "TX", 4096, NULL, 2, NULL);
}

void loop()
{
    // Tout le travail est dans les tasks FreeRTOS
    vTaskDelete(NULL);
}
