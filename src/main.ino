/*
  APP4 — Communication Manchester via RMT (dual-core)

  Câblage physique (boucle locale sur un seul ESP32) :
    GPIO_TX_A (12) ──────────────── GPIO_RX_B (14)
    GPIO_RX_A (27) ──────────────── GPIO_TX_B (26)

  Core 0 : Nœud B (récepteur principal)
  Core 1 : Nœud A (émetteur principal)

  Les deux nœuds sont full-duplex : chacun a son propre
  canal RMT TX et RMT RX. Les canaux doivent tous être distincts.

  Canaux RMT utilisés :
    Nœud A : TX → RMT_CHANNEL_0 | RX → RMT_CHANNEL_1
    Nœud B : TX → RMT_CHANNEL_2 | RX → RMT_CHANNEL_3
*/

#include <Arduino.h>
#include "Manchester.h"

// ----- Pins -----
#define GPIO_TX_A 12   // Sortie du nœud A  →  entrée du nœud B
#define GPIO_RX_A 27   // Entrée du nœud A  ←  sortie du nœud B
#define GPIO_TX_B 26   // Sortie du nœud B  →  entrée du nœud A
#define GPIO_RX_B 14   // Entrée du nœud B  ←  sortie du nœud A

// ----- Instances Manchester (full-duplex) -----
// Nœud A : TX=canal 0, RX=canal 2 (occupe 2,3,4,5)
Manchester nodeA(GPIO_TX_A, GPIO_RX_A, RMT_CHANNEL_0, RMT_CHANNEL_2);

// Nœud B : TX=canal 1, RX=canal 6 (occupe 6,7)
Manchester nodeB(GPIO_TX_B, GPIO_RX_B, RMT_CHANNEL_1, RMT_CHANNEL_6);

// ----- Handles des tâches FreeRTOS -----
TaskHandle_t TaskA;
TaskHandle_t TaskB;

// ============================================================
//  Tâche A — Core 1 : Émetteur
//  Envoie un message toutes les 5 secondes.
// ============================================================
void taskNodeA(void *pvParameters)
{
    uint8_t message[] =
        "Lorem ipsum dolor sit amet, consectetur adipiscing elit. "
        "Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.";
    size_t msgLen = sizeof(message) - 1; // sans le '\0'

    for (;;)
    {
        Serial.println("[A] >>> Début de l'envoi...");
        nodeA.TransmitMessage(message, msgLen);
        Serial.println("[A] >>> Envoi terminé.");

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

// ============================================================
//  Tâche B — Core 0 : Récepteur
//  Reconstitue le message fragment par fragment.
// ============================================================
void taskNodeB(void *pvParameters)
{
    uint8_t frameBuffer[MAX_PAYLOAD];
    uint8_t type, seq, vol;

    // Buffer de reconstitution
    uint8_t assemblyBuffer[512];
    size_t  assemblyOffset      = 0;
    uint8_t expectedSeq         = 1;
    uint8_t totalPacketsExpected = 0;

    for (;;)
    {
        int bytesRead = nodeB.ReceiveFrame(frameBuffer, type, seq, vol);

        if (bytesRead >= 0)
        {
            switch (type)
            {
                // ---- Trame START ----
                case TYPE_START:
                    assemblyOffset       = 0;
                    expectedSeq          = 1;
                    totalPacketsExpected = vol;
                    Serial.printf("[B] Connexion établie — %d paquet(s) attendu(s).\n",
                                  totalPacketsExpected);
                    break;

                // ---- Trame DATA ----
                case TYPE_DATA:
                    if (seq == expectedSeq)
                    {
                        if (assemblyOffset + bytesRead < sizeof(assemblyBuffer))
                        {
                            memcpy(assemblyBuffer + assemblyOffset, frameBuffer, bytesRead);
                            assemblyOffset += bytesRead;
                            expectedSeq++;
                            Serial.printf("[B] Paquet %d/%d reçu (%d octets)\n",
                                          seq, totalPacketsExpected, bytesRead);
                        }
                        else
                        {
                            Serial.println("[B] ERREUR : buffer de reconstitution plein !");
                        }
                    }
                    else
                    {
                        Serial.printf("[B] ERREUR séquence : reçu %d, attendu %d\n",
                                      seq, expectedSeq);
                    }
                    break;

                // ---- Trame END ----
                case TYPE_END:
                    assemblyBuffer[assemblyOffset] = '\0'; // sécurité pour Serial.print
                    Serial.println("[B] ====== MESSAGE RECONSTITUÉ ======");
                    Serial.printf("[B] %zu octets : %s\n", assemblyOffset,
                                  (char *)assemblyBuffer);
                    Serial.println("[B] ====================================");
                    break;

                default:
                    Serial.printf("[B] Type inconnu : 0x%02X\n", type);
                    break;
            }
        }
        else if (bytesRead != -1) // -1 = simple timeout, pas une vraie erreur
        {
            Serial.printf("[B] Erreur décodage trame : code %d\n", bytesRead);
        }

        // Cède le CPU brièvement pour ne pas affamer les autres tâches
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// ============================================================
//  setup / loop
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== APP4 — Manchester RMT dual-core ===");
    Serial.printf("Vitesse : demi-bit = %d µs  (~%.0f bps)\n",
                  Pilote::halfBit(),
                  1e6f / (2.0f * Pilote::halfBit()));

    // Nœud B sur Core 0 (démarre en premier pour être prêt avant A)
    xTaskCreatePinnedToCore(taskNodeB, "NodeB", 8192, NULL, 2, &TaskB, 0);
    delay(100);

    // Nœud A sur Core 1
    xTaskCreatePinnedToCore(taskNodeA, "NodeA", 8192, NULL, 2, &TaskA, 1);
}

void loop()
{
    // Tout le travail est dans les tâches FreeRTOS
    vTaskDelete(NULL);
}
