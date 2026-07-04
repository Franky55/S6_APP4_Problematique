/*
  APP4 — Communication Manchester via RMT (dual-core)

  Câblage physique (boucle locale sur un seul ESP32) :
    GPIO_TX_A (12) ──────────────── GPIO_RX_B (14)
    GPIO_RX_A (27) ──────────────── GPIO_TX_B (26)

  Core 0 : Nœud B (récepteur principal)
  Core 1 : Nœud A (émetteur principal)

  Canaux RMT utilisés (mem_block_num = 1 par canal, pas de chevauchement) :
    Nœud A : TX → RMT_CHANNEL_0 | RX → RMT_CHANNEL_1
    Nœud B : TX → RMT_CHANNEL_2 | RX → RMT_CHANNEL_3

  CORRECTIF bug #3 : les canaux TX/RX de chaque nœud sont maintenant
  distincts et ne se chevauchent plus. Avec mem_block_num=1 (64 items
  par canal), les 4 canaux utilisés occupent les blocs 0,1,2,3 sans
  collision. rmt_write_items() gère les trames plus longues que 64
  items automatiquement en mode bloquant.
*/

#include <Arduino.h>
#include "Manchester.h"

// ----- Pins -----
#define GPIO_TX_A 12   // Sortie nœud A  →  entrée nœud B (GPIO_RX_B)
#define GPIO_RX_A 27   // Entrée nœud A  ←  sortie nœud B (GPIO_TX_B)
#define GPIO_TX_B 26   // Sortie nœud B  →  entrée nœud A (GPIO_RX_A)
#define GPIO_RX_B 14   // Entrée nœud B  ←  sortie nœud A (GPIO_TX_A)

// ----- Instances Manchester -----
// Canaux 0 et 1 : non-chevauchants avec mem_block_num=1
Manchester nodeA(GPIO_TX_A, GPIO_RX_A, RMT_CHANNEL_0, RMT_CHANNEL_1);

// Canaux 2 et 3 : non-chevauchants avec mem_block_num=1
Manchester nodeB(GPIO_TX_B, GPIO_RX_B, RMT_CHANNEL_2, RMT_CHANNEL_3);

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
    size_t msgLen = sizeof(message) - 1;  // sans le '\0'

    for (;;)
    {
        Serial.println("[A] >>> Debut de l'envoi...");
        nodeA.TransmitMessage(message, msgLen);
        Serial.println("[A] >>> Envoi termine.");

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

    uint8_t assemblyBuffer[512];
    size_t  assemblyOffset       = 0;
    uint8_t expectedSeq          = 1;
    uint8_t totalPacketsExpected = 0;

    for (;;)
    {
        int bytesRead = nodeB.ReceiveFrame(frameBuffer, type, seq, vol);

        if (bytesRead >= 0)
        {
            switch (type)
            {
                case TYPE_START:
                    assemblyOffset       = 0;
                    expectedSeq          = 1;
                    totalPacketsExpected = vol;
                    Serial.printf("[B] Connexion etablie — %d paquet(s) attendu(s).\n",
                                  totalPacketsExpected);
                    break;

                case TYPE_DATA:
                    if (seq == expectedSeq)
                    {
                        if (assemblyOffset + bytesRead < sizeof(assemblyBuffer))
                        {
                            memcpy(assemblyBuffer + assemblyOffset, frameBuffer, bytesRead);
                            assemblyOffset += bytesRead;
                            expectedSeq++;
                            Serial.printf("[B] Paquet %d/%d recu (%d octets)\n",
                                          seq, totalPacketsExpected, bytesRead);
                        }
                        else
                        {
                            Serial.println("[B] ERREUR : buffer de reconstitution plein !");
                        }
                    }
                    else
                    {
                        Serial.printf("[B] ERREUR sequence : recu %d, attendu %d\n",
                                      seq, expectedSeq);
                    }
                    break;

                case TYPE_END:
                    assemblyBuffer[assemblyOffset] = '\0';
                    Serial.println("[B] ====== MESSAGE RECONSTITUE ======");
                    Serial.printf("[B] %zu octets : %s\n", assemblyOffset,
                                  (char *)assemblyBuffer);
                    Serial.println("[B] ====================================");
                    break;

                default:
                    Serial.printf("[B] Type inconnu : 0x%02X\n", type);
                    break;
            }
        }
        else if (bytesRead != -1)  // -1 = simple timeout, pas une erreur
        {
            Serial.printf("[B] Erreur decodage trame : code %d\n", bytesRead);
        }

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
    Serial.printf("Vitesse : demi-bit = %d us (~%.0f bps)\n",
                  Pilote::halfBit(),
                  1e6f / (2.0f * Pilote::halfBit()));

    // Nœud B démarre en premier (doit être prêt avant que A commence à émettre)
    xTaskCreatePinnedToCore(taskNodeB, "NodeB", 8192, NULL, 2, &TaskB, 0);
    delay(100);

    xTaskCreatePinnedToCore(taskNodeA, "NodeA", 8192, NULL, 2, &TaskA, 1);
}

void loop()
{
    vTaskDelete(NULL);
}
