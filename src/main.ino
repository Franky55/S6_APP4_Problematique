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

// ----- Instance unique Manchester -----
// Les deux canaux RMT sont distincts et non-chevauchants (mem_block_num=1).
Manchester node(GPIO_TX, GPIO_RX, RMT_CHANNEL_1, RMT_CHANNEL_0);

// ----- Queue de communication TX→RX (optionnel, pour debug croisé) -----
// Protège Serial contre l'accès concurrent des deux tasks.
static SemaphoreHandle_t serialMutex;

// Macro thread-safe pour Serial
#define SERIAL_PRINT(...) \
    do { \
        xSemaphoreTake(serialMutex, portMAX_DELAY); \
        Serial.printf(__VA_ARGS__); \
        xSemaphoreGive(serialMutex); \
    } while(0)

// ============================================================
//  Task TX — Core 1
//  Envoie un message toutes les 5 secondes.
//  Modifie ce message pour personnaliser ce que cet ESP32 envoie.
// ============================================================
void taskTX(void *pvParameters)
{
    // Petit délai au démarrage pour laisser la task RX s'initialiser
    vTaskDelay(200 / portTICK_PERIOD_MS);

    uint8_t message[] = "Salut ca va, j'essaie de faire quelque chose de plus gros";
    size_t msgLen = sizeof(message) - 1;  // sans le '\0'

    for (;;)
    {
        SERIAL_PRINT("[TX] >>> Debut envoi (%d octets)...\n", (int)msgLen);

        node.TransmitMessage(message, msgLen);

        SERIAL_PRINT("[TX] >>> Envoi termine.\n");

        // Attend 5 secondes avant le prochain envoi
        vTaskDelay(5000 / portTICK_PERIOD_MS);
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
                    assemblyOffset       = 0;
                    expectedSeq          = 1;
                    totalPacketsExpected = vol;
                    SERIAL_PRINT("[RX] Connexion etablie — %d paquet(s) attendu(s).\n",
                                 totalPacketsExpected);
                    break;

                // ---- Trame DATA ----
                // Copie le payload dans le buffer d'assemblage.
                case TYPE_DATA:
                    if (seq == expectedSeq)
                    {
                        if (assemblyOffset + bytesRead < sizeof(assemblyBuffer))
                        {
                            memcpy(assemblyBuffer + assemblyOffset, frameBuffer, bytesRead);
                            assemblyOffset += bytesRead;
                            expectedSeq++;
                            SERIAL_PRINT("[RX] Paquet %d/%d recu (%d octets)\n",
                                         seq, totalPacketsExpected, bytesRead);
                        }
                        else
                        {
                            SERIAL_PRINT("[RX] ERREUR : buffer d'assemblage plein !\n");
                        }
                    }
                    else
                    {
                        SERIAL_PRINT("[RX] ERREUR sequence : recu %d, attendu %d\n",
                                     seq, expectedSeq);
                        // Remet à zéro pour attendre le prochain START
                        assemblyOffset = 0;
                        expectedSeq    = 1;
                    }
                    break;

                // ---- Trame END ----
                // Affiche le message reconstitué.
                case TYPE_END:
                    assemblyBuffer[assemblyOffset] = '\0';
                    SERIAL_PRINT("[RX] ====== MESSAGE RECONSTITUE ======\n");
                    SERIAL_PRINT("[RX] %d octets : %s\n",
                                 (int)assemblyOffset, (char *)assemblyBuffer);
                    SERIAL_PRINT("[RX] ====================================\n");
                    // Remet à zéro pour le prochain message
                    assemblyOffset = 0;
                    expectedSeq    = 1;
                    break;

                default:
                    SERIAL_PRINT("[RX] Type inconnu : 0x%02X\n", type);
                    break;
            }
        }
        else if (bytesRead == -1)
        {
            // Timeout normal : rien reçu pendant 500 ms, on reboucle silencieusement
        }
        else
        {
            // Vraie erreur de décodage
            SERIAL_PRINT("[RX] Erreur decodage : code %d\n", bytesRead);
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

    // RX démarre en premier sur Core 0 (priorité 3)
    xTaskCreatePinnedToCore(taskRX, "RX", 8192, NULL, 3, NULL, 0);

    // TX démarre ensuite sur Core 1 (priorité 2)
    // Le délai de 200 ms dans taskTX lui laisse le temps de s'initialiser
    xTaskCreatePinnedToCore(taskTX, "TX", 4096, NULL, 2, NULL, 1);
}

void loop()
{
    // Tout le travail est dans les tasks FreeRTOS
    vTaskDelete(NULL);
}
