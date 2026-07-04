/*
  Laboratoire : Interruption
  Connexion : Relier le GPIO 12 (Générateur) au 
              GPIO 14 (Interruption) avec un fil.
*/

#include <Arduino.h>
#include "Manchester.h"

#define TX_PIN 12
#define RX_PIN 14

Manchester txNode(TX_PIN, false);
Manchester rxNode(RX_PIN, true);

TaskHandle_t TaskTx;
TaskHandle_t TaskRx;

// Tâche d'émission (Core 1)
void txTask(void *pvParameters) {
    // uint8_t longMessage[] = "Allo comment ca va moi ca va bien mais je veux plus que 80 octets pour voir que les 2 messages se fait envoyer, mais seulement sur max 80 octets a la fois";
    uint8_t longMessage[] = "Voici mon uatre message qui est grand et qui est plus que 80 octes pour satifaires les demande de la prob";
    size_t messageLen = sizeof(longMessage) - 1;

    for(;;) {
        Serial.println(">>> Début de l'envoi du grand message...");
        txNode.TransmitMessage(longMessage, messageLen);
        Serial.println(">>> Envoi complet terminé.");
        
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Attend 5 secondes avant de recommencer
    }
}

// Tâche de réception (Core 0)
void rxTask(void *pvParameters) {
    uint8_t frameBuffer[80]; 
    uint8_t type, seq, vol;

    // Variables d'état pour la reconstruction du grand message
    uint8_t assemblyBuffer[256]; 
    size_t assemblyOffset = 0;
    uint8_t expectedSeq = 1;
    uint8_t totalPacketsExpected = 0;

    for(;;) {
        int bytesRead = rxNode.ReceiveFrame(frameBuffer, type, seq, vol);
        
        if (bytesRead >= 0) {
            switch(type) {
                case 0x01: // Trame de Début
                    assemblyOffset = 0;
                    expectedSeq = 1;
                    totalPacketsExpected = vol;
                    Serial.printf("[Rx] Connexion établie. Attente de %d paquets.\n", totalPacketsExpected);
                    break;

                case 0x02: // Trame de Données
                    // Vérification de la suite logique des numéros de séquence
                    if (seq == expectedSeq) {
                        if (assemblyOffset + bytesRead < sizeof(assemblyBuffer)) {
                            memcpy(assemblyBuffer + assemblyOffset, frameBuffer, bytesRead);
                            assemblyOffset += bytesRead;
                            expectedSeq++;
                            Serial.printf("[Rx] Paquet %d/%d reçu (%d octets)\n", seq, totalPacketsExpected, bytesRead);
                        }
                    } else {
                        Serial.printf("[Rx] ERREUR : Paquet hors séquence ! Reçu %d, attendu %d\n", seq, expectedSeq);
                        // Note : Étant sur un lien filaire direct unidirectionnel (GPIO 12 -> 14), 
                        // le récepteur ne peut pas physiquement retransmettre un paquet NACK à l'émetteur.
                    }
                    break;

                case 0x03: // Trame de Fin
                    Serial.println("--- MESSAGE RECONSTITUÉ COMPLET ---");
                    Serial.print("Contenu : ");
                    for(size_t i = 0; i < assemblyOffset; i++) {
                        Serial.print((char)assemblyBuffer[i]);
                    }
                    Serial.println("\n-----------------------------------");
                    break;
            }
        } 
        else if (bytesRead != -1) { 
            Serial.printf("Erreur de décodage trame. Code : %d\n", bytesRead);
        }
        
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000); 

    xTaskCreatePinnedToCore(txTask, "TransmitTask", 10000, NULL, 1, &TaskTx, 1);
    xTaskCreatePinnedToCore(rxTask, "ReceiveTask", 10000, NULL, 1, &TaskRx, 0);
}

void loop() {
    vTaskDelete(NULL);
}