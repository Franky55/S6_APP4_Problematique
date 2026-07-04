/*
  Laboratoire : Interruption
  Connexion : Relier le GPIO 12 (Générateur) au 
              GPIO 14 (Interruption) avec un fil.
*/

#include <Arduino.h>
#include "Manchester.h"

#define TX_PIN 12
#define RX_PIN 14

// Instanciation des objets
Manchester txNode(TX_PIN, false);
Manchester rxNode(RX_PIN, true);

// Pointeurs pour les tâches FreeRTOS
TaskHandle_t TaskTx;
TaskHandle_t TaskRx;

// Tâche d'émission (Core 1)
void txTask(void *pvParameters) {
    uint8_t message[] = "Voici mon uatre message qui est grand et qui est plus que 80 octes pour satifaires les demande de la prob";
    
    for(;;) {
        Serial.println(">>> Début de l'envoi du grand message...");
        txNode.TransmitMessage(message, sizeof(message));
        Serial.println(">>> Envoi complet terminé.");
        
        // Délai obligatoire pour céder le processeur et éviter un blocage du Watchdog
        vTaskDelay(2000 / portTICK_PERIOD_MS); 
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
    delay(1000); // Laisse le temps au port série de s'initialiser

    // Épingle la tâche de transmission sur le Core 1 (App Core)
    xTaskCreatePinnedToCore(
        txTask,         /* Fonction de la tâche */
        "TransmitTask", /* Nom de la tâche */
        10000,          /* Taille de la pile (Stack size) */
        NULL,           /* Paramètres passés à la tâche */
        1,              /* Priorité de la tâche */
        &TaskTx,        /* Handle de la tâche */
        1               /* Épinglée au Core 1 */
    );

    // Épingle la tâche de réception sur le Core 0 (Pro Core)
    xTaskCreatePinnedToCore(
        rxTask,         /* Fonction de la tâche */
        "ReceiveTask",  /* Nom de la tâche */
        10000,          /* Taille de la pile */
        NULL,           /* Paramètres passés à la tâche */
        1,              /* Priorité de la tâche */
        &TaskRx,        /* Handle de la tâche */
        0               /* Épinglée au Core 0 */
    );
}

void loop() {
    // La boucle loop() tourne par défaut sur le Core 1.
    // On la supprime car nos tâches FreeRTOS s'occupent de tout.
    vTaskDelete(NULL);
}