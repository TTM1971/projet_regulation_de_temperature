#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL28Z7.h"
#include "fsl_debug_console.h"
#include "fsl_lpi2c.h"
#include "delay/delay.h"
#include "LiquidCrystal_I2C.h"
#include "fsl_clock.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "bmp280.h"  // Inclure la bibliothèque pour le capteur BMP280
#include "bmp280_defs.h"  // Inclure les définitions du capteur BMP280

#define I2C_BUFFER_LEN 25

float previous_temperature = -300.0; // Valeur initiale impossible pour indiquer qu'aucune lecture n'a été effectuée
char buffer[16];
int tempo = 0; // Variable pour compter le temps d'activation de l'alerte

// Déclaration des fonctions I2C nécessaires
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

// Variables globales
volatile uint32_t g_systickCounter; // Compteur pour les délais
struct bmp280_dev bmp280; // Déclarez une instance du capteur BMP280

// Prototypes des fonctions
void SysTick_Init(void);
void LPI2C1_Init(void);
void LCD_Init(void);
void BMP280_Init(void);
void Hardware_Init(void);
void SysTick_Handler(void);
void Delay_ms(uint32_t ms);
float BMP280_ReadTemperature(void);
void process_temperature(void);
void activation_alerte(void);
void desactivation_alerte(void);
void PinOut_Init(void);

// Handler de l'interruption SysTick
void SysTick_Handler(void)
{
    if (g_systickCounter != 0U) // Décrémenter le compteur tant qu'il n'est pas à zéro
    {
        g_systickCounter--;
    }
}

// Initialisation du SysTick pour générer une interruption toutes les millisecondes
void SysTick_Init(void)
{
    if (SysTick_Config(SystemCoreClock / 1000U)) // Configurer le SysTick pour une interruption toutes les millisecondes
    {
        PRINTF("Échec dans l'initialisation du systick\n"); // Afficher un message d'erreur si l'initialisation échoue
    }
}

// Fonction de délai utilisant le SysTick
void Delay_ms(uint32_t ms)
{
    g_systickCounter = ms; // Initialiser le compteur avec la valeur du délai
    while (g_systickCounter != 0U) // Attendre jusqu'à ce que le compteur atteigne zéro
    {
        // Attente active
    }
}

// Initialisation du module LPI2C1 pour la communication avec l'écran LCD et le capteur BMP280
void LPI2C1_Init(void)
{
    // Activer les horloges pour LPI2C et PortC
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_Lpi2c1);

    // Configurer les broches pour LPI2C1 (SCL -> PTC1, SDA -> PTC2)
    PORT_SetPinMux(PORTC, 1U, kPORT_MuxAlt2); // SCL LPI2C1 -> PTC1
    PORT_SetPinMux(PORTC, 2U, kPORT_MuxAlt2); // SDA LPI2C1 -> PTC2

    // Configuration du maître LPI2C
    lpi2c_master_config_t sConfig;
    LPI2C_MasterGetDefaultConfig(&sConfig); // Obtenir la configuration par défaut
    sConfig.enableMaster = true; // Activer le mode maître
    sConfig.baudRate_Hz = 100000U; // Configurer la vitesse de communication I2C à 100 kHz
    LPI2C_MasterInit(LPI2C1, &sConfig, CLOCK_GetFreq(kCLOCK_Osc0ErClk)); // Initialiser LPI2C1 avec la configuration
}

// Initialisation du module LCD
void LCD_Init(void)
{
    begin();       // Initialiser le LCD
    backlight();   // Activer le rétroéclairage
    clear();       // Effacer le contenu de l'écran lors de l'initialisation
}

// Initialisation du capteur BMP280
void BMP280_Init(void)
{
    bmp280.dev_id = BMP280_I2C_ADDR_PRIM; // Adresse I2C du BMP280
    bmp280.intf = BMP280_I2C_INTF; // Interface de communication I2C
    bmp280.read = user_i2c_read;  // Fonction utilisateur pour la lecture via I2C
    bmp280.write = user_i2c_write; // Fonction utilisateur pour l'écriture via I2C
    bmp280.delay_ms = Delay_ms; // Fonction de délai

    int8_t result = bmp280_init(&bmp280); // Initialiser le capteur BMP280
    if (result != BMP280_OK) { // Vérifier si l'initialisation a réussi
        PRINTF("Erreur d'initialisation du BMP280\n\r"); // Afficher un message d'erreur en cas d'échec
        return;
    }

    // Configuration du BMP280 pour la mesure de température
    struct bmp280_config conf;
    bmp280_get_config(&conf, &bmp280); // Obtenir la configuration actuelle du capteur
    conf.os_temp = BMP280_OS_4X;  // Suréchantillonnage pour améliorer la précision
    conf.os_pres = BMP280_OS_4X; // Suréchantillonnage de la pression (non utilisé ici)
    conf.filter = BMP280_FILTER_COEFF_16; // Utiliser un filtre pour réduire le bruit
    conf.odr = BMP280_ODR_1000_MS; // Taux de rafraîchissement des mesures
    bmp280_set_config(&conf, &bmp280); // Appliquer la configuration
    bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp280); // Mettre le capteur en mode normal
}

// Fonction pour lire la température depuis le BMP280
float BMP280_ReadTemperature(void)
{
    struct bmp280_uncomp_data uncomp_data;  // Structure pour stocker les données non compensées
    double temperature;
    int8_t rslt;

    // Lire les données non compensées
    rslt = bmp280_get_uncomp_data(&uncomp_data, &bmp280);
    if (rslt != BMP280_OK) { // Vérifier si la lecture a réussi
        PRINTF("Erreur lors de l'obtention des données non compensées. Code: %d\n\r", rslt); // Afficher un message d'erreur
        return -273.15; // Retourner une valeur indicative d'erreur (température impossible)
    }

    // Calculer la température compensée
    rslt = bmp280_get_comp_temp_double(&temperature, uncomp_data.uncomp_temp, &bmp280);
    if (rslt != BMP280_OK) { // Vérifier si la compensation a réussi
        PRINTF("Erreur lors de la compensation de la température. Code: %d\n\r", rslt); // Afficher un message d'erreur
        return -273.15; // Retourner une valeur indicative d'erreur
    }

    return (float)temperature; // Retourner la température compensée
}

///////////////////////////////////////////////////////
//////////////// traitement function //////////////////

// Fonction qui traite la température lue
void process_temperature() {
    float temperature = BMP280_ReadTemperature(); // Lire la température actuelle depuis le capteur BMP280

    // Si la lecture de température échoue, attendre une seconde et quitter la fonction
    if (temperature == -273.15) {
        Delay_ms(1000);
        return;
    }

    // Vérifier si la température a changé par rapport à la précédente pour mettre à jour l'affichage
    if (temperature != previous_temperature) {
        setCursor(0, 0); // Positionner le curseur à la première ligne de l'écran LCD
        sprintf(buffer, "Temp: %d C", (int)temperature); // Formater la température en une chaîne de caractères
        PRINTF("%d\n\r", (int)temperature); // Imprimer la température sur la console de débogage
        print(buffer);  // Afficher la température sur l'écran LCD
        previous_temperature = temperature; // Mettre à jour la température précédente
    }



    // Si la température dépasse 30 °C, activer l'alerte
    if (temperature > 30) {
        GPIO_PortClear(GPIOD, 1U << 3U); // Éteindre la LED bleue
        activation_alerte(); // Appeler la fonction pour activer l'alerte
    } else if (temperature < 30) { // Si la température est inférieure à 30 °C, désactiver l'alerte
        desactivation_alerte(); // Appeler la fonction pour désactiver l'alerte
    }
}

// Fonction pour activer l'alerte lorsque la température est élevée
void activation_alerte() {
    GPIO_PortClear(GPIOD, 1U << 3U); // Éteindre la LED bleue
    GPIO_PortSet(GPIOD, 1U << 1U); // Allumer la LED rouge pour indiquer une température élevée
    Delay_ms(1000); // Attendre une seconde
    GPIO_PortSet(GPIOD, 1U << 0U); // Allumer le ventilateur
    GPIO_PortSet(GPIOD, 1U << 2U); // Allumer la LED verte pour indiquer que le ventilateur est en marche
}

// Fonction pour désactiver l'alerte lorsque la température redescend
void desactivation_alerte() {
    if (tempo > 5) { // Si le temps de maintien de l'alerte est écoulé
        GPIO_PortClear(GPIOD, 1U << 1U); // Éteindre la LED rouge
        GPIO_PortClear(GPIOD, 1U << 0U); // Éteindre le ventilateur
        GPIO_PortClear(GPIOD, 1U << 2U); // Éteindre la LED verte
        GPIO_PortSet(GPIOD, 1U << 3U); // Allumer la LED bleue pour signifier que le système est en mode veille/zone normale
        tempo = 0; // Réinitialiser le compteur de tempo
    } else { // Sinon, augmenter le compteur et attendre
        tempo++;
        Delay_ms(10);
    }
}

//////////////// end traitement function /////////////////////
///////////////////////////////////////////////////////

// Initialisation des sorties pour les LED et le ventilateur
void PinOut_Init(void)
{
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0, // Configurer les broches comme sortie numérique
    };

    CLOCK_EnableClock(kCLOCK_PortD); // Activer l'horloge pour le port D
    PORT_SetPinMux(PORTD, 0U, kPORT_MuxAsGpio); // Configurer la broche 0 du port D comme GPIO
    GPIO_PinInit(GPIOD, 0U, &led_config); // Initialiser la broche 0 comme sortie
    PORT_SetPinMux(PORTD, 1U, kPORT_MuxAsGpio); // Configurer la broche 1 du port D comme GPIO
    GPIO_PinInit(GPIOD, 1U, &led_config); // Initialiser la broche 1 comme sortie
    PORT_SetPinMux(PORTD, 2U, kPORT_MuxAsGpio); // Configurer la broche 2 du port D comme GPIO
    GPIO_PinInit(GPIOD, 2U, &led_config); // Initialiser la broche 2 comme sortie
    PORT_SetPinMux(PORTD, 3U, kPORT_MuxAsGpio); // Configurer la broche 3 du port D comme GPIO
    GPIO_PinInit(GPIOD, 3U, &led_config); // Initialiser la broche 3 comme sortie
}

// Initialisation du matériel de la carte
void Hardware_Init(void)
{
    BOARD_InitBootPins(); // Initialiser les broches de la carte
    BOARD_InitBootClocks(); // Initialiser les horloges de la carte
    BOARD_InitBootPeripherals(); // Initialiser les périphériques
    BOARD_InitDebugConsole(); // Initialiser la console de débogage
}

// Fonction pour initialiser tout le système
void init_system()
{
    Hardware_Init(); // Initialiser le matériel
    SysTick_Init(); // Initialiser le SysTick
    LPI2C1_Init(); // Initialiser LPI2C1
    LCD_Init(); // Initialiser le LCD
    BMP280_Init(); // Initialiser le capteur BMP280
    PinOut_Init(); // Initialiser les sorties GPIO
}

////////////////////////////////////////////////////
//////////////////main function/////////////////////
int main(void)
{
    init_system(); // Initialiser le système complet
    clear();
    while (1) // Boucle infinie pour traiter la température
    {
        process_temperature(); // Traiter la température lue
        Delay_ms(10); // Attendre un court délai avant la prochaine lecture
    }
    return 0; // Retourner 0 (ne sera jamais atteint dans la boucle infinie)
}
////////////////end main function /////////////////////
///////////////////////////////////////////////////////

// Implémentation des fonctions user_i2c_read et user_i2c_write
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    uint32_t status = 0;
    uint8_t array[I2C_BUFFER_LEN] = {0}; // Créer un tableau pour stocker les données lues
    uint8_t stringpos = 0;
    array [0] = reg_addr; // Premier élément : adresse du registre à lire

    lpi2c_master_transfer_t masterXfer; // Structure pour la transaction I2C
    masterXfer.slaveAddress = dev_id; // Adresse du périphérique I2C
    masterXfer.direction = kLPI2C_Read; // Direction de la lecture
    masterXfer.subaddress = reg_addr; // Adresse du registre
    masterXfer.subaddressSize = 1; // Taille de l'adresse du registre
    masterXfer.data = &array[0]; // Données à lire
    masterXfer.dataSize = len; // Taille des données
    masterXfer.flags = kLPI2C_TransferDefaultFlag; // Flags par défaut

    status = LPI2C_MasterTransferBlocking(LPI2C1, &masterXfer); // Effectuer la lecture bloquante
    for (stringpos = 0; stringpos < len; stringpos++){
        *(data + stringpos) = array[stringpos]; // Copier les données lues dans le buffer de destination
    }

    return (status == kStatus_Success) ? BMP280_OK : BMP280_E_COMM_FAIL; // Retourner le statut
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    uint32_t status = 0;
    uint8_t array[I2C_BUFFER_LEN] = {0}; // Créer un tableau pour stocker les données à écrire
    uint8_t stringpos = 0;
    array [0] = reg_addr; // Premier élément : adresse du registre à écrire

    lpi2c_master_transfer_t masterXfer; // Structure pour la transaction I2C
    masterXfer.slaveAddress = dev_id; // Adresse du périphérique I2C
    masterXfer.direction = kLPI2C_Write; // Direction de l'écriture
    masterXfer.subaddress = reg_addr; // Adresse du registre
    masterXfer.subaddressSize = 1; // Taille de l'adresse du registre
    masterXfer.data = &array[0]; // Données à écrire
    masterXfer.dataSize = len; // Taille des données
    masterXfer.flags = kLPI2C_TransferDefaultFlag; // Flags par défaut

    for (stringpos = 0; stringpos < len; stringpos++){
        array[stringpos] = *(data + stringpos); // Copier les données dans le tableau
    }

    status = LPI2C_MasterTransferBlocking(LPI2C1, &masterXfer); // Effectuer l'écriture bloquante

    return (status == kStatus_Success) ? BMP280_OK : BMP280_E_COMM_FAIL; // Retourner le statut
}
