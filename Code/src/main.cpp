#include <Arduino.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include <DS1307.h>
#include <SPI.h>
#include <SD.h>
#include <ChainableLED.h>
#include <BME280I2C.h>

// ---------------------- DEFINITION DES PINS ----------------------
#define btnRed 2
#define btnGreen 3
#define lightSensor A0
#define gpsRx 8
#define gpsTx 9
#define SD_CS 4

// ---------------------- DEFINITION DES VARIABLES ----------------------
ChainableLED led(5, 6, 1);        // LED RGB 2 broches
DS1307 rtc;                       // Horloge RTC
BME280I2C bme;                    // Capteur de température, pression et hygrométrie
SoftwareSerial gps(gpsRx, gpsTx); // Module GPS

// Variables de configuration
byte mode = 0;                                                      // 0 = mode standard, 1 = mode eco, 2 = mode maintenance
bool backEco = false;                                               // Retour au mode eco après mode maintenance
byte LOG_INTERVAL = 0;                                              // Intervalle entre 2 mesures (en minutes)
bool BtnRpush = false;                                              // Appui sur le bouton rouge
bool BtnGpush = false;                                              // Appui sur le bouton vert
unsigned long tmpBtnPush = 0;                                       // Temps depuis lequel un bouton est appuié
unsigned long tmpLog = 0;                                           // Temps depuis lequel une mesure a été effectuée
bool incoData = false;                                              // Données incohérentes
unsigned int FILE_MAX_SIZE = 4096; // Taille maximale d'un fichier de log en octets
byte numLog = 0;                                                    // Numéro du fichier de log actuel
unsigned int TIMEOUT = EEPROM.read(3) * 1000;                       // Temps maximal d'attente d'une donnée (en secondes)

// Variables d'activation des capteurs
bool GPS = true;                  // Varible d'activation du GPS
bool PRESSURE = EEPROM.read(14);  // Variable d'activation du capteur de pression
bool LUMIN = EEPROM.read(4);      // Variable d'activation du capteur de luminosité
bool TEMP_AIR = EEPROM.read(8);   // Variable d'activation du capteur de température de l'air
bool HYGR = EEPROM.read(11);      // Variable d'activation du capteur d'hygrométrie
bool readPress = EEPROM.read(14); // Variable d'activation de la lecture de la pression

// Variables de configuration des capteurs
byte LUMIN_LOW = EEPROM.read(5);                            // Seuil de luminosité faible
int LUMIN_HIGH = EEPROM.read(7) * 256 + EEPROM.read(6);     // Seuil de luminosité forte
int MIN_TEMP_AIR = (int8_t)EEPROM.read(9);                  // Seuil de température de l'air en dessous duquel le capteur se mettra en erreur
byte MAX_TEMP_AIR = EEPROM.read(10);                        // Seuil de température de l'air au-dessus duquel le capteur se mettra en erreur
byte HYGR_MINT = EEPROM.read(12);                           // Température en dessous de laquelle les mesures d'hygrométrie ne seront pas prises en compte
byte HYGR_MAXT = EEPROM.read(13);                           // Température au-dessus de laquelle les mesures d'hygrométrie ne seront pas prises en compte
int PRESSURE_MIN = EEPROM.read(16) * 256 + EEPROM.read(15); // Seuil de pressure atmosphérique en dessous duquel le capteur se mettra en erreur
int PRESSURE_MAX = EEPROM.read(18) * 256 + EEPROM.read(17); // Seuil de pressure atmosphérique au-dessus duquel le capteur se mettra en erreur

// Structure de données recueillies
struct Data
{
    float latitude = -1;
    float longitude = -1;
    int tempAir = -1;
    int pressure = -1;
    int hygrometrie = -1;
    int brightness = -1;
} data;

// ---------------------- DEFINITION DES FONCTIONS ----------------------
void modeStandard();
void modeEco();
void modeMaintenance();
void writeData();
void getSensors();
void getGPS(float *lat, float *lon);
void configParam();
void errRTC();
void errSD();
void SDfull();
void errGPS();
void errSensors();
void dataInco();
void pushBtnR();
void pushBtnG();
void checkFile(char *fileName);

// ---------------------- FONCTIONS DE GESTION DES ERREURS ----------------------
// LED intermittente rouge et jaune (fréquence 1Hz)
void errRTC()
{
    int time = millis() % 1000;
    led.setColorRGB(0, (time < 500) ? 255 : 0, 0, (time < 500) ? 0 : 255);
}

// LED intermittente rouge et jaune (fréquence 1Hz)
void errSD()
{
    int time = millis() % 1000;
    led.setColorRGB(0, 255, (time < 333) ? 0 : 255, (time < 333) ? 0 : 255);
}

// LED intermittente rouge et bleue (fréquence 1Hz)
void SDfull()
{
    int time = millis() % 1000;
    led.setColorRGB(0, 255, (time < 500) ? 0 : 255, (time < 500) ? 0 : 255);
}

// LED intermittente rouge et verte (fréquence 1Hz)
void errGPS()
{
    int time = millis() % 1000;
    led.setColorRGB(0, 255, (time < 500) ? 0 : 255, 0);
}

// LED intermittente rouge et bleue (fréquence 1Hz)
void errSensors()
{
    if (mode != 2)
        while (millis() - tmpLog < 2000)
        {
            int time = millis() % 1000;
            led.setColorRGB(0, (time < 500) ? 255 : 0, (time < 500) ? 0 : 255, 0);
            if (digitalRead(btnRed) == LOW || digitalRead(btnGreen) == LOW)
                return;
        }
}

// LED intermittente rouge et verte (fréquence 1Hz, durée 2 fois plus longue pour le vert)
void dataInco()
{
    int time = millis() % 1000;
    led.setColorRGB(0, (time < 333) ? 255 : 0, (time < 333) ? 0 : 255, 0);
}

// ---------------------- FONCTIONS DE CONFIGURATION ----------------------
// Fonction de configuration des paramètres de la station météo
void configParam()
{
    rtc.begin();
    rtc.getTime();
    while (rtc.hour > 23)
    {
        errRTC();
        rtc.getTime();
    }
    Serial.println("Mode configuration");
    Serial.println("Veuillez vous référer au manuel d'utilisation pour plus d'informations");
    led.setColorRGB(0, 255, 200, 0);
    unsigned long configStartTime = millis();

    while (millis() - configStartTime < 1800000 && digitalRead(btnGreen) == HIGH)
    {
        if (Serial.available() > 0)
        {
            String input = Serial.readStringUntil('\n');
            Serial.println(input);
            int value = input.substring(input.indexOf("=") + 1).toInt();
            if (input.startsWith("LOG_INTERVAL"))
                LOG_INTERVAL = value;
            else if (input.startsWith("FILE_MAX_SIZE"))
                FILE_MAX_SIZE = value;
            else if (input.startsWith("VERSION"))
                Serial.println("V-b0.1");
            else if (input.startsWith("TIMEOUT"))
                TIMEOUT = value;
            else if (input.startsWith("LUMIN_LOW"))
                LUMIN_LOW = value;
            else if (input.startsWith("LUMIN_HIGH"))
                LUMIN_HIGH = value;
            else if (input.startsWith("LUMIN"))
                LUMIN = value;
            else if (input.startsWith("TEMP_AIR"))
                TEMP_AIR = value;
            else if (input.startsWith("MIN_TEMP_AIR"))
                MIN_TEMP_AIR = value;
            else if (input.startsWith("MAX_TEMP_AIR"))
                MAX_TEMP_AIR = value;
            else if (input.startsWith("HYGR_MINT"))
                HYGR_MINT = value;
            else if (input.startsWith("HYGR_MAXT"))
                HYGR_MAXT = value;
            else if (input.startsWith("HYGR"))
                HYGR = value;
            else if (input.startsWith("PRESSURE_MIN"))
                PRESSURE_MIN = value;
            else if (input.startsWith("PRESSURE_MAX"))
                PRESSURE_MAX = value;
            else if (input.startsWith("PRESSURE"))
                PRESSURE = value;
            else if (input.startsWith("DATE"))
            {
                String date = input.substring(5);
                byte MONTH = date.substring(0, date.indexOf(",")).toInt();
                date = date.substring(date.indexOf(",") + 1);
                byte DAY = date.substring(0, date.indexOf(",")).toInt();
                date = date.substring(date.indexOf(",") + 1);
                byte YEAR = date.toInt();
                rtc.fillByYMD(YEAR, MONTH, DAY);
                rtc.setTime();
            }
            else if (input.startsWith("CLOCK"))
            {
                String time = input.substring(5);
                byte HOUR = time.substring(0, time.indexOf(",")).toInt();
                time = time.substring(time.indexOf(",") + 1);
                byte MINUTE = time.substring(0, time.indexOf(",")).toInt();
                time = time.substring(time.indexOf(",") + 1);
                byte SECOND = time.toInt();
                rtc.fillByHMS(HOUR, MINUTE, SECOND);
                rtc.setTime();
            }
            else if (input.startsWith("RESET"))
            {
                // Réinitialisation des paramètres par défaut
                LOG_INTERVAL = 10;
                FILE_MAX_SIZE = 4096;
                TIMEOUT = 30;
                LUMIN = 1;
                LUMIN_LOW = 255;
                LUMIN_HIGH = 768;
                TEMP_AIR = 1;
                MIN_TEMP_AIR = -10;
                MAX_TEMP_AIR = 60;
                HYGR = 1;
                HYGR_MINT = 0;
                HYGR_MAXT = 50;
                PRESSURE = 1;
                PRESSURE_MIN = 450;
                PRESSURE_MAX = 1030;
            }
        }
    }
    Serial.println("Save...");
    EEPROM.write(0, LOG_INTERVAL);
    EEPROM.write(1, lowByte(FILE_MAX_SIZE));
    EEPROM.write(2, highByte(FILE_MAX_SIZE));
    EEPROM.write(3, TIMEOUT);
    EEPROM.write(4, LUMIN);
    EEPROM.write(5, LUMIN_LOW);
    EEPROM.write(6, lowByte(LUMIN_HIGH));
    EEPROM.write(7, highByte(LUMIN_HIGH));
    EEPROM.write(8, TEMP_AIR);
    EEPROM.write(9, MIN_TEMP_AIR);
    EEPROM.write(10, MAX_TEMP_AIR);
    EEPROM.write(11, HYGR);
    EEPROM.write(12, HYGR_MINT);
    EEPROM.write(13, HYGR_MAXT);
    EEPROM.write(14, PRESSURE);
    EEPROM.write(15, lowByte(PRESSURE_MIN));
    EEPROM.write(16, highByte(PRESSURE_MIN));
    EEPROM.write(17, lowByte(PRESSURE_MAX));
    EEPROM.write(18, highByte(PRESSURE_MAX));
}

void setup()
{
    led.init();

    // Initialisation des pins
    pinMode(btnRed, INPUT_PULLUP);
    pinMode(btnGreen, INPUT_PULLUP);
    pinMode(lightSensor, INPUT);

    // Initialisation des communications
    gps.begin(9600);
    Serial.begin(9600);
    Wire.begin();

    // Passage en mode configuration si demandé
    if (digitalRead(btnRed) == LOW)
    {
        configParam();
    }

    // Initialisation de la carte SD
    while (!SD.begin(SD_CS))
    {
        errSD();
    }

    // Definition des interruptions de changement de mode
    attachInterrupt(digitalPinToInterrupt(btnRed), pushBtnR, FALLING);
    attachInterrupt(digitalPinToInterrupt(btnGreen), pushBtnG, FALLING);

    // Passage en mode par défaut
    modeStandard();
}

// ---------------------- FONCTIONS DE GESTION DES CAPTEURS ----------------------
// Fonction de récupération des données GPS (latitude et longitude) et de vérification de leur validité
void getGPS(float *lat, float *lon)
{
    if (GPS)
    {
        while (gps.available() > 0 && millis() - tmpLog < TIMEOUT)
        {
            String input = gps.readStringUntil('\n');
            if (input.startsWith("$GPGGA"))
            {
                String latStr = input.substring(17, 19) + "." + input.substring(19, 21) + input.substring(22, 27);
                String lonStr = input.substring(30, 32) + "." + input.substring(32, 34) + input.substring(35, 41);

                *lat = latStr.toDouble();
                *lon = lonStr.toDouble();

                if (input.charAt(27) == 'S')
                {
                    *lat *= -1.0;
                }
                if (input.charAt(40) == 'W')
                {
                    *lon *= -1.0;
                }
            }
        }
        if (*lat == 0 || *lon == 0 || *lat > 90 || *lat < -90 || *lon > 180 || *lon < -180)
        {
            *lat = -1;
            *lon = -1;
            while (millis() - tmpLog < TIMEOUT)
            {
                errGPS();
            }
        }
    }
    else
    {
        *lat = -1;
        *lon = -1;
    }
}

// Fonction de récupération des données des capteurs et de vérification de leur validité
void getSensors()
{
    int temp = -1;
    if (mode == 1 && data.latitude == -1)
    {
        getGPS(&data.latitude, &data.longitude);
    }
    else
    {
        getGPS(&data.latitude, &data.longitude);
    }

    if (LUMIN)
    {
        temp = analogRead(lightSensor);
        if (temp < LUMIN_LOW || temp > LUMIN_HIGH)
        {
            if (data.brightness == -1)
                errSensors();
            else
                data.brightness = -1;
            incoData = true;
        }
        else
        {
            data.brightness = temp;
        }
    }
    else
    {
        data.brightness = -1;
    }

    if (bme.begin())
    {
        if (TEMP_AIR)
        {
            temp = bme.temp();
            if (temp < MIN_TEMP_AIR || temp > MAX_TEMP_AIR)
            {
                if (data.tempAir == -1)
                    errSensors();
                else
                    data.tempAir = -1;
                incoData = true;
            }
            else
            {
                data.tempAir = temp;
            }
        }

        if (PRESSURE)
        {
            temp = bme.pres();
            if (temp < PRESSURE_MIN || temp > PRESSURE_MAX)
            {
                if (data.pressure == -1)
                    errSensors();
                else
                    data.pressure = -1;
                incoData = true;
            }
            else
            {
                data.pressure = temp;
            }
        }

        if (HYGR)
        {
            temp = bme.hum();
            if (temp < HYGR_MINT || temp > HYGR_MAXT)
            {
                if (data.hygrometrie == -1)
                    errSensors();
                else
                    data.hygrometrie = -1;
                incoData = true;
            }
            else
            {
                data.hygrometrie = temp;
            }
        }
    }
    else
    {
        while (!bme.begin() && millis() - tmpLog < TIMEOUT)
        {
            errSensors();
        }
        if (bme.begin())
        {
            getSensors();
        }
        else
        {
            data.tempAir = -1;
            data.hygrometrie = -1;
            data.pressure = -1;
        }
    }
}

// ---------------------- FONCTIONS DE GESTION DE LA CARTE SD ----------------------
// Ecriture des données dans un fichier de log sur la carte SD
void writeData()
{
    rtc.getTime();
    char fileName[13];
    sprintf(fileName, "%02d%02d%02d_%d.LOG", rtc.year, rtc.month, rtc.dayOfMonth, numLog);
    File dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile)
    {
        if (dataFile.size() == 0)
        {
            dataFile.print("Y,M,D,H,M,LAT,LON,TEMP,PRESS,HYGR,BRIGHT");
            dataFile.println();
        }
        dataFile.print(rtc.year);
        dataFile.print(",");
        dataFile.print(rtc.month);
        dataFile.print(",");
        dataFile.print(rtc.dayOfMonth);
        dataFile.print(",");
        dataFile.print(rtc.hour);
        dataFile.print(",");
        dataFile.print(rtc.minute);
        dataFile.print(",");
        dataFile.print(data.latitude);
        dataFile.print(",");
        dataFile.print(data.longitude);
        dataFile.print(",");
        dataFile.print(data.tempAir);
        dataFile.print(",");
        dataFile.print(data.pressure);
        dataFile.print(",");
        dataFile.print(data.hygrometrie);
        dataFile.print(",");
        dataFile.print(data.brightness);
        dataFile.println();
        dataFile.close();
    }
    else
    {
        while (!dataFile && millis() - tmpLog < TIMEOUT)
        {
            errSD();
            dataFile = SD.open(fileName, FILE_WRITE);
        }
        if (dataFile)
        {
            writeData();
        }
        else
        {
            SDfull();
        }
    }

    if (dataFile.size() > FILE_MAX_SIZE * 8)
    {
        numLog++;
    }
}
// ---------------------- FONCTIONS DE GESTION DES MODES ----------------------
// Fonction d'interruption de changement de mode bouton rouge
void pushBtnR()
{
    BtnRpush = true;
    tmpBtnPush = millis();
}

// Fonction d'interruption de changement de mode bouton vert
void pushBtnG()
{
    BtnGpush = true;
    tmpBtnPush = millis();
}

// Fonction de changement de mode standard (mode par défaut)
void modeStandard()
{
    // Initialisation de la LED
    led.setColorRGB(0, 0, 255, 0);
    mode = 0;
    LOG_INTERVAL = EEPROM.read(0);
}

// Fonction de changement de mode eco
void modeEco()
{
    // Initialisation de la LED
    led.setColorRGB(0, 0, 0, 255);
    mode = 1;
    LOG_INTERVAL = EEPROM.read(0) * 2;
}

// Fonction de changement de mode maintenance
void modeMaintenance()
{
    // Initialisation de la LED
    led.setColorRGB(0, 255, 75, 0);
    mode = 2;
    LOG_INTERVAL = 1;
}

void loop()
{
    if (BtnGpush)
    {
        if (millis() - tmpBtnPush > 5000)
        {
            BtnGpush = false;
            tmpBtnPush = 0;

            if (mode == 0)
                modeEco();
            else if (mode == 1)
            {
                backEco = true;
                modeMaintenance();
            }
        }
        else if (digitalRead(btnGreen) == HIGH)
        {
            BtnGpush = false;
        }
    }
    else if (BtnRpush)
    {
        if (millis() - tmpBtnPush > 5000)
        {
            BtnRpush = false;
            tmpBtnPush = 0;
            if (mode == 0)
            {
                backEco = false;
                modeMaintenance();
            }
            else if (mode == 1)
                modeStandard();
            else if (mode == 2 && backEco)
                modeEco();
            else
                modeStandard();
        }
        else if (digitalRead(btnRed) == HIGH)
        {
            BtnRpush = false;
        }
        incoData = false;
    }

    if (!BtnRpush && !BtnGpush && millis() - tmpLog > LOG_INTERVAL * 60000) // 60000 = 1 minute
    {
        tmpLog = millis();
        getSensors();
        if (mode != 2)
        {
            writeData();
        }
        else
        {
            rtc.getTime();
            Serial.print(rtc.year);
            Serial.print(",");
            Serial.print(rtc.month);
            Serial.print(",");
            Serial.print(rtc.dayOfMonth);
            Serial.print(",");
            Serial.print(rtc.hour);
            Serial.print(",");
            Serial.print(rtc.minute);
            Serial.print(",");
            Serial.print(data.latitude);
            Serial.print(",");
            Serial.print(data.longitude);
            Serial.print(",");
            Serial.print(data.tempAir);
            Serial.print(",");
            Serial.print(data.pressure);
            Serial.print(",");
            Serial.print(data.hygrometrie);
            Serial.print(",");
            Serial.print(data.brightness);
            Serial.println();
        }
    }
    else if (incoData)
    {
        while (millis() - tmpLog < TIMEOUT)
        {
            dataInco();
        }
        incoData = false;
    }
    else if (mode == 0)
        modeStandard();
    else if (mode == 1)
        modeEco();
    else if (mode == 2)
        modeMaintenance();
    else
        modeEco();
}
