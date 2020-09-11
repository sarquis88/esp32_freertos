#include <Wire.h>
#include "MPU6050.h"

//Pin del esp32 conectado a las interrupciones
#define BUTTON_PIN_BITMASK 0x400000000 // 2^34
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int pasosMem = 0;

MPU6050 mpu;
const byte interruptPin = 33;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int16_t ax, ay, az;
int16_t m_ap;

//Variables usadas para la deteccion de pasos
int valorActual = 0;
int valorAntiguo = 0;
bool subiendo;
int pasos = 0;

bool verbose = false;

//DEBUG
bool debugTime= false;
bool debugSensorLecure=false;
bool debugMessages= false;
bool debugOthers=false;
bool debugConfig=false;

typedef enum {
    umbral0 = 12168,
    umbral1 = 12777,
    umbral2 = 13385,
    umbral3 = 13994,
    umbral4 = 14603,
    umbral5 = 15211,
    umbral6 = 15820,
    umbral7 = 16429,
    umbral8 = 17268,
    umbral9 = 18107,
    umbral10 = 18947,
    umbral11 = 19786,
    umbral12 = 20625,
    umbral13 = 21465,
    umbral14 = 22304,
    umbral15 = 23144,
} umbral;

int getUmbral(int n)
{
    switch (n) {
    case 0:
        return umbral0;
        break;
    case 1:
        return umbral1;
        break;
    case 2:
        return umbral2;
        break;
    case 3:
        return umbral3;
    case 4:
        return umbral4;
        break;
    case 5:
        return umbral5;
        break;
    case 6:
        return umbral6;
        break;
    case 7:
        return umbral7;
        break;
    case 8:
        return umbral8;
    case 9:
        return umbral9;
        break;
    case 10:
        return umbral10;
        break;
    case 11:
        return umbral11;
        break;
    case 12:
        return umbral12;
        break;
    case 13:
        return umbral13;
        break;
    case 14:
        return umbral14;
        break;
    case 15:
        return umbral15;
        break;
    /*
    default:
    // if nothing else matches, do the default
    // default is optional
        break;
    */
    }
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(); //Iniciando I2C
    Serial.println("Initialize MPU6050");

    while (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G)) {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }

    if (bootCount == 0) {
        mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);

        mpu.setIntFreeFallEnabled(false);
        mpu.setIntZeroMotionEnabled(false);
        mpu.setIntMotionEnabled(true);

        mpu.setDHPFMode(MPU6050_DHPF_5HZ);

        mpu.setMotionDetectionThreshold(2);
        mpu.setMotionDetectionDuration(50);

        mpu.setZeroMotionDetectionThreshold(4);
        mpu.setZeroMotionDetectionDuration(2);

        checkSettings();

    }

    //Pin del esp32 que recibirá la interrupcion del MPU
    pinMode(GPIO_NUM_34, INPUT);
    
    //Increment boot number and print it every reboot
    ++bootCount;
    if (debugConfig)
      Serial.println("Boot number: " + String(bootCount));

    //Activa el wake up por interrupciones externas
    esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
}

void checkSettings()
{
    Serial.println();

    Serial.print(" * Sleep Mode:                ");
    Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

    Serial.print(" * Motion Interrupt:     ");
    Serial.println(mpu.getIntMotionEnabled() ? "Enabled" : "Disabled");

    Serial.print(" * Zero Motion Interrupt:     ");
    Serial.println(mpu.getIntZeroMotionEnabled() ? "Enabled" : "Disabled");

    Serial.print(" * Free Fall Interrupt:       ");
    Serial.println(mpu.getIntFreeFallEnabled() ? "Enabled" : "Disabled");

    Serial.print(" * Motion Threshold:          ");
    Serial.println(mpu.getMotionDetectionThreshold());

    Serial.print(" * Motion Duration:           ");
    Serial.println(mpu.getMotionDetectionDuration());

    Serial.print(" * Zero Motion Threshold:     ");
    Serial.println(mpu.getZeroMotionDetectionThreshold());

    Serial.print(" * Zero Motion Duration:      ");
    Serial.println(mpu.getZeroMotionDetectionDuration());

    Serial.print(" * Clock Source:              ");
    switch (mpu.getClockSource()) {
    case MPU6050_CLOCK_KEEP_RESET:
        Serial.println("Stops the clock and keeps the timing generator in reset");
        break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ:
        Serial.println("PLL with external 19.2MHz reference");
        break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ:
        Serial.println("PLL with external 32.768kHz reference");
        break;
    case MPU6050_CLOCK_PLL_ZGYRO:
        Serial.println("PLL with Z axis gyroscope reference");
        break;
    case MPU6050_CLOCK_PLL_YGYRO:
        Serial.println("PLL with Y axis gyroscope reference");
        break;
    case MPU6050_CLOCK_PLL_XGYRO:
        Serial.println("PLL with X axis gyroscope reference");
        break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:
        Serial.println("Internal 8MHz oscillator");
        break;
    }

    Serial.print(" * Accelerometer:             ");
    switch (mpu.getRange()) {
    case MPU6050_RANGE_16G:
        Serial.println("+/- 16 g");
        break;
    case MPU6050_RANGE_8G:
        Serial.println("+/- 8 g");
        break;
    case MPU6050_RANGE_4G:
        Serial.println("+/- 4 g");
        break;
    case MPU6050_RANGE_2G:
        Serial.println("+/- 2 g");
        break;
    }

    Serial.print(" * Accelerometer offsets:     ");
    Serial.print(mpu.getAccelOffsetX());
    Serial.print(" / ");
    Serial.print(mpu.getAccelOffsetY());
    Serial.print(" / ");
    Serial.println(mpu.getAccelOffsetZ());

    Serial.print(" * Accelerometer power delay: ");
    switch (mpu.getAccelPowerOnDelay()) {
    case MPU6050_DELAY_3MS:
        Serial.println("3ms");
        break;
    case MPU6050_DELAY_2MS:
        Serial.println("2ms");
        break;
    case MPU6050_DELAY_1MS:
        Serial.println("1ms");
        break;
    case MPU6050_NO_DELAY:
        Serial.println("0ms");
        break;
    }

    Serial.println();
}

long leerDato(bool lecturaRapida)
{
    if (!lecturaRapida)
        delay(50);
    mpu.getAcceleration(&ax, &ay, &az);
    long ret = sqrt(sq(ax) + sq(ay) + sq(az));
    if (debugSensorLecure){
        Serial.print(ret);
        Serial.print(" - ");
    }
    return ret;
}

//nivel(int valor): dada la lectura del sensor, retorna el nivel en el que se encuentra
int nivel(int valor)
{
    String stringUmbral1;
    String stringUmbral2;

    for (int i = 0; i < 15; i++) {
        stringUmbral1 = String("umbral" + i);
        stringUmbral2 = String("umbral" + i + 1);

        if (valor > getUmbral(i) && valor < getUmbral(i + 1)) {
            return i + 1;
        }
    }
}


void detectarPasos()
{
    if (debugMessages)
        Serial.println("entro a detectarPasos");
    valorAntiguo = valorActual;
    valorActual = leerDato(false);
    
    int count = 0;
    int decartarpaso = 20;
    int fallas;
    bool caminando = false;
    int pasos = 0;

    while (true) {
        if (debugMessages)
          Serial.println("While de afuera");

        while (true) {
            if (debugMessages)
              Serial.println("While de adentro");

            fallas = 0;

            if (debugMessages)
                Serial.println("entro a while 1");
                
            //Mientras no supere el umbral superior (esto quiere decir que esta en reposo)
            //buscar que saltanto este umbral salte la interrupcion
            int while2 = 0;
            while (valorActual < umbral9) {
                if (debugMessages)
                    Serial.println("entro a while 2");

                valorAntiguo = valorActual;
                valorActual = leerDato(false);
                count++;
                if (count == 50) {
                    Serial.println("Going to sleep now");
                    esp_deep_sleep_start();
                }
                while2 = count;
            }
            if (debugMessages)
                Serial.println("Salgo del reposo");

            
            debugSensorLecure= true;
                if (debugSensorLecure){
                    Serial.print(valorAntiguo);
                    Serial.print(" - ");
                    Serial.print(valorActual);
                    Serial.print(" - ");
                }
            
            int time1 = millis();
            count = 0;
            int while3 = 0;
            //Mientras las lecturas esten por encima del umbral superior
            while (valorActual > umbral9 && count < decartarpaso) {
                if (debugMessages)
                    Serial.println("entro a while 3");
                valorAntiguo = valorActual;
                valorActual = leerDato(false);
                count++;
            }
            while3 = count;
            int time3 = millis();
            if (count == decartarpaso && fallas >= 2) {
                if (!caminando) {
                    Serial.println("Going to sleep now");
                    esp_deep_sleep_start();
                }
                else
                    break;
            }
            else {
                fallas++;
            }

            if (debugMessages)
                Serial.println("Entro por el umbral de arriba");

            bool subiendo = false;
            int while4 = 0;
            //Una vez que entra por el umbral de arriba, lo dejo bajar libremente -> bajar = !subiendo
            while (!subiendo && count < decartarpaso) {
                if (debugMessages)
                    Serial.println("entro a while 4");
                valorAntiguo = valorActual;
                valorActual = leerDato(false);
                if (nivel(valorAntiguo) < nivel(valorActual)) {
                    subiendo = true;
                }
                count++;
            }
            int time4 = millis();
            while4 = count;
            if (count == decartarpaso && fallas >= 2) {
                if (!caminando) {
                    Serial.println("Going to sleep now");
                    esp_deep_sleep_start();
                }
                else
                    break;
            }
            else {
                fallas++;
            }
            if (debugMessages)
                Serial.println("Encontré minimo");

            count = 0;
            int while5 = 0;
            while (subiendo && count < decartarpaso) {
                if (debugMessages)
                    Serial.println("entro a while 5");
                valorAntiguo = valorActual;
                valorActual = leerDato(false);
                if (nivel(valorActual) < nivel(valorAntiguo)) {
                    subiendo = false;
                }
                count++;
            }
            int time5 = millis();
            while5 = count;
            if (count == decartarpaso && fallas >= 2) {
                if (!caminando) {
                    Serial.println("Going to sleep now");
                    esp_deep_sleep_start();
                }
                else
                    break;
            }
            else {
                fallas++;
            }
            if (debugMessages)
                Serial.println("Encontré maximo");

            count = 0;
            int while6 = 0;
            while (!subiendo && count < decartarpaso) {
                if (debugMessages)
                    Serial.println("entro a while 6");
                valorAntiguo = valorActual;
                valorActual = leerDato(false);
                if (nivel(valorActual) > nivel(valorAntiguo)) {
                    subiendo = true;
                }
                count++;
            }
            int time6 = millis();
            while6 = count;
            if (count == decartarpaso && fallas >= 2) {
                if (!caminando) {
                    Serial.println("Going to sleep now");
                    esp_deep_sleep_start();
                }
                else
                    break;
            }
            else {
                fallas++;
            }

            if (debugMessages)
                Serial.println("Encontré minimo");

            count = 0;
            int while7 = 0;
            while (valorActual < umbral6 && count < decartarpaso) {
                if (debugMessages)
                    Serial.println("entro a while 7");
                valorAntiguo = valorActual;
                valorActual = leerDato(false);
                count++;
            }
            while7 = count;
            int time2 = millis();
            int duracion = time2 - time1;

            if (duracion < 380) {
                if (!caminando) {
                    Serial.println("Going to sleep now");
                    esp_deep_sleep_start();
                }
                else
                    break;
            }

            pasosMem++;

            if (!caminando) {
                pasos++;
                if (pasos == 5){
                    caminando = true;
                    if (debugMessages)
                      Serial.println("Caminando");
                }
            }

            Serial.print("PASO: ");
            Serial.print(pasosMem);
            Serial.print(" - ");

            if (debugTime){
                Serial.print("Duracion: ");
                Serial.print(duracion);
                Serial.print(" - ");
    
                Serial.print("t3: ");
                Serial.print(time3 - time1);
                Serial.print(" - ");
                Serial.print("t4: ");
                Serial.print(time4 - time3);
                Serial.print(" - ");
                Serial.print("t5: ");
                Serial.print(time5 - time4);
                Serial.print(" - ");
                Serial.print("t6: ");
                Serial.print(time6 - time5);
            }
            Serial.println("");
            
        }
    }
}

void loop()
{

    detectarPasos();

    Activites act = mpu.readActivites();
    // Sleep on low.
    if (!act.isActivity) {
        //Go to sleep now
        Serial.println("Going to sleep now");
        esp_deep_sleep_start();
        Serial.println("This will never be printed");
    }
}
