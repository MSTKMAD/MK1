/**
 * @file TestMKx.ino
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-09-29
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SSD1306.h"
#include "MilliTimer.h"

const uint16_t C_PIN_USB_D_POS = 0;
const uint16_t C_PIN_USB_D_NEG = 1;
const uint16_t C_PIN_VIN_17 = 2;
const uint16_t C_PIN_VIN_20 = 3;
const uint16_t C_PIN_R_3v3 = 4;
const uint16_t C_PIN_R2A = 5;
const uint16_t C_PIN_R3A = 6;
const uint16_t C_PIN_OVC_ALARM = 7;
const uint16_t C_PIN_PEDAL = 8;
const uint16_t C_PIN_MUXSEL0 = 13;
const uint16_t C_PIN_MUXSEL1 = 14;
const uint16_t C_PIN_S3 = 15;
const uint16_t C_PIN_S2 = 16;
const uint16_t C_PIN_DSP_RST = 17;
const uint16_t C_PIN_DSP_SCK = 18;
const uint16_t C_PIN_MOSI = 19;
const uint16_t C_PIN_MISO = 20;
const uint16_t C_PIN_SS = 21;
const uint16_t C_PIN_S1 = 22;
const uint16_t C_PIN_VOSEN = A0;
const uint16_t C_PIN_ISENSE = A1;
const uint16_t C_PIN_ANALOG_MUX = A2;

const uint16_t C_ST_TEST_1 = 1;
const uint16_t C_ST_TEST_2 = 2;
const uint16_t C_ST_TEST_3 = 3;
const uint16_t C_ST_TEST_4 = 4;
const uint16_t C_ST_TEST_5 = 5;
const uint16_t C_ST_TEST_6 = 6;
const uint16_t C_ST_TEST_7 = 7;

const uint32_t C_3v3 = 3300;
const uint32_t C_V_PEDAL = 1370;
const uint32_t C_V_EN_DCDC = 1400;
const uint32_t C_IOUT_2A = 2000;
const uint32_t C_IOUT_3A = 2000;
const uint32_t C_VOUT_2A = 6200;
const uint32_t C_VOUT_3A = 6200;

const uint32_t C_TOLERANCE = 5;

Adafruit_SSD1306 display(128, 64, C_PIN_MOSI, C_PIN_DSP_SCK, C_PIN_MISO, C_PIN_DSP_RST, C_PIN_SS);

uint16_t status = 0;
uint16_t cont_errores = 0;

uint32_t vcc_3v3 = 0;
uint32_t pedal = 0;
uint32_t enable_dcdc = 0;
uint32_t iout_sample = 0;
uint32_t vout_sample = 0;

bool test_status = true;

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    pinMode(C_PIN_VIN_17, OUTPUT);
    pinMode(C_PIN_VIN_20, OUTPUT);
    pinMode(C_PIN_R_3v3, OUTPUT);
    pinMode(C_PIN_R2A, OUTPUT);
    pinMode(C_PIN_R3A, OUTPUT);
    pinMode(C_PIN_OVC_ALARM, OUTPUT);
    pinMode(C_PIN_PEDAL, OUTPUT);
    pinMode(C_PIN_MUXSEL0, OUTPUT);
    pinMode(C_PIN_MUXSEL1, OUTPUT);

    pinMode(C_PIN_S3, INPUT);
    pinMode(C_PIN_S2, INPUT);
    pinMode(C_PIN_S1, INPUT);

    pinMode(C_PIN_ANALOG_MUX, INPUT);
    pinMode(C_PIN_ISENSE, INPUT);
    pinMode(C_PIN_VOSEN, INPUT);

    analogReadResolution(12);

    digitalWrite(C_PIN_VIN_17, LOW);
    digitalWrite(C_PIN_VIN_20, LOW);

    digitalWrite(C_PIN_R_3v3, LOW);

    digitalWrite(C_PIN_R2A, LOW);
    digitalWrite(C_PIN_R3A, LOW);

    digitalWrite(C_PIN_OVC_ALARM, LOW);
    digitalWrite(C_PIN_PEDAL, LOW);

    digitalWrite(C_PIN_MUXSEL0, LOW);
    digitalWrite(C_PIN_MUXSEL1, LOW);

    display.begin();
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("HI");
    display.setCursor(0, 25);
    display.display();
}

void loop()
{

    if (digitalRead(C_PIN_S3) == HIGH)
    {
        status++;
        constrain(status, C_ST_TEST_1, C_ST_TEST_7);
        test_status = true;
    }
    else if (digitalRead(C_PIN_S2) == HIGH)
    {
        test_status = true;
    }
    else if (digitalRead(C_PIN_S1) == HIGH)
    {
        status = 0;
        test_status = false;
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.print("HI");
        display.setCursor(0, 25);
        display.display();
    }

    if (status == C_ST_TEST_1)
    {
        if (test_status)
        {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("TEST 1:");
            display.setCursor(0, 25);
            display.display();
            /*  Control de los actuadores */
            digitalWrite(C_PIN_VIN_17, LOW);
            digitalWrite(C_PIN_VIN_20, HIGH);

            digitalWrite(C_PIN_R_3v3, LOW);

            digitalWrite(C_PIN_R2A, LOW);
            digitalWrite(C_PIN_R3A, LOW);

            digitalWrite(C_PIN_OVC_ALARM, LOW);
            digitalWrite(C_PIN_PEDAL, LOW);

            digitalWrite(C_PIN_MUXSEL0, HIGH);
            digitalWrite(C_PIN_MUXSEL1, HIGH);

            vcc_3v3 = 0;

            for (int i = 0; i < 8; i++)
            {
                vcc_3v3 += analogRead(C_PIN_ANALOG_MUX);
                delay(50);
            }

            vcc_3v3 = vcc_3v3 / 8;

            vcc_3v3 = vcc_3v3 * 3300 / 4095;
            Serial.printf("3v3:\t%d\n", vcc_3v3);

            if ((vcc_3v3 >= ((C_3v3 * (100 - C_TOLERANCE)) / 100)) && ((vcc_3v3 <= ((C_3v3 * (100 + C_TOLERANCE)) / 100))))
            {
                digitalWrite(C_PIN_R_3v3, HIGH);
                vcc_3v3 = 0;

                for (int i = 0; i < 8; i++)
                {
                    vcc_3v3 += analogRead(C_PIN_ANALOG_MUX);
                    delay(50);
                }

                vcc_3v3 = vcc_3v3 / 8;

                vcc_3v3 = vcc_3v3 * 3300 / 4095;
                display.print(vcc_3v3);
                if ((vcc_3v3 >= ((C_3v3 * (100 - C_TOLERANCE)) / 100)) && ((vcc_3v3 <= ((C_3v3 * (100 + C_TOLERANCE)) / 100))))
                {
                    display.setCursor(50, 50);
                    display.print("OK");
                    test_status = false;
                }
                else
                {
                    display.setCursor(50, 50);
                    display.print("ERROR");
                    test_status = false;
                }
            }
            else
            {
                display.print("ERROR");
                // test_status = false;
            }
            display.display();
        }
    }
    else if (status == C_ST_TEST_2)
    { /*
         if (test_status)
         {
             display.clearDisplay();
             display.setCursor(0, 0);
             display.print("TEST 2:");
             display.setCursor(0, 25);
             display.display();
             //  Control de los actuadores
        digitalWrite(C_PIN_VIN_17, LOW);
        digitalWrite(C_PIN_VIN_20, LOW);

        digitalWrite(C_PIN_R_3v3, LOW);

        digitalWrite(C_PIN_R2A, LOW);
        digitalWrite(C_PIN_R3A, LOW);

        digitalWrite(C_PIN_OVC_ALARM, LOW);
        digitalWrite(C_PIN_PEDAL, LOW);

        digitalWrite(C_PIN_MUXSEL0, LOW);
        digitalWrite(C_PIN_MUXSEL1, LOW);
        delay(1000);
        display.print("20");
        display.display();
        digitalWrite(C_PIN_VIN_17, LOW);
        digitalWrite(C_PIN_VIN_20, HIGH);

        // Esperar a que se libere el boton
        while (digitalRead(C_PIN_S1) == HIGH)
        {
        }
        // Esperar a la pulsacion
        while (digitalRead(C_PIN_S1) == LOW)
        {
        }
        // Esperar a que se libere el boton
        while (digitalRead(C_PIN_S1) == HIGH)
        {
        }
        delay(1000);
        display.setCursor(32, 25);
        display.print("17");
        display.display();
        digitalWrite(C_PIN_VIN_17, LOW);
        digitalWrite(C_PIN_VIN_20, LOW);

        delay(1000);

        digitalWrite(C_PIN_VIN_17, LOW);
        digitalWrite(C_PIN_VIN_20, HIGH);

        // Esperar a que se libere el boton
        while (digitalRead(C_PIN_S1) == HIGH)
        {
        }
        // Esperar a la pulsacion
        while (digitalRead(C_PIN_S1) == LOW)
        {
        }
        // Esperar a que se libere el boton
        while (digitalRead(C_PIN_S1) == HIGH)
        {
        }
        display.setCursor(60, 50);
        display.print("OK");
        display.display();
        test_status = false;
        delay(1000);
    }
    */
    }
    else if (status == C_ST_TEST_3)
    {
        // status++;
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("TEST 3:");
        display.setCursor(0, 25);
        display.display();
        // test_status = true;
        /*
                // Esperar a que se libere el boton
                while (digitalRead(C_PIN_S1) == HIGH)
                {
                }
                // Esperar a la pulsacion
                while (digitalRead(C_PIN_S1) == LOW)
                {
                }
                // Esperar a que se libere el boton
                while (digitalRead(C_PIN_S1) == HIGH)
                {
                }*/
    }
    else if (status == C_ST_TEST_4)
    {
        if (test_status)
        {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("TEST 4:");
            display.setCursor(0, 25);
            display.display();
            /*  Control de los actuadores */
            digitalWrite(C_PIN_VIN_17, LOW);
            digitalWrite(C_PIN_VIN_20, HIGH);

            digitalWrite(C_PIN_R_3v3, LOW);

            digitalWrite(C_PIN_R2A, LOW);
            digitalWrite(C_PIN_R3A, LOW);

            digitalWrite(C_PIN_OVC_ALARM, LOW);
            digitalWrite(C_PIN_PEDAL, HIGH);

            digitalWrite(C_PIN_MUXSEL0, HIGH);
            digitalWrite(C_PIN_MUXSEL1, LOW);

            pedal = 0;

            for (int i = 0; i < 8; i++)
            {
                pedal += analogRead(C_PIN_ANALOG_MUX);
            }

            pedal = pedal >> 3;

            pedal = pedal * 3300 / 4095;
            Serial.printf("Pedal:\t%d\n", pedal);
            display.print(pedal);
            if ((pedal >= (C_V_PEDAL * (100 - C_TOLERANCE) / 100)) && ((pedal <= (C_V_PEDAL * (100 + C_TOLERANCE) / 100))))
            {
                display.setCursor(0, 50);
                display.print("OK");
            }
            else
            {
                display.setCursor(0, 50);
                display.print("ERROR");
                // test_status = false;
            }

            bool set_7v = false;
            display.display();
            while (!set_7v)
            {
                vout_sample = 0;
                for (int i = 0; i < 16; i++)
                {
                    vout_sample += analogRead(C_PIN_VOSEN);
                    delay(50);
                }
                vout_sample = vout_sample >> 4;

                vout_sample = vout_sample * 3300 / 4095;

                vout_sample = vout_sample * 177 / 27;
                if ((vout_sample >= (7300 * (100 - C_TOLERANCE) / 100)) && (vout_sample <= (7300 * (100 + C_TOLERANCE) / 100)))
                {
                    set_7v = true;
                    test_status = false;
                    display.setCursor(60, 25);
                    display.print(vout_sample);
                    display.setCursor(60, 50);
                    display.print("OK");
                }
                else
                {
                    cont_errores++;
                    if (cont_errores == 100)
                    {
                        set_7v = true;
                        display.setCursor(60, 25);
                        display.print(vout_sample);
                        display.setCursor(60, 50);
                        display.print("ERROR");
                    }
                }
            }

            display.display();
            delay(1000);
        }
    }
    else if (status == C_ST_TEST_5)
    {
        if (test_status)
        { /*
             display.clearDisplay();
             display.setCursor(0, 0);
             display.print("TEST 5:");
             display.setCursor(0, 25);
             display.display();
             //  Control de los actuadores
            digitalWrite(C_PIN_VIN_17, LOW);
            digitalWrite(C_PIN_VIN_20, HIGH);

            digitalWrite(C_PIN_R_3v3, LOW);

            digitalWrite(C_PIN_R2A, LOW);
            digitalWrite(C_PIN_R3A, LOW);

            digitalWrite(C_PIN_OVC_ALARM, LOW);
            digitalWrite(C_PIN_PEDAL, LOW);

            digitalWrite(C_PIN_MUXSEL0, LOW);
            digitalWrite(C_PIN_MUXSEL1, LOW);

            display.display();
            // Esperar a que se libere el boton
            while (digitalRead(C_PIN_S1) == HIGH)
            {
            }
            // Esperar a la pulsacion
            while (digitalRead(C_PIN_S1) == LOW)
            {
            }
            // Esperar a que se libere el boton
            while (digitalRead(C_PIN_S1) == HIGH)
            {
            }
            delay(1000);
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("TEST 5:");
            display.setCursor(0, 25);
            display.print("OVC: HIGH");
            display.display();
            digitalWrite(C_PIN_OVC_ALARM, HIGH);
            // Esperar a que se libere el boton
            while (digitalRead(C_PIN_S1) == HIGH)
            {
            }
            // Esperar a la pulsacion
            while (digitalRead(C_PIN_S1) == LOW)
            {
            }
            // Esperar a que se libere el boton
            while (digitalRead(C_PIN_S1) == HIGH)
            {
            }

            test_status = false;
            delay(1000);
            */
        }
    }
    else if (status == C_ST_TEST_6)
    {
        if (test_status)
        {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("TEST 6:");
            display.setCursor(0, 25);
            display.display();
            /*  Control de los actuadores */
            digitalWrite(C_PIN_VIN_17, LOW);
            digitalWrite(C_PIN_VIN_20, HIGH);

            digitalWrite(C_PIN_R_3v3, LOW);

            digitalWrite(C_PIN_R2A, LOW);
            digitalWrite(C_PIN_R3A, LOW);

            digitalWrite(C_PIN_OVC_ALARM, LOW);
            digitalWrite(C_PIN_PEDAL, HIGH);

            digitalWrite(C_PIN_MUXSEL0, LOW);
            digitalWrite(C_PIN_MUXSEL1, LOW);

            // test_status = false;

            enable_dcdc = 0;

            for (int i = 0; i < 8; i++)
            {
                enable_dcdc += analogRead(C_PIN_ANALOG_MUX);
            }
            enable_dcdc = enable_dcdc >> 3;

            enable_dcdc = enable_dcdc * 3300 / 4095;
            display.print(enable_dcdc);
            if ((enable_dcdc >= (C_V_EN_DCDC * (100 - C_TOLERANCE) / 100)) && ((enable_dcdc <= (C_V_EN_DCDC * (100 + C_TOLERANCE) / 100))))
            {
                display.setCursor(50, 50);
                display.print("OK");
                test_status = false;
            }
            else
            {
                display.setCursor(50, 50);
                display.print("ERROR");
                // test_status = false;
            }

            display.display();
            delay(1000);
        }
    }
    else if (status == C_ST_TEST_7)
    {
        if (test_status)
        {
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("TEST 7:");
            display.setCursor(0, 25);
            display.display();
            /*  Control de los actuadores */
            digitalWrite(C_PIN_VIN_17, LOW);
            digitalWrite(C_PIN_VIN_20, HIGH);

            digitalWrite(C_PIN_R_3v3, LOW);

            digitalWrite(C_PIN_R2A, HIGH);
            digitalWrite(C_PIN_R3A, LOW);

            digitalWrite(C_PIN_OVC_ALARM, LOW);

            digitalWrite(C_PIN_MUXSEL0, LOW);
            digitalWrite(C_PIN_MUXSEL1, LOW);

            digitalWrite(C_PIN_PEDAL, HIGH);
            bool test_1 = false;
            while (!test_1)
            {
                iout_sample = 0;
                vout_sample = 0;
                delay(500);
                for (int i = 0; i < 16; i++)
                {
                    iout_sample += analogRead(C_PIN_ISENSE);
                    vout_sample += analogRead(C_PIN_VOSEN);
                    delay(50);
                }
                iout_sample = iout_sample >> 4;
                vout_sample = vout_sample >> 4;

                iout_sample = iout_sample * 3300 / 4095;
                vout_sample = vout_sample * 3300 / 4095;

                iout_sample = iout_sample * 1000 / 625;
                vout_sample = vout_sample * 177 / 27;

                Serial.printf("--------------------\n");
                Serial.printf("1-A:\t%d\n", iout_sample);
                Serial.printf("1-V:\t%d\n", vout_sample);
                if ((iout_sample >= (C_IOUT_2A * (100 - C_TOLERANCE) / 100)) && (iout_sample <= (C_IOUT_2A * (100 + C_TOLERANCE) / 100)))
                {
                    if ((vout_sample >= (C_VOUT_2A * (100 - C_TOLERANCE) / 100)) && (vout_sample <= (C_VOUT_2A * (100 + C_TOLERANCE) / 100)))
                    {
                        display.setTextSize(1);
                        display.setCursor(0, 25);
                        display.print(iout_sample);
                        display.setCursor(0, 40);
                        display.print(vout_sample);
                        display.setCursor(30, 30);
                        display.setTextSize(2);
                        display.print("OK");
                        test_1 = true;
                    }
                    else
                    {
                    }
                }
                else
                {
                    cont_errores++;
                    if (cont_errores == 10)
                    {
                        display.setCursor(0, 30);
                        display.setTextSize(2);
                        display.print("ERROR");
                        test_1 = true;
                    }
                }
            }
            display.display();
            digitalWrite(C_PIN_R2A, LOW);
            delay(100);
            digitalWrite(C_PIN_R3A, HIGH);
            delay(500);
            cont_errores = 0;
            bool test_2 = false;
            while (!test_2)
            {
                /* code */

                iout_sample = 0;
                vout_sample = 0;
                for (int i = 0; i < 16; i++)
                {
                    iout_sample += analogRead(C_PIN_ISENSE);
                    vout_sample += analogRead(C_PIN_VOSEN);
                    delay(50);
                }
                iout_sample = iout_sample >> 4;
                vout_sample = vout_sample >> 4;

                iout_sample = iout_sample * 3300 / 4095;
                vout_sample = vout_sample * 3300 / 4095;

                iout_sample = iout_sample * 1000 / 625;
                vout_sample = vout_sample * 177 / 27;

                Serial.printf("2-A:\t%d\n", iout_sample);
                Serial.printf("2-V:\t%d\n", vout_sample);

                if ((iout_sample >= (C_IOUT_3A * (100 - C_TOLERANCE) / 100)) && (iout_sample <= (C_IOUT_3A * (100 + C_TOLERANCE) / 100)))
                {
                    if ((vout_sample >= (C_VOUT_3A * (100 - C_TOLERANCE) / 100)) && (vout_sample <= (C_VOUT_3A * (100 + C_TOLERANCE) / 100)))
                    {
                        display.setTextSize(1);
                        display.setCursor(70, 25);
                        display.print(iout_sample);
                        display.setCursor(70, 40);
                        display.print(vout_sample);
                        display.setCursor(100, 30);
                        display.setTextSize(2);
                        display.print("OK");
                        test_status = false;
                        test_2 = true;
                    }
                    else
                    {
                    }
                }
                else
                {
                    cont_errores++;
                    if (cont_errores == 10)
                    {
                        display.setCursor(70, 30);
                        display.setTextSize(2);
                        test_status = false;
                        display.print("ERROR");
                        test_1 = true;
                    }
                }
                display.display();
            }
        }
        digitalWrite(C_PIN_PEDAL, LOW);
    }
}