/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************
   Resources:

   Wire.h (included with Arduino IDE)
   SparkFunTMP102.h

   Development environment specifics:
   Arduino 1.0+
   MySensors 2.0

   DESCRIPTION

   Capteur temperature sur pile
   capteur I2C TMP102
   Pro mini 8Mhz 3.3V modifié en 1MHz (RC interne ou crystal externe 8MHz) BOD 1.8V, LDO et LED dessoudé
   Alimentation par 2 piles AA ou pile 3V CR2032
   inversion polarité avec P-MOS (voir schema raspberry pi B+ sur Adafruit)
   Ajouter une capa de 100uF au bornes de la pile pour limiter à 15mA max les appels de courant sur la pile bouton
   Capa 4.7uF à 10uF sur Vcc NRF24L01

  topic pour trouver les bootloaders 1Mhz: http://forum.arduino.cc/index.php?topic=160647.15

   Ajouter dans Arduino\hardware\arduino\avr\boards.txt:

  ## Arduino Pro or Pro Mini (BOD 1.8V, internal 1MHz, 4800baud) w/ ATmega328
  ## --------------------------------------------------
  pro.menu.cpu.RC1MHZatmega328=Optiboot internalRC 1MHz BOD1.8V 4800baud

  pro.menu.cpu.RC1MHZatmega328.upload.maximum_size=32256
  pro.menu.cpu.RC1MHZatmega328.upload.maximum_data_size=2048

  pro.menu.cpu.RC1MHZatmega328.upload.speed=4800

  pro.menu.cpu.RC1MHZatmega328.bootloader.low_fuses=0x62
  pro.menu.cpu.RC1MHZatmega328.bootloader.high_fuses=0xde
  pro.menu.cpu.RC1MHZatmega328.bootloader.extended_fuses=0xfe
  pro.menu.cpu.RC1MHZatmega328.bootloader.file=optiboot/optiboot_atmega328_pro_1MHz_4800.hex

  pro.menu.cpu.RC1MHZatmega328.build.mcu=atmega328p
  pro.menu.cpu.RC1MHZatmega328.build.f_cpu=1000000L

  Pour le contenu des bootloaders :

    copier le contenu suivant dans un fichier optiboot_atmega328_pro_1MHz_4800.hex sous
  /bootloaders/optiboot/:

  :107E0000112484B714BE81FFE6D085E08093810001
  :107E100082E08093C00088E18093C10086E0809377
  :107E2000C20089E18093C4008EE0BFD0259A86E02D
  :107E300023EC3FEF91E0309385002093840096BBC4
  :107E4000B09BFECF1D9AA8958150A9F7EE24FF2480
  :107E5000AA24A394B5E0CB2EA1E1BA2EF3E0DF2E45
  :107E600098D0813461F495D0082FA5D0023829F13B
  :107E7000013811F485E001C083E083D07FC08234F3
  :107E800011F484E103C0853419F485E09CD076C0F8
  :107E9000853579F47ED0E82EFF247BD0082F10E0C2
  :107EA000102F00270E291F29000F111F84D07801E1
  :107EB00065C0863521F484E086D080E0DECF84364C
  :107EC00009F040C066D065D0082F63D080E0E81686
  :107ED00080E7F80618F4F701D7BEE895C0E0D1E0D6
  :107EE00058D089930C17E1F7F0E0EF16F0E7FF06A2
  :107EF00018F0F701D7BEE8955ED007B600FCFDCFBD
  :107F0000A701A0E0B1E02C9130E011968C9111977F
  :107F100090E0982F8827822B932B1296FA010C0160
  :107F2000A7BEE89511244E5F5F4FF1E0A038BF0770
  :107F300051F7F701C7BEE89507B600FCFDCFB7BE05
  :107F4000E8951CC0843761F424D023D0082F21D0B9
  :107F500032D0F70185917F0114D00150D1F70EC0C6
  :107F6000853739F428D08EE10CD085E90AD08FE02E
  :107F700084CF813511F488E018D01DD080E101D084
  :107F80006FCF982F8091C00085FFFCCF9093C600E3
  :107F900008958091C00087FFFCCF8091C00084FDD0
  :107FA00001C0A8958091C6000895E0E6F0E098E150
  :107FB000908380830895EDDF803219F088E0F5DF4B
  :107FC000FFCF84E1DECF1F93182FE3DF1150E9F7D5
  :107FD000F2DF1F910895282E80E0E7DFEE27FF27CC
  :027FE000099402
  :027FFE0000057C
  :0400000300007E007B
  :00000001FF

  pour programmer le bootloader, telecharger le sketch ArduinoISP sur un Uno, et programmer le mini pro
  connecté au Uno en SPI en choisissant le programmateur Arduino as ISP, alimenter en 5v le pro mini

*/

//enable debug function for voltage and frequency measurements
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Serial output baud rate (debug prints and serial gateway speed) is 115200 by default for 16MHz clock.
//for 1MHz clock the max baud rate is 4800baud
// see http://wormfood.net/avrbaudcalc.php
#define MY_BAUD_RATE 4800

#include <Wire.h>
#include <MySensors.h>
#include "SparkFunTMP102.h"

unsigned long SLEEP_TIME = 900000;  // sleep time between reads (seconds * 1000 milliseconds)

#define RELEASE "1.0"
#define CHILD_ID_TEMP 0
#define CHILD_ID_MULTIMETER 1

int oldBatteryPcnt = 0;
float oldtemperature = 0;
float oldbatteryV = 0;
int BATTERY_SENSE_PIN = A0;  // select the input pin for the battery sense point

TMP102 sensor0(0x48); // Initialize sensor at I2C address 0x48

// Initialize messages
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgBatt(CHILD_ID_MULTIMETER, V_VOLTAGE);

void setup()
{
  sensor0.begin();  // Join I2C bus

  // Initialize sensor0 settings
  // These settings are saved in the sensor, even if it loses power
  // set the Conversion Rate (how quickly the sensor gets a new reading)
  //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
  sensor0.setConversionRate(2);

  //set Extended Mode.
  //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
  sensor0.setExtendedMode(0);

  // use the 1.1 V internal reference
  analogReference(INTERNAL);

}

void presentation()
{
  // Send the sketch name and sketch version information to the gateway and Controller
  sendSketchInfo("Temperature", "1.1");

  //present sensor to controller
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_MULTIMETER, S_MULTIMETER);

}

void loop()
{

  // Turn sensor on to start temperature measurement.
  // Current consumtion typically ~10uA.
  sensor0.wakeup();

  // read temperature data
  float temperature = sensor0.readTempC();

  // Place sensor in sleep mode to save power.
  // Current consumtion typically <0.5uA.
  sensor0.sleep();

  // Measure battery voltage with adc
  int sensorValue = analogRead(BATTERY_SENSE_PIN);

  // 1M, 470K divider across battery and using internal ADC ref of 1.1V
  // Sense point is bypassed with 0.1 uF cap to reduce noise at that point
  // ((1e6+470e3)/470e3)*1.1 = Vmax = 3.44 Volts
  // 3.44/1023 = Volts per bit = 0.003363075
  // real resistor values:
  // ((992e3+464e3)/464e3)*1.1 = Vmax = 3.45 Volts
  // 3.45/1023 = Volts per bit = 0.003374119

  int batteryPcnt = sensorValue / 10;

  float batteryV  = sensorValue * 0.003374119;

  if (oldtemperature != temperature) {
    oldtemperature = temperature;
    send(msgTemp.set(temperature, 1));
  }

  if (oldbatteryV != batteryV) {
    oldbatteryV = batteryV;
    send(msgBatt.set(batteryV, 3));
  }

  if (oldBatteryPcnt != batteryPcnt) {
    oldBatteryPcnt = batteryPcnt;
    sendBatteryLevel(batteryPcnt);
  }

  // go to sleep cpu
  sleep(SLEEP_TIME);
}

