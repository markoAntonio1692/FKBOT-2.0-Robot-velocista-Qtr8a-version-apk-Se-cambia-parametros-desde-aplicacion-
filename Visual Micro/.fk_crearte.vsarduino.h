/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Nano w/ ATmega328, Platform=avr, Package=arduino
*/

#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define ARDUINO 163
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define F_CPU 16000000L
#define ARDUINO 163
#define ARDUINO_AVR_NANO
#define ARDUINO_ARCH_AVR
extern "C" void __cxa_pure_virtual() {;}

//
//
void funcion_pausa();
void frenos_contorno(int flanco_comparacion);
void pid(int linea, int velocidad, float Kp, float Ki, float Kd);
void motores(int motor_izq, int motor_der);
void botones();
void recepcion_data();
void guardar_eeprom(float e_kp, float e_kd, float e_ki, float e_veocidad);
void leer_eeprom();

#include "F:\MICROCONTROLADORES\arduino\arduino-1.6.3\hardware\arduino\avr\variants\eightanaloginputs\pins_arduino.h" 
#include "F:\MICROCONTROLADORES\arduino\arduino-1.6.3\hardware\arduino\avr\cores\arduino\arduino.h"
#include <fk_crearte.ino>
