// by Amir Sk
// Apr. 5rd 2020

#include <Arduino.h>
#include <configuration.h>
#include <avr/wdt.h>
#include <openGLCD.h>
#include <sysconfig.h>
#include <motor.h>
#include <motor_driver.h>
#include <buzzer.h>
#include <button.h>
#include <led.h>
#include <Potentiometer.h>
#include <LCD.h>
#include <timers.h>
#include <callBacks.h>
#include <trajectory.h>
#include <PID.h>
#include <string.h>
#include <Stream.h>
#include <math.h>

long testTimer = 0;

/* Global Objects */
SysConfig *Global_SysConfig;

Button *onButton;
Button *open_uSwitch;

LED *gLED;
LED *ardLED;
Buzzer *coolBuzz;

Potentiometer *respVolume;
Potentiometer *respCycle;
Potentiometer *IERatio;

PID *pid;

int table_RV[] = {200, 300, 400, 500, 600, 700, 800};
int table_RC[23];
int table_IE[] = {1, 2, 3, 4};

int RV = 0;
int RC = 0;

int j = 0, k=0;
int motorStart = 0;
int openSwitchHitTime=0;


float x[50];
float RPM[50];

const int iterationLimit=50;
int timeStepValid = 0;
int motorSpeeds[14]={10,0,12,0,14,0,16,0,20,0,25,0,30,0};
float motorCalcedRpms[iterationLimit];
int pidVal[iterationLimit];
int pidSetVal=0;
//nt pc[600*2+1];

LED *wLED;

/* ------------- Initial Check ------------*/

void static initial_Check()
{
	if (open_uSwitch->get_Status() == BSTATE_HIGH)
	{
		Serial.println("Initial Setup");
		Motor::getInstance()->setDirection(DIRECTION_OPEN);
		Motor::getInstance()->motorStart();
		while (open_uSwitch->get_Clicked() == false)
			delay(1);
		open_uSwitch->set_Clicked(false);
		Motor::getInstance()->motorStop();
	}
}


void static setRequiredSpeed(float requiredSpeed)
{
    pidSetVal = pid->Calc((float)requiredSpeed, Motor::getInstance()->getEncRPM());
    Motor::getInstance()->setSpeed(pidSetVal);
}

void setup()
{
	noInterrupts();
	float KP=1.4 ;
    float KI=8  ;                        
    float KD=5e-2;

	for (size_t i = 8; i <= 30; i++)
		table_RC[i - 8] = i;

	Init_Timer1();
	Init_Timer3();
	Init_Timer4();
	Init_Timer5();

	Serial.begin(115200);

	Global_SysConfig = new SysConfig(2, 20, 0);
	PinConfiguration::getInstance()->pinConfiguration();

	coolBuzz = new Buzzer(PinConfiguration::buzzerPin);

	onButton = new Button(PinConfiguration::onButton_pin);
	onButton->setPressCallback(onButton_callback);

	open_uSwitch = new Button(PinConfiguration::open_uSw_pin, INPUT, open_uSw_callback, LOW);

	wLED = new LED(PinConfiguration::wLED_pin);

	respVolume = new Potentiometer(PinConfiguration::Potentiometer_Volume, 7);
	respCycle = new Potentiometer(PinConfiguration::Potentiometer_Cycle, 23);
	IERatio = new Potentiometer(PinConfiguration::Potentiometer_IE, 4);

	respCycle->set_Range(table_RC, sizeof table_RC);
	respVolume->set_Range(table_RV, sizeof table_RV);
	IERatio->set_Range(table_IE, sizeof table_IE);

	pid = new PID(KP, KI, KD, MINIUM_MOTOR_SPEED_IN_PWM, PID_IGNORE_COUNT, PID_GAURD_COUNT);

	interrupts();
	digitalWrite(PinConfiguration::motorDriverOnOff, HIGH);
	Motor::getInstance()->setSpeed(MINIUM_MOTOR_SPEED_IN_PWM);
	//Motor::getInstance()->motorStart();
	Motor::getInstance()->setDirection(DIRECTION_CLOSE);
	Motor::getInstance()->initEnc(PinConfiguration::motorEncoderPin, INPUT, enc_callback, FALLING);
	//initial_Check();
}
void loop()
{

	//mot_Driver->update_resp_rate(Global_SysConfig);
	//mot_Driver->check();
	//LCD::getInstance()->LCD_Menu(respVolume->Potentiometer_Read(), respCycle->Potentiometer_Read(), IERatio->Potentiometer_Read());
	onButton->check();
	if (onButton->get_Clicked() == true && onButton->get_On_Off() == BSTATE_ON)
	{
		Motor::getInstance()->resetEncPeriod();
		Motor::getInstance()->resetPC();
		Motor::getInstance()->setSpeed(MINIUM_MOTOR_SPEED_IN_PWM);	
		Motor::getInstance()->motorStart();
		Timer1Start(77);
		onButton->set_Clicked(false);
	}
	else if (onButton->get_Clicked() == true && onButton->get_On_Off() == BSTATE_OFF)
	{
		Motor::getInstance()->motorStop();
		Motor::getInstance()->resetEncPeriod();
		Motor::getInstance()->resetPC();
		k=0;
		j=0;
		openSwitchHitTime=0;		
		onButton->set_Clicked(false);
	}

	if (open_uSwitch->get_Clicked() == true)
	{
		Serial.println(Motor::getInstance()->getPC());
		Serial.println(Motor::getInstance()->getEncRPM());
		Serial.println(openSwitchHitTime);
		Timer1Start(15624);
		open_uSwitch->set_Clicked(false);
	}

	if (Motor::getInstance()->getStatus() == MOTOR_IS_ON)
	{	
		if (timeStepValid){
			timeStepValid=0;
			wLED->switch_led();			
			//setRequiredSpeed(REQURIED_SPEED);
			int motorSpeed=map(analogRead(PinConfiguration::Potentiometer_Cycle), 0, 1023, 0, 255);
			Motor::getInstance()->setSpeed(motorSpeed);
			//pidVal[j]=pidSetVal;
			//motorCalcedRpms[j]=Motor::getInstance()->getEncRPM();	
			j++;
			if(j%iterationLimit==0){
				Serial.print(motorSpeed);
				Serial.print("\t");
				Serial.println(Motor::getInstance()->getEncRPM());
				j=0;						
			}
			
		}
		
	}

	wdt_reset();
}
