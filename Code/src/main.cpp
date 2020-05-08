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
#include <PID.h>
#include <trajectory.h>
#include <string.h>
#include <Stream.h>
#include <math.h>

long testTimer = 0;

/* Global Objects */
SysConfig *Global_SysConfig;

Button *ON_button;
Button *open_uSwitch;

LED *gLED;
LED *ardLED;
boolean a = false;
Buzzer *coolBuzz;
Motor_Driver *mot_Driver;

Potentiometer *respVolume;
Potentiometer *respCycle;
Potentiometer *IERatio;

PID *pid;

Trajectory *trajectory;

int table_RV[] = {200, 300, 400, 500, 600, 700, 800};
int table_RC[23];
int table_IE[] = {1, 2, 3, 4};

int RV = 0;
int RC = 0;

volatile int encFalled = 0;

int motorSpeed = 0;
unsigned long lastMilis = 0;

char tbp[30] = "";
int printCounter = 0;
int j = 0, k=0;
const int loopParam = 4;

float x[50];
float RPM[50];

int timeStepValid = 0;
int motorSpeeds[20]={230,255,210,255,190,255,170,255,150,255,130,255,110,255,90,255,70,255,50,255};

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

void setup()
{
	noInterrupts();

	for (size_t i = 8; i <= 30; i++)
		table_RC[i - 8] = i;

	Init_Timer3();
	Init_Timer4();
	Init_Timer5();
	TCCR1B = (TCCR1B & 0b11111000) | 0x01;

	Serial.begin(9600);

	Global_SysConfig = new SysConfig(2, 20, 0);
	PinConfiguration::getInstance()->pinConfiguration();

	coolBuzz = new Buzzer(PinConfiguration::buzzerPin);

	//mot_Driver = new Motor_Driver(Motor::getInstance());

	ON_button = new Button(PinConfiguration::onButton_pin, INPUT, onButton_callback, LOW);

	open_uSwitch = new Button(PinConfiguration::open_uSw_pin, INPUT, open_uSw_callback, LOW);

	gLED = new LED(PinConfiguration::gLED_pin);

	//ardLED = new LED(PinConfiguration::ardLED);

	/*LCD::getInstance()->LCD_Cover();
	delay(2000);
	LCD::getInstance()->LCD_Clear();*/

	respVolume = new Potentiometer(PinConfiguration::Potentiometer_Volume, 7);
	respCycle = new Potentiometer(PinConfiguration::Potentiometer_Cycle, 23);
	IERatio = new Potentiometer(PinConfiguration::Potentiometer_IE, 4);

	respCycle->set_Range(table_RC, sizeof table_RC);
	respVolume->set_Range(table_RV, sizeof table_RV);
	IERatio->set_Range(table_IE, sizeof table_IE);

	pid = new PID((float)3, (float)48, (float)0.025);
	pid->setTimeStep(8e-3);
	pid->setOutputRange(0, 255);

	float duration = 1;
	trajectory = new Trajectory(duration / (loopParam * pid->getTimeStep()), 360, 0, 0, duration);
	trajectory->calcTrajec();

	interrupts();

	Motor::getInstance()->setSpeed(255);
	//Motor::getInstance()->setDirection(DIRECTION_CLOSE);
	Motor::getInstance()->initEnc(PinConfiguration::motorEncoderPin, INPUT, enc_callback, RISING);
	//initial_Check();
}
void loop()
{

	//mot_Driver->update_resp_rate(Global_SysConfig);
	//mot_Driver->check();
	//LCD::getInstance()->LCD_Menu(respVolume->Potentiometer_Read(), respCycle->Potentiometer_Read(), IERatio->Potentiometer_Read());
	if (ON_button->get_Clicked() == true && ON_button->get_On_Off() == BSTATE_ON)
	{
		Motor::getInstance()->resetEncPeriod();
		Motor::getInstance()->resetPC();
		Motor::getInstance()->setSpeed(motorSpeeds[k]);	
		Motor::getInstance()->motorStart();
		Serial.println(Motor::getInstance()->getEncRPM());
		OCR4A  = 77;
		TCCR4B |= (1 << WGM12)|(1<<CS10) | (1<<CS12) ;
		ON_button->set_Clicked(false);
		Global_SysConfig->set_Start_Time();
	}
	else if (ON_button->get_Clicked() == true && ON_button->get_On_Off() == BSTATE_OFF)
	{
		Motor::getInstance()->motorStop();
		Motor::getInstance()->resetEncPeriod();
		Motor::getInstance()->resetPC();
		pid->resetParams();	
		ON_button->set_Clicked(false);
	}

	if (open_uSwitch->get_Clicked() == true)
	{
		TCNT5 = 0;
		encFalled = 0;
		Motor::getInstance()->changeDirection();
		pid->resetParams();
		open_uSwitch->set_Clicked(false);
	}

	if (Motor::getInstance()->getStatus() == MOTOR_IS_ON)
	{	
		if (timeStepValid){
			timeStepValid=0;
			Motor::getInstance()->setSpeed(motorSpeeds[k]);
			Serial.println(Motor::getInstance()->getEncRPM());	
			j++;
			if(j==200){
				j=0;
				k++;
				if (k==20){
					Motor::getInstance()->motorStop();
					Motor::getInstance()->resetEncPeriod();
					Motor::getInstance()->resetPC();	
				}				
			}
		}
		
		/*while (j < loopParam * trajectory->getResolution())
		{

			if (millis() - lastMilis >= (pid->getTimeStep()) * 1e3)
			{
				motorSpeed = pid->Calc(trajectory->getRPM((int)(j / loopParam)), Motor::getInstance()->getEncRPM());
				Motor::getInstance()->setSpeed(motorSpeed);
				Serial.print(j);
				Serial.print("\t");
				Serial.print(trajectory->getRPM((int)(j / loopParam)));
				Serial.print("\t");
				Serial.println(Motor::getInstance()->getSpeed());
				//sprintf(tbp,"%d\t%ld\t", respCycle->Potentiometer_Read(), round(Motor::getInstance()->getEncRPM()*100)/100);
				//Serial.print(tbp);
				//Serial.print(pid->getError());
				//Serial.print("\t");
				//Serial.println(pid->getPidRealVal());
				j++;
			}
		}
		j = 0;
		Serial.println(Motor::getInstance()->getPC());
		Motor::getInstance()->motorStop();
		Motor::getInstance()->resetEncPeriod();		
		pid->resetParams();
		ON_button->set_On_Off();*/
		
	}

	wdt_reset();
}
