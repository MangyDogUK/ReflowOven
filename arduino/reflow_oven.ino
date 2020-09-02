#include "Arduino.h"
#include <stdio.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "max6675.h"
//#define nextSerial softSerial // change to serial when going to hardware mode

#define thermoDO A1
#define  thermoCS  A2
#define  thermoCLK  A3

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

#define nextSerial  Serial

SoftwareSerial debugSerial(A4, A5); // RX, TX //will later swap for hardware, with software debug out.

#include <ThreadController.h>
ThreadController controll = ThreadController();

Thread nextSerialRCV = Thread();
Thread buttonCheck = Thread();
Thread heatPWM = Thread();
Thread cuTemp = Thread();
Thread beeper = Thread();

//uint8_t heatupRate;
//uint8_t heatupTime;
//uint8_t soakTemp;
//uint8_t soakTime;
//uint8_t rampRate;
//uint8_t rampTime;
//uint8_t reflowTemp;
//uint8_t reflowTime;
//uint8_t coolRate;
//uint8_t coolTime;

#define mainPage 0
#define presetsPage 1
#define keypadPage 2

uint8_t page = 0;

uint8_t stage = 0;
uint8_t profile = 0;

#define zeroDet PD2 //digital pin 2
#define heatPin PD4 //digital pin 4
#define tempOffset 30 //the offset of temp value for 1 byte storage. so stored value of 0 = 50c and 255 = 305c

#define beepPin 5

#define graphX 15
#define graphY 27
#define graphW 210
#define graphH 111
#define graphHeadroom 40

#define startTemp  40

//dimmer vars
float dutyTrig = 0; //0-100% dimmer value
//volatile uint16_t oldAC_time = 0;
//volatile uint16_t newAC_time = 0;
float targetAC_time = 0;

float targ = 0;
float icr1 = 0;

bool ledFlipper = false;

#define flipLed1 6
#define flipLed2 7

char serialBuffer[30]; // serial string command passthrough, temp during early debug
uint8_t serialbuffcount = 0;

uint8_t nextCMDBuff[10]; // command buffer to build off nextion serial buffer
uint8_t nextbuffcount = 0;
uint8_t nextendConf = 0;

bool nextCMDrdy = false;

//buttons
//basic control
bool startBut = false;
bool abortBut = false;
bool presetBut = false;

//settings
bool huRateBut = false;
bool huTimeBut = false;
bool soakTempBut = false;
bool soakTimeBut = false;
bool rpRateBut = false;
bool rpTimeBut = false;
bool rTempBut = false;
bool rTimeBut = false;

//profiles
bool leadProfBut = false;
bool leadfreeProfBut = false;
bool prof1But = false;
bool prof2But = false;
bool prof3But = false;

bool prof1Set = false;
bool prof2Set = false;
bool prof3Set = false;

//Keypad
bool kp0But = false;
bool kp1But = false;
bool kp2But = false;
bool kp3But = false;
bool kp4But = false;
bool kp5But = false;
bool kp6But = false;
bool kp7But = false;
bool kp8But = false;
bool kp9But = false;
bool entBut = false;
bool cnclBut = false;

uint8_t leadProf[10] = { 4, 1, 120, 80, 4, 1, 200, 30, 6, 1 };
uint8_t unleadProf[10] = { 4, 1, 160, 80, 4, 1, 230, 40, 6, 1 };

uint8_t prof1[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t prof2[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t prof3[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint8_t loadedProf[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint8_t bLength = 0;
uint8_t bCnt = 0;
uint16_t beepTime = 0;

void beep(uint16_t length, uint8_t count)
	{
		for (uint8_t b = 0; b <= count - 1; b++)
			{
				digitalWrite(beepPin, HIGH);
				delay(length);
				digitalWrite(beepPin, LOW);
				delay(length);
			}
	}
void NoneBlockbeep(uint16_t length, uint8_t count)
	{
		beepTime = millis() + length;
		bLength = length;
		bCnt = count * 2 + 1;

//		for (uint8_t b = 0; b <= count - 1; b++)
//			{
//				digitalWrite(beepPin, HIGH);
//				delay(length);
//				digitalWrite(beepPin, LOW);
//				delay(length);
//			}

	}

void doBeep()
	{
		if (bCnt > 0)
			{
				//debugSerial.println(bCnt);
				//debugSerial.println(beepTime);

				if (beepTime > millis())
					{
						if (bCnt % 2 == 0)
							{
								digitalWrite(beepPin, HIGH);
							}
						else
							{
								digitalWrite(beepPin, LOW);
							}
					}
				else
					{
						if (bCnt != 0)
							{
							//	debugSerial.println(bCnt);
								bCnt--;
							}

						beepTime = millis() + bLength;

					}

			}

	}

void recoverPresets()
	{
		//do flash memory retreaval...
		for (uint8_t i = 0; i < 10; i++)
			{
				prof1[i] = EEPROM.read(i);
				prof2[i] = EEPROM.read(i + 10);
				prof3[i] = EEPROM.read(i + 20);
			}
	}

void savePreset(uint8_t preset)
	{

		int offset = 0;

		if (preset == 1)
			{
				offset = 0;
			}
		else if (preset == 2)
			{
				offset = 10;
			}
		else if (preset == 3)
			{
				offset = 20;
			}
		for (int i = 0; i < 10; i++)
			{
				EEPROM.update(i + offset, loadedProf[i]);
			}
		recoverPresets();
	}

class heater
	{

private:

	uint8_t PWMcnt = 0;

	uint16_t timeE = 0;
	uint8_t prevTemp = 0;

///////////PID copied in
//	float temperature_read = 0.0;
//	float PID_error = 0;
//	float previous_error = 0;
//	float elapsedTime, Time, timePrev;
//	float PID_value = 0;
//	int button_pressed = 0;
//	int menu_activated = 0;
//	float last_set_temperature = 0;
////
////	//PID constants
//////////////////////////////////////////////////////////////
//	int kp = 90;
//	int ki = 10;
//	int kd = 10;
//////////////////////////////////////////////////////////////
////
//	int PID_p = 0;
//	int PID_i = 0;
//	int PID_d = 0;
//	float last_kp = 0;
//	float last_ki = 0;
//	float last_kd = 0;
//
//	int PID_values_fixed = 0;
	////////////// PID copied in^^^^

public:
	uint8_t hpin;
	heater(uint8_t input) :
			hpin(input)
		{

		}

	uint16_t target = 0;
	uint16_t current = 0;
	uint8_t Duty = 0;

	bool heating = false;
	bool running = false;

	void runheaterPWM()
		{
			if ((timeE + 100) < millis())
				{
					setDelta();
					timeE = millis();
				}

			if (running == true)
				{
					//debugSerial.println(dutyTrig);
					//	dutyTrig=Duty;
					if (dutyTrig < Duty)
						{
							dutyTrig++;
						}
					else
						{
							dutyTrig--;
						}

					if (dutyTrig < 0)
						{
							dutyTrig = 0;
						}
					if (dutyTrig > 100)
						{
							dutyTrig = 100;
						}

					//dutyTrig=dutyTrig+(Duty-dutyTrig/10);

//					debugSerial.println(dutyTrig);

//					if (Duty > PWMcnt)
//						{
//							digitalWrite(hpin, HIGH);
//							heating = true;
//						}
//					else
//						{
//							digitalWrite(hpin, LOW);
//							heating = false;
//						}
//
//					PWMcnt++;
//					if (PWMcnt >= 100)
//						{
//							PWMcnt = 0;
//						}
				}
			else
				{

					Duty = 0;
					dutyTrig = 0;
					//	PWMcnt = 0;

				}

		}

	void run()
		{
			running = true;
		}

	void stop()
		{
			Duty = 0;
			dutyTrig = 0;
			digitalWrite(hpin, LOW);
			heating = false;
			running = false;
		}

#define dWidth 10 //the error scale in 'c
	void setDelta()
		{
//			// First we read the real value of temperature
//							temperature_read = current;
//
//							//Next we calculate the error between the setpoint and the real value
//							PID_error = target - temperature_read + 3;
//
//							//Calculate the P value
//							PID_p = 0.01 * kp * PID_error;
//
//							//Calculate the I value in a range on +-3
//							PID_i = 0.01 * PID_i + (ki * PID_error);
//
//							//For derivative we need real time to calculate speed change rate
//							timePrev = Time; // the previous time is stored before the actual time read
//							Time = millis();                            // actual time read
//
//							elapsedTime = (Time - timePrev) / 1000;
//							//Now we can calculate the D calue
//
//							PID_d = 0.01 * kd * ((PID_error - previous_error) / elapsedTime);
//
//							//Final total PID value is the sum of P + I + D
//							PID_value = PID_p + PID_i + PID_d;
//
//							//We define PWM range between 0 and 255
//							if (PID_value < 0)
//								{
//									PID_value = 0;
//								}
//							if (PID_value > 255)
//								{
//									PID_value = 255;
//								}
//							Duty=PID_value;
////							//Now we can write the PWM signal to the mosfet on digital pin D3
////
////							//Since we activate the MOSFET with a 0 to the base of the BJT, we write 255-PID value (inverted)
////							Duty=map(PID_value,0,255,0,100);//analogWrite(PWM_pin, 255 - PID_value);
//						previous_error = PID_error; //Remember to store the previous error for next loop.

			int d = target+3 - current;

			if (target+3 > current)  //for the offeset observed when holding temp
				{
					if (d > dWidth)
						{
							Duty = 100; // in %
						}
					else
						{
							Duty = (100 / dWidth) * d;
						}
				}
			else
				{
					Duty = 0;
				}
		}

	};

heater heat(heatPin);

void runHeaterPWM() //only way to link the thread to the class
	{
		heat.runheaterPWM();
	}

void startHeater()
	{
		controll.add(&heatPWM);

		//	interrupts();
		heat.run();
	}

void stopHeater()
	{
		controll.remove(&heatPWM);
		//	noInterrupts();
		heat.stop();
		char buffer[25];
		sprintf(buffer, "running.val=%i", 0);
		nextSerial.print(buffer);
		nextConf();
	}

void updateDispTemps(int tar, int cur, int sec)
	{

		char buffer[25];
		sprintf(buffer, "tarTemp.txt=\"%i °C\"", tar);
		nextSerial.print(buffer);
		nextConf();

		sprintf(buffer, "curTemp.txt=\"%i °C\"", cur);
		nextSerial.print(buffer);
		nextConf();

		uint8_t minutes = sec / 60;
		uint8_t seconds = sec - (60 * minutes);
		uint8_t dec= seconds / 10;
		seconds=seconds - (dec*10);

		sprintf(buffer, "stopclock.txt=\"%i:%i%i\"", minutes, dec, seconds);
		nextSerial.print(buffer);
		nextConf();

	}

void ledFlip(bool on)
	{
		if (on == true)
			{
				if (ledFlipper == true)
					{
						digitalWrite(flipLed1, HIGH);
						digitalWrite(flipLed2, LOW);

					}
				else
					{
						digitalWrite(flipLed1, LOW);
						digitalWrite(flipLed2, HIGH);

					}

				ledFlipper = !ledFlipper;

			}
		else
			{
				digitalWrite(flipLed1, LOW);
				digitalWrite(flipLed2, LOW);
			}

	}

void updateCurrentTemp()
	{
		if (page == mainPage)
			{
				int temp = thermocouple.readCelsius();

				char buffer[25];
				sprintf(buffer, "curTemp.txt=\"%i °C\"", temp);
				nextSerial.print(buffer);
				nextConf();
			}
	}


void drawFlowgraph(uint16_t temp, uint16_t seconds, uint8_t rangeLow,
		uint16_t rangeHigh, uint16_t timeTotal, uint16_t col, uint8_t width)
	{
		char buffer[25];

		float range = rangeHigh - rangeLow + graphHeadroom;
		int xPos = graphX + (float(graphW / float(timeTotal)) * seconds);
		int yPos = graphY + graphH - (float(graphH / range) * (temp - rangeLow));

		sprintf(buffer, "cirs %i,%i,%i,%i", xPos, yPos, width, col);
		nextSerial.print(buffer);
		nextConf();

	}

void showStat()
	{
//		debugSerial.print(dutyTrig);
//		debugSerial.print("  ");
//		debugSerial.print(targetAC_time);
//		debugSerial.print("  ");
//		debugSerial.print(icr1);
//		debugSerial.print("  ");
//		debugSerial.print((100 / icr1) * (targetAC_time - icr1));
//		debugSerial.println(" %");
	}

void reflowCycle()
	{
		loadPage(mainPage);

//		nextSerial.print("baud=9600");
//		nextConf();
//		delay(500);
//		nextSerial.end();
//		nextSerial.begin(9600);

		//debugSerial.println("reflowCycle");
		controll.remove(&cuTemp);
		char buffer[25];

		uint16_t secs = 0;

		if (page != mainPage)
			{
				//debugSerial.println("stopping heater");
				ledFlip(false);
				stopHeater(); //most certainly not needed but ust enforces that its turned off
			}
		else
			{
			//	debugSerial.println("starting heater");
				startHeater();

				beep(1000, 1);

				uint16_t targ = 100;
				uint16_t curr = thermocouple.readCelsius();

				float preheatRateTime = ((loadedProf[2] + tempOffset - curr)
						/ loadedProf[0]); //-25 average room temp for starting could replace with ambient temp from sensor at boot time
				uint16_t soaktime = loadedProf[3];
				uint16_t heatRampTime =
						((loadedProf[6] - loadedProf[2]) / loadedProf[4]);
				uint16_t reflow = loadedProf[7];
				uint16_t cooldown = (loadedProf[6] - curr) / 6;

				uint16_t totalTime = preheatRateTime + soaktime + heatRampTime + reflow
						+ cooldown;

				uint16_t red = 63488;
				uint16_t green = 2016;

				uint32_t time = millis();

				uint32_t secint = millis();

////////////////////////////////////////////////////////////////////////////////////////
//////////////////* BASE PREHEAT TO STARTING TEMP eg 40c *////////////////////
///////////////////////////////////////////////////////////////////////////////////////

				sprintf(buffer, "running.val=%i", 1);
				nextSerial.print(buffer);
				nextConf();

				stage = 1;
				sprintf(buffer, "stageVal.val=%i", 1);
				nextSerial.print(buffer);
				nextConf();

				nextSerial.print("starButt.pic=18");
				nextConf();

				updateDispTemps(startTemp, curr, 0);

				curr = thermocouple.readCelsius();
				updateDispTemps(startTemp, curr, 0);

				while (curr < startTemp && heat.running == true) //preheat to starting temp
					{

						if ((secint + 1000) < millis())
							{
//								debugSerial.println("preheat");
//								debugSerial.println(curr);
								ledFlip(true);
								beep(10, 1);
								//secs++;
								secint = millis();
								curr = thermocouple.readCelsius();
								updateDispTemps(startTemp, curr, secs);

								showStat();

							}

						heat.current = curr;
						heat.target = 100;

						controll.run(); //runs other threads including button checking

					}

				////////////////////////////////////////////////////////////////////////////////////////
				////////////////////////* HEATUP  TO SOAK TEMP eg 180c *////////////////////////
				///////////////////////////////////////////////////////////////////////////////////////
				//                										0                1                   2                   3                   4                 5                   6                       7                      8                 10
				//												 heatupRate 	heatupTime    soakTemp    soakTime    rampRate    rampTime    reflowTemp    reflowTime    coolRate     coolTime
				//leadProf[10] = {    3, 	   							 1, 	   						 130, 	            70, 	               5, 	             1, 	                190,                30, 	              6,                1};

				preheatRateTime = ((loadedProf[2] + tempOffset - curr) / loadedProf[0]);//updated from the base preheat
				float preheatRateRate = (loadedProf[2] + tempOffset - curr)
						/ preheatRateTime;
				uint16_t heatUpSec = 0;
				while (heat.running == true && curr < (loadedProf[2] + tempOffset - 5))	//stage 1 preheat to soak
					{

						if ((time + 1000) < millis())
							{
//								debugSerial.println("soakramp");
//								debugSerial.println(curr);

								showStat();

								ledFlip(true);
								beep(20, 1);
								secs++;
								time = millis();

								curr = thermocouple.readCelsius();
								heatUpSec++;

								targ = 40 + (preheatRateRate * heatUpSec);//(loadedProf[2] + tempOffset);

								if (targ > loadedProf[2] + tempOffset)
									{
										targ = (loadedProf[2] + tempOffset);
									}

								updateDispTemps(targ, curr, secs);

								if (curr > (targ - 5) && curr < (targ + 5))
									{
										drawFlowgraph(curr, secs, startTemp,
												(loadedProf[6] + tempOffset), totalTime, green, 1);
									}
								else
									{
										drawFlowgraph(curr, secs, startTemp,
												(loadedProf[6] + tempOffset), totalTime, red, 1);
									}

							}

						heat.current = curr;
						heat.target = targ;

						controll.run(); //runs other threads including button checking

					}

////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////* SOAK PHASE*////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
				stage = 2;
				sprintf(buffer, "stageVal.val=%i", 2);
				nextSerial.print(buffer);
				nextConf();

				int endTime = secs + loadedProf[3];

				while (heat.running == true && endTime > secs) //stage 2 soak
					{

						if ((time + 1000) < millis())
							{
//								debugSerial.println("soakramp");
//								debugSerial.println(curr);

								ledFlip(true);
								beep(20, 1);
								secs++;
								time = millis();
								updateDispTemps(targ, curr, secs);

								targ = loadedProf[2] + tempOffset;

								curr = thermocouple.readCelsius();

								showStat();

								if (curr > (targ - 5) && curr < (targ + 5))
									{
										drawFlowgraph(curr, secs, startTemp,
												(loadedProf[6] + tempOffset), totalTime, green, 1);
									}
								else
									{
										drawFlowgraph(curr, secs, startTemp,
												(loadedProf[6] + tempOffset), totalTime, red, 1);
									}
							}

						heat.current = curr;
						heat.target = targ;

						controll.run(); //runs other threads including button checking

					}

				////////////////////////////////////////////////////////////////////////////////////////
				/////////////////////////////* RAMPUP TO REFLOW*////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////////////
				stage = 3;
				sprintf(buffer, "stageVal.val=%i", 3);
				nextSerial.print(buffer);
				nextConf();
//                										0                1                   2                   3                   4                 5                   6                       7                      8                 10
//												 heatupRate 	heatupTime    soakTemp    soakTime    rampRate    rampTime    reflowTemp    reflowTime    coolRate     coolTime
//leadProf[10] = {    3, 	   							 1, 	   						 130, 	            70, 	               5, 	             1, 	                190,                30, 	              6,                1};

				endTime = secs + heatRampTime;
				int secsOffset = secs;

				heatRampTime = ((loadedProf[6] + tempOffset - curr) / loadedProf[4]);	//updated from the base preheat
				float heatRampRate = (loadedProf[6] + tempOffset - curr) / heatRampTime;
				heatUpSec = 0;
				while (heat.running == true && curr < loadedProf[6] + tempOffset - 5) //stage 2 soak
					{

						if ((time + 1000) < millis())
							{
								ledFlip(true);
								beep(20, 1);
								secs++;
								time = millis();
								updateDispTemps(targ, curr, secs);
								heatUpSec++;

								targ = (loadedProf[2] + tempOffset)
										+ (heatRampRate * heatUpSec);

								showStat();

								if (targ > loadedProf[6] + tempOffset)
									{
										targ = (loadedProf[6] + tempOffset); // ((leadProf[6] + tempOffset)); // / heatRampTime) * secs;
									}

								curr = thermocouple.readCelsius();

								if (curr > (targ - 5) && curr < (targ + 5))
									{
										drawFlowgraph(curr, secs, startTemp,
												(loadedProf[6] + tempOffset), totalTime, green, 1);
									}
								else
									{
										drawFlowgraph(curr, secs, startTemp,
												(loadedProf[6] + tempOffset), totalTime, red, 1);
									}
							}

						heat.current = curr;
						heat.target = targ;

						controll.run(); //runs other threads including button checking

					}

				////////////////////////////////////////////////////////////////////////////////////////
				//////////////////////////////////* REFLOW  PHASE*////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////////////

				stage = 4;
				sprintf(buffer, "stageVal.val=%i", 4);
				nextSerial.print(buffer);
				nextConf();

				endTime = secs + loadedProf[7];

				while (heat.running == true && endTime > secs) //stage 2 soak
					{
						if ((time + 1000) < millis())
							{
								ledFlip(true);
								beep(20, 1);
								secs++;
								time = millis();
								updateDispTemps(targ, curr, secs);
								targ = loadedProf[6] + tempOffset;
								curr = thermocouple.readCelsius();

								showStat();

								if (curr > (targ - 5) && curr < (targ + 5))
									{
										drawFlowgraph(curr, secs, startTemp,
												(loadedProf[6] + tempOffset), totalTime, green, 1);
									}
								else
									{
										drawFlowgraph(curr, secs, startTemp,
												(loadedProf[6] + tempOffset), totalTime, red, 1);
									}
							}

						heat.current = curr;
						heat.target = targ;

						controll.run(); //runs other threads including button checking

					}

				////////////////////////////////////////////////////////////////////////////////////////
				//////////////////////////////* COOLDOWN  PHASE*////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////////////

				stage = 5;
				sprintf(buffer, "stageVal.val=%i", 5);
				nextSerial.print(buffer);
				nextConf();

				endTime = secs + loadedProf[7];

				time = millis();
				uint32_t doubleTime = millis();

				bool beepState = false;
				uint16_t stopBeep = secs + 30;
				stopHeater();
				while (curr > 40) //stage 2 soak
					{
						if ((time + 1000) < millis())
							{
								secs++;
								time = millis();
								targ = 40;
								curr = thermocouple.readCelsius();

								sprintf(buffer, "tarTemp.txt=\"%i °C\"", targ);
								nextSerial.print(buffer);
								nextConf();

								sprintf(buffer, "curTemp.txt=\"%i °C\"", curr);
								nextSerial.print(buffer);
								nextConf();

								if (curr > (targ - 5) && curr < (targ + 5))
									{
										drawFlowgraph(curr, secs, startTemp,
												(loadedProf[6] + tempOffset), totalTime, green, 1);
									}
								else
									{
										drawFlowgraph(curr, secs, startTemp,
												(loadedProf[6] + tempOffset), totalTime, red, 1);
									}

							}

						if ((doubleTime + 250) < millis())
							{
								doubleTime = millis();

								if (secs < stopBeep)
									{
										if (beepState == true)
											{
												digitalWrite(beepPin, HIGH);
											}
										else
											{
												digitalWrite(beepPin, LOW);
											}
										beepState = !beepState;
									}
								else
									{
										digitalWrite(beepPin, LOW);
									}

								ledFlip(true);

							}

						//heat.current = curr;
						heat.target = 40;

						controll.run(); //runs other threads including button checking

					}
				digitalWrite(beepPin, LOW);
				updateDispTemps(targ, curr, secs);

				nextSerial.print("starButt.pic=18");
				nextConf();

				sprintf(buffer, "p8.pic=15", 0);
				nextSerial.print(buffer);
				nextConf();

				sprintf(buffer, "T0.txt=\"TIME\"", 0);
				nextSerial.print(buffer);
				nextConf();

				sprintf(buffer, "running.val=%i", 0);
				nextSerial.print(buffer);
				nextConf();

				sprintf(buffer, "stageVal.val=%i", 0);
				nextSerial.print(buffer);
				nextConf();

				controll.add(&cuTemp);
			}

		updateDispTemps(0, thermocouple.readCelsius(), secs);

		ledFlip(false);
		stopHeater();

		nextSerial.print("starButt.pic=18");
		nextConf();

		sprintf(buffer, "p8.pic=15", 0);
		nextSerial.print(buffer);
		nextConf();

		sprintf(buffer, "t0.txt=\"TIME\"", 0);
		nextSerial.print(buffer);
		nextConf();

		sprintf(buffer, "running.val=%i", 0);
		nextSerial.print(buffer);
		nextConf();

		sprintf(buffer, "stageVal.val=%i", 0);
		nextSerial.print(buffer);
		nextConf();

		nextSerial.print("baud=57600");
		nextConf();
		delay(500);
		nextSerial.end();
		nextSerial.begin(57600);

	}


#define upperClamp 95
ISR(TIMER1_CAPT_vect)
	{

		if (dutyTrig > upperClamp) // allowing true 100% though loosing that last 10% of control
			{
				PORTD |= (1 << 4);
			}
		else
			{
				PORTD &= ~(1 << 4);
			}

		//turn off heating pin
		//digitalWrite(heatPin,LOW); //swap for faster direct port call
		TCCR1B ^= (1 << ICES1); //flips rising to falling to rising... capture for state change capture

		icr1 = ICR1;
		targ = 100 - dutyTrig;
		targetAC_time = (icr1 / 100) * targ; // set when to trig dimmer triac based on duty option set elseware, 100-duty as trigger triggers earlier and earlier in the cycle as duty goes up

		OCR1A = targetAC_time; // target for timer1 compa, to trigger duty turn on.

		if (dutyTrig > 5) //limiting how far up the AC cycle it can trigger to avoid over lap to next AC cycle
			{
				TIMSK1 |= (1 << OCIE1A);  //Output compare eneabled
				TIMSK1 |= (1 << OCIE1B);  //Output compare eneabled
				OCR1A = targetAC_time; //+ 1; // target for timer1 compa, to trigger duty turn on. +1 other wise it doesnt trigger when 0
				OCR1B = targetAC_time + 3; // target turn off
			}
		else
			{
				TIMSK1 &= ~(1 << OCIE1A); //turn off COMPA trig when not running
			}
		TCNT1 = 0; //reset to zero
	}

ISR(TIMER1_COMPA_vect)
	{
		if (dutyTrig != 0)
			{
				//debugSerial.println("trig");
				PORTD |= (1 << 4); //turn on heat pin
				//	digitalWrite(heatPin,HIGH); //swap for faster direct port call
			}
	}

ISR(TIMER1_COMPB_vect)
	{

		if (dutyTrig > upperClamp) // allowing true 100% though loosing that last 10% of control
					{
						PORTD |= (1 << 4);
					}
		else
			{
				PORTD &= ~(1 << 4);
			}

	}

void setup()
	{
		pinMode(beepPin, OUTPUT);

		pinMode(flipLed1, OUTPUT);
		pinMode(flipLed2, OUTPUT);

		pinMode(heatPin, OUTPUT);

		pinMode(8, INPUT);

		digitalWrite(heatPin, LOW);

		//beep(20, 10);
		beep(500, 2);

		debugSerial.begin(115200);

		/*Valid values are: 2400, 4800, 9600, 19200, 31250, 38400, 57600, and 115200, 230400, 250000, 256000, 512000, and 921600*/

		nextSerial.begin(9600); //increase baud rate when going serial
		delay(500);
		nextSerial.print("baud=115200");
		nextConf();
		delay(500);

		nextSerial.end();
		nextSerial.begin(115200);
		delay(500);

		recoverPresets();

		//////////////////////
		/* AC Timing stuff*/
		noInterrupts(); // disable all interrupts

		TCCR1A = 0;
		TCCR1B = 0;

		TCCR1B &= ~(1 << WGM12); // WGm02 bit set to zero? for normal mode, to not clear on compare match? Is this right?
		/*WGM02:0=0 according to 328 datasheet, another example calls for WGM12...confusing*/

		TCCR1B |= (1 << CS12);    // 256 prescaler
		TCCR1B &= ~(1 << CS11);
		TCCR1B &= ~(1 << CS10);

		//TCCR1B |= (1 << ICNC1); //filter

		TIMSK1 |= (1 << OCIE1A);  //Output compare eneabled
		TIMSK1 |= (1 << ICIE1);  // input capture enabled

		TCNT1 = 0; //reset to zero

		interrupts();             // enable all interrupts

		/////////////////////

		nextSerialRCV.onRun(nextSerialCHK);
		nextSerialRCV.setInterval(10);
		controll.add(&nextSerialRCV);

		buttonCheck.onRun(checkButs);
		buttonCheck.setInterval(50);
		controll.add(&buttonCheck);

		heatPWM.onRun(runHeaterPWM);
		heatPWM.setInterval(20);
		controll.add(&heatPWM);

		cuTemp.onRun(updateCurrentTemp);
		cuTemp.setInterval(1000);
		controll.add(&cuTemp);

//		beeper.onRun(doBeep);
//		beeper.setInterval(1);
//		controll.add(&beeper);

		delay(2000);

		loadProf(0);
		loadPage(mainPage);

	}

void loadPage(uint8_t pages)
	{

		if (pages == mainPage)
			{
				beep(20, 2);
				nextSerial.print("page main");
				nextConf();
				delay(70);
				page = mainPage;
				plotProfile();

			}

		if (pages == presetsPage)
			{
				beep(50, 1);
				nextSerial.print("page presets");
				nextConf();
				page = presetsPage;
				delay(70);
			}

		if (pages == keypadPage)
			{
				beep(50, 1);
				nextSerial.print("page keypad");
				nextConf();
				page = keypadPage;
				delay(70);
			}

	}
//                0                1                            2                   3                   4                     5                       6                       7                      8                 10
//		 heatupRate 	heatupTime    soakTemp    soakTime    rampRate    rampTime    reflowTemp    reflowTime    coolRate     coolTime
//leadProf[10] = {    3, 	    1, 	        130, 	     70, 	  5, 	      1, 	  190,          30, 	      6,           1};
void loadKeypad(uint8_t *value, uint8_t arr)
	{
		nextSerial.print("page keypad");
		nextConf();
		delay(50);
		page = keypadPage;

		uint8_t keypadBuffer[3] { 0, 0, 0 };
		while (true)
			{
				nextSerialCHK();
				delay(50);

				uint8_t reply = checkKeypad();
				if (reply != 100)
					{
						if (reply != 20 && reply != 30)
							{
//								debugSerial.println(reply);
								keypadBuffer[2] = keypadBuffer[1];
								keypadBuffer[1] = keypadBuffer[0];
								keypadBuffer[0] = reply; //redesign to use a circular buffer read...

								char buff[20];
								sprintf(buff, "n0.val=%i%i%i", keypadBuffer[2], keypadBuffer[1],
										keypadBuffer[0]);
								nextSerial.print(buff);
								nextConf();

//								debugSerial.println(
//										(100 * keypadBuffer[2]) + (10 * keypadBuffer[1])
//												+ (keypadBuffer[0]));
							}
						else if (reply == 20)
							{
								if (arr == 2 || arr == 6)
									{
										value[arr] = (100 * keypadBuffer[2])
												+ (10 * keypadBuffer[1])
												+ (keypadBuffer[0] - tempOffset); //edit field value
									}
								else
									{
										value[arr] = (100 * keypadBuffer[2])
												+ (10 * keypadBuffer[1]) + (keypadBuffer[0]); //edit field value
									}
								loadPage(mainPage);
								break;
							}
						else if (reply == 30)
							{
								loadPage(mainPage); //cancel and reload main+
								break;
							}
					}
			}
	}

void nextConf()
	{
		nextSerial.write(0xff);
		nextSerial.write(0xff);
		nextSerial.write(0xff);
	}
void drawDot(uint8_t x, uint8_t y, uint8_t w, uint16_t col)
	{
		char buffer[30];

		int a = graphX + x;
		int b = graphY + graphH - y;

		sprintf(buffer, "cirs %i,%i,%i,%i", a, b, w, col);

		nextSerial.print(buffer);
		nextConf();

	}

void renderLine(uint8_t xA, uint8_t yA, uint8_t xB, uint8_t yB, uint16_t col) // drawing a line primitive
	{
		int xAa = graphX + xA;
		int xBb = graphX + xB;

		int yAa = graphY + graphH - yA;
		int yBb = graphY + graphH - yB;

		char buffer[30];

		sprintf(buffer, "line %i,%i,%i,%i,%i", xAa, yAa, xBb, yBb, col);
		nextSerial.print(buffer);
		nextConf();
	}

void drawLine(uint8_t fromX, uint8_t fromY, uint8_t toX, uint8_t toY) // drawing line with circles along a path give thickness
	{
		int16_t stepX = toX - fromX;
		int16_t stepY = toY - fromY;

		float stepsY = (float(stepY) / float(stepX));

		for (uint8_t d = 0; d <= stepX; d++)
			{
				drawDot(fromX + d, fromY + (stepsY * d), 1, 64960);
			}

	}
//                0                1                            2                   3                   4                     5                       6                       7                      8                 10
//		 heatupRate 	heatupTime    soakTemp    soakTime    rampRate    rampTime    reflowTemp    reflowTime    coolRate     coolTime
//leadProf[10] = {    3, 	    1, 	        130, 	     70, 	  5, 	      1, 	  190,          30, 	      6,           1};

void plotProfile()
	{
		uint8_t curr = 25; //replace this with temp probe reading

		uint8_t preheatRateTime = ((loadedProf[2] - curr + tempOffset)
				/ loadedProf[0]); //-25 average room temp for starting could replace with ambient temp from sensor at boot time
		uint8_t soaktime = loadedProf[3];
		uint8_t heatRampTime = ((loadedProf[6] - loadedProf[2]) / loadedProf[4]);
		uint8_t reflow = loadedProf[7];
		uint8_t cooldown = (loadedProf[6] - curr) / 6;

//		debugSerial.print(preheatRateTime);
//		debugSerial.print(" ");
//		debugSerial.print(soaktime);
//		debugSerial.print(" ");
//		debugSerial.print(heatRampTime);
//		debugSerial.print(" ");
//		debugSerial.print(reflow);
//		debugSerial.print(" ");
//		debugSerial.print(cooldown);

		uint16_t totalTime = preheatRateTime + soaktime + heatRampTime + reflow
				+ cooldown;
//
//		debugSerial.print(" = ");
//		debugSerial.println(totalTime);

		uint8_t preheatX = (float(graphW) / float(totalTime))
				* float(preheatRateTime);
		uint8_t preheatY = (float(graphH) / (float(loadedProf[6] + graphHeadroom)))
				* float(loadedProf[2]);

//		debugSerial.print(preheatX);
//		debugSerial.print(" ");
//		debugSerial.println(preheatY);

		//drawDot(3, 3, 3, 64960);

		//drawDot(preheatX, preheatY, 3, 64960);

		uint8_t flowrampX = (float(graphW) / float(totalTime))
				* float(preheatRateTime + loadedProf[3]);
		// drawDot(flowrampX, preheatY, 3, 64960);

		uint8_t peakdot = (float(graphW) / float(totalTime))
				* float(preheatRateTime + loadedProf[3] + heatRampTime);
		uint8_t peakY = (float(graphH) / (float(loadedProf[6] + graphHeadroom)))
				* loadedProf[6];
		// drawDot(peakdot, peakY, 3, 64960);

		uint16_t peakdotend = (float(graphW) / float(totalTime))
				* float(
						(preheatRateTime + loadedProf[3] + heatRampTime + loadedProf[7]));
		uint8_t peakYend = (float(graphH) / (float(loadedProf[6] + graphHeadroom)))
				* loadedProf[6];
		// drawDot(peakdotend, peakYend, 3, 64960);

		// drawDot(graphW - 3, 3, 3, 64960);

		//draw the connecting line

		drawLine(2, 2, preheatX, preheatY);
		drawLine(preheatX, preheatY, flowrampX, preheatY);
		drawLine(flowrampX, preheatY, peakdot, peakY);
		drawLine(peakdot, peakY, peakdotend, peakYend);
		drawLine(peakdotend, peakYend, graphW - 2, 2);

		renderLine(2, 2, preheatX, preheatY, 64960);
		renderLine(preheatX, preheatY, flowrampX, preheatY, 64960);
		renderLine(flowrampX, preheatY, peakdot, peakY, 64960);
		renderLine(peakdot, peakY, peakdotend, peakYend, 64960);
		renderLine(peakdotend, peakYend, graphW - 2, 2, 64960);

		renderLine(preheatX, 0, preheatX, graphH, 19017);
		renderLine(flowrampX, 0, flowrampX, graphH, 19017);
		renderLine(peakdot, 0, peakdot, graphH, 19017);
		renderLine(peakdotend, 0, peakdotend, graphH, 19017);

		char buffer[25];

		sprintf(buffer, "huRate.txt=\"%i  \"", loadedProf[0]);
		nextSerial.print(buffer);
		nextConf();

		sprintf(buffer, "huTime.txt=\"%i  \"", loadedProf[1]);
		nextSerial.print(buffer);
		nextConf();

		sprintf(buffer, "soakTemp.txt=\"%i\"", loadedProf[2] + tempOffset);
		nextSerial.print(buffer);
		nextConf();

		sprintf(buffer, "soakTime.txt=\"%i\"", loadedProf[3]);
		nextSerial.print(buffer);
		nextConf();

		sprintf(buffer, "rpRate.txt=\"%i  \"", loadedProf[4]);
		nextSerial.print(buffer);
		nextConf();

		sprintf(buffer, "rpTime.txt=\"%i  \"", loadedProf[5]);
		nextSerial.print(buffer);
		nextConf();

		sprintf(buffer, "rTemp.txt=\"%i\"", loadedProf[6] + tempOffset);
		nextSerial.print(buffer);
		nextConf();

		sprintf(buffer, "rTime.txt=\"%i\"", loadedProf[7]);
		nextSerial.print(buffer);
		nextConf();

		sprintf(buffer, "tarTemp.txt=\"%i °C\"", 0);
		nextSerial.print(buffer);
		nextConf();

		uint8_t minutes = totalTime / 60;
		uint8_t seconds = totalTime - (60 * minutes);

		sprintf(buffer, "stopclock.txt=\"%i:%i\"", minutes, seconds);
		nextSerial.print(buffer);
		nextConf();

		switch (profile)
			{
		case 0:
			{
				nextSerial.print("profName.txt=\"LEADED\"");
				nextConf();
				break;
			}
		case 1:
			{
				nextSerial.print("profName.txt=\"LEAD FREE\"");
				nextConf();
				break;
			}
		case 2:
			{
				nextSerial.print("profName.txt=\"Profile 1\"");
				nextConf();
				break;
			}
		case 3:
			{
				nextSerial.print("profName.txt=\"Profile 2\"");
				nextConf();
				break;
			}
		case 4:
			{
				nextSerial.print("profName.txt=\"Profile 3\"");
				nextConf();
				break;
			}
			}

	}

void nextSerialCHK()
	{
		if (nextSerial.available() > 0)
			{
				uint8_t data = nextSerial.read();
				if (data == 0xFF)
					{
						goto end;
						// ignore nextions ff ff ff on startup
					}
				// eg, of being sent 0a 01 b0 ee ee ee

//				char buff[20];
//				sprintf(buff, "%x",data);
//				debugSerial.println(buff);

				if (data == 0x0A) //capture header for added robust sync
					{
						nextCMDBuff[0] = data;
						nextbuffcount = 1;
					}
				else if (data != 0xEE)
					{
						nextCMDBuff[nextbuffcount] = data;
						nextbuffcount++;
					}
				else if (data == 0xEE)
					{

						nextCMDBuff[nextbuffcount] = data;
						nextbuffcount++;
						nextendConf++;
					}
			}

		if (nextendConf == 3)
			{
				nextbuffcount = 0;
				nextendConf = 0;
				nextCMDrdy = true;
			}

		end: // ignore nextions ff ff ff on startup
		if (nextCMDrdy == true)
			{
				nextCMDrdy = false;

				if (nextCMDBuff[1] == 0x01) //mainpage buttons
					{
						switch (nextCMDBuff[2])
							{
						case 0xb0:
							startBut = true;
							break;
						case 0xb1:
							abortBut = true;
							break;
						case 0xb2:
							presetBut = true;
							break;
						case 0xd0:
							huRateBut = true;
							break;
						case 0xd1:
							//huTimeBut = true;
							break;
						case 0xd2:
							soakTempBut = true;
							break;
						case 0xd3:
							soakTimeBut = true;
							break;
						case 0xd4:
							rpRateBut = true;
							break;
						case 0xd5:
							//rpTimeBut = true;
							break;
						case 0xd6:
							rTempBut = true;
							break;
						case 0xd7:
							rTimeBut = true;
							break;
							}

					}

				if (nextCMDBuff[1] == 0x02) //profile buttons
					{
						switch (nextCMDBuff[2])
							{
						case 0xc0:
							leadProfBut = true;
							break;
						case 0xc1:
							leadfreeProfBut = true;
							break;
						case 0xc2:
							prof1But = true;
							break;
						case 0xc3:
							prof2But = true;
							break;
						case 0xc4:
							prof3But = true;
							break;
							}
					}

				if (nextCMDBuff[1] == 0x0E) //profile save command
					{
						switch (nextCMDBuff[2])
							{
						case 0x01:
							prof1Set = true;
							break;
						case 0x02:
							prof2Set = true;
							break;
						case 0x03:
							prof3Set = true;
							break;
							}
					}

				if (nextCMDBuff[1] == 0x03) //keypads buttons
					{
						switch (nextCMDBuff[2])
							{
						case 0xb0:
							kp0But = true;
							break;
						case 0xb1:
							kp1But = true;
							break;
						case 0xb2:
							kp2But = true;
							break;
						case 0xb3:
							kp3But = true;
							break;
						case 0xb4:
							kp4But = true;
							break;
						case 0xb5:
							kp5But = true;
							break;
						case 0xb6:
							kp6But = true;
							break;
						case 0xb7:
							kp7But = true;
							break;
						case 0xb8:
							kp8But = true;
							break;
						case 0xb9:
							kp9But = true;
							break;

						case 0xe0:
							entBut = true;
							break;

						case 0xc0:
							cnclBut = true;
							break;
							}
					}
			}
	}

void checkButs()
	{
		if (startBut == true)
			{
				startBut = false;
				if (heat.heating != true)
					{
						reflowCycle();
					}
			}
		if (abortBut == true)
			{
				abortBut = false;
				//	if (heat.heating == true)
				//	{
				stopHeater();
				beep(30, 8);
				//loadPage(mainPage);
				//	}
			}
		if (presetBut == true)
			{
				presetBut = false;
				if (heat.heating == false)
					{
						beep(50, 1);
						loadPage(presetsPage);
					}
			}
//              0            					1                        2                   3                  4                      5                        6                        7                    8                   10
//		 heatupRate 	heatupTime    soakTemp    soakTime    rampRate    rampTime    reflowTemp    reflowTime    coolRate     coolTime
//leadProf[10] = {    3, 	    1, 	        130, 	     70, 	  5, 	      1, 	  190,          30, 	      6,           1};
		//settings
		if (huRateBut == true)
			{
				huRateBut = false;		//disabled fixing it to 2c psec, ovens limit.
				if (heat.heating != true)
					{
						beep(50, 1);
						loadKeypad(loadedProf, 0);
					}
			}
		if (huTimeBut == true)
			{
				huTimeBut = false;
				//loadKeypad(loadedProf, 1);
			}
		if (soakTempBut == true)
			{
				soakTempBut = false;
				if (heat.heating != true)
					{
						beep(50, 1);

						loadKeypad(loadedProf, 2);
					}
			}
		if (soakTimeBut == true)
			{
				soakTimeBut = false;
				if (heat.heating != true)
					{
						beep(50, 1);
						loadKeypad(loadedProf, 3);

					}
			}
		if (rpRateBut == true)
			{
				rpRateBut = false; //disabled fixing it to 2c psec, ovens limit.
				if (heat.heating != true)
					{
						beep(50, 1);

						loadKeypad(loadedProf, 4);
					}
			}
		if (rpTimeBut == true)
			{
				rpTimeBut = false;
				//	loadKeypad(loadedProf, 5);
			}
		if (rTempBut == true)
			{
				rTempBut = false;
				if (heat.heating != true)
					{
						beep(50, 1);

						loadKeypad(loadedProf, 6);
					}
			}
		if (rTimeBut == true)
			{
				rTimeBut = false;
				if (heat.heating != true)
					{
						beep(50, 1);
						loadKeypad(loadedProf, 7);
					}
			}

		//profiles
		if (leadProfBut == true)
			{
				beep(50, 1);
				leadProfBut = false;
				loadProf(0);
				loadPage(mainPage);

			}
		if (leadfreeProfBut == true)
			{
				beep(50, 1);
				leadfreeProfBut = false;
				loadProf(1);
				loadPage(mainPage);

			}
		if (prof1But == true)
			{
				beep(50, 1);
				prof1But = false;
				loadProf(2);
				loadPage(mainPage);

			}
		if (prof2But == true)
			{
				beep(50, 1);
				prof2But = false;
				loadProf(3);
				loadPage(mainPage);

			}
		if (prof3But == true)
			{
				beep(50, 1);
				prof3But = false;
				loadProf(4);
				loadPage(mainPage);

			}

		//save presets
		if (prof1Set == true)
			{
				beep(500, 1);
				prof1Set = false;
				savePreset(1);
				//loadPage(mainPage);
			}

		if (prof2Set == true)
			{
				beep(500, 1);
				prof2Set = false;
				savePreset(2);
				//	loadPage(mainPage);
			}

		if (prof3Set == true)
			{
				beep(500, 1);
				prof3Set = false;
				savePreset(3);
				//	loadPage(mainPage);
			}

	}

uint8_t checkKeypad()
	{
		//Keypad
		if (kp0But == true)
			{
				beep(50, 1);
				kp0But = false;
				return 0;

			}
		else if (kp1But == true)
			{
				beep(50, 1);
				kp1But = false;
				return 1;

			}
		else if (kp2But == true)
			{
				beep(50, 1);
				kp2But = false;
				return 2;

			}
		else if (kp3But == true)
			{
				beep(50, 1);
				kp3But = false;
				return 3;

			}
		else if (kp4But == true)
			{
				beep(50, 1);
				kp4But = false;
				return 4;

			}
		else if (kp5But == true)
			{
				beep(50, 1);
				kp5But = false;
				return 5;

			}
		else if (kp6But == true)
			{
				beep(50, 1);
				kp6But = false;
				return 6;

			}
		else if (kp7But == true)
			{
				beep(50, 1);
				kp7But = false;
				return 7;

			}
		else if (kp8But == true)
			{
				beep(50, 1);
				kp8But = false;
				return 8;

			}
		else if (kp9But == true)
			{
				beep(50, 1);
				kp9But = false;
				return 9;

			}

		else if (entBut == true)
			{
				beep(100, 1);
				entBut = false;
				return 20;
			}
		else if (cnclBut == true)
			{
				beep(50, 2);
				cnclBut = false;
				return 30;
			}
		else
			{
				return 100;
			}

	}
void passThrough()
	{

		char data = debugSerial.read();

		if (data != '\n')
			{
				serialBuffer[serialbuffcount] = data;
				serialbuffcount++;
			}
		else
			{

				serialBuffer[serialbuffcount] = data;
				serialbuffcount = 0;

				debugSerial.print("passing ");
				debugSerial.print(serialBuffer);

				for (int i = 0; i < 30; i++)
					{
						if (serialBuffer[i] != '\n')
							{
								nextSerial.write(serialBuffer[i]);
							}
						else
							{
								break;
							}
					}
				nextConf();

				for (int i = 0; i < 30; i++)
					{
						serialBuffer[i] = NULL;
					}
			}
	}

void loadProf(uint8_t prof)
	{
		profile = prof;
		if (profile == 0)
			{
				setProf(leadProf);
			}
		else if (profile == 1)
			{
				setProf(unleadProf);
			}
		else if (profile == 2)
			{
				setProf(prof1);
			}
		else if (profile == 3)
			{
				setProf(prof2);
			}
		else if (profile == 4)
			{
				setProf(prof3);
			}
	}

void setProf(uint8_t *prof)
	{
		for (int i = 0; i < 10; i++)
			{
				loadedProf[i] = prof[i];
			}
	}

void loop()
	{
		controll.run();
		//digitalWrite(heatPin, LOW);

		//plotProfile(unleadProf);

//		if (debugSerial.available())
//			{
//				passThrough();
//			}
	}
