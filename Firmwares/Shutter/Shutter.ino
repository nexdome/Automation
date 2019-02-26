/*
* PDM NexDome Shutter kit firmware. NOT compatible with original NexDome ASCOM driver or Rotation kit firmware.
*
* Copyright (c) 2018 Patrick Meloy
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
*  files (the Software), to deal in the Software without restriction, including without limitation the rights to use, copy,
*  modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
*  is furnished to do so, subject to the following conditions:
*  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED AS IS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
*  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
*  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
*  OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*  Inspired by the original official NexDome firmware by grozzie2 but completely incompatible.
*  https://github.com/grozzie2/NexDome
*/


/*
** Basic operation
** Communications between the rotator and shutter are generally kept at a minimum with the only constant communication being periodic
** rain sensor checks. The driver talks to the rotator and the rotator will pass on any shutter commands to the shutter over wireless.
** Changing shutter settings from the driver usually initiates a one-way message to the shutter with the new setting. Since this has
** to pass through the rotator the rotator will store the value and the shutter doesn't have to respond to the rotator.
**
** When the shutter gets a movement command it will start updating the rotator once per second until the move is complete. That way the
** rotator doesn't have to ask for the values, cutting down on the number of messages going back and forth.
**
** On startup the shutter sends a Hello message over wireless. If the rotator is present it will ask the shutter for all the settings.
**
** If the rotator is powered up after the shutter then the rotator will send out a Hello message and if the shutter is powered up they
** will initiate the data exchange.
**
** One of the drawbacks of a "dumb" stepper driver is that if the Arduino is doing something else the stepper is not updated causing
** it to try and stop during a move. This doesn't last long but it does create a noticable "tick" sound. Each time it ticks there is
** extra stress on the motor and gearing so reducing the ticking by not having to receive lots of update requests as well as sending
** them is a good thing.
**
*/

#include <AccelStepper.h>
#include <EEPROM.h>
#include "ShutterClass.h"

#define Computer Serial
String serialBuffer;

#define Wireless Serial1
String wirelessBuffer;

const String version = "2.1";

#pragma endregion

#pragma region Command character constants
const char ABORT_CMD				= 'a';
const char ACCELERATION_SHUTTER_CMD = 'E'; // Get/Set stepper acceleration
const char CLOSE_SHUTTER_CMD		= 'C'; // Close shutter
const char ELEVATION_SHUTTER_CMD	= 'G'; // Get/Set altitude
const char HELLO_CMD				= 'H'; // Let rotator know we're here
const char OPEN_SHUTTER_CMD			= 'O'; // Open the shutter
const char POSITION_SHUTTER_GET		= 'P'; // Get step position
const char WATCHDOG_INTERVAL_SET		= 'I'; // Tell us how long between checks in seconds
const char RAIN_ROTATOR_GET			= 'F'; // Rotator telling us if it's raining or not
const char SLEEP_SHUTTER_CMD		= 'S'; // Get/Set radio sleep settings
const char SPEED_SHUTTER_CMD		= 'R'; // Get/Set step rate (speed)
const char REVERSED_SHUTTER_CMD		= 'Y'; // Get/Set stepper reversed status
const char STATE_SHUTTER_GET		= 'M'; // Get shutter state
const char STEPSPER_SHUTTER_CMD		= 'T'; // Get/Set steps per stroke
const char VERSION_SHUTTER_GET		= 'V'; // Get version string
const char VOLTS_SHUTTER_CMD		= 'K'; // Get volts and get/set cutoff
const char SHUTTER_PING					= 'L'; // use to reset watchdong timer.
const char VOLTSCLOSE_SHUTTER_CMD	= 'B';
const char INIT_XBEE						= 'x'; // force a ConfigXBee

#pragma endregion


ShutterClass Shutter;

int configStep = 0;
String ATString = "";

bool SentHello = false, XbeeStarted = false;
bool isRaining = false;

// StopWatch nextUpdateTimer;
// StopWatch nextStepTimer;

// unsigned long updateInterval;
unsigned long stepInterval;

// StopWatch nextVoltageUpdateTimer;
unsigned long voltUpdateInterval = 5000;

// StopWatch nextRainCheckTimer;
bool doFinalUpdate = false;

void setup()
{
	Computer.begin(9600);
	Wireless.begin(9600);
	// updateInterval = 1000;
	stepInterval = 100;
	watchdogTimer.reset();
}

void loop()
{
	if (Computer.available() > 0)
		ReceiveSerial();

	if (Wireless.available())
		ReceiveWireless();

	if (!XbeeStarted) {
		if (!Shutter.radioIsConfigured  && !Shutter.isConfiguringWireless) {
			StartWirelessConfig();
		}
		else if (Shutter.radioIsConfigured) {
			XbeeStarted = true;
			wirelessBuffer = "";
			DBPrintln("Radio configured");
		}
	}

	if(watchdogTimer.elapsed() >= Shutter.watchdogInterval) {
		// we lost communication with the rotator.. close everything.
		if (Shutter.GetState() != CLOSED && Shutter.GetState() != CLOSING)
			Shutter.Close();
	}

	Shutter.DoButtons();
	Shutter.Run();
}

///<SUMMARY>
///Send all data to rotator every second but stagger the messages over that second.
///</SUMMARY>
/* Run through this once per second when the stepper is (or was) running.
** Shutter.sendUpdates is set to true when a movement command is received and set to false
** in Shutter.run() if the motor is stopped.
** It's entirely possible for the motor to stop part way through the update steps which would mean the next time around
** no updates would be sent even though the data is different from what was already sent to the rotator. To prevent this
** the sendUpdates state is stored at the start of the update steps and checked at the end. If they are different then
** doFinalUpdate is set to true so the update cycle will run one final time even though the motor is stopped.
*/

/*
void UpdateRotator()
{
	static bool sentState = false, sentPosition = false, runningAtaStart = false;

	runningAtaStart = Shutter.sendUpdates; // Store motion state to comparison at end
	if(nextVoltageUpdateTimer.elapsed() >= voltUpdateInterval) {
		Wireless.print(VOLTS_SHUTTER_CMD + Shutter.GetVoltString() + "#");
		nextVoltageUpdateTimer.reset();
	}

	if(nextStepTimer.elapsed() < stepInterval) {
		return;
	}

	if (!sentState) {
		Wireless.print(String(STATE_SHUTTER_GET) + String(Shutter.GetState()) + "#");
		sentState = true;
		nextStepTimer.reset();
		return;
	}

	//TODO: Not ready yet - when handbox is done finish this up
	//if (!sentElevation) {
	//	Wireless.print(String(ELEVATION_SHUTTER_CMD) + String(Shutter.GetElevation()) + "#");
	//	sentElevation = true;
	// nextStepTimer.reset()
	//	return;
	//}

	if (!sentPosition) {
		Wireless.print(String(POSITION_SHUTTER_GET) + String(Shutter.GetPosition()) + "#");
		sentPosition = true;
		nextStepTimer.reset();
		return;
	}

	nextUpdateTimer.reset();

	sentState = sentPosition = false; // Reset the bools for the next cycle
	Shutter.sendUpdates = false;
	if (runningAtaStart != Shutter.sendUpdates) { // See if motor stopped after update steps started
		doFinalUpdate = true; // Motor stopped after starting the update steps so do one more update to catch those up.
	}
	else {
		doFinalUpdate = false;
	}
}
*/

#pragma region XBeeRoutines
void StartWirelessConfig()
{
	Computer.println("Xbee configuration started");
	delay(1100); // guard time before and after
	Shutter.isConfiguringWireless = true;
	DBPrintln("Sending +++");
	Wireless.print("+++");
	delay(1100);
}

inline void ConfigXBee(String result)
{
	if (configStep == 0) {
		// ATString = "ATCE0,ID7734,AP0,SM0,RO0,WR,CN";
		//  CE0 for end device, shutter MY is 1
		ATString = "ATCE0,ID7734,CH0C,MY1,DH0,DL0,AP0,SM0,BD3,WR,CN";
		DBPrintln("AT String " + ATString);
		Wireless.println(ATString);
		Wireless.flush();

	}
	DBPrintln("Result " + String(configStep) + ":" + result);
	if (configStep > 9) {
		Shutter.isConfiguringWireless = false;
		Shutter.radioIsConfigured = true;
		XbeeStarted = true;
		Shutter.WriteEEProm();
		delay(10000);
		Computer.println("Xbee configuration finished");
		while(Wireless.available() > 0) {
			Wireless.read();
		}
	}
	configStep++;
}

#pragma endregion

#pragma region Communications
void ReceiveSerial()
{
	char character = Computer.read();

	if (character == '\r' || character == '\n') {
		// End of message
		if (serialBuffer.length() > 0) {
			ProcessMessages(serialBuffer);
			serialBuffer = "";
		}
	}
	else {
		serialBuffer += String(character);
	}
}

void ReceiveWireless()
{
	char character;
	// read as much as possible in one call to ReceiveWireless()
	while(Wireless.available()) {
		character = Wireless.read();

		if (character == '\r' || character == '#') {
			if (wirelessBuffer.length() > 0) {
				if (Shutter.isConfiguringWireless) {
					DBPrint("Configuring XBee");
					ConfigXBee(wirelessBuffer);
				}
				else {
					watchdogTimer.reset(); // communication are working
					ProcessMessages(wirelessBuffer);
				}
				wirelessBuffer = "";
			}
		}
		else {
			wirelessBuffer += String(character);
		}
	} // end while
}

void ProcessMessages(String buffer)
{
	// float localFloat;
	int32_t local32;
	int16_t local16;

	String value, computerMessage="", wirelessMessage="";
	char command;


	if (buffer.equals("OK")) {
		DBPrint("Buffer == OK");
		return;
	}

	command = buffer.charAt(0); // If "H" the second letter says what it's from. We only care if it's "R", the rotator.
	value = buffer.substring(1); // Payload if the command has data.
	DBPrintln("<<< Command:" + String(command) + " Value:" + value);

	switch (command) {
		case ACCELERATION_SHUTTER_CMD:
			if (value.length() > 0) {
				DBPrintln("Set acceleration to " + value);
				local32 = value.toInt();
				Shutter.SetAcceleration(local32);
			}
			wirelessMessage = String(ACCELERATION_SHUTTER_CMD) + String(Shutter.GetAcceleration());
			DBPrintln("Acceleration is " + String(Shutter.GetAcceleration()));
			break;

		case ABORT_CMD:
			// Rotator update will be through UpdateRotator
			DBPrintln("STOP!");
			Shutter.Stop();
			wirelessMessage = String(ABORT_CMD);
			break;

		case CLOSE_SHUTTER_CMD:
			// Rotator update will be through UpdateRotator
			DBPrintln("Close shutter");
			if (Shutter.GetState() != CLOSED) {
				Shutter.Close();
			}
			wirelessMessage = String(STATE_SHUTTER_GET) + String(Shutter.GetState());
			break;

		case HELLO_CMD:
			DBPrintln("Rotator says hello!");
			wirelessMessage = String(HELLO_CMD);
			DBPrintln("Sending hello back");
			break;

		case OPEN_SHUTTER_CMD:
			// Rotator update will be through UpdateRotator
			DBPrintln("Received Open Shutter Command");
			if (isRaining) {
				wirelessMessage = "OR"; // (O)pen command (R)ain cancel
				DBPrintln("Raining");
			}
			else if (Shutter.GetVoltsAreLow()) {
				wirelessMessage = "OL"; // (O)pen command (L)ow voltage cancel
				DBPrintln("Voltage Low");
			}
			else {
				if (Shutter.GetState() != OPEN) Shutter.Open();
			}

			break;

		case POSITION_SHUTTER_GET:
			 //Rotator update will be through UpdateRotator
			wirelessMessage = String(POSITION_SHUTTER_GET) + String(Shutter.GetPosition());
			DBPrintln(wirelessMessage);
			break;

		case WATCHDOG_INTERVAL_SET:
			if (value.length() > 0) {
				Shutter.SetWatchdogInterval(value.toInt());
				DBPrintln("Watchdog interval set to " + value + "ms");
			}
			else {
				DBPrintln("Rain check interval " + String(Shutter.watchdogInterval));
			}
			wirelessMessage = String(WATCHDOG_INTERVAL_SET) + String(Shutter.watchdogInterval);
			break;

		case RAIN_ROTATOR_GET:
			local16 = value.toInt();
			if (local16 == 1) {
				if (!isRaining) {
					if (Shutter.GetState() != CLOSED && Shutter.GetState() != CLOSING)
						Shutter.Close();
					isRaining = true;
					DBPrintln("It's raining! (" + value + ")");
				}
			}
			else {
				isRaining = false;
				DBPrintln("It's not raining");
			}
			wirelessMessage = String(RAIN_ROTATOR_GET);
			break;

		case REVERSED_SHUTTER_CMD:
			if (value.length() > 0) {
				Shutter.SetReversed(value.equals("1"));
				DBPrintln("Set Reversed to " + value);
			}
			wirelessMessage = String(REVERSED_SHUTTER_CMD) + String(Shutter.GetReversed());
			DBPrintln(wirelessMessage);
			break;

		case SPEED_SHUTTER_CMD:
			if (value.length() > 0) {
				local32 = value.toInt();
				DBPrintln("Set speed to " + value);
				if (local32 > 0) Shutter.SetMaxSpeed(value.toInt());
			}
			wirelessMessage = String(SPEED_SHUTTER_CMD) + String(Shutter.GetMaxSpeed());
			DBPrintln(wirelessMessage);
			break;

		case STATE_SHUTTER_GET:
			wirelessMessage = String(STATE_SHUTTER_GET) + String(Shutter.GetState());
			DBPrintln(wirelessMessage);
			break;

		case STEPSPER_SHUTTER_CMD:
			if (value.length() > 0) {
				local32 = value.toInt();
				if (local32 > 0) {
					Shutter.SetStepsPerStroke(local32);
				}
			}
			else {
				DBPrintln("Get Steps " + String(Shutter.GetStepsPerStroke()));
			}
			wirelessMessage = String(STEPSPER_SHUTTER_CMD) + String(Shutter.GetStepsPerStroke());
			break;

		case VERSION_SHUTTER_GET:
			wirelessMessage = "V" + version;
			DBPrintln(wirelessMessage);
			break;

		case VOLTS_SHUTTER_CMD:
			if (value.length() > 0) {
				Shutter.SetVoltsFromString(value);
				DBPrintln("Set volts to " + value);
			}
			wirelessMessage = "K" + Shutter.GetVoltString();
			DBPrintln(wirelessMessage);
			break;

		case VOLTSCLOSE_SHUTTER_CMD:
			if (value.length() > 0) {
				DBPrintln("Close on low voltage value inn" + String(value));
				Shutter.SetVoltsClose(value.toInt());
			}
			wirelessMessage = String(VOLTSCLOSE_SHUTTER_CMD) + String(Shutter.GetVoltsClose());
			DBPrintln("Close on low voltage " + String(Shutter.GetVoltsClose()));
			break;

		case INIT_XBEE:
			Shutter.radioIsConfigured = false;
			Shutter.isConfiguringWireless = false;
			XbeeStarted = false;
			configStep = 0;
			wirelessMessage = String(INIT_XBEE);
			break;

		case SHUTTER_PING:
			wirelessMessage = String(SHUTTER_PING);
			watchdogTimer.reset();
			break;

		default:
			DBPrintln("Unknown command " + String(command));
			break;
	}

	if (wirelessMessage.length() > 0) {
		DBPrintln(">>> Sending " + wirelessMessage);
		Wireless.print(wirelessMessage +"#");
	}
}
#pragma endregion
