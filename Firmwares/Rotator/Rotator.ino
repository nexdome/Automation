/*
* NexDome Shutter kit firmware. NOT compatible with original NexDome ASCOM driver or Rotation kit firmware.
*
* Copyright (c) 2018 Patrick Meloy
* Copyright (c) 2019 Rodolphe Pineau, Ron Crouch, NexDome
*
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

// Uncomment #define DEBUG in RotatorClass.h to enable printing debug messages in serial

#include "RotatorClass.h"
#include "RemoteShutterClass.h"
#include <AccelStepper.h>

//todo: Implement low voltage safety
// Decide on a "comment" char to start all debug printing with so
// ASCOM definately won't be affected (all messages validated anyway but
// that'll stop mistaken "Invalid" messages.

#pragma region Devices
RotatorClass Rotator ;
RemoteShutterClass RemoteShutter;

#define MAX_TIMEOUT 10
#define ERR_NO_DATA	-1
#define OK	0


#define VERSION "2.12"

#define Computer Serial
String computerBuffer;

#define Wireless Serial1
String wirelessBuffer;

#pragma endregion

#pragma region Declarations and Variables

// Flag to do XBee startup on first boot in loop(). Could do in setup but
// serial may not be ready so debugging prints won't show. Also used
// to make sure the XBee has started and configured itself before
// trying to send any wireless messages.

bool XbeeStarted = false, sentHello = false, isConfiguringWireless = false, gotHelloFromShutter = false;
int configStep;
int sleepMode = 0, sleepPeriod = 300, sleepDelay = 30000;
String ATString="";
static const unsigned long pingInterval = 30000; // 30 seconds, can't be changed

// Once booting is done and XBee is ready, broadcast a hello message
// so a shutter knows you're around if it is already running. If not,
// the shutter will send a hello when it's booted up.
bool SentHello = false;

// Timer to periodically checks for rain and shutter ping.

StopWatch Rainchecktimer;
StopWatch PingTimer;

bool bIsRaining = false;
bool bShutterPresnt = false;
#pragma endregion

#pragma region command constants

const char DEBUG_MSG_CMD			= '*'; // start of a debug message sent to the Shutter Arduino.
const char ACCELERATION_ROTATOR_CMD		= 'e'; // Get/Set stepper acceleration
const char ABORT_MOVE_CMD				= 'a'; // Tell everything to STOP!
const char CALIBRATE_ROTATOR_CMD		= 'c'; // Calibrate the dome
const char ERROR_AZ_ROTATOR_GET			= 'o'; // Azimuth error when I finally implement it
const char GOTO_ROTATOR_CMD				= 'g'; // Get/set dome azimuth
const char HOME_ROTATOR_CMD				= 'h'; // Home the dome
const char HOMEAZ_ROTATOR_CMD			= 'i'; // Get/Set home position
const char HOMESTATUS_ROTATOR_GET		= 'z'; // Get homed status
const char MOVE_RELATIVE_ROTATOR_CMD	= 'b'; // Move relative - steps from current position +/-
const char PARKAZ_ROTATOR_CMD			= 'l'; // Get/Set park azimuth
const char POSITION_ROTATOR_CMD			= 'p'; // Get/Set step position
const char RAIN_ROTATOR_ACTION			= 'n'; // Get/Set action when rain sensor triggered none, home, park
const char RAIN_ROTATOR_CMD				= 'f'; // Get or Set Rain Check Interval
const char RAIN_ROTATOR_TWICE_CMD		= 'j'; // Get/Set Rain check requires to hits
const char REVERSED_ROTATOR_CMD			= 'y'; // Get/Set stepper reversed status
const char SEEKSTATE_GET				= 'd'; // None, homing, calibration steps.
const char SLEW_ROTATOR_GET				= 'm'; // Get Slewing status/direction
const char SPEED_ROTATOR_CMD			= 'r'; // Get/Set step rate (speed)
const char STEPSPER_ROTATOR_CMD			= 't'; // GetSteps per rotation
const char SYNC_ROTATOR_CMD				= 's'; // Sync to telescope
const char VERSION_ROTATOR_GET			= 'v'; // Get Version string
const char VOLTS_ROTATOR_CMD			= 'k'; // Get volts and get/set cutoff
const char INIT_XBEE						= 'x'; // force a ConfigXBee

const char ACCELERATION_SHUTTER_CMD		= 'E'; // Get/Set stepper acceleration
const char CLOSE_SHUTTER_CMD			= 'C'; // Close shutter
//const char ELEVATION_SHUTTER_CMD		= 'G'; // Get/Set altitude
const char HELLO_CMD					= 'H'; // Let shutter know we're here
const char HOMESTATUS_SHUTTER_GET		= 'Z'; // Get homed status (has it been closed)
const char INACTIVE_SHUTTER_CMD			= 'X'; // Get/Set how long before shutter closes
const char OPEN_SHUTTER_CMD				= 'O'; // Open the shutter
const char POSITION_SHUTTER_GET			= 'P'; // Get step position
const char WATCHDOG_INTERVAL_SET			= 'I'; // Tell shutter when to trigger the wtachdog for communication loss with rotator
const char RAIN_SHUTTER_GET				= 'F'; // Get rain status (from client) or tell shutter it's raining (from Rotator)
const char SPEED_SHUTTER_CMD			= 'R'; // Get/Set step rate (speed)
const char REVERSED_SHUTTER_CMD			= 'Y'; // Get/Set stepper reversed status
const char SLEEP_SHUTTER_CMD			= 'S'; // Get/Set radio sleep settings
const char STATE_SHUTTER_GET			= 'M'; // Get shutter state
const char STEPSPER_SHUTTER_CMD			= 'T'; // Get/Set steps per stroke
const char VERSION_SHUTTER_GET			= 'V'; // Get version string
const char VOLTS_SHUTTER_CMD			= 'K'; // Get volts and get/set cutoff
const char SHUTTER_PING					= 'L'; // use to reset watchdong timer.
const char VOLTSCLOSE_SHUTTER_CMD			= 'B'; // Get/Set if shutter closes and rotator homes on shutter low voltage
#pragma endregion

/*
** An XBee may or may not be present and we don't want to waste time trying to
** talk to an XBee that isn't there. Good thing is that all serial comms are asychronous
** so just try to start a read-only check of it's configuration then if it responds go
** ahead and use it. if it doesn't respond, there's nothing that will talk to it. Config
** routine sets XBee.present to true if the XBee responds.
*/

#pragma region "Arduino Setup and Loop"
void setup()
{
	Computer.begin(9600);
	Wireless.begin(9600);
	Rainchecktimer.reset();
	PingTimer.reset();
	DBPrint("Ready");
}

void loop()
{

	if (!XbeeStarted) {
		if (!Rotator._radioIsConfigured && !isConfiguringWireless) {
			DBPrint("Xbee reconfiguring");
			StartWirelessConfig();
			DBPrint("Rotator._radioIsConfigured : " + String(Rotator._radioIsConfigured));
			DBPrint("isConfiguringWireless : " + String(isConfiguringWireless));
		}
		else if (Rotator._radioIsConfigured) {
			XbeeStarted = true;
			wirelessBuffer = "";
			DBPrint("Radio configured");
			SendHello();
		}
	}

	Rotator.Run();
	CheckForCommands();
	PingShutter();
	CheckForRain();
	if(gotHelloFromShutter) {
		requestShutterData();
		gotHelloFromShutter = false;
	}

}
#pragma endregion

#pragma region Periodic and Helper functions

void StartWirelessConfig()
{
	Computer.println("Xbee configuration started");
	delay(1100); // guard time before and after
	isConfiguringWireless = true;
	DBPrint("Sending +++");
	Wireless.print("+++");
	delay(1100);
}

void ConfigXBee(String result)
{
	DBPrint("[ConfigXBee]");

	if (configStep == 0) {
		ATString = "ATCE1,ID7734,CH0C,MY0,DH0,DLFFFF,AP0,SM0,BD3,WR,CN";
		Wireless.println(ATString);
		DBPrint("Sending : " + ATString);
	}
	DBPrint("Result " + String(configStep) + ":" + result);

	if (configStep > 9) {
		isConfiguringWireless = false;
		Rotator._radioIsConfigured = true;
		XbeeStarted = true;
		Rotator.SaveToEEProm();
		delay(10000);
		Computer.print("Xbee configuration finished");
		while(Wireless.available() > 0) {
			Wireless.read();
		}
	}
	configStep++;
}

// <SUMMARY>Broadcast that you exist</SUMMARY>
void SendHello()
{
	DBPrint("Sending hello");
	Wireless.print(String(HELLO_CMD) + "#");
	ReceiveWireless();
	SentHello = true;
}

void requestShutterData()
{
		Wireless.print(String(STATE_SHUTTER_GET) + "#");
		ReceiveWireless();
		stepper.run(); // we don't want the stepper to stop

		Wireless.print(String(VERSION_SHUTTER_GET) + "#");
		ReceiveWireless();
		stepper.run(); // we don't want the stepper to stop

		Wireless.print(String(REVERSED_SHUTTER_CMD) + "#");
		ReceiveWireless();
		stepper.run(); // we don't want the stepper to stop

		Wireless.print(String(STEPSPER_SHUTTER_CMD) + "#");
		ReceiveWireless();
		stepper.run(); // we don't want the stepper to stop

		Wireless.print(String(SPEED_SHUTTER_CMD) + "#");
		ReceiveWireless();
		stepper.run(); // we don't want the stepper to stop

		Wireless.print(String(ACCELERATION_SHUTTER_CMD) + "#");
		ReceiveWireless();
		stepper.run(); // we don't want the stepper to stop

		Wireless.print(String(POSITION_SHUTTER_GET) + "#");
		ReceiveWireless();
		stepper.run(); // we don't want the stepper to stop

		Wireless.print(String(VOLTS_SHUTTER_CMD) + "#");
		ReceiveWireless();
		stepper.run(); // we don't want the stepper to stop

		Wireless.print(String(VOLTSCLOSE_SHUTTER_CMD) + "#");
		ReceiveWireless();
		stepper.run(); // we don't want the stepper to stop
}

//<SUMMARY>Check for Serial and Wireless data</SUMMARY>
void CheckForCommands()
{
	if (Computer.available() > 0) {
		ReceiveComputer();
	}

	if (Wireless.available() > 0) {
		ReceiveWireless();
	}
}

//<SUMMARY>Tells shutter the rain sensor status</SUMMARY>
void CheckForRain()
{
	// Only check periodically (fast reads seem to mess it up)
	// Disable by setting rain check interval to 0;
	if(Rotator.GetRainCheckInterval() == 0)
		return;
	if(Rainchecktimer.elapsed() >= (Rotator.GetRainCheckInterval() * 1000) ) {
		bIsRaining = Rotator.GetRainStatus();
		// send value to shutter
		if(bShutterPresnt) {
			Wireless.print(String(RAIN_SHUTTER_GET) + String(bIsRaining ? "1" : "0") + "#");
			ReceiveWireless();
		}
		if (bIsRaining) {
			if (Rotator.GetRainAction() == 1)
				Rotator.SetAzimuth(Rotator.GetHomeAzimuth());

			if (Rotator.GetRainAction() == 2)
				Rotator.SetAzimuth(Rotator.GetParkAzimuth());
		}
		Rainchecktimer.reset();
	}
}


void PingShutter()
{
	if(PingTimer.elapsed() >= pingInterval) {
		Wireless.print(String(SHUTTER_PING )+ "#");
		ReceiveWireless();
		PingTimer.reset();
		}
}

#pragma endregion

#pragma region Serial handling
// All ASCOM comms are terminated with # but left if the \r\n for XBee config
// with other programs.
void ReceiveComputer()
{
	char computerCharacter = Computer.read();
	if (computerCharacter != ERR_NO_DATA) {
		if (computerCharacter == '\r' || computerCharacter == '\n' || computerCharacter == '#') {
			// End of message
			if (computerBuffer.length() > 0) {
				ProcessSerialCommand();
				computerBuffer = "";
			}
		}
		else {
			computerBuffer += String(computerCharacter);
		}
	}
}

void ProcessSerialCommand()
{
	float localFloat;
	char command; //, localChar;
	String value, wirelessMessage;
	unsigned long localULong;
	String serialMessage, localString;
	bool hasValue = false; //, localBool = false;
	long localLong;

	// Split the buffer into command char and value
	// Command character
	command = computerBuffer.charAt(0);
	// Payload
	value = computerBuffer.substring(1);
	// payload has data (better one comparison here than many in code. Even though
	// it's still executed just once per loop.
	if (value.length() > 0)
		hasValue = true;

	serialMessage = "";
	wirelessMessage = "";

	DBPrint("ProcessSerialCommand");
	DBPrint("Command = \"" + String(command) +"\"");
	DBPrint("Value = \"" + String(value) +"\"");


	// Grouped by Rotator and Shutter then put in alphabetical order
	switch (command) {
#pragma region Rotator commands
		case ABORT_MOVE_CMD:
			localString = String(ABORT_MOVE_CMD);
			serialMessage = localString;
			Rotator.Stop();
			wirelessMessage = localString;
			Wireless.print(wirelessMessage + "#");
			ReceiveWireless();
			break;

		case ACCELERATION_ROTATOR_CMD:
			if (hasValue) {
				Rotator.SetAcceleration(value.toInt());
			}
			serialMessage = String(ACCELERATION_ROTATOR_CMD) + String(Rotator.GetAcceleration());
			break;

		case CALIBRATE_ROTATOR_CMD:
			Rotator.StartCalibrating();
			serialMessage = String(CALIBRATE_ROTATOR_CMD);
			break;

		case ERROR_AZ_ROTATOR_GET:
			// todo: See if azimuth error is needed (when passing home switch check to see if the
			// actual position matches where the stepper thinks it is.
			serialMessage = String(ERROR_AZ_ROTATOR_GET) + "0";
			break;

		case GOTO_ROTATOR_CMD:
			if (hasValue) {
				localFloat = value.toFloat();
				if ((localFloat >= 0.0) && (localFloat <= 360.0)) {
					Rotator.SetAzimuth(localFloat);
				}
			}
			serialMessage = String(GOTO_ROTATOR_CMD) + String(Rotator.GetAzimuth());
			break;

		case HELLO_CMD:
			SendHello();
			serialMessage = String(HELLO_CMD);
			break;

		case HOME_ROTATOR_CMD:
			Rotator.StartHoming();
			serialMessage = String(HOME_ROTATOR_CMD);
			break;

		case HOMEAZ_ROTATOR_CMD:
			if (hasValue) {
				localFloat = value.toFloat();
				if ((localFloat >= 0) && (localFloat < 360))
					Rotator.SetHomeAzimuth(localFloat);
			}
			serialMessage = String(HOMEAZ_ROTATOR_CMD) + String(Rotator.GetHomeAzimuth());
			break;

		case HOMESTATUS_ROTATOR_GET:
			serialMessage = String(HOMESTATUS_ROTATOR_GET) + String(Rotator.GetHomeStatus());
			break;

		case MOVE_RELATIVE_ROTATOR_CMD:
			if (hasValue) {
				if (!Rotator.GetVoltsAreLow()) {
					localLong = value.toInt();
					Rotator.MoveRelative(localLong);
				}
				else {
					serialMessage = String(MOVE_RELATIVE_ROTATOR_CMD) + "L";
				}
			}
			break;

		case PARKAZ_ROTATOR_CMD:
			// Get/Set Park Azumith
			localString = String(PARKAZ_ROTATOR_CMD);
			if (hasValue) {
				localFloat = value.toFloat();
				if ((localFloat >= 0) && (localFloat < 360)) {
					Rotator.SetParkAzimuth(localFloat);
					serialMessage = localString + String(Rotator.GetParkAzimuth());
				}
				else {
					serialMessage = localString + "E";
				}
			}
			else {
				serialMessage = localString + String(Rotator.GetParkAzimuth());
			}
			break;

		case POSITION_ROTATOR_CMD:
			if (hasValue)
			{
				if (!Rotator.GetVoltsAreLow()) {
					Rotator.SetPosition(value.toInt());
					serialMessage = String(POSITION_ROTATOR_CMD) + String(Rotator.GetPosition());
				}
				else {
					serialMessage = String(POSITION_ROTATOR_CMD) + "L";
				}
			}
			else {
				serialMessage = String(POSITION_ROTATOR_CMD) + String(Rotator.GetPosition());
			}
			break;

		case RAIN_ROTATOR_ACTION:
			if (hasValue) {
				Rotator.SetRainAction(value.toInt());
			}
			serialMessage = String(RAIN_ROTATOR_ACTION) + String(Rotator.GetRainAction());
			break;

		case RAIN_ROTATOR_TWICE_CMD:
			if (hasValue) {
				Rotator.SetCheckRainTwice(value.equals("1"));
			}
			serialMessage = String(RAIN_ROTATOR_TWICE_CMD) + String(Rotator.GetRainCheckTwice());
			break;

		case RAIN_ROTATOR_CMD:
			if (hasValue) {
				localULong = (unsigned long)value.toInt();
				Rotator.SetRainInterval(localULong);
			}
			serialMessage = String(RAIN_ROTATOR_CMD) + String(Rotator.GetRainCheckInterval());
			break;

		case SPEED_ROTATOR_CMD:
			if (hasValue)
				Rotator.SetMaxSpeed(value.toInt());
			serialMessage = String(SPEED_ROTATOR_CMD) + String(Rotator.GetMaxSpeed());
			break;

		case REVERSED_ROTATOR_CMD:
			if (hasValue)
				Rotator.SetReversed(value.toInt());
			serialMessage = String(REVERSED_ROTATOR_CMD) + String(Rotator.GetReversed());
			break;

		case SEEKSTATE_GET:
			serialMessage = String(SEEKSTATE_GET) + String(Rotator.GetSeekMode());
			break;

		case SLEW_ROTATOR_GET:
			serialMessage = String(SLEW_ROTATOR_GET) + String(Rotator.GetDirection());
			break;

		case STEPSPER_ROTATOR_CMD:
			if (hasValue)
				Rotator.SetStepsPerRotation(value.toInt());
			serialMessage = String(STEPSPER_ROTATOR_CMD) + String(Rotator.GetStepsPerRotation());
			break;

		case SYNC_ROTATOR_CMD:
			if (hasValue) {
				localFloat = value.toFloat();
				if (localFloat >= 0 && localFloat < 360) {
					Rotator.SyncHome(localFloat);
					Rotator.SyncPosition(localFloat);
					serialMessage = String(SYNC_ROTATOR_CMD) + String(Rotator.GetPosition());
				}
			}
			else {
					serialMessage = String(SYNC_ROTATOR_CMD) + "E";
			}
			break;

		case VERSION_ROTATOR_GET:
			serialMessage = String(VERSION_ROTATOR_GET) + VERSION;
			break;

		case VOLTS_ROTATOR_CMD:
			// value only needs infrequent updating.
			if (hasValue) {
				Rotator.SetLowVoltageCutoff(value.toInt());
			}
			serialMessage = String(VOLTS_ROTATOR_CMD) + String(Rotator.GetVoltString());
			break;

		case RAIN_SHUTTER_GET:
			serialMessage = String(RAIN_SHUTTER_GET) + String(bIsRaining ? "1" : "0");
			break;

		case INIT_XBEE:
			localString = String(INIT_XBEE);
			Rotator._radioIsConfigured = false;
			isConfiguringWireless = false;
			XbeeStarted = false;
			configStep = 0;
			serialMessage = localString;
			Wireless.print(localString + "#");
			ReceiveWireless();
			DBPrint("trying to reconfigure radio");
			break;

	#pragma endregion

	#pragma region Shutter Commands
		case ACCELERATION_SHUTTER_CMD:
			localString = String(ACCELERATION_SHUTTER_CMD);
			if (hasValue) {
				RemoteShutter.acceleration = value;
				wirelessMessage = localString + RemoteShutter.acceleration;
			}
			else {
				wirelessMessage = localString;
			}
			Wireless.print(wirelessMessage + "#");
			ReceiveWireless();
			serialMessage = localString + RemoteShutter.acceleration;
			break;

		case CLOSE_SHUTTER_CMD:
			localString = String(CLOSE_SHUTTER_CMD);
			Wireless.print(localString+ "#");
			ReceiveWireless();
			serialMessage = localString;
			break;

		//case ELEVATION_SHUTTER_CMD:
		//		localString = String(ELEVATION_SHUTTER_CMD);
		//		if (hasValue)
		//			wirelessMessage = localString + value;
		//		serialMessage = localString + RemoteShutter.elevation;
		//		break;

		case HOMESTATUS_SHUTTER_GET: // TODO: Figure this out if it's necessary
				// todo: Create shutter calibration and get that status here
				localString = String(HOMESTATUS_SHUTTER_GET);
				Wireless.print(localString + "#");
				ReceiveWireless();
				serialMessage = localString + String(RemoteShutter.homedStatus);
				break;

		case OPEN_SHUTTER_CMD:
				localString = String(OPEN_SHUTTER_CMD);
				Wireless.print(localString + "#");
				ReceiveWireless();
				serialMessage = localString;
				break;

		case POSITION_SHUTTER_GET:
				localString = String(POSITION_SHUTTER_GET);
				Wireless.print(localString + "#");
				ReceiveWireless();
				serialMessage = localString + String(RemoteShutter.position);
				break;

		case REVERSED_SHUTTER_CMD:
			localString = String(REVERSED_SHUTTER_CMD);
			if (hasValue) {
				RemoteShutter.reversed = value;
				wirelessMessage = localString + value;
			}
			else {
				wirelessMessage = localString;
			}
			Wireless.print(wirelessMessage + "#");
			ReceiveWireless();
			serialMessage = localString + RemoteShutter.reversed;
			break;

		case SLEEP_SHUTTER_CMD:
			localString = String(SLEEP_SHUTTER_CMD);
			if (hasValue) {
				RemoteShutter.sleepSettings = value;
				wirelessMessage = localString + value;
			}
			else {
				wirelessMessage = localString;
			}
			Wireless.print(wirelessMessage + "#");
			ReceiveWireless();
			serialMessage = localString + RemoteShutter.sleepSettings;
			break;

		case SPEED_SHUTTER_CMD:
			localString = String(SPEED_SHUTTER_CMD);
			if (hasValue) {
				RemoteShutter.speed = value;
				wirelessMessage = localString + String(value.toInt());
			}
			else {
				wirelessMessage = localString;
			}
			Wireless.print(wirelessMessage + "#");
			ReceiveWireless();
			serialMessage = localString + RemoteShutter.speed;
			break;

		case STATE_SHUTTER_GET:
			localString = String(STATE_SHUTTER_GET);
			Wireless.print(localString + "#");
			ReceiveWireless();
			serialMessage = localString + RemoteShutter.state;
			break;

		case STEPSPER_SHUTTER_CMD:
			localString = String(STEPSPER_SHUTTER_CMD);
			if (hasValue) {
				RemoteShutter.stepsPerStroke = value;
				wirelessMessage = localString + value;
			}
			else {
				wirelessMessage = localString;
			}
			Wireless.print(wirelessMessage + "#");
			ReceiveWireless();
			serialMessage = localString + RemoteShutter.stepsPerStroke;
			break;

		case VERSION_SHUTTER_GET:
			// Rotator gets this upon Hello and it's not going to change so don't ask for it wirelessly
			localString = String(VERSION_SHUTTER_GET);
			Wireless.print(localString + "#");
			ReceiveWireless();
			serialMessage = localString + RemoteShutter.version;
			break;

		case VOLTS_SHUTTER_CMD:
			localString = String(VOLTS_SHUTTER_CMD);
			wirelessMessage = localString;
			if (hasValue)
				wirelessMessage += String(value);

			Wireless.print(wirelessMessage + "#");
			ReceiveWireless();
			serialMessage = localString + RemoteShutter.volts;
			break;

		case VOLTSCLOSE_SHUTTER_CMD:
			localString = String(VOLTSCLOSE_SHUTTER_CMD);
			if (value.length() > 0) {
				RemoteShutter.voltsClose = value;
				wirelessMessage = localString+ value;
			}
			else {
				wirelessMessage = localString;
			}
			Wireless.print(wirelessMessage + "#");
			ReceiveWireless();
			serialMessage = localString + RemoteShutter.voltsClose;
			break;

		case WATCHDOG_INTERVAL_SET:
			localString = String(WATCHDOG_INTERVAL_SET);
			if (value.length() > 0) {
				wirelessMessage = localString + value;
			}
			else {
				wirelessMessage = localString;
			}
			Wireless.print(wirelessMessage + "#");
			ReceiveWireless();
			serialMessage = localString + RemoteShutter.watchdogInterval;
			break;

		case DEBUG_MSG_CMD:
			break;

	#pragma endregion

		default:
			serialMessage = "Unknown command:" + String(command);
			break;
	}



	// Send messages if they aren't empty.
	if (serialMessage.length() > 0) {
		Computer.print(serialMessage + "#");
	}
}

#pragma endregion

#pragma region Wireless Communications

#define MAX_TIMEOUT 10
#define ERR_NO_DATA	-1
#define OK	0

int ReceiveWireless()
{
	int timeout = 0;
	char wirelessCharacter;

	wirelessBuffer = "";
	if (isConfiguringWireless) {
		DBPrint("[ReceiveWireless] isConfiguringWireless : " + String(isConfiguringWireless));
		// read the response
		do {
			wirelessCharacter = Wireless.read();
			if(wirelessCharacter != '\r' && wirelessCharacter != ERR_NO_DATA) {
				wirelessBuffer += String(wirelessCharacter);
			}
		} while (wirelessCharacter != '\r');

		DBPrint("[ReceiveWireless] wirelessBuffer = " + wirelessBuffer);

		ConfigXBee(wirelessBuffer);
		return;
	}

	// wait for response, timeout after MAX_TIMEOUT times 10ms
	while(Wireless.available() == 0) {
		delay(10);
		timeout++;
		if(timeout >= MAX_TIMEOUT) {
			return ERR_NO_DATA;
			}
	}

	// read the response
	do {
		wirelessCharacter = Wireless.read();
		if(wirelessCharacter != ERR_NO_DATA && wirelessCharacter != '#') {
			wirelessBuffer += String(wirelessCharacter);
		}
	} while (wirelessCharacter != '#');

	if (wirelessBuffer.length() > 0) {
		ProcessWireless();
	}
	return OK;
}

void ProcessWireless()
{
	char command;
	String value, wirelessMessage;

	DBPrint("<<< Received: " + wirelessBuffer);
	command = wirelessBuffer.charAt(0);
	value = wirelessBuffer.substring(1);

	wirelessMessage = "";

	switch (command) {
		case ACCELERATION_SHUTTER_CMD:
			RemoteShutter.acceleration = value;
			break;

		case HELLO_CMD:
			gotHelloFromShutter = true;
			bShutterPresnt = true;
			break;

		case POSITION_SHUTTER_GET:
			RemoteShutter.position = value;
			break;

		case SPEED_SHUTTER_CMD:
			RemoteShutter.speed = value;
			break;

		case RAIN_SHUTTER_GET:
			break;

		case REVERSED_SHUTTER_CMD:
			RemoteShutter.reversed = value;
			break;

		case SLEEP_SHUTTER_CMD: // Sleep settings mode,period,delay
			RemoteShutter.sleepSettings = value;
			break;

		case STATE_SHUTTER_GET: // Dome status
			RemoteShutter.state = value;
			break;

		case STEPSPER_SHUTTER_CMD:
			RemoteShutter.stepsPerStroke = value;
			break;

		case VERSION_SHUTTER_GET:
			RemoteShutter.version = value;
			break;

		case VOLTS_SHUTTER_CMD: // Sending battery voltage and cutoff
			RemoteShutter.volts = value;
			break;

		case VOLTSCLOSE_SHUTTER_CMD:
			RemoteShutter.voltsClose = value;
			break;

		case WATCHDOG_INTERVAL_SET:
			RemoteShutter.watchdogInterval = value;
			break;

		case SHUTTER_PING:
			bShutterPresnt = true;
			break;

		default:
			break;
	}

	if (wirelessMessage.length() > 0) {
		DBPrint(">>> Sending " + wirelessMessage);
		Wireless.print(wirelessMessage + "#");
	}
}
#pragma endregion



