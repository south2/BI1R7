/* body interaction 1 script "p5"
   hardware: body interaction 1 board
   www.bodyinteraction.com
   info@bodyinteraction.com

   The body interaction 1 board is based on the Jeenode Micro that uses for the communication the Hope RFM12b. In the context
   of wireless communication each body interaction board is a node.
   Motion is measured with the accelerometer BMA020. A vibration motor is driven with a simple  with transitor and diode.

   The script reads data from the accelorometer. Depending of the measured motion a vibration motor is speed up or slowed
   down. (Slow motion = reduce viration motor speed, fast motion = speed up). When the measured motion changes the motion data are send out and can be received by other BI1. In addition you can
   use a jeenode to receive the data and send them to a PC for further processing.
   At the same time this script listen to other body interaction boards which send their motion data
   to all listening nodes. When the measured motion of other body interaction board are different from the motion
   measured by this script, the vibration motor is adjusted (speed up or slowed down). So two or more body interaction boards
   can influence each other and syncronize after some time.

   There are 3 states (modes): active, pause, sleep

   active: In the active state the body interaction board behave like described above. When there is no motion and other boards don't
   send motion data the vibration motor is set to low vibration. After some time the state changes to pause.

   pause: In the pause state the vibration motor is off. But the body interaction board still listen to other body interaction board.
   If it receives motion data from an other board or wehn to motion of the borad itself changes the state is changed to active.
   If nothing happens the state is changes to sleeping after some time.

   sleeping: The body interaction board is set to sleep modus and saves battery power. After some tiem the board wakes up
   and read out the motion data. If the board is moved the state is changed to active.

   This code is in the public domain.
  */
#include <JeeLib.h> // Jeelab Library
#include <avr/sleep.h> // sleep module
ISR(WDT_vect)
{
	Sleepy::watchdogEvent();
}

// states
const int active = 2;
const int pause = 1;
const int sleeping = 0;
int state = active; // current state

const int myId = 3; //Id number of this node
const int myNetwork = 20; // number of the network, only nodes in the same network can communicate with one another

// constants are compared with calcDiff()
const int sensorNoise = 20;
const int diffLowSpeed = 35; //v1: 70
const int maxMotorSpeed = 255; //255
const int decreaseStep = 20; //v1: 20
const int wakeUpDiff = 20; //wake up from sleep state
const int motorPin = 3;  // pin number for driving the node with PWM

const int sendMax = 8;
const int sendNormal = 3;

PortI2C myBus (1); // the accelerometer is connected with I2C port number 1 (ports are a convention in the JeeLib)

GravityPlug sensor (myBus); // the accelerometer is named as "sensor"

// Jeelib timer
MilliTimer sendTimer; // sending data is allowed when timer fires
MilliTimer readGTimer; // when timer fires gravity (motion) data should be read
MilliTimer pauseTimer; // when timer fires BI1 board can go into state pause
MilliTimer sleepTimer; // when timer fires BI1 board can go into stat sleep

int payload[] = {diffLowSpeed, state}; // payload is an array send to other BI1

//global variables
int p1[3] = {0, 0, 0}; // Motion data and time=1 in (x,y,z) direction
int p0[3] = {0, 0, 0}; // Motion data and time=0 in (x,y,z) direction
int d[3] = {0, 0, 0}; // differnce between time=1 and time=0 in (x,y,z) direction

// character to integer casting
int c2i(char c)
{
	return (((int)c) + 127);
}
// integer to character casting
char i2c (int i)
{
	return ((char)i - 127);
}

// heuristic function for calculating the motion difference between time=1 and time=0
// returns the maximum difference from each (x,y,z) direction
int calcDiff()
{
	int diff;
	d[0] = abs(p1[0] - p0[0]);
	d[1] = abs(p1[1] - p0[1]);
	d[2] = abs(p1[2] - p0[2]);
	diff = max(d[0], d[1]);
	diff = max(diff, d[2]);
	return diff;
}

// read out the acceleromter data
// function return the difference based on calcdiff()
int gravityRead ()
{
	const int* p = sensor.getAxes();
	int difference;
	p0[0] = p1[0]; // store old data in p0[] (time = 0)
	p0[1] = p1[1];
	p0[2] = p1[2];
	p = sensor.getAxes(); // read out sensor
	p1[0] = p[0]; // store new data in p1[] (time = 1)
	p1[1] = p[1];
	p1[2] = p[2];
	difference = calcDiff();
	return difference;
}


boolean shouldSend()   // interval between data sending
{
	sendTimer.poll(100); // 100 millisecond = 0.1 second (1 second = 1000 milliseconds)
}
boolean shouldReadG()   // intervall between motion measurement
{
	readGTimer.poll(300); // approx 0.3 seconds
}
void resetReadGTimer()   // reset timer
{
	readGTimer.set(0);
}
boolean shouldPause()  // interval between last measured motion change and changing state to pause (motor off)
{
	pauseTimer.poll();
}
boolean armPauseTimer()  // init timer
{
	pauseTimer.set(1000 * 5 * 2); // aprox 10 seconds
}
boolean shouldSleep()   // interval between entering pause state and changing to sleep state
{
	sleepTimer.poll();
}
void armSleepTimer()   // init timer
{
	sleepTimer.set(1000 * 5 * 4); // approx 10 seconds
}



int diff_t0; // last measured difference in motion
int diff; // variable for current difference in motion

int repeatSendN; // repeat data send n times
boolean received; // boolean variable true when data received

/* in this loop the following tasks are done:
   1. measure motion data
   2. change vibration motor speed according to motion data
   3. change states
   4. listen to the receiver
   5. change vibration motor speed if motion data from othe body interaction 1 (BI1) are received
*/

void loop()
{
	if (shouldReadG())
	{
		diff = gravityRead(); //measure motion data
		if (state == pause && diff < diffLowSpeed) diff = 0; // set variable diff to 0 when in pause state
		else   // motion detected: calculated diff, smoothing
		{
			if (diff + decreaseStep < diff_t0)
			{
				diff = max(diff_t0 - decreaseStep, diffLowSpeed);    // motion reduced
				state = active;
			}
			else diff = max((diff + diff_t0) / 2, diffLowSpeed);
			state = active; // motion increased
		}

		if (diff > diff_t0 + sensorNoise)
		{
			armPauseTimer();    // motion increased: reset pause timer
			repeatSendN = sendMax;
		}

		if (diff == diffLowSpeed && shouldPause())
		{
			state = pause;    // change to pause state
			diff = 0;
			armSleepTimer();
		}

		diff = min(maxMotorSpeed, diff); // measured motion (diff) becomes vibration motor speed, motor speed not about maximum

		analogWrite(motorPin, diff);  // set vibration motor speed; use variable diff

		payload[0] = i2c(diff); // prepare payload for sending, put diff in the first byte
	} //(shouldReadG())

	// sleep mode: stay in sleep state until motion is measured
	if (state == pause && shouldSleep()) // go into sleep state when in pause state for some time
	{
		delay(100);
		rf12_sleep(RF12_SLEEP); // set RFM12 module asleep and save battery
		delay(100);
		diff = gravityRead(); // check motion
		while (diff < wakeUpDiff) // if motion is less than treshold: fall asleep for approx 30 seconds
		{
			delay(100);
			Sleepy::loseSomeTime(30000); // now fall asleep
			delay(100); // needs some time to wake up
			diff = gravityRead(); //measure motion again
		}
		delay(100); // end of sleep state
		state = active; // go to active state
		armPauseTimer(); // reset timer
		resetReadGTimer();
		diff = diffLowSpeed; // set diff to "öow speed" - vibration motor will start to say hello
		diff_t0 = diffLowSpeed;
	}

	diff_t0 = diff; // go to next time cycle; current diff becomes old diff (time=0);

	// sending data
	if (shouldSend() && (repeatSendN > 0)) // check if data have to be send
	{
		if (rf12_canSend())   // check if no other radio is sending
		{
			rf12_sendStart(0, payload, sizeof payload); // send
			repeatSendN--; //  number of times to repeat sending is reduced by 1
		}
	}

	// listen to other body interaction 1 boards
	received = rf12_recvDone() && rf12_crc == 0; // call important rf12_recvDone() function, returns true if data are received
	if (received)
	{
		diff = c2i(rf12_data[0]); // received motion data are used to drive vibration motor
		// you could add code for smoothing the new motion data
		if (diff > -1)   // ignore -1 (measurement or transmission error)
		{
			diff = min(maxMotorSpeed, diff); // set motor speed to variable diff
			diff_t0 = diff;
			if (diff >= diffLowSpeed) analogWrite(motorPin, diff);
		}
	}
}


void setup()
{
	//Serial.begin(115200);
	//Serial.println("\n[body interaction 1 script ""p1""]");
	rf12_initialize(myId, RF12_868MHZ, myNetwork); // init RFM12 module; for some countries (eg. US) use RF12_915MHZ
	armPauseTimer();
	repeatSendN = sendNormal;
}
