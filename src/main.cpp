#include <Arduino.h>
#include <FastAccelStepper.h>

//User-specific setup data
#define dirPinStepper 12 //Number of DIR+ pin on ESP32
#define enablePinStepper 13 //Number of ENA+ pin on ESP32
#define stepPinStepper 14 //Number of PUL+ pin on ESP32
const int stepsPerRev = 6400; //CHOOSE VALUE Microstepper setting of pulse/rev

//Variables to control static and cyclic motion flow
int movementsTraveled = 0; //The number of movements traveled during static motion
double cyclesTraveled = 0.0; //The number of cycles traveled during cyclic motion (always a multiple of 1 or 0.5)
bool movingForward = true; //The direction of movement (true = clockwise, false = counterclockwise)

//An enum to control the possible states of void loop()
enum State {
  IDLE,
  STATIC_MOTION,
  CYCLIC_MOTION
};
State currentState = IDLE; //Initializing the state of motion to IDLE

//The variables to be assigned during user input
char* mode = nullptr; //The mode of motion, assigned during user input
char* userIdentifier = nullptr; //A user identifier, determined during input to help post-processing
char* dataCollection = nullptr; //A string representing readability of data and hz. Looks something like H100 or C20. Refer to dataRate and readability for further explanation
int dataRate; //dataRate is the rate to collect data in Hz.
char readability; //Extracted from dataCollection, either H or C where H = human readability, C = computer readability,
double startPos, endPos, maxSpeed, acceleration; //The start position, end position, max speed, and acceleration to be assigned during user input
int special; //Special is the miscellaneous variable which varies based on mode. For static motion it is the number of individual movements. For cyclic motion it is the number of cycles.
bool inputGiven = false; //Determining if input has been given, used to control the flow of input. False upon initialization because no input initially
//rename to waitForInput??

//Initializing the FastAccelStepper engine
FastAccelStepperEngine engine = FastAccelStepperEngine(); 
FastAccelStepper *stepper = NULL;




//A helper method to acquire stepper position data in degrees based on the data acquisition rate (top of code)
void acquireData() {

  //Determining the computer-generated identifier
  String compIdentifier;
  if (String(mode) == "STATIC") {
    compIdentifier = "M" + String(movementsTraveled);
  } else if (String(mode) == "CYCLIC") {
    compIdentifier = "C" + String(cyclesTraveled);
  }

  static int previousMillis = 0;
  int currentMillis = millis();

  //Check if enough time has elapsed since the last iteration
  if (currentMillis - previousMillis >= 1000 / dataRate) {
    //If dataRate has been elapsed (the data collection rate), then set previous time to current time and print position data
    previousMillis = currentMillis;
    //If human readable data is desired, print this specific output
    if (readability == 'H') { 
      Serial.println("Mode: " + String(mode) + ", Current Position: " + String(stepper->getCurrentPosition() / (double) stepsPerRev * 360) + " degrees");

      //If computer readable data is desired, the print this specific output
    } else if (readability == 'C') {
      Serial.println("<" + String(int(stepper->getCurrentPosition() / (double) stepsPerRev * 36000)) + //Position in degrees * 100
                     "," + String(!(stepper->getCurrentSpeedInMilliHz() == 0)) + //Boolean on whether the stepper is running
                     "," + String(mode) + //The mode of motion
                     "," + String(userIdentifier) + "_" + String(compIdentifier) + //The user-specified identifier and computer generated identifier
                     ">");
    }
  }
}



//A helper method to parse the input and check if its format is acceptable
bool parseInput(String input) {
  int result = sscanf(input.c_str(), "%m[^,],%lf,%lf,%lf,%lf,%d,%m[^,],%m[^,]", &mode, &startPos, &endPos, &maxSpeed, &acceleration, &special, &userIdentifier, &dataCollection);

  //Check if sscanf successfully parsed all required values
  if (result == 8) {
    sscanf(dataCollection, "%c%d", &readability, &dataRate);
    return true;
  } 
  return false;
}



//A non-blocking method to perform a static sweep based on the user input.
//Returns a boolean depicting whether or not the static sweep is complete
bool staticMotion(double startPos, double endPos, int movements) {

  //Calculating the partitioned distance to move
  double splitDistance = (endPos - startPos) / movements;
  
  //First checking if the stepper is at the correct starting position. Will only ever run ONCE for a given input
  //If the stepper is still AND it has not begun the static motion AND it is not already at startPos then move it to startPos
  if (!stepper->isRunning() && (movementsTraveled == 0) && (startPos != stepper->getCurrentPosition() / (double) stepsPerRev * 360)) {
      Serial.println("Moving to START_POS...");
      stepper->move(round(startPos / 360.0 * stepsPerRev - stepper->getCurrentPosition()));
  }

  //Invoke the data acquisition method which obtains the stepper's position at a desired rate
  acquireData();

  //If the stepper is not moving and has not completed the number of desired movements
  if (!stepper->isRunning() && movementsTraveled < movements) {

    //Move to the next position
    stepper->move(round(splitDistance / 360 * stepsPerRev));

    //Increase movementsTraveled as we have completed a single partition of the sweep
    movementsTraveled++;
  }

  //Return a boolean on if the stepper is running and if it has completed the desired movements
  return !stepper->isRunning() && movementsTraveled >= movements;
}



//A non-blocking method to perform cyclic motion based on the user input. A single cycle moves from startPos to endPos and back to startPos.
//Returns a boolean depicting whether or not all cycles are complete.
bool cyclicMotion(double startPos, double endPos, int cycles) {

  //Workaround for infinite cycles (which is when cycles input is -1)
  //If the user desires infinite cycles, then keep setting cycles to one more than cyclesTraveled, so this method always returns false and runs indefinitely
  if (cycles == -1) {
    cycles = cyclesTraveled + 1;
  } 

  //Calculating the distance to move (2 * amplitude of cycle)
  double distance = endPos - startPos;

  //First checking if the stepper is at the correct starting position. Will only ever run ONCE for a given input
  //If the stepper is still AND it has not begun the cyclic motion AND it is not already at startPos then move it to startPos
  if (!stepper->isRunning() && (cyclesTraveled == 0.0) && (startPos != stepper->getCurrentPosition() / (double) stepsPerRev * 360)) {
      Serial.println("Moving to START_POS...");
      stepper->move(round(startPos / 360.0 * stepsPerRev - stepper->getCurrentPosition()));
  }

  //Invoke the data acquisition method which obtains the stepper's position at a desired rate
  acquireData();

  //Check if the stepper motor is still and if not all cycles have been completed
  if (!stepper->isRunning() && cyclesTraveled < cycles) {

    //Move to the next position
    if (movingForward) {
      //Moving the stepper, changing the direction of motion for the next movement, and adding 0.5 to cyclesTraveled
      stepper->move(round(distance / 360 * stepsPerRev));
      movingForward = !movingForward;
      cyclesTraveled += 0.5;
    } else {
      //Moving the stepper, changing the direction of motion for the next movement, and adding 0.5 to cyclesTraveled
      stepper->move(-round(distance / 360 * stepsPerRev));
      movingForward = !movingForward;
      cyclesTraveled += 0.5;
    }
  }


  //Return a boolean on if the stepper is running and if it has completed the desired cycles
  return !stepper->isRunning() && cyclesTraveled >= cycles;
}







void setup() {
  Serial.begin(115200);

  //Initialize the FastAccelStepper engine
  engine.init();

  //Giving the pin data to the engine
  stepper = engine.stepperConnectToPin(stepPinStepper);
  stepper->setDirectionPin(dirPinStepper);
  stepper->setEnablePin(enablePinStepper);
  stepper->setAutoEnable(true);
}


//The main loop() method is a state machine which controls the flow of code and is designed to allow for easy addition of future motional modes.
//Also note that all methods are written to specifically ensure that nothing blocks the flow of code (no while/for loops). This allows for force stops.
void loop() {

  switch (currentState) {

    //The default state is IDLE, which asks the user for input. The program stays in the IDLE state until a user input is given
    case IDLE:

      //If there is no input given, then prompt the user.
      if (!inputGiven) {
        Serial.println("ENTER FORMATTED COMMAND: ");
        inputGiven = true;
      }

      //If there is an input given, then we parse the input and assign variables accordingly
      if (Serial.available() > 0) {

        //Utilize sscanf to parse data. NO INPUT FORMAT OR EDGE CASE CHECKING. INPUT MUST BE GIVEN IN THE DOCUMENTED FORMAT OR ELSE IT WILL BREAK THE CODE
        String input = Serial.readStringUntil('\n');

        //If the input is able to be parsed (formatted correctly), then we continue to prepare movement
        if (parseInput(input)) {

          //Since we have dynamically allocated char arrays, ensure to free up unused space
          free(mode); 
          free(userIdentifier);
          free(dataCollection);
          
          //Setting the max speed and acceleration of the motor based on the respective inputs. Converting from degrees to steps
          stepper->setSpeedInHz(maxSpeed / 360.0 * stepsPerRev);
          stepper->setAcceleration(acceleration / 360.0 * stepsPerRev);

          //Set the appropriate state based on user input. Add another elseif statement if another mode is added
          if (String(mode) == "STATIC") {
            currentState = STATIC_MOTION;
          } else if (String(mode) == "CYCLIC") {
            currentState = CYCLIC_MOTION;
          }
          
          //Reset inputGiven to reprompt the user in the future
          inputGiven = false;
          
        } else { //If the input is not able to be parsed, then the state machine is unchanged and we print a message about the input
          Serial.println("Incorrect input format. Please check documentation for proper input: MODE,START_POS,END_POS,MAX_SPEED,ACCELERATION,SPECIAL,USER_ID,DATA_INFO");
          Serial.println("ENTER FORMATTED COMMAND: "); //inputGiven = false??
        }
      }
      break;
    

    //If the mode input is STATIC
    case STATIC_MOTION:

      //Run staticMotion and check if it has been completed
      if (!staticMotion(startPos, endPos, special)) {

        //Serial checker to see if the enter button has been pressed (force stop, return to IDLE state)
        if (Serial.available() > 0 && Serial.peek() == '\n') {
          //Consume the newline character, reset movementsTraveled, and send the state machine to IDLE
          Serial.read();
          Serial.println("FORCE STOPPED");
          movementsTraveled = 0;
          currentState = IDLE;
        } else {
          //If staticMotion is not complete and has not been force stopped, then keep the state machine in STATIC_MOTION.
          currentState = STATIC_MOTION;
        }

      } else {
        //If the static sweep is complete, reset the movementTraveled counter and send the state machine to IDLE
        acquireData(); //Final data log
        Serial.println("Static sweep complete.");
        movementsTraveled = 0;
        currentState = IDLE;
      }
      break;


    //If the mode input is CYCLIC
    case CYCLIC_MOTION:

      //Run cyclicMotion and check if it has been completed
      if (!cyclicMotion(startPos, endPos, special)) {

        //Serial checker to see if the enter button has been pressed (force stop, return to IDLE state)
        if (Serial.available() > 0 && Serial.read() == '\n') {
          //Reset cyclesTraveled and send the state machine to IDLE
          Serial.println("FORCE STOPPED");
          cyclesTraveled = 0;
          currentState = IDLE;
        } else {
          //If cyclicMotion is not complete and has not been force stopped, then keep the state machine CYCLIC_MOTION.
          currentState = CYCLIC_MOTION;
        }

      } else {
        //If cyclicMotion is complete, then reset the cyclesTraveled counter and send the state machine to IDLE
        acquireData(); //Final data log
        Serial.println("Cyclic motion complete.");
        cyclesTraveled = 0;
        currentState = IDLE;
      }
      break;

    //Add more states/modes of motion here if desired, always ensure to reset currentState to IDLE afterwards
  }
}