/*****************************************************************************************************************
Course: ROBT1270 - C Programming

Program: Lab5: SCARA Robot Simulator Intermediate Contol

Purpose: To demonstrate intermediate control over the SCARA Robot Simulator
         Programming methods: formatted I/O, conditional statements, pointers, functions, arrays, structures, 
         strings, dynamic arrays

Author: ????

Declaration: I/We, ??????, declare that the following program was written by me/us.

Date Created: ?????

*****************************************************************************************************************/

//-------------------------- Standard library prototypes ---------------------------------------------------------
#include <stdlib.h> // standard functions and constant
#include <stdio.h>  // i/o functions
#include <math.h>   // math functions
#include <string.h> // string functions
#include <ctype.h>  // character functions
#include "robot.h"  // robot functions

CRobot robot; // the global robot Class.  Can be used everywhere

//---------------------------- Program Constants -----------------------------------------------------------------
const double PI = 3.14159265358979323846;    // the one and only
const double L1 = 350.0;                     // length of the inner arm
const double L2 = 250.0;                     // length of the outer arm
const double ABS_THETA1_DEG_MAX = 150.0;     // maximum magnitude of shoulder angle in degrees
const double ABS_THETA2_DEG_MAX = 170.0;     // maximum magnitude of elbow angle in degrees
const double LMAX = L1 + L2;                 // max L -> maximum reach of robot
const double LMIN = sqrt(L1 * L1 + L2 * L2 - 2.0 * L1 * L2 * cos(PI - ABS_THETA2_DEG_MAX * PI / 180.0)); // min L

const double ERROR_VALUE = DBL_MAX;

const int LOW_RESOLUTION_POINTS_PER_500_UNITS = 3;
const int MEDIUM_RESOLUTION_POINTS_PER_500_UNITS = 11;
const int HIGH_RESOLUTION_POINTS_PER_500_UNITS = 21;


#define COMMAND_STRING_ARRAY_SIZE 502        // size of array to store commands for robot. NOTE: 2 elements must be
                                             // reserved for trailing '\n' and '\0' in command string

enum { LEFT, RIGHT };      // left arm or right arm configuration
enum { PEN_UP, PEN_DOWN }; // pen position
enum { MOTOR_SPEED_LOW, MOTOR_SPEED_MEDIUM, MOTOR_SPEED_HIGH }; // motor speed
enum { FORWARD_CALC_METHOD, INVERSE_CALC_METHOD }; // calculation method index

const unsigned char PLUSMINUS_SYMBOL = 241;  // the plus/minus ascii symbol
const unsigned char DEGREE_SYMBOL = 248;     // the degree symbol

const int PRECISION = 2;      // for printing values to console
const int FIELD_WIDTH = 8;    // for printing values to console

// RGB color
typedef struct RGB
{
   int r, g, b;  // ranges are 0-255
}
RGB;

const RGB RED = {255,0,0};
const RGB GREEN = {0,255,0};
const RGB BLUE = {0,0,255};
const RGB BLACK = {0,0,0};

//---------------------------- Structure Constants ---------------------------------------------------------------

// SCARA tooltip coordinates
typedef struct TOOL_POSITION
{
   double x, y;  
}
TOOL_POSITION;

// SCARA joint angles (degrees)
typedef struct JOINT_ANGLES
{
   double theta1Deg, theta2Deg;
}
JOINT_ANGLES;

// pen state
typedef struct PEN_STATE
{
   RGB penColor;
   int penPos;
}
PEN_STATE;

// robot state data
typedef struct ROBOT_STATE
{
   JOINT_ANGLES jointAngles;
   TOOL_POSITION toolPos;
   PEN_STATE pen;
   int motorSpeed;
}
ROBOT_STATE;

typedef struct INVERSE_SOLUTION
{
   JOINT_ANGLES jointAngles[2];  // joint angles (in degrees).  Left and Right arm solutions
   bool bCanReach[2];            // true if robot can reach, false if not.  Left and right arm configurations
}
INVERSE_SOLUTION;

typedef struct PATH_CHECK
{
   bool bCanDraw[2];    // true if robot can draw, false if not.  Left and right arm configurations
   double dThetaDeg[2]; // total angle changes required to draw path
}
PATH_CHECK;

//----------------------------- Function Prototypes --------------------------------------------------------------
bool flushInputBuffer();         // flushes any characters left in the standard input buffer
void waitForEnterKey();          // waits for the Enter key to be pressed
int nint(double d);              // computes nearest integer to a double value
double degToRad(double);         // returns angle in radians from input angle in degrees
double radToDeg(double);         // returns angle in degrees from input angle in radians
double mapAngle(double angRad);  // make sure inverseKinematic angled are mapped in range robot understands
void pauseRobotThenClear();      // pauses the robot for screen capture, then clears everything

void robotState(ROBOT_STATE *pState, bool bSetState);  // stores/retrieves the current state of the robot

INVERSE_SOLUTION inverseKinematics(TOOL_POSITION); // get left/right arm joint angles from x,y pos

//----------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Program to demonstrate basic control of the SCARA robot simulator
// ARGUMENTS:    none
// RETURN VALUE: an int that tells the O/S how the program ended.  0 = EXIT_SUCCESS = normal termination
int main()
{
   // open connection with robot
   if(!robot.Initialize()) return 0;



   robot.Send("END\n"); // close remote connection
   robot.Close(); // close the robot

   printf("\n\nPress ENTER to end the program...\n");
   waitForEnterKey();
   return EXIT_SUCCESS;
}

//----------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Find x,y position corresponding to shoulder and elbow angles
// ARGUMENTS:    theta1Deg, theta2Deg:  shoulder/joint angles.  px, py: pointers to pen position x,y
// RETURN VALUE: none
void robotState(ROBOT_STATE *pState, bool bSetState)
{
   static ROBOT_STATE state = {0.0, 0.0, LMAX, 0.0, RED, PEN_DOWN, MOTOR_SPEED_MEDIUM};

   if(bSetState)
      state = *pState;
   else
      *pState = state;
}

//----------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Find shoulder and elbow angles for both left/right arm configurations for a x,y coordinate 
// ARGUMENTS:    tp: the x,y coordinates of a tool tip position
// RETURN VALUE: a structure that contains the left/right arm joint angles corresponding to tp and a true/false 
//               value for each arm indicating if the coordinate is reachable.
INVERSE_SOLUTION inverseKinematics(TOOL_POSITION tp)
{
   INVERSE_SOLUTION isol = {ERROR_VALUE, ERROR_VALUE, ERROR_VALUE, ERROR_VALUE, false, false};  // solution values


   return isol;
}

//----------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Pauses the robot then clears everthing after user presses ENTER
// ARGUMENTS:    none
// RETURN VALUE: none
void pauseRobotThenClear()
{
   waitForEnterKey();
   system("cls");
   robot.Send("HOME\n");
   robot.Send("CLEAR_TRACE\n");
   robot.Send("PEN_COLOR 0 0 255\n");
   robot.Send("CLEAR_REMOTE_COMMAND_LOG\n");
   robot.Send("CLEAR_POSITION_LOG\n");
}

//----------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Maps an angle in radians into a an equivalent angle understood by the robot (-PI <= ang <= +PI)
// ARGUMENTS:    ang: the angle in radians 
// RETURN VALUE: the mapped angle in radians
double mapAngle(double angRad)
{
   angRad = fmod(angRad, 2.0 * PI);  // put in range -2*PI <= ang <= +2*PI

   // map into range -PI <= ang <= +PI
   if(angRad > PI)
      angRad -= 2.0 * PI;
   else if(angRad < -PI)
      angRad += 2.0 * PI;

   return angRad;
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in degrees from input angle in radian
// ARGUMENTS:    angDeg:  angle in degrees
// RETURN VALUE: angle in radians
double degToRad(double angDeg)
{
   return (PI / 180.0) * angDeg;
}

//-----------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in radians from input angle in degrees
// ARGUMENTS:    angRad:  angle in radians
// RETURN VALUE: angle in degrees
double radToDeg(double angRad)
{
   return (180.0 / PI) * angRad;
}

//------------------------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  This function flushes the input buffer to avoid scanf issues
// ARGUMENTS:    none
// RETURN VALUE: false if nothing or only '\n' in stdin. true if extra keystrokes precede the '\n'.
//               Good for detecting left over garbage from scanf_s in the input buffer
bool flushInputBuffer()
{
   int ch; // temp character variable
   bool bHasGarbage = false;

   // exit loop when all characters are flushed
   while((ch = getchar()) != '\n' && ch != EOF)
   {
      if(!bHasGarbage) bHasGarbage = true;
   }
   return bHasGarbage;
}

//-----------------------------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Waits for user to press enter.  flushes stdin if keystrokes precede enter
// ARGUMENTS:    none
// RETURN VALUE: none
void waitForEnterKey()
{
   unsigned char ch;
   if((ch = (unsigned char)getchar()) != EOF && ch != '\n') flushInputBuffer();
}

//-----------------------------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  computes nearest integer to given double
// ARGUMENTS:    d: double value
// RETURN VALUE: nearest int
int nint(double d)
{
   return (int)floor(d + 0.5);
}

