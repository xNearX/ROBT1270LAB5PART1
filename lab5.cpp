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

//-------------------------- WINDOWS SPECIFIC CODE ---------------------------------------------------------------
#ifdef  _WIN32
#include "robot.h"  // robot functions
CRobot robot; // the global robot Class.  Can be used everywhere
#endif

//---------------------------- Program Constants -----------------------------------------------------------------
const double PI = 3.14159265358979323846;    // the one and only
const double L1 = 350.0;                     // length of the inner arm
const double L2 = 250.0;                     // length of the outer arm
const double ABS_THETA1_DEG_MAX = 150.0;     // maximum magnitude of shoulder angle in degrees
const double ABS_THETA2_DEG_MAX = 170.0;     // maximum magnitude of elbow angle in degrees
const double LMAX = L1 + L2;                 // max L -> maximum reach of robot
const double LMIN = sqrt(L1 * L1 + L2 * L2 - 2.0 * L1 * L2 * cos(PI - ABS_THETA2_DEG_MAX * PI / 180.0)); // min L

#ifdef _WIN32
const double ERROR_VALUE = DBL_MAX;
#elif __APPLE__
const double ERROR_VALUE = -1;
#endif

const int LOW_RESOLUTION_POINTS_PER_500_UNITS = 3;
const int MEDIUM_RESOLUTION_POINTS_PER_500_UNITS = 11;
const int HIGH_RESOLUTION_POINTS_PER_500_UNITS = 21;


#define COMMAND_STRING_ARRAY_SIZE 502        // size of array to store commands for robot. NOTE: 2 elements must be
                                             // reserved for trailing '\n' and '\0' in command string
#define _CRT_SECURE_NO_WARNINGS              // disables _s warnings for windows users

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

#ifdef _WIN32
// robot state data
typedef struct ROBOT_STATE
{
   JOINT_ANGLES jointAngles;
   TOOL_POSITION toolPos;
   PEN_STATE pen;
   int motorSpeed;
}
ROBOT_STATE;
#endif

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

//----------------------------- Windows Only Functions -----------------------------------------------------------
#ifdef _WIN32
void robotState(ROBOT_STATE *pState, bool bSetState);  // stores/retrieves the current state of the robot
void drawLine();
#endif

INVERSE_SOLUTION inverseKinematics(TOOL_POSITION); // get left/right arm joint angles from x,y pos
TOOL_POSITION* getLinePoints(double, double, double, double, char, int*);
PATH_CHECK checkPath(TOOL_POSITION*, int);

#ifdef __APPLE__
void macDrawLine();
#endif


//----------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Program to demonstrate basic control of the SCARA robot simulator
// ARGUMENTS:    none
// RETURN VALUE: an int that tells the O/S how the program ended.  0 = EXIT_SUCCESS = normal termination
int main()
{
   // windows only
   #ifdef _WIN32
   // open connection with robot
   if(!robot.Initialize()) return 0;
   drawLine();
   pauseRobotThenClear();
   robot.Send("END\n"); // close remote connection
   robot.Close(); // close the robot

   // Mac Only
   #elif __APPLE__
   macDrawLine();
   #endif


   printf("\n\nPress ENTER to end the program...\n");
   waitForEnterKey();
   return EXIT_SUCCESS;
}

#ifdef _WIN32
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
#endif

//----------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Find shoulder and elbow angles for both left/right arm configurations for a x,y coordinate 
// ARGUMENTS:    tp: the x,y coordinates of a tool tip position
// RETURN VALUE: a structure that contains the left/right arm joint angles corresponding to tp and a true/false 
//               value for each arm indicating if the coordinate is reachable.
INVERSE_SOLUTION inverseKinematics(TOOL_POSITION toolPos)
{
    INVERSE_SOLUTION isol = { ERROR_VALUE, ERROR_VALUE, ERROR_VALUE, ERROR_VALUE, false, false };

    double distance = 0.0;
    double beta = 0.0;
    double alpha = 0.0;
    double tanExpression = 0.0;

    double leftTheta1 = 0.0;
    double leftTheta2 = 0.0;
    double leftTheta1Deg = 0.0;
    double leftTheta2Deg = 0.0;

    double rightTheta1 = 0.0;
    double rightTheta2 = 0.0;
    double rightTheta1Deg = 0.0;
    double rightTheta2Deg = 0.0;

    distance = sqrt(toolPos.x * toolPos.x + toolPos.y * toolPos.y);

    // test that distance from robot origin to entered point is within range
    if (distance > LMIN && distance < LMAX)
    {
        beta = atan2(toolPos.y, toolPos.x);
        alpha = acos((L2 * L2 - distance * distance - L1 * L1) / (-2 * distance * L1));

        leftTheta1 = mapAngle(beta + alpha);
        tanExpression = atan2(toolPos.y - L1 * sin(leftTheta1), toolPos.x - L1 * cos(leftTheta1));
        leftTheta2 = mapAngle(tanExpression - leftTheta1);


        leftTheta1Deg = radToDeg(leftTheta1);
        leftTheta2Deg = radToDeg(leftTheta2);

        if (fabs(leftTheta1Deg) < ABS_THETA1_DEG_MAX && fabs(leftTheta2Deg) < ABS_THETA2_DEG_MAX)
        {
            isol.jointAngles[LEFT].theta1Deg = leftTheta1Deg;
            isol.jointAngles[LEFT].theta2Deg = leftTheta2Deg;
            isol.bCanReach[LEFT] = true;

        }

        rightTheta1 = mapAngle(beta - alpha);
        tanExpression = atan2(toolPos.y - L1 * sin(rightTheta1), toolPos.x - L1 * cos(rightTheta1));
        rightTheta2 = mapAngle(tanExpression - rightTheta1);

        rightTheta1Deg = radToDeg(rightTheta1);
        rightTheta2Deg = radToDeg(rightTheta2);

        if (fabs(rightTheta1Deg) < ABS_THETA1_DEG_MAX && fabs(rightTheta2Deg) < ABS_THETA2_DEG_MAX)
        {
            isol.jointAngles[RIGHT].theta1Deg = rightTheta1Deg;
            isol.jointAngles[RIGHT].theta2Deg = rightTheta2Deg;
            isol.bCanReach[RIGHT] = true;

        }
    }
    return isol;

}

TOOL_POSITION* getLinePoints(double x1, double y1, double x2, double y2, char resolution, int* numPoints)
{

    double lineLength = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));

    int NP = 0;

    if (resolution == 'l')
    {
        NP = nint(((lineLength) / 500) * LOW_RESOLUTION_POINTS_PER_500_UNITS);
    }
    else if (resolution == 'm')
    {
        NP = nint(((lineLength) / 500) * MEDIUM_RESOLUTION_POINTS_PER_500_UNITS);
    }
    else if (resolution == 'h')
    {
        NP = nint(((lineLength) / 500) * HIGH_RESOLUTION_POINTS_PER_500_UNITS);
    }

    printf("The number of of points is %d\n", NP);

    TOOL_POSITION* toolPoints = (TOOL_POSITION*)malloc(NP * sizeof(TOOL_POSITION));

    if (toolPoints != NULL)
    {
        for (int n = 0; n < NP; ++n)
        {
            toolPoints[n].x = x1 + (x2 - x1) * ((double)n) / ((double)NP - 1);
            toolPoints[n].y = y1 + (y2 - y1) * ((double)n) / ((double)NP - 1);
        }
    }
    else
    {
        return NULL;
    }


    printf("The list of points are:\n");
    for (int i = 0; i < NP; ++i)
    {
        printf("(%lf, %lf) \n", toolPoints[i].x, toolPoints[i].y);
    }

    *numPoints = NP;

    return toolPoints;
}
#ifdef _WIN32
void drawLine()
{
   double x1, y1, x2, y2;
   char resolution;
   int numPoints;
   TOOL_POSITION *points;
   PATH_CHECK pathCheck;
   char commandString[COMMAND_STRING_ARRAY_SIZE];

   printf("Enter line start point (x y): ");
   scanf_s("%lf %lf", &x1, &y1);
   flushInputBuffer();
   printf("Enter line end point (x y): ");
   scanf_s("%lf %lf", &x2, &y2);
   flushInputBuffer();

   printf("Please enter line resolution (l = low, m = medium, h = high): ");
   scanf_s("%c", &resolution, sizeof(&resolution));
   flushInputBuffer();
   INVERSE_SOLUTION testInv[200];

   points = getLinePoints(x1, y1, x2, y2, resolution, &numPoints);

   for(int i = 0; i < numPoints; i++)
   {
   testInv[i] = inverseKinematics(points[i]);
   }

  

   if (points != NULL)
   {
      //pathCheck = checkPath(points, numPoints);

      pathCheck = {false, true, 900, 300};

      if (pathCheck.bCanDraw[LEFT] || pathCheck.bCanDraw[RIGHT])
      {
         RGB color = { 255, 255 ,255 };
         int motorSpeed = 1;

         printf("Enter pen color (r g b): ");
         scanf_s("%d %d %d", &color.r, &color.g, &color.b);

         sprintf_s(commandString, COMMAND_STRING_ARRAY_SIZE, "PEN_COLOR %d %d %d\n", color.r, color.g, color.b);
         robot.Send(commandString);
         

         printf("Enter motor speed (0 = low, 1 = medium, 2 = high): ");
         scanf_s("%d", &motorSpeed);

         sprintf_s(commandString, COMMAND_STRING_ARRAY_SIZE, "MOTOR_SPEED %s\n", motorSpeed == 0 ? "LOW" : motorSpeed == 1 ? "MEDIUM" : "HIGH");

         robot.Send("PEN_UP\n");
         if(pathCheck.bCanDraw[LEFT] && pathCheck.bCanDraw[RIGHT])
         {
            if(pathCheck.dThetaDeg[LEFT] < pathCheck.dThetaDeg[RIGHT])
            {
               for(int i = 0; i < numPoints; i++)
               {
               
                  INVERSE_SOLUTION isol = inverseKinematics(points[i]);

                  sprintf_s(commandString, COMMAND_STRING_ARRAY_SIZE, "ROTATE_JOINT ANG1 %.2f ANG2 %.2f\n", isol.jointAngles[LEFT].theta1Deg, isol.jointAngles[LEFT].theta2Deg);
                  robot.Send(commandString);
               
                  robot.Send("PEN_DOWN\n");
            
               }
            }
            else
            {
               for(int i = 0; i < numPoints; i++)
               {
                  INVERSE_SOLUTION isol = inverseKinematics(points[i]);

                  sprintf_s(commandString, COMMAND_STRING_ARRAY_SIZE, "ROTATE_JOINT ANG1 %.2f ANG2 %.2f\n", isol.jointAngles[RIGHT].theta1Deg, isol.jointAngles[RIGHT].theta2Deg);
                  robot.Send(commandString);

                  robot.Send("PEN_DOWN\n");
                 
               }
            }
         }
 
         else if (pathCheck.bCanDraw[RIGHT])
         {
            for(int i = 0; i < numPoints; i++)
            {
               INVERSE_SOLUTION isol = inverseKinematics(points[i]);

               sprintf_s(commandString, COMMAND_STRING_ARRAY_SIZE, "ROTATE_JOINT ANG1 %.2f ANG2 %.2f\n", isol.jointAngles[RIGHT].theta1Deg, isol.jointAngles[RIGHT].theta2Deg);
               robot.Send(commandString);
               if(i == 0)
               {
                  robot.Send("PEN_DOWN\n");
               }

            }
         }
         else if(pathCheck.bCanDraw[LEFT])
         {
            for(int i = 0; i < numPoints; i++)
            {
               INVERSE_SOLUTION isol = inverseKinematics(points[i]);

               sprintf_s(commandString, COMMAND_STRING_ARRAY_SIZE, "ROTATE_JOINT ANG1 %.2f ANG2 %.2f\n", isol.jointAngles[LEFT].theta1Deg, isol.jointAngles[LEFT].theta2Deg);
               robot.Send(commandString);
               if(i == 0)
               {
                  robot.Send("PEN_DOWN");
               }
            }
         }
      }
      else
      {
         printf("Neither arm can draw the line.\n");
      }
      free(points);
   }
}
#endif


void macDrawLine()
{
   TOOL_POSITION toolPos = {-230.303030, 300};
   INVERSE_SOLUTION isol = inverseKinematics(toolPos);
   double x1 = -400, y1 = 300, x2 = 400, y2 = 300;
   char resolution = 'l';
   int numPoints;
   TOOL_POSITION* points;
   PATH_CHECK pathCheck;

   printf("left theta 1 is: %lf\n", isol.jointAngles[LEFT].theta1Deg);
   printf("left theta 2 is: %lf\n", isol.jointAngles[LEFT].theta2Deg);
   printf("right theta 1 is: %lf\n", isol.jointAngles[RIGHT].theta1Deg);
   printf("right theta 2 is: %lf\n", isol.jointAngles[RIGHT].theta2Deg);
   printf("Left can reach is %d\n", isol.bCanReach[LEFT]);
   printf("Right can reach is %d\n", isol.bCanReach[RIGHT]);

   points = getLinePoints(x1, y1, x2, y2, resolution, &numPoints);
   
   printf("The number of points at high resolution is %d\n", numPoints);

   for(size_t i = 0; i < numPoints; i++)
   {
      printf("(%lf, %lf)\n", points[i].x, points[i].y);
   }

   pathCheck = checkPath(points, numPoints);

   if (pathCheck.bCanDraw[RIGHT] && (!pathCheck.bCanDraw[LEFT] || pathCheck.dThetaDeg[RIGHT] < pathCheck.dThetaDeg[LEFT]))
   {
      for(size_t i = 0; i < numPoints; i++)
      {
         printf("(%lf, %lf)\n", points[i].x, points[i].y);
      }
   }
}

#ifdef _WIN32
//----------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Pauses the robot then clears everything after user presses ENTER
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
#endif

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

