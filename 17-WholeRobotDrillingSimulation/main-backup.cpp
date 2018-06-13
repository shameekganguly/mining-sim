//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 1869 $



    main_2_drills_move_in_3_dimensions! Today's Final version 
    May 20, 2018
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <math.h>
#include "timer/LoopTimer.h"
#include <fstream>
#include "voxelBoxNode.h"
#include "voxelCollision.h"
#include "drill.h"
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------


// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a small cylinder (cursor) representing the haptic device 
int numDrills = 4;
std::map<int, cShapeCylinder*> cursor;

//two spheres moving along with cursor to make it a capsule
std::map<int, cShapeSphere*> cursorS1;
std::map<int, cShapeSphere*> cursorS2;





// a line representing the velocity vector of the haptic device
cShapeLine* velocity;



// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a label to display the haptic device model
cLabel* labelHapticDeviceModel;

// a label to display the position [m] of the haptic device
cLabel* labelHapticDevicePosition;



// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a flag for using damping (ON/OFF)
bool useDamping = false;

// a flag for using force field (ON/OFF)
bool useForceField = true;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;





//------------------------------------------------------------------------------
// DECLARED NEW VARIABLES
//------------------------------------------------------------------------------

const double Rc = 0.03; // radius of the cylinder 0.023
const double Hc = 0.015; //0.10
const double Rs = 0.005; // 0.01  radius of the sphere obstacle
#define PI 3.14159265

//------------------------------------------------------------------------------
//
// parallel and not staggered
//
//
// const double halfOffsetDistY = 1.0 * Rc;//1.0 * Rc;//5.0 * Rc;//
// const double halfOffsetDistZ = 0.0 * Rc;//1.0 * Rc;//5.0 * Rc;//
// const double halfTiltAngle = 0.0;//30.0


// const double drillangle1[3][3] = {{1,                              0,                              0},
//                                   {0,-sin(PI * (-halfTiltAngle / 180.0)), cos(PI * (-halfTiltAngle / 180.0))},
//                                   {0, cos(PI * (-halfTiltAngle / 180.0)), sin(PI * (-halfTiltAngle / 180.0))}};

// const double drillangle2[3][3] = {{1,                             0,                              0},
//                                   {0, -sin(PI * (halfTiltAngle / 180.0)), cos(PI * (halfTiltAngle / 180.0))},
//                                   {0, cos(PI * (halfTiltAngle / 180.0)), sin(PI * (halfTiltAngle / 180.0))}};

// const double drillangle1[3][3] = {{1,                              0,                              0},
//                                   {0, cos(PI * (-halfTiltAngle / 180.0)), -sin(PI * (-halfTiltAngle / 180.0))},
//                                   {0, sin(PI * (-halfTiltAngle / 180.0)), cos(PI * (-halfTiltAngle / 180.0))}};

// const double drillangle2[3][3] = {{1,                             0,                              0},
//                                   {0, cos(PI * (halfTiltAngle / 180.0)), -sin(PI * (halfTiltAngle / 180.0))},
//                                   {0, sin(PI * (halfTiltAngle / 180.0)), cos(PI * (halfTiltAngle / 180.0))}};

// const double drillangle1[3][3] = {{1,                              0,                              0},
//                                   {0,                              1,                              0},
//                                   {0,                              0,                              1}};

// const double drillangle2[3][3] = {{1,                             0,                              0},
//                                   {0,                             1,                              0},
//                                   {0,                             0,                              1}};

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//
// parallel and staggered
//
//
// const double halfOffsetDistY = 1.0 * Rc;//1.0 * Rc;//5.0 * Rc;//
// const double halfOffsetDistZ = 1.0 * Rc;//1.5 * Rc;//5.0 * Rc;//
// const double halfTiltAngle = 0.0;//30.0
// const double drillangle1[3][3] = {{1,                              0,                              0},
//                                   {0, cos(PI * (-halfTiltAngle / 180.0)), -sin(PI * (-halfTiltAngle / 180.0))},
//                                   {0, sin(PI * (-halfTiltAngle / 180.0)), cos(PI * (-halfTiltAngle / 180.0))}};
// const double drillangle2[3][3] = {{1,                             0,                              0},
//                                   {0, cos(PI * (halfTiltAngle / 180.0)), -sin(PI * (halfTiltAngle / 180.0))},
//                                   {0, sin(PI * (halfTiltAngle / 180.0)), cos(PI * (halfTiltAngle / 180.0))}};
//------------------------------------------------------------------------------





//------------------------------------------------------------------------------
//
// one straight drill and the other tilt and not staggered
//
//
// const double halfOffsetDistY = 1.5 * Rc;//1.0 * Rc;//5.0 * Rc;//
// const double halfOffsetDistZ = 0.0 * Rc;
// const double halfTiltAngle = 45.0;//30.0
// const double drillangle1[3][3] = {{1,                     0,                      0},
//                                   {0, cos(PI * (0 / 180.0)), -sin(PI * (0 / 180.0))},
//                                   {0, sin(PI * (0 / 180.0)),  cos(PI * (0 / 180.0))}};

// const double drillangle2[3][3] = {{1,                                     0,                                      0},
//                                   {0, cos(PI * (2 * halfTiltAngle / 180.0)), -sin(PI * (2 * halfTiltAngle / 180.0))},
//                                   {0, sin(PI * (2 * halfTiltAngle / 180.0)), cos(PI * (2 * halfTiltAngle / 180.0))}};
//------------------------------------------------------------------------------




//------------------------------------------------------------------------------
//
// mining robot
//
//
const double halfOffsetDistX = 0.8 * Rc;//1.0 * Rc;//5.0 * Rc;// 1.2
const double halfOffsetDistY = 0.8 * Rc;//1.0 * Rc;//5.0 * Rc;// 1.2
const double halfOffsetDistZ = 0.0 * Rc;//1.0 * Rc;//5.0 * Rc;//


const double drillangle1[3][3] = {{1,                              0,                              0},
                                  {0,                              1,                              0},
                                  {0,                              0,                              1}};

const double drillangle2[3][3] = {{1,                             0,                              0},
                                  {0,                             1,                              0},
                                  {0,                             0,                              1}};
const double drillangle3[3][3] = {{1,                              0,                              0},
                                  {0,                              1,                              0},
                                  {0,                              0,                              1}};
const double drillangle4[3][3] = {{1,                              0,                              0},
                                  {0,                              1,                              0},
                                  {0,                              0,                              1}};

//---



//flags for haptic interaction
bool fHapticDeviceEnabled = false;
std::map<int, cShapeSphere*> obstacleS;
double scale_factor = 5.0;
cLabel* labelProxyPosition;
cVector3d textProxyPosition;




// int numObstacles = 100;//64;//64;//1024
// int sqrtNumObs = sqrt(numObstacles);
//cVector3d home_pos(0.0, -0.5*Rs, (2.0 * Rs * (sqrtNumObs / 2 + 1) + Rs) + Hc / 2.0 - 5.0 * Rs); //the final 2 * Rs is for a little free space of the capsule

int cbrtNumObs = 8;
int cbrtNumObsBigger = 16;
//int numObstacles = pow(cbrtNumObs, 3);//64;//1024

int numObstacles = cbrtNumObsBigger * cbrtNumObsBigger * cbrtNumObs;//64;//1024


//cVector3d home_pos(0.0, 0.0, (2.0 * Rs * (cbrtNumObs / 2 + 1) + Rs) + Hc / 2.0 + 2 * Rs); //the final 2 * Rs is for a little free space of the capsule
cVector3d home_pos(0.0, 0.0, (2.0 * Rs * (cbrtNumObs / 2 + 1) + Rs) + Hc / 2.0 + 10 * Rs); //the final 2 * Rs is for a little free space of the capsule


cLabel* labelFContact;
cLabel* labelBLIndex;
cLabel* labelInCollision;
//cVector3d textFContact;
std::string textFContact;

double drillingAngularSpeed = 1; //eventually this should be controlled by the gripper angle

double epsForElimination = 0.1 * Rs;

int boundaryLayerIndex = 0; // here the boundary layer is just one ball, and as they are listed as a column, the first ball will be the boundary layer. 
std::map<int, cShapeSphere*> boundaryLayer;

bool capsuleInCollision;

const double proxy_b_no_contact = 0.01;//will change between contact and no contact condition

// double ka = 10.0 / drillingAngularSpeed;
// double kl = 30.0 / drillingAngularSpeed;//these will be the ka and kl used in general algorithm


double ka = 10.0 / drillingAngularSpeed; // 10.0
double kl = 100.0 / drillingAngularSpeed;//30.0  these will be the ka and kl used in general algorithm


ofstream myfile;//used for output velocity and force wrt time

cVector3d lastLinearVel;

voxelCollision vC;



std::map<int, drill*> dr;

//std::map<int, cShapeLine*> boundingBoxLine;



//drillObj params
//turn the drill head direction down
// const double drillObjAngleConst[3][3] = {{1,       0,        0},
//                                          {0, cos(PI), -sin(PI)},
//                                          {0, sin(PI), cos(PI)}};


double alpha = 0;
double beta = 0;
double gamma1 = 1.5707;
const double drillObjAngleConst[3][3] = {{cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma1)-sin(alpha)*cos(gamma1), cos(alpha)*sin(beta)*cos(gamma1)+sin(alpha)*sin(gamma1)},
                                         {sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma1)+cos(alpha)*cos(gamma1), sin(alpha)*sin(beta)*cos(gamma1)-cos(alpha)*sin(gamma1)},
                                         {-sin(beta),                                            cos(beta)*sin(gamma1),                                  cos(beta)*cos(gamma1)}};

double alpha2 = 0;
double beta2 = 0;
double gamma2 = 0;
const double forebodyObjAngleConst[3][3] = {{cos(alpha2)*cos(beta2), cos(alpha2)*sin(beta2)*sin(gamma2)-sin(alpha2)*cos(gamma2), cos(alpha2)*sin(beta2)*cos(gamma2)+sin(alpha2)*sin(gamma2)},
                                         {sin(alpha2)*cos(beta2), sin(alpha2)*sin(beta2)*sin(gamma2)+cos(alpha2)*cos(gamma2), sin(alpha2)*sin(beta2)*cos(gamma2)-cos(alpha2)*sin(gamma2)},
                                         {-sin(beta2),                                            cos(beta2)*sin(gamma2),                                  cos(beta2)*cos(gamma2)}};

double alpha3 = 0;
double beta3 = 0;
double gamma3 = 0.261;//0.261;
// double gamma3 = 1.5707;
const double rearbodyObjAngleConst[3][3] = {{cos(alpha3)*cos(beta3), cos(alpha3)*sin(beta3)*sin(gamma3)-sin(alpha3)*cos(gamma3), cos(alpha3)*sin(beta3)*cos(gamma3)+sin(alpha3)*sin(gamma3)},
                                         {sin(alpha3)*cos(beta3), sin(alpha3)*sin(beta3)*sin(gamma3)+cos(alpha3)*cos(gamma3), sin(alpha3)*sin(beta3)*cos(gamma3)-cos(alpha3)*sin(gamma3)},
                                         {-sin(beta3),                                            cos(beta3)*sin(gamma3),                                  cos(beta3)*cos(gamma3)}};




// const double drillObjAngleConst[3][3] = {{1,       0,        0},
//                                          {0,       0,        1},
//                                          {0,       1,        0}};


std::map<int, cMultiMesh*> drillObj;
//cVector3d drillObjLocalPos(0, 0, -0.85*Hc);//0.85*Hc
cVector3d drillObjLocalPos(0, 1.6*Hc, 0);//0.85*Hc
bool showCapsuleFlag = false;
bool showObjFlag = true;


cLabel* labelDrillingLevel;
int drillingLevel = 1;



// Robot mesh files
const string robot_rearbody = "../resources/robot1/graphics/rearbody.obj";
const string robot_forebody = "../resources/robot1/graphics/forebody.obj";
const string robot_drill = "../resources/robot1/graphics/roller_cone_drill_bit_02.obj";
const string robot_cam1 = "../resources/robot1/graphics/brake_cam.obj";
const string robot_cam2 = "../resources/robot1/graphics/brake_cam2.obj";

cMultiMesh* forebody;
cVector3d forebodyLocalPos(0, 0, 2*Hc);//0.85*Hc
cMultiMesh* rearbody;
std::map<int, cMultiMesh*> frontCam;
std::map<int, cVector3d> frontCamLocalPos;
cVector3d rearbodyLocalPos;

std::map<int, cMultiMesh*> rearCam;
std::map<int, cVector3d> rearCamLocalPos;

int numFrontCams = 4;
int numRearCams = 4;







//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//------------------------------------------------------------------------------
// DECLARED NEW FUNCTIONS
//------------------------------------------------------------------------------


//==============================================================================
/*
    DEMO:   17-WholeRobotDrillingSimulation.cpp

    This application illustrates how to program forces, torques and gripper
    forces to your haptic device.

    In this example the application opens an OpenGL window and displays a
    3D cursor for the device connected to your computer. If the user presses 
    onto the user button (if available on your haptic device), the color of 
    the cursor changes from blue to green.

    In the main haptics loop function  "updateHaptics()" , the position,
    orientation and user switch status are read at each haptic cycle. 
    Force and torque vectors are computed and sent back to the haptic device.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "ROBOT_MINING_SIM" << endl;
    cout << "Demo: 17-WholeRobotDrillingSimulation" << endl;
    cout << "Copyright 2003-2018" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Enable/Disable potential field" << endl;
    cout << "[2] - Enable/Disable damping" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;


    lastLinearVel.zero();


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);


    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif

    //--------------------------------------------------------------------------
    // SET HIGHER FREQUENCY
    //--------------------------------------------------------------------------
    freqCounterGraphics.setTimePeriod(0.001);
    freqCounterHaptics.setTimePeriod(0.001);


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( cVector3d (0.5, 0.0, 0.0),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // camera->set( cVector3d (0.5, 0.0, 0.2),    // camera position (eye)
    //              cVector3d (0.0, 0.0, 0.0),    // look at position (target)
    //              cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // camera->set( cVector3d (0.5, 0.5, 0.5),    // camera position (eye)
    //              cVector3d (0.0, 0.0, 0.0),    // look at position (target)
    //              cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // camera->set( cVector3d (0.0, 0.0, -2),    // camera position (eye)
    //              cVector3d (0.0, 0.0, 0.0),    // look at position (target)
    //              cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // camera->set( cVector3d (0.5, 0.0, 0.0),    // camera position (eye)
    //              cVector3d (0.0, 0.0, 0.0),    // look at position (target)
    //              cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    camera->setClippingPlanes(0.01, 10.0);


    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0);



    //--------------------------------------------------------------------------
    // A CAPSULE SHAPE CURSOR
    //--------------------------------------------------------------------------


    // create two moving spheres and a moving cylinder (cursor) to render a capsule which represents the haptic device
    


    for (int i = 0; i < numDrills; ++i){
        cursor[i] = new cShapeCylinder(Rc, Rc, Hc);
        cursorS1[i] = new cShapeSphere(Rc);
        cursorS2[i] = new cShapeSphere(Rc);

        drillObj[i] = new cMultiMesh();

        drillObj[i]->loadFromFile("resources/17-WholeRobotDrillingSimulation/roller_cone_drill_bit_02.obj");
        drillObj[i]->scaleXYZ(0.003, 0.003, 0.003); //0.0045
        dr[i] = new drill(cursor[i], cursorS1[i], cursorS2[i], drillObj[i], world);
        dr[i]->setCapsuleShowEnabled(showCapsuleFlag);
        
        //dr[i]->setCapsuleShowEnabled(true);
        
    }

    forebody = new cMultiMesh();
    forebody->loadFromFile("resources/17-WholeRobotDrillingSimulation/forebody.obj");
    forebody->scaleXYZ(1.0, 1.0, 1.0);
    world->addChild(forebody);
    // forebody->setShowEnabled(false);

    rearbody = new cMultiMesh();
    rearbody->loadFromFile("resources/17-WholeRobotDrillingSimulation/rearbody.obj");
    rearbody->scaleXYZ(1.0, 1.0, 1.0);
    world->addChild(rearbody);
    //rearbody->setShowEnabled(false);

    //forebodyLocalPos.addr(0.000, 0.015, -0.092, rearbodyLocalPos);
    forebodyLocalPos.addr(0.000, 0.015, 0, rearbodyLocalPos);



    for (int i = 0; i < numFrontCams; ++i){

        //cMultiMesh
        frontCam[i] = new cMultiMesh();
        if (i % 2 == 0){
            frontCam[i]->loadFromFile("resources/17-WholeRobotDrillingSimulation/brake_cam.obj");
        }
        else{
            frontCam[i]->loadFromFile("resources/17-WholeRobotDrillingSimulation/brake_cam2.obj");
        }
        frontCam[i]->scaleXYZ(1.0, 1.0, 1.0);
        world->addChild(frontCam[i]);

        //local position
        // frontCamLocalPos[i].zero();
        frontCamLocalPos[i] = forebodyLocalPos;
    }
    // frontCam[0]->setShowEnabled(false); //right
    // frontCam[1]->setShowEnabled(false);
    // frontCam[2]->setShowEnabled(false); //right
    // frontCam[3]->setShowEnabled(false);

    frontCamLocalPos[0].add( 0.02, -0.015, 0);
    frontCamLocalPos[1].add(-0.02, -0.015, 0);
    frontCamLocalPos[2].add( 0.02, -0.015, 0);
    frontCamLocalPos[3].add(-0.02, -0.015, 0);


    // frontCamLocalPos[0].add( 0.02, -0.015, -0.038);
    // frontCamLocalPos[1].add(-0.02, -0.015, -0.038);
    // frontCamLocalPos[2].add( 0.02, -0.015, -0.038);
    // frontCamLocalPos[3].add(-0.02, -0.015, -0.038);


    for (int i = 0; i < numRearCams; ++i){

        //cMultiMesh
        rearCam[i] = new cMultiMesh();
        if (i % 2 == 0){
            rearCam[i]->loadFromFile("resources/17-WholeRobotDrillingSimulation/brake_cam.obj");
        }
        else{
            rearCam[i]->loadFromFile("resources/17-WholeRobotDrillingSimulation/brake_cam2.obj");
        }
        rearCam[i]->scaleXYZ(1.0, 1.0, 1.0);
        world->addChild(rearCam[i]);

        //local position
        // frontCamLocalPos[i].zero();
        //rearCamLocalPos[i] = rearbodyLocalPos;
        rearCamLocalPos[i] = forebodyLocalPos;
    }

    // rearCam[0]->setShowEnabled(false); //right
    // rearCam[1]->setShowEnabled(false);
    // rearCam[2]->setShowEnabled(false); //right
    // rearCam[3]->setShowEnabled(false);

    // rearCamLocalPos[0].add( 0.02, 0.015, 0);
    // rearCamLocalPos[1].add(-0.02, 0.015, 0);
    // rearCamLocalPos[2].add( 0.02, 0.015, 0);
    // rearCamLocalPos[3].add(-0.02, 0.015, 0);

    // rearCamLocalPos[0].add( 0.02, -0.015, -0.038);
    // rearCamLocalPos[1].add(-0.02, -0.015, -0.038);
    // rearCamLocalPos[2].add( 0.02, -0.015, -0.038);
    // rearCamLocalPos[3].add(-0.02, -0.015, -0.038);

    rearCamLocalPos[0].add( 0.02, 0.015, 0+0.092);
    rearCamLocalPos[1].add(-0.02, 0.015, 0+0.092);
    rearCamLocalPos[2].add( 0.02, 0.015, 0+0.092);
    rearCamLocalPos[3].add(-0.02, 0.015, 0+0.092);







    cMatrix3d rotObstacle;
    rotObstacle.identity();
    
    
    cVector3d testObstaclePos;
    int testObstacleIdx;


    // for (int i = 0; i < cbrtNumObs; i++){
    //     for (int j = 0; j < cbrtNumObs; j++){
    //         for (int k = 0; k < cbrtNumObs; k++){
    //             int obsIdx = (cbrtNumObs * cbrtNumObs) * k + cbrtNumObs * i + j;
    //             obstacleS[obsIdx] = new cShapeSphere(Rs);
    //             cVector3d posObstacle(-(k - (cbrtNumObs / 2)) * (2 * Rs) - Rs * ((cbrtNumObs + 1) % 2),
    //                                    (j - (cbrtNumObs / 2)) * (2 * Rs) + Rs * ((cbrtNumObs + 1) % 2),
    //                                   -(i - (cbrtNumObs / 2)) * (2 * Rs) - Rs * ((cbrtNumObs + 1) % 2));
    //             obstacleS[obsIdx]->setLocalPos(posObstacle);
    //             obstacleS[obsIdx]->setLocalRot(rotObstacle);
    //             world->addChild(obstacleS[obsIdx]);

    //             //obstacleS[obsIdx]->setShowEnabled(false);
    //         }
    //     }
    // }
    for (int i = 0; i < cbrtNumObs; i++){
        for (int j = 0; j < cbrtNumObsBigger; j++){
            for (int k = 0; k < cbrtNumObsBigger; k++){
                int obsIdx = (cbrtNumObs * cbrtNumObsBigger) * k + cbrtNumObsBigger * i + j;
                obstacleS[obsIdx] = new cShapeSphere(Rs);
                cVector3d posObstacle(-(k - (cbrtNumObsBigger / 2)) * (2 * Rs) - Rs * ((cbrtNumObsBigger + 1) % 2),
                                       (j - (cbrtNumObsBigger / 2)) * (2 * Rs) + Rs * ((cbrtNumObsBigger + 1) % 2),
                                      -(i - (cbrtNumObs / 2)) * (2 * Rs) - Rs * ((cbrtNumObs + 1) % 2));
                obstacleS[obsIdx]->setLocalPos(posObstacle);
                obstacleS[obsIdx]->setLocalRot(rotObstacle);
                world->addChild(obstacleS[obsIdx]);

                //obstacleS[obsIdx]->setShowEnabled(false);
            }
        }
    }


    // voxelCollision vC;
    vC.initialize(obstacleS, numObstacles);
    //vC.printTree();







    //------------------------------------------------------------------------------------
    //only for bounding volume hierarchy test, and now we are using it to show the bounding box of the capsule

    // cVector3d mn;
    // cVector3d mx;
    // // int depth = 6;
    // // int order[] = {0,1,1,1,1,1};//problem
    
    // int depth = 2;//another problem
    // int order[] = {0,0,0,1};

    // bool coutInfo = false;
    // vC.render(depth, order, mn, mx, coutInfo);
    // //vC.render(vC.m_nodes.size()-4, mn, mx);

    // // voxelBoxNode voxelN(testObstacleIdx);
    // // voxelN.fitBBox(Rs, testObstaclePos);


    // cVector3d mnDr;
    // cVector3d mxDr;

    // //dr1->m_node.render(mnDr, mxDr);
    // dr[0]->m_node.render(mnDr, mxDr);


    // cVector3d nnn(mn.x(),mn.y(), mn.z()),
    //           xxx(mx.x(),mx.y(), mx.z()),
    //           xnn(mx.x(),mn.y(), mn.z()),
    //           nxn(mn.x(),mx.y(), mn.z()),
    //           nnx(mn.x(),mn.y(), mx.z()),
    //           xxn(mx.x(),mx.y(), mn.z()),
    //           xnx(mx.x(),mn.y(), mx.z()),
    //           nxx(mn.x(),mx.y(), mx.z());





    // boundingBoxLine[0] = new cShapeLine(nnn, nnx);
    // boundingBoxLine[1] = new cShapeLine(nnn, nxn);
    // boundingBoxLine[2] = new cShapeLine(nnn, xnn);

    // boundingBoxLine[3] = new cShapeLine(xxx, nxx);
    // boundingBoxLine[4] = new cShapeLine(xxx, xnx);
    // boundingBoxLine[5] = new cShapeLine(xxx, xxn);

    // boundingBoxLine[6] = new cShapeLine(nxn, nxx);
    // boundingBoxLine[7] = new cShapeLine(nxn, xxn);

    // boundingBoxLine[8] = new cShapeLine(nnx, nxx);
    // boundingBoxLine[9] = new cShapeLine(nnx, xnx);

    // boundingBoxLine[10] = new cShapeLine(xnn, xxn);
    // boundingBoxLine[11] = new cShapeLine(xnn, xnx);


    // for (int k = 0; k < 12; ++k){
    // world->addChild(boundingBoxLine[k]);
    // }



    // cColorf LineColor(0.7f, 0.7f, 0.7f);
    // // set size on lines
    // glLineWidth(1.0);

    // // set color of boundary box
    // glColor4fv(LineColor.getData());
    // cDrawWireBox(-0.5, 0.5, -0.5, 0.5,-0.5, 0.5);
    // glEnable(GL_LIGHTING);

    // //------------------------------------------------------------------------------------

























    // create small line to illustrate the velocity of the haptic device
    velocity = new cShapeLine(cVector3d(0,0,0), 
                              cVector3d(0,0,0));

    // insert line inside world
    world->addChild(velocity);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    handler->getDevice(hapticDevice, 0);

    // open a connection to haptic device
    hapticDevice->open();

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    /*------------------set up flags------------------*/
    fHapticDeviceEnabled = true;



    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // display a reference frame if haptic device supports orientations
    if (info.m_sensedRotation == true)
    {

 
        //  // display reference frame
        // dr1->m_cylinder->setShowFrame(true);

        // // set the size of the reference frame
        // dr1->m_cylinder->setFrameSize(0.05);   

        for (int i = 0; i < numDrills; ++i){
            // display reference frame
            dr[i]->m_cylinder->setShowFrame(true);
            // set the size of the reference frame
            dr[i]->m_cylinder->setFrameSize(0.05);

        }

    }

    // if the device has a gripper, enable the gripper to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);



    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI40();

    // create a label to display the haptic device model
    labelHapticDeviceModel = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDeviceModel);
    labelHapticDeviceModel->setText(info.m_modelName);

    // create a label to display the position of haptic device
    labelHapticDevicePosition = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDevicePosition);


    labelProxyPosition = new cLabel(font);
    camera->m_frontLayer->addChild(labelProxyPosition);

    labelFContact = new cLabel(font);
    camera->m_frontLayer->addChild(labelFContact);

    labelBLIndex = new cLabel(font);
    camera->m_frontLayer->addChild(labelBLIndex);

    labelInCollision = new cLabel(font);
    camera->m_frontLayer->addChild(labelInCollision);

    labelDrillingLevel = new cLabel(font);
    camera->m_frontLayer->addChild(labelDrillingLevel);



    
    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);




    //--------------------------------------------------------------------------
    // PREPARE TO WRITE DATA INTO A .TXT FILE
    //--------------------------------------------------------------------------    
    myfile.open("resources/17-WholeRobotDrillingSimulation/F_contact_and_Linear_Vel.txt");
    myfile << "Time \t\t F_contact \t\t Linear_Vel \t\t InCollision\n";







    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetFramebufferSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    //close myfile
    myfile.close();

    // exit
    return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;

}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - enable/disable force field
    else if (a_key == GLFW_KEY_1)
    {
        useForceField = !useForceField;
        if (useForceField)
            cout << "> Enable force field     \r";
        else
            cout << "> Disable force field    \r";
    }

    // option - enable/disable damping
    else if (a_key == GLFW_KEY_2)
    {
        useDamping = !useDamping;
        if (useDamping)
            cout << "> Enable damping         \r";
        else
            cout << "> Disable damping        \r";
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

    delete hapticsThread;
    delete world;
    delete handler;


    for (int i = 0; i < numDrills; ++i){
        delete dr[i];
    }

    //delete forebody; //already deleted by world
    //delete rearbody; //already deleted by world
    
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update position data
    std::string textOfDevicePosition      = "Device Position: ";
    std::string textOfProxyPosition       = "Proxy   Position: ";
    std::string textOfContactForce        = "Contact   Force: ";
    std::string textOfBoundaryLayerIndex  = "Boundary Layer Index: ";
    std::string textOfInCollision  = "InCollision: ";
    std::string textOfDrillingSpeedLevel = "Drilling Speed Level (1-5): ";
    std::string capsuleInCollisionText = capsuleInCollision ? "True" : "False";


    labelHapticDevicePosition->setText(textOfDevicePosition + hapticDevicePosition.str(3));
    labelProxyPosition->setText(textOfProxyPosition + textProxyPosition.str(3));
    //labelFContact->setText(textOfContactForce + textFContact.str(6));
    labelFContact->setText(textOfContactForce + textFContact);
    labelBLIndex->setText(textOfBoundaryLayerIndex + std::to_string(boundaryLayerIndex));
    labelInCollision->setText(textOfInCollision + capsuleInCollisionText);
    labelDrillingLevel->setText(textOfDrillingSpeedLevel + std::to_string(drillingLevel));



    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);




    // update position of label

    labelHapticDeviceModel->setLocalPos(20, height - 40, 0);

    // update position of label
    labelHapticDevicePosition->setLocalPos(20, height - 70, 0);

    labelProxyPosition->setLocalPos(20, height - 100, 0);

    labelFContact->setLocalPos(20, height - 130, 0);

    labelBLIndex->setLocalPos(20, height - 160, 0);

    labelInCollision->setLocalPos(20, height - 190, 0);

    labelDrillingLevel->setLocalPos(20, height - 220, 0);
    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all OpenGL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------





void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;


    //create a timer
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(1000); //1000Hz timer
    double last_time = timer.elapsedTime(); //secs

    bool fTimerDidSleep = true;    


    //--------------------------------------------------------------------------
    // PROXY_SETUP AND INITIALIZATION
    //--------------------------------------------------------------------------
    cVector3d proxy_position, device_position, F_proxy, T_proxy, F_contact, T_contact;
    
    cMatrix3d device_rotation;

    std::map<int, cMatrix3d> proxy_rotation;

    F_contact.zero();
    T_contact.zero();
    const double proxy_kp = 5.0;//3.0
    const double proxy_kr = 5.0;//0.5
    //const double proxy_b = 0.003;
    const double haptic_force_scale = 20;
    const double contact_kp = 1.5;

    const double cursorMass = 1.0;
    const double cursorMomentumOfInertia = 5.0;




    cVector3d position;
    cMatrix3d rotation;
    cMatrix3d rotation2;
    cMatrix3d rotation3;
    cMatrix3d rotation4;




    cVector3d proxyLinearVel;
    cVector3d proxyAngularVel;
    cVector3d proxyLinearAcc;
    cVector3d proxyAngularAcc;

    cMatrix3d drillObjAngle;
    drillObjAngle.set(drillObjAngleConst);


    rotation.set(drillangle1);
    rotation2.set(drillangle2);
    rotation3.set(drillangle3);
    rotation4.set(drillangle4);




    proxyLinearVel.zero();
    proxyAngularVel.zero();
    proxyLinearAcc.zero();
    proxyAngularAcc.zero();

    cVector3d s1LocalPos(0, 0, -1/2.0*Hc);
    cVector3d s2LocalPos(0, 0, 1/2.0*Hc);


    std::map<int, cVector3d> offsetPos;


    offsetPos[0].set(halfOffsetDistX, halfOffsetDistY, halfOffsetDistZ);
    offsetPos[1].set(-halfOffsetDistX, halfOffsetDistY, halfOffsetDistZ);
    offsetPos[2].set(-halfOffsetDistX, -halfOffsetDistY, halfOffsetDistZ);
    offsetPos[3].set(halfOffsetDistX, -halfOffsetDistY, halfOffsetDistZ);




    

    boundaryLayer[boundaryLayerIndex] = obstacleS[boundaryLayerIndex];
 


    //initialization
    if (fHapticDeviceEnabled){
        hapticDevice->getPosition(position);//Find the position and give to raw_position
        //hapticDevice->getRotation(rotation);
        position.x(0);
        position.y(0);
        position = scale_factor * position + home_pos;


        device_position = position;
        device_rotation = rotation;
        proxy_position = device_position;

        proxy_rotation[0] = device_rotation;
        proxy_rotation[1] = rotation2;
        proxy_rotation[2] = rotation3;
        proxy_rotation[3] = rotation4;


        // // update position and orientation of cursor



        std::map<int, cVector3d> drillPos1, drillPos2, drillObjPos;


        for (int i = 0; i < numDrills; ++i){
            drillPos1[i] = proxy_position + proxy_rotation[i] * s1LocalPos + offsetPos[i];
            drillPos2[i] = proxy_position + proxy_rotation[i] * s2LocalPos + offsetPos[i];
            drillObjPos[i] = proxy_position + proxy_rotation[i] * drillObjAngle * drillObjLocalPos + offsetPos[i];
            dr[i]->setLocalPos(drillPos1[i], drillPos2[i], drillObjPos[i]);
            dr[i]->setLocalRot(proxy_rotation[i], drillObjAngle);
        }

        cVector3d forebodyPos;
        cMatrix3d forebodyRot;


        forebodyRot.set(forebodyObjAngleConst);

        forebodyPos = proxy_position + rotation * forebodyRot * forebodyLocalPos;
        //forebodyPos = proxy_position;
        forebody->setLocalPos(forebodyPos);
        forebody->setLocalRot(rotation * forebodyRot);

        std::map<int, cVector3d> frontCamPos;
        for (int i = 0; i < numFrontCams; i++){
            frontCamPos[i] = proxy_position + rotation * forebodyRot * frontCamLocalPos[i];
            frontCam[i]->setLocalPos(frontCamPos[i]);
            frontCam[i]->setLocalRot(rotation * forebodyRot);
        }


        cVector3d rearbodyPos;
        cMatrix3d rearbodyRot;
        rearbodyRot.set(rearbodyObjAngleConst);
        rearbodyPos = proxy_position + rotation * rearbodyRot * rearbodyLocalPos;
        rearbody->setLocalPos(rearbodyPos);
        rearbody->setLocalRot(rotation * rearbodyRot);



        // std::map<int, cVector3d> rearCamPos;
        // for (int i = 0; i < numRearCams; i++){
        //     rearCamPos[i] = proxy_position + rotation * rearbodyRot * rearCamLocalPos[i];
        //     rearCam[i]->setLocalPos(rearCamPos[i]);
        //     //rearCam[i]->setLocalPos(rearbodyPos);
        //     rearCam[i]->setLocalRot(rotation * rearbodyRot);
        // }
        std::map<int, cVector3d> rearCamPos;
        for (int i = 0; i < numRearCams; i++){
            rearCamPos[i] = proxy_position + rotation * forebodyRot * rearCamLocalPos[i];
            rearCam[i]->setLocalPos(rearCamPos[i]);
            //rearCam[i]->setLocalPos(rearbodyPos);
            rearCam[i]->setLocalRot(rotation * forebodyRot);
        }


    }
    else{
        position.x(0);
        position.y(0);
        position.z(0);
        position = scale_factor * position + home_pos;


        device_position = position;
        device_rotation = rotation;
        proxy_position = device_position;

        proxy_rotation[0] = device_rotation;
        proxy_rotation[1] = rotation2;
        proxy_rotation[2] = rotation3;
        proxy_rotation[3] = rotation4;


        // // update position and orientation of cursor



        std::map<int, cVector3d> drillPos1, drillPos2, drillObjPos;


        for (int i = 0; i < numDrills; ++i){
            drillPos1[i] = proxy_position + proxy_rotation[i] * s1LocalPos + offsetPos[i];
            drillPos2[i] = proxy_position + proxy_rotation[i] * s2LocalPos + offsetPos[i];
            drillObjPos[i] = proxy_position + proxy_rotation[i] * drillObjAngle * drillObjLocalPos + offsetPos[i];
            dr[i]->setLocalPos(drillPos1[i], drillPos2[i], drillObjPos[i]);
            dr[i]->setLocalRot(proxy_rotation[i], drillObjAngle);
        }

        cVector3d forebodyPos;
        cMatrix3d forebodyRot;


        forebodyRot.set(forebodyObjAngleConst);

        forebodyPos = proxy_position + rotation * forebodyRot * forebodyLocalPos;
        //forebodyPos = proxy_position;
        forebody->setLocalPos(forebodyPos);
        forebody->setLocalRot(rotation * forebodyRot);

        std::map<int, cVector3d> frontCamPos;
        for (int i = 0; i < numFrontCams; i++){
            frontCamPos[i] = proxy_position + rotation * forebodyRot * frontCamLocalPos[i];
            frontCam[i]->setLocalPos(frontCamPos[i]);
            frontCam[i]->setLocalRot(rotation * forebodyRot);
        }


        // right now we will just fix the rearbody and rearcam
        cVector3d rearbodyPos;
        cMatrix3d rearbodyRot;
        rearbodyRot.set(rearbodyObjAngleConst);
        rearbodyPos = proxy_position + rotation * rearbodyRot * rearbodyLocalPos;
        rearbody->setLocalPos(rearbodyPos);
        rearbody->setLocalRot(rotation * rearbodyRot);

        // std::map<int, cVector3d> rearCamPos;
        // for (int i = 0; i < numRearCams; i++){
        //     rearCamPos[i] = proxy_position + rotation * rearbodyRot * rearCamLocalPos[i];
        //     rearCam[i]->setLocalPos(rearCamPos[i]);
        //     //rearCam[i]->setLocalPos(rearbodyPos);
        //     rearCam[i]->setLocalRot(rotation * rearbodyRot);
        // }
        std::map<int, cVector3d> rearCamPos;
        for (int i = 0; i < numRearCams; i++){
            rearCamPos[i] = proxy_position + rotation * forebodyRot * rearCamLocalPos[i];
            rearCam[i]->setLocalPos(rearCamPos[i]);
            //rearCam[i]->setLocalPos(rearbodyPos);
            rearCam[i]->setLocalRot(rotation * forebodyRot);
        }
    }

















    // main haptic simulation loop
    while(simulationRunning)
    {
        fTimerDidSleep = timer.waitForNextLoop();

        // update time
        double curr_time = timer.elapsedTime();
        double loop_dt = curr_time - last_time;

        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // read position 
        //cVector3d position;
        hapticDevice->getPosition(position);
        position = scale_factor * position +home_pos;
        device_position = position;
        



        // read gripper position
        double gripperAngle;
        hapticDevice->getGripperAngleRad(gripperAngle);

        // read linear velocity 
        cVector3d linearVelocity;
        hapticDevice->getLinearVelocity(linearVelocity);

        // read angular velocity
        cVector3d angularVelocity;
        hapticDevice->getAngularVelocity(angularVelocity);

        // read gripper angular velocity
        double gripperAngularVelocity;
        hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);

        // read user-switch status (button 0)
        bool button0, button1, button2, button3;
        button0 = false;
        button1 = false;
        button2 = false;
        button3 = false;

        hapticDevice->getUserSwitch(0, button0);
        hapticDevice->getUserSwitch(1, button1);
        hapticDevice->getUserSwitch(2, button2);
        hapticDevice->getUserSwitch(3, button3);


        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////



        // adjust the  color of the cursor according to the status of
        // the user-switch (ON = TRUE / OFF = FALSE)


        if (button0)
        {
            for (int i = 0; i < numDrills; ++i){
                dr[i]->setColor(0);
            }
            drillingAngularSpeed = 2.0;
            drillingLevel = 2;
        }
        else if (button1)
        {
            for (int i = 0; i < numDrills; ++i){
                dr[i]->setColor(1);
            } 
            drillingAngularSpeed = 3.0;
            drillingLevel = 3;
        }
        else if (button2)
        {
            for (int i = 0; i < numDrills; ++i){
                dr[i]->setColor(2);
            }
            drillingAngularSpeed = 4.0;
            drillingLevel = 4;
        }
        else if (button3)
        {
            for (int i = 0; i < numDrills; ++i){
                dr[i]->setColor(3);
            }
            drillingAngularSpeed = 5.0;
            drillingLevel = 5;
        }
        else
        {
            for (int i = 0; i < numDrills; ++i){
                dr[i]->setColor(4);
            }
            drillingAngularSpeed = 1.0;
            drillingLevel = 1;
        }

        ka = 10.0 / drillingAngularSpeed;
        kl = 100.0 / drillingAngularSpeed;//these will be the ka and kl used in general algorithm




        // update global variable for graphic display update
        hapticDevicePosition = device_position;
        textProxyPosition = proxy_position;


        /////////////////////////////////////////////////////////////////////
        // COMPUTE AND APPLY FORCES
        /////////////////////////////////////////////////////////////////////


        //unitL may be used in other situations


        //F_proxy caused by the deviation from the device position
        F_proxy = proxy_kp * (device_position - proxy_position);


        //is it possible for us to change the boundaryLayer size at the same time as executing the for loop
        



        //1->collision
        //2->crush
        //3->no collision
        //4->already eliminated     no 4 currently


        cVector3d F_contact;
        capsuleInCollision = false;
        bool stickyFlag = true;


        int * collisionType = new int[numDrills];
        for (int i = 0; i < numDrills; ++i){
            collisionType[i] = vC.computeCollision(dr[i], lastLinearVel, stickyFlag);// if one stickyFlag is false, all should be false. Because now it is in drilling state
            // if (collisionType[i] == 1){
            //     capsuleInCollision = true;
            // }
        }


        if (collisionType[0] == 1 && collisionType[1] == 1){
            capsuleInCollision = true;
        }



        cVector3d F_vibration; //try first add the vibration force around x axis (which is originally 0 all the time)
        //This force should only be added to the haptic device, not on F_total
        cVector3d F_total;
        F_total.zero();
        F_contact.zero();
        F_vibration.zero();
        cMatrix3d A_For_Vel;

        double A_For_Vel_source[3][3] = {
            {proxy_b_no_contact,                  0,                  0},
            {                 0, proxy_b_no_contact,                  0},
            {                 0,                  0, proxy_b_no_contact}
        };
        A_For_Vel.set(A_For_Vel_source);



        for (int i = 0; i < numDrills; ++i){
        //for (int i = 0; i < 1; ++i){
            if (collisionType[i] == 1 && !stickyFlag){
                cVector3d unitL_i = dr[i]->axisDirection();
                cMatrix3d A_For_Vel_I;

                double A_For_Vel_I_source[3][3] = {
                    {kl,  0,  0},
                    { 0, kl,  0},
                    { 0,  0, kl}
                };
                A_For_Vel_I.set(A_For_Vel_I_source);

                double a_i = unitL_i.x();
                double b_i = unitL_i.y();
                double c_i = unitL_i.z();

                cMatrix3d abc_i;

                double abc_i_source[3][3] = {
                    {(ka - kl) * a_i * a_i, (ka - kl) * a_i * b_i, (ka - kl) * a_i * c_i},
                    {(ka - kl) * a_i * b_i, (ka - kl) * b_i * b_i, (ka - kl) * b_i * c_i},
                    {(ka - kl) * a_i * c_i, (ka - kl) * b_i * c_i, (ka - kl) * c_i * c_i}
                };

                abc_i.set(abc_i_source);

                A_For_Vel_I.add(abc_i);

                A_For_Vel.add(A_For_Vel_I);
                //F_contact = F_contact + dr[i]->contactForce(F_proxy, proxy_b_no_contact, proxy_b_contact_dir, proxy_b_contact_lat);
                //F_vibration = F_vibration + dr[i]->vibrationForce(true, curr_time);
            }
        }

        
        cMatrix3d A_For_Vel_inv;
        A_For_Vel.invertr(A_For_Vel_inv);

        F_total = F_proxy + F_vibration;


        //if we want to add some vibration feelings, just directly add the disturbance here in F_proxy
        A_For_Vel_inv.mulr(F_proxy, proxyLinearVel);




        F_contact = F_proxy - proxy_b_no_contact * proxyLinearVel;

        //textFContact = std::to_string(F_contact.length());  
        textFContact = A_For_Vel.str(3);    

        //record the velocity
        lastLinearVel = proxyLinearVel;

        std::string txtCollision = capsuleInCollision ? "True" : "False";
        myfile << curr_time << "\t" <<  F_contact.str(6) << "\t" << proxyLinearVel.str(6) << "\t"<< txtCollision <<"\n";




        velocity->m_pointA = proxy_position;
        velocity->m_pointB = cAdd(proxy_position, proxyLinearVel);

        proxy_position = proxy_position + proxyLinearVel * loop_dt;





        std::map<int, cVector3d> drillPos1, drillPos2, drillObjPos;
        cMatrix3d d_DrillObjLocalRot, d_DrillObjLocalRotRev;
        
        cVector3d rotAxis(0.0, 0.0, -1.0);

        
        d_DrillObjLocalRot.setAxisAngleRotationRad(rotAxis, drillingAngularSpeed * loop_dt);
        d_DrillObjLocalRotRev.setAxisAngleRotationRad(-rotAxis, drillingAngularSpeed * loop_dt);



        for (int i = 0; i < numDrills; ++i){
            drillPos1[i] = proxy_position + proxy_rotation[i] * s1LocalPos + offsetPos[i];
            drillPos2[i] = proxy_position + proxy_rotation[i] * s2LocalPos + offsetPos[i];
            drillObjPos[i] = proxy_position + proxy_rotation[i] * drillObjAngle * drillObjLocalPos + offsetPos[i];
            dr[i]->setLocalPos(drillPos1[i], drillPos2[i], drillObjPos[i]);
            if (i % 2 == 0){
                proxy_rotation[i] = proxy_rotation[i] * d_DrillObjLocalRot;
            }else{
                proxy_rotation[i] = proxy_rotation[i] * d_DrillObjLocalRotRev;
            }
            dr[i]->setLocalRot(proxy_rotation[i], drillObjAngle);
        }


        cVector3d forebodyPos;
        cMatrix3d forebodyRot;


        forebodyRot.set(forebodyObjAngleConst);

        forebodyPos = proxy_position + rotation * forebodyRot * forebodyLocalPos;
        //forebodyPos = proxy_position;

        forebody->setLocalPos(forebodyPos);
        forebody->setLocalRot(rotation * forebodyRot);

        std::map<int, cVector3d> frontCamPos;
        for (int i = 0; i < numFrontCams; i++){
            frontCamPos[i] = proxy_position + rotation * forebodyRot * frontCamLocalPos[i];
            frontCam[i]->setLocalPos(frontCamPos[i]);
            frontCam[i]->setLocalRot(rotation * forebodyRot);
        }

        // send computed force, torque, and gripper force to haptic device
        //hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

        //hapticDevice->setForceAndTorqueAndGripperForce(-F_proxy * haptic_force_scale, cVector3d(), 0);

        hapticDevice->setForceAndTorqueAndGripperForce((-F_proxy+F_vibration) * haptic_force_scale, cVector3d(), 0);
        F_contact.zero();
        F_vibration.zero();
        //hapticDevice->setForceAndTorqueAndGripperForce(F_proxy * haptic_force_scale, T_proxy, 0);

        // -------------------------------------------
        // update last time
        last_time = curr_time;
        freqCounterHaptics.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//-----------------------------------------------------------------------------