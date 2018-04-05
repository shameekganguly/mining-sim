/*  01-cylinders
Simplest construction of the mining robot simulation

Author: Shameek Ganguly shameekg@stanford.edu
Date: 1/15/18
*/

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;
using namespace Eigen;

const string world_fname = "resources/01-cylinders/world.urdf";
const string robot_fname = "../resources/robot1/robot1.urdf";
const string robot_name = "Robot";
// string camera_name = "camera_side";
// string camera_name = "camera_isometric";
string camera_name = "camera_front";
// string camera_name = "camera_top";

// global variables
Eigen::VectorXd q_home;
chai3d::cGenericObject* fore_cam1;
chai3d::cGenericObject* fore_cam2;
chai3d::cGenericObject* fore_cam3;
chai3d::cGenericObject* fore_cam4;
chai3d::cGenericObject* rear_cam1;
chai3d::cGenericObject* rear_cam2;
chai3d::cGenericObject* rear_cam3;
chai3d::cGenericObject* rear_cam4;
chai3d::cMaterialPtr cam_disengaged_mat;
chai3d::cMaterialPtr cam_engaged_mat;

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

bool f_global_sim_pause = false; // use with caution!
// bool f_global_sim_pause = true; // use with caution!

bool f_camera_view_changed = false;

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fRotPanTilt = false;

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);
	Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	graphics->_world->setBackgroundColor(0.4, 0.4, 0.4);

	// load objects for cams
	chai3d::cGenericObject* rearbody_graphic = graphics->_world->getChild(0)->getChild(0)->getChild(0)->getChild(0)->getChild(0)->getChild(0)->getChild(0);
	rear_cam1 = rearbody_graphic->getChild(8)->getChild(0);
	rear_cam2 = rearbody_graphic->getChild(9)->getChild(0);
	rear_cam3 = rearbody_graphic->getChild(10)->getChild(0);
	rear_cam4 = rearbody_graphic->getChild(11)->getChild(0);
	chai3d::cGenericObject* forebody_graphic = graphics->_world->getChild(0)->getChild(0)->getChild(0)->getChild(0)->getChild(0)->getChild(0)->getChild(0)->getChild(7)->getChild(0);
	fore_cam1 = forebody_graphic->getChild(4)->getChild(0);
	fore_cam2 = forebody_graphic->getChild(5)->getChild(0);
	fore_cam3 = forebody_graphic->getChild(6)->getChild(0);
	fore_cam4 = forebody_graphic->getChild(7)->getChild(0);

	cam_engaged_mat = chai3d::cMaterial::create();
	cam_engaged_mat->setRedFireBrick();
	cam_disengaged_mat = chai3d::cMaterial::create();
	cam_disengaged_mat->setGreenLime();
	rear_cam1->setMaterial(cam_disengaged_mat);
	rear_cam2->setMaterial(cam_disengaged_mat);
	rear_cam3->setMaterial(cam_disengaged_mat);
	rear_cam4->setMaterial(cam_disengaged_mat);
	fore_cam4->setMaterial(cam_disengaged_mat);
	fore_cam3->setMaterial(cam_disengaged_mat);
	fore_cam2->setMaterial(cam_disengaged_mat);
	fore_cam1->setMaterial(cam_disengaged_mat);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);
	sim->setCollisionRestitution(0.3);
    // set co-efficient of friction also to zero for now as this causes jitter
    sim->setCoeffFrictionStatic(1.0);
    sim->setCoeffFrictionDynamic(1.0);

	// set initial condition
	q_home.setZero(robot->dof());
	robot->_q = q_home;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

	// information about computer screen and GLUT display window
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// start the simulation
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    // cache variables
	double last_cursorx, last_cursory;

	Eigen::MatrixXd G;
	Eigen::Matrix3d R;
	Eigen::Vector3d center_point = Eigen::Vector3d::Zero();
    while (!glfwWindowShouldClose(window)) {
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

	    // poll for events
	    glfwPollEvents();
	
		// move scene camera as required
		if (f_camera_view_changed) {
			graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
			f_camera_view_changed = true;
		}
    	
    	Eigen::Vector3d cam_up_axis;
    	// cam_up_axis = camera_vertical;
    	// cam_up_axis.normalize();
    	cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
	    Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
    	cam_roll_axis.normalize();
    	Eigen::Vector3d cam_lookat_axis = camera_lookat;
    	cam_lookat_axis.normalize();
    	if (fTransXp) {
	    	camera_pos = camera_pos + 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_roll_axis;
	    }
	    if (fTransXn) {
	    	camera_pos = camera_pos - 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_roll_axis;
	    }
	    if (fTransYp) {
	    	// camera_pos = camera_pos + 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos + 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_up_axis;
	    }
	    if (fTransYn) {
	    	// camera_pos = camera_pos - 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos - 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_up_axis;
	    }
	    if (fRotPanTilt) {
	    	// get current cursor position
	    	double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
	    }
	    graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
	    glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer
	double last_time = timer.elapsedTime(); //secs

	// States
	enum OperationStates {
		NoOp = 0,
		Drilling,
		MovingForeBody,
		MovingRearBody
	};

	// dof counts
	int dof = robot->dof();
	int act_dof = robot->dof() - 6; // 3D free base

	VectorXd tau, tau_act;
	tau.setZero(dof);
	
	// cache variables
	bool fTimerDidSleep = true;

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// check if paused
		if (f_global_sim_pause) { continue;}

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);

		// update model every once in a while
		// TODO: this should be in a separate thread
		if (timer.elapsedCycles() % 10 == 1) {
			robot->updateModel();
			// robot->gravityVector(gj, Eigen::Vector3d(0.0, 0.0, -3.00));
		}

		// assemble full tau vector for simulation
		// tau.tail(act_dof) = tau_act;
		sim->setJointTorques(robot_name, tau);

		// -------------------------------------------

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1500); //1.5kHz timer
	double last_time = timer.elapsedTime(); //secs

	/* ---- START: VARIABLES FOR KINEMATIC SIMULATION ---- */
	enum OperationStates {
		NoOp = 0,
		EngageRearCams,
		DisengageForeCams,
		MovingForeBody,
		EngageForeCams,
		DisengageRearCams,
		MovingRearBody,
	};
	OperationStates robot_state = EngageRearCams;
	auto base = sim->_world->getBaseNode(robot_name);
	uint num_drill_cycles = 0;
	double state_current_time = 0;
	double last_state_stop_time = 2;
	double last_rear_body_pos = 0.0;
	bool forecams_engaged = false;
	/* ---- END: VARIABLES FOR KINEMATIC SIMULATION ---- */

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();
		double curr_time = timer.elapsedTime();

		// // integrate forward
		// double curr_time = timer.elapsedTime();
		// double loop_dt = curr_time - last_time;
		// if (!f_global_sim_pause) {
		// 	sim->integrate(loop_dt);
		// }

		/* ---- START: KINEMATIC SIMULATION FOR DEMO ---- */
		// Use this for kinematic only simulation. This does not use the control torques
		// at all. It simply moves the joints
		state_current_time = curr_time - last_state_stop_time;

		if (curr_time < 2) {
			continue;
		}
		// Move drills
		base->getJoint("jdrill1-rz")->setPos(curr_time*0.3);
		base->getJoint("jdrill2-rz")->setPos(curr_time*0.3);
		base->getJoint("jdrill3-rz")->setPos(curr_time*0.3);

		if (robot_state == EngageRearCams) {
			if (base->getJoint("jrear_bcam1")->getPos() > 1.4) {
				if (forecams_engaged) {
					robot_state = DisengageForeCams;
					fore_cam1->setMaterial(cam_disengaged_mat);
					fore_cam2->setMaterial(cam_disengaged_mat);
					fore_cam3->setMaterial(cam_disengaged_mat);
					fore_cam4->setMaterial(cam_disengaged_mat);
				} else {
					robot_state = MovingForeBody;
				}
				num_drill_cycles = 0;
				last_state_stop_time = curr_time;
				// change cam color
				rear_cam1->setMaterial(cam_engaged_mat);
				rear_cam2->setMaterial(cam_engaged_mat);
				rear_cam3->setMaterial(cam_engaged_mat);
				rear_cam4->setMaterial(cam_engaged_mat);
				forecams_engaged = false;
			} else {
				base->getJoint("jrear_bcam1")->setPos(state_current_time*(0.0 - (-1.4))/2.0);
				base->getJoint("jrear_bcam2")->setPos(state_current_time*(0.0 - (1.4))/2.0);
				base->getJoint("jrear_bcam3")->setPos(state_current_time*(0.0 - (-0.7))/2.0);
				base->getJoint("jrear_bcam4")->setPos(state_current_time*(0.0 - (0.7))/2.0);
			}
		}
		else if (robot_state == DisengageForeCams) {
			if (base->getJoint("jfront_bcam1")->getPos() < 0.0) {
				forecams_engaged = false;
				robot_state = MovingForeBody;
				last_state_stop_time = curr_time;
			} else {
				base->getJoint("jfront_bcam1")->setPos(1.4 - state_current_time*(0.0 - (-1.4))/2.0);
				base->getJoint("jfront_bcam2")->setPos(-1.4 - state_current_time*(0.0 - (1.4))/2.0);
				base->getJoint("jfront_bcam3")->setPos(0.7 - state_current_time*(0.0 - (-0.7))/2.0);
				base->getJoint("jfront_bcam4")->setPos(-0.7 - state_current_time*(0.0 - (0.7))/2.0);
			}
		}
		else if (robot_state == MovingForeBody) {
			if (num_drill_cycles > 2 && base->getJoint("jbody-pitch")->getPos() > 0.0) {
				robot_state = EngageForeCams;
				last_state_stop_time = curr_time;
			} else {
				const double T = 6.0;
				base->getJoint("jbody-pitch")->setPos(0.16*sin(state_current_time*(2.0*M_PI)/(T)));
				if (state_current_time > T) { ++num_drill_cycles; }
				base->getJoint("jbody-extend")->setPos((-0.02)*state_current_time/T);
			}
		}
		else if (robot_state == EngageForeCams) {
			if (base->getJoint("jfront_bcam1")->getPos() > 1.4) {
				robot_state = DisengageRearCams;
				last_state_stop_time = curr_time;
				last_rear_body_pos = base->getJoint("j1")->getPos();
				forecams_engaged = true;
				// change cam color
				fore_cam1->setMaterial(cam_engaged_mat);
				fore_cam2->setMaterial(cam_engaged_mat);
				fore_cam3->setMaterial(cam_engaged_mat);
				fore_cam4->setMaterial(cam_engaged_mat);

				rear_cam1->setMaterial(cam_disengaged_mat);
				rear_cam2->setMaterial(cam_disengaged_mat);
				rear_cam3->setMaterial(cam_disengaged_mat);
				rear_cam4->setMaterial(cam_disengaged_mat);
			} else {
				base->getJoint("jfront_bcam1")->setPos(state_current_time*(0.0 - (-1.4))/2.0);
				base->getJoint("jfront_bcam2")->setPos(state_current_time*(0.0 - (1.4))/2.0);
				base->getJoint("jfront_bcam3")->setPos(state_current_time*(0.0 - (-0.7))/2.0);
				base->getJoint("jfront_bcam4")->setPos(state_current_time*(0.0 - (0.7))/2.0);
			}
		}
		else if (robot_state == DisengageRearCams) {
			if (base->getJoint("jrear_bcam1")->getPos() < 0.0) {
				robot_state = MovingRearBody;
				last_state_stop_time = curr_time;
			} else {
				base->getJoint("jrear_bcam1")->setPos(1.4 - state_current_time*(0.0 - (-1.4))/2.0);
				base->getJoint("jrear_bcam2")->setPos(-1.4 - state_current_time*(0.0 - (1.4))/2.0);
				base->getJoint("jrear_bcam3")->setPos(0.7 - state_current_time*(0.0 - (-0.7))/2.0);
				base->getJoint("jrear_bcam4")->setPos(-0.7 - state_current_time*(0.0 - (0.7))/2.0);
			}
		}
		else if (robot_state == MovingRearBody) {
			if (base->getJoint("jbody-extend")->getPos() > 0.0) {
				robot_state = EngageRearCams;
				last_state_stop_time = curr_time;
			} else {
				const double T = 4.0;
				base->getJoint("jbody-extend")->setPos(-0.02 - (-0.02)*state_current_time/T);
				base->getJoint("j1")->setPos(last_rear_body_pos + (-0.02)*state_current_time/T);
			}
		}

		/* ---- END: KINEMATIC SIMULATION FOR DEMO ---- */

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - Mining Sim", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
    if ((key == 'P' || key == 'p') && action == GLFW_PRESS)
    {
        // pause simulation
        f_global_sim_pause = !f_global_sim_pause;
    }
    if ((key == '1') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_front";
        f_camera_view_changed = true;
    }
    if ((key == '2') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_side";
        f_camera_view_changed = true;
    }
    if ((key == '3') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_top";
        f_camera_view_changed = true;
    }
    if ((key == '4') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_isometric";
        f_camera_view_changed = true;
    }
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}
