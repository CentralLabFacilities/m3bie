///////////////////////////////////////////////////////
//These names should match the create_xxx() and destroy_xxx() function names.
//They should also match the names used for component definition in m3_config.yml 
#define JOINTS_CONTROLLER_NAME "joints_controller"
#define SIN_CONTROLLER_NAME "sin_controller"
//#define CART_CONTROLLER_NAME "cart_controller"
#define TORQUE_CONTROLLER_NAME "torque_controller"

#include <stdio.h>
#include <m3rt/base/component.h>
#ifdef JOINTS_CONTROLLER_NAME
	#include <joints_controller/joints_controller.h>
#endif
#ifdef SIN_CONTROLLER_NAME
	#include <sin_controller/sin_controller.h>
#endif
#ifdef CART_CONTROLLER_NAME
	#include <cart_controller/cart_controller.h>
#endif
#ifdef TORQUE_CONTROLLER_NAME
	#include <torque_controller/torque_controller.h>
#endif

///////////////////////////////////////////////////////
extern "C" 
{
///////////////////////////////////////////////////////
//Creators and Deletors
#ifdef JOINTS_CONTROLLER_NAME
m3rt::M3Component* create_joints_controller(){return new m3_controllers::JointsController;}
void destroy_joints_controller(m3rt::M3Component* c) {delete c;}
#endif

#ifdef SIN_CONTROLLER_NAME
m3rt::M3Component* create_sin_controller(){return new m3_controllers::SinController;}
void destroy_sin_controller(m3rt::M3Component* c) {delete c;}
#endif

#ifdef CART_CONTROLLER_NAME
m3rt::M3Component* create_cart_controller(){return new m3_controllers::CartController;}
void destroy_cart_controller(m3rt::M3Component* c) {delete c;}
#endif

#ifdef TORQUE_CONTROLLER_NAME
m3rt::M3Component* create_torque_controller(){return new m3_controllers::TorqueController;}
void destroy_torque_controller(m3rt::M3Component* c) {delete c;}
#endif

///////////////////////////////////////////////////////
class M3FactoryProxy 
{ 
public:
	M3FactoryProxy()
	{	
	#ifdef JOINTS_CONTROLLER_NAME
		m3rt::creator_factory[JOINTS_CONTROLLER_NAME] = create_joints_controller;
		m3rt::destroyer_factory[JOINTS_CONTROLLER_NAME] = destroy_joints_controller;
	#endif
	#ifdef SIN_CONTROLLER_NAME
		m3rt::creator_factory[SIN_CONTROLLER_NAME] = create_sin_controller;
		m3rt::destroyer_factory[SIN_CONTROLLER_NAME] = destroy_sin_controller;
	#endif
	#ifdef CART_CONTROLLER_NAME
		m3rt::creator_factory[CART_CONTROLLER_NAME] = create_cart_controller;
		m3rt::destroyer_factory[CART_CONTROLLER_NAME] = destroy_cart_controller;
	#endif
	#ifdef TORQUE_CONTROLLER_NAME
		m3rt::creator_factory[TORQUE_CONTROLLER_NAME] = create_torque_controller;
		m3rt::destroyer_factory[TORQUE_CONTROLLER_NAME] = destroy_torque_controller;
	#endif
	}
};
///////////////////////////////////////////////////////
// The library's one instance of the proxy
M3FactoryProxy proxy;
}
