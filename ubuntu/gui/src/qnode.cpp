/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <sstream>
#include <iostream>
#include "../include/gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"gui");
	if ( ! ros::master::check() ) {
		return false;
	}
    IniciarNodos();
    FlagSisLoc=false;
    FlagSisForm=false;
    FlagSisNav=false;
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"gui");
	if ( ! ros::master::check() ) {
		return false;
	}
    IniciarNodos();
	return true;
}

void QNode::IniciarNodos(){
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    IniciarSistema_publisher = n.advertise<std_msgs::Bool>("IniciarSistema", 1000);
    DefinirTrayectoria_publisher = n.advertise<std_msgs::Int8>("NumeroTrayectoria", 1000);
    DefinirPosicionX_publisher = n.advertise<std_msgs::Float32>("PosicionX", 1000);
    DefinirPosicionY_publisher = n.advertise<std_msgs::Float32>("PosicionY", 1000);
    TipoNavegacion_publisher = n.advertise<std_msgs::Bool>("OpcionNavegacion", 1000);
    RefposeR1=n.subscribe("/Robot1/refpose", 1000, &QNode::refposeR1Callback, this);
    RefposeR2=n.subscribe("/Robot2/refpose", 1000, &QNode::refposeR2Callback, this);
    RefposeR3=n.subscribe("/Robot3/refpose", 1000, &QNode::refposeR3Callback, this);
    RolR1=n.subscribe("/Robot1/role", 1, &QNode::RolR1Callback, this);
    RolR2=n.subscribe("/Robot2/role", 1, &QNode::RolR2Callback, this);
    RolR3=n.subscribe("/Robot3/role", 1, &QNode::RolR3Callback, this);
    histogramaR1=n.subscribe("/Robot1/hist", 1000, &QNode::histogramR1Callback, this);
    histogramaR2=n.subscribe("/Robot2/hist", 1000, &QNode::histogramR2Callback, this);
    histogramaR3=n.subscribe("/Robot3/hist", 1000, &QNode::histogramR3Callback, this);
    ICPxR1=n.subscribe("/Robot1/map_icpx", 1, &QNode::ICPxR1Callback, this);
    ICPxR2=n.subscribe("/Robot2/map_icpx", 1, &QNode::ICPxR2Callback, this);
    ICPxR3=n.subscribe("/Robot3/map_icpx", 1, &QNode::ICPxR3Callback, this);
    ICPyR1=n.subscribe("/Robot1/map_icpy", 1, &QNode::ICPyR1Callback, this);
    ICPyR2=n.subscribe("/Robot2/map_icpy", 1, &QNode::ICPyR2Callback, this);
    ICPyR3=n.subscribe("/Robot3/map_icpy", 1, &QNode::ICPyR3Callback, this);
    ErrorDistR1=n.subscribe("/Robot1/error_d", 1000, &QNode::errorDistR1Callback, this);
    ErrorDistR2=n.subscribe("/Robot2/error_d", 1000, &QNode::errorDistR2Callback, this);
    ErrorDistR3=n.subscribe("/Robot3/error_d", 1000, &QNode::errorDistR3Callback, this);
    ErrorAngR1=n.subscribe("/Robot1/error_a", 1000, &QNode::errorAngR1Callback, this);
    ErrorAngR2=n.subscribe("/Robot2/error_a", 1000, &QNode::errorAngR2Callback, this);
    ErrorAngR3=n.subscribe("/Robot3/error_a", 1000, &QNode::errorAngR3Callback, this);
    LastPosxR1=n.subscribe("/Robot1/last_pos_x", 1000, &QNode::LastPosxR1Callback, this);
    LastPosxR2=n.subscribe("/Robot2/last_pos_x", 1000, &QNode::LastPosxR2Callback, this);
    LastPosxR3=n.subscribe("/Robot3/last_pos_x", 1000, &QNode::LastPosxR3Callback, this);
    LastPosyR1=n.subscribe("/Robot1/last_pos_y", 1000, &QNode::LastPosyR1Callback, this);
    LastPosyR2=n.subscribe("/Robot2/last_pos_y", 1000, &QNode::LastPosyR2Callback, this);
    LastPosyR3=n.subscribe("/Robot3/last_pos_y", 1000, &QNode::LastPosyR3Callback, this);
    SisLoc=n.subscribe("/sys_loc", 1, &QNode::SisLocCallback, this);
    SisForm=n.subscribe("/sys_form", 1, &QNode::SisFormCallback, this);
    SisNav=n.subscribe("/sys_nav", 1, &QNode::SisNavCallback, this);
    SisNavEnd=n.subscribe("/finish_flag", 1, &QNode::SisNavEndCallback, this);
    start();
}

/*****************************************************************************
** Funciones Callback
*****************************************************************************/

void QNode::refposeR1Callback(const geometry_msgs::PoseWithCovariance::ConstPtr& msg)
{
    xr1=msg->pose.position.x;
    yr1=msg->pose.position.y;
    zr1=msg->pose.orientation.z;
}

void QNode::refposeR2Callback(const geometry_msgs::PoseWithCovariance::ConstPtr& msg)
{
    xr2=msg->pose.position.x;
    yr2=msg->pose.position.y;
    zr2=msg->pose.orientation.z;
}

void QNode::refposeR3Callback(const geometry_msgs::PoseWithCovariance::ConstPtr& msg)
{
    xr3=msg->pose.position.x;
    yr3=msg->pose.position.y;
    zr3=msg->pose.orientation.z;
}

void QNode::RolR1Callback(const std_msgs::String::ConstPtr& msg)
{
    rol1=msg->data.c_str();
}

void QNode::RolR2Callback(const std_msgs::String::ConstPtr& msg)
{
    rol2=msg->data.c_str();
}

void QNode::RolR3Callback(const std_msgs::String::ConstPtr& msg)
{
    rol3=msg->data.c_str();
}

void QNode::histogramR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    distanciasR1=msg->data;
    Q_EMIT HistogramasUpdate();
}

void QNode::histogramR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    distanciasR2=msg->data;
    Q_EMIT HistogramasUpdate();
}

void QNode::histogramR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    distanciasR3=msg->data;
    Q_EMIT HistogramasUpdate();
}

void QNode::ICPxR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    MapxR1=msg->data;
}

void QNode::ICPxR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    MapxR2=msg->data;
}

void QNode::ICPxR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    MapxR3=msg->data;
}

void QNode::ICPyR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    MapyR1=msg->data;
}

void QNode::ICPyR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    MapyR2=msg->data;
}

void QNode::ICPyR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    MapyR3=msg->data;
}

void QNode::errorDistR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    errordistR1=msg->data;
    Q_EMIT ErrorDistUpdate();
}

void QNode::errorDistR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    errordistR2=msg->data;
    Q_EMIT ErrorDistUpdate();
}

void QNode::errorDistR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    errordistR3=msg->data;
    Q_EMIT ErrorDistUpdate();
}

void QNode::errorAngR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    errorangR1=msg->data;
    Q_EMIT ErrorAngUpdate();
}

void QNode::errorAngR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    errorangR2=msg->data;
    Q_EMIT ErrorAngUpdate();
}

void QNode::errorAngR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    errorangR3=msg->data;
    Q_EMIT ErrorAngUpdate();
}

void QNode::LastPosxR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    lastpxR1=msg->data;
    Q_EMIT PosUpdate();
}

void QNode::LastPosxR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    lastpxR2=msg->data;
    Q_EMIT PosUpdate();
}

void QNode::LastPosxR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    lastpxR3=msg->data;
    Q_EMIT PosUpdate();
}

void QNode::LastPosyR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    lastpyR1=msg->data;
    Q_EMIT PosUpdate();
}

void QNode::LastPosyR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    lastpyR2=msg->data;
    Q_EMIT PosUpdate();
}

void QNode::LastPosyR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    lastpyR3=msg->data;
    Q_EMIT PosUpdate();
}

void QNode::SisLocCallback(const std_msgs::Bool::ConstPtr& msg){
    FlagSisLoc=msg->data;
    Q_EMIT SisLocFlag(FlagSisLoc);
}

void QNode::SisFormCallback(const std_msgs::Bool::ConstPtr& msg){
    FlagSisForm=msg->data;
    Q_EMIT SisFormFlag(FlagSisForm);
}

void QNode::SisNavCallback(const std_msgs::Bool::ConstPtr& msg){
    FlagSisNav=msg->data;
    Q_EMIT SisNavFlag(FlagSisNav);
}

void QNode::SisNavEndCallback(const std_msgs::Bool::ConstPtr& msg){
    FlagSisNavEnd=msg->data;
    Q_EMIT SisNavEndFlag(FlagSisNavEnd);
}
/*****************************************************************************
** Funciones predefinidas
*****************************************************************************/

void QNode::run() {
    ros::Rate loop_rate(10);
    int count = 0, contadorpruebas=10;
    while ( ros::ok() && count<=contadorpruebas) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Mensaje de prueba " << count;
        msg.data = ss.str();
        chatter_publisher.publish(msg);
        log(Info,std::string(" ")+msg.data);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    while ( ros::ok() ){
        Q_EMIT PosicionesUpdate(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3);
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

/*****************************************************************************
** Funciones propias
*****************************************************************************/

void QNode::InitSistema(bool estado)
{
    Iniciar.data=estado;
    IniciarSistema_publisher.publish(Iniciar);
    //Imprimir dato
    std::stringstream ss;
    std::string texto;
    ss << "Iniciar sistema: " << std::boolalpha << estado;
    texto=ss.str();
    MostrarInfo(texto);
}

void QNode::DefinirTrayectoria(int num){
    Trayectoria.data=num;
    DefinirTrayectoria_publisher.publish(Trayectoria);
    //Imprimir dato
    std::stringstream ss;
    std::string texto;
    ss << "Trayectoria: " << num;
    texto=ss.str();
    MostrarInfo(texto);
}

void QNode::DefinirPunto(float posx, float posy){
    PosicionX.data=posx;
    PosicionY.data=posy;
    DefinirPosicionX_publisher.publish(PosicionX);
    DefinirPosicionY_publisher.publish(PosicionY);
    //Imprimir dato
    std::stringstream ssx,ssy;
    std::string textox,textoy;
    ssx << "Punto en x: " << posx;
    textox=ssx.str();
    MostrarInfo(textox);
    ssy << "Punto en y: " << posy;
    textoy=ssy.str();
    MostrarInfo(textoy);
}

void QNode::MostrarInfo(const std::string texto){
    std_msgs::String msg;
    msg.data=texto;
    chatter_publisher.publish(msg);
    log(Debug,std::string(" ")+msg.data);
}


void QNode::DefinirTipoNavegacion(bool estado)
{
    TipoNav.data=estado;
    TipoNavegacion_publisher.publish(TipoNav);
    //Imprimir dato
    std::stringstream ss;
    std::string texto;
    ss << "Tipo de navegacion: " << std::boolalpha << estado;
    texto=ss.str();
    MostrarInfo(texto);
}


}  // namespace gui
