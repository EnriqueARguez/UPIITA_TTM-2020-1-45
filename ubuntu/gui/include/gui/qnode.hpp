/**
 * @file /include/gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef gui_QNODE_HPP_
#define gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseWithCovariance.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
        QNode(int argc, char** argv );
        virtual ~QNode();
        bool init();
        bool init(const std::string &master_url, const std::string &host_url);
        void IniciarNodos();
        void run();

        /*********************
        ** Logging
        **********************/
        enum LogLevel {
                 Debug,
                 Info,
                 Warn,
                 Error,
                 Fatal
         };

        /*********************
        ** variables
        **********************/
        double xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3;
        std::string rol1,rol2,rol3;
        std::vector<double> distanciasR1,distanciasR2,distanciasR3;
        std::vector<double> MapxR1,MapxR2,MapxR3;
        std::vector<double> MapyR1,MapyR2,MapyR3;
        std::vector<double> errordistR1,errordistR2,errordistR3;
        std::vector<double> errorangR1,errorangR2,errorangR3;
        std::vector<double> lastpxR1,lastpxR2,lastpxR3;
        std::vector<double> lastpyR1,lastpyR2,lastpyR3;
        bool FlagSisLoc;
        bool FlagSisForm;
        bool FlagSisNav;
        bool FlagSisNavEnd;

        QStringListModel* loggingModel() { return &logging_model; }
        void log( const LogLevel &level, const std::string &msg);
        void MostrarInfo(const std::string texto);
        void InitSistema(bool estado);
        void DefinirTrayectoria(int num);
        void DefinirPunto(float posx, float posy);
        void DefinirTipoNavegacion(bool estado);
        void refposeR1Callback(const geometry_msgs::PoseWithCovariance::ConstPtr& msg);
        void refposeR2Callback(const geometry_msgs::PoseWithCovariance::ConstPtr& msg);
        void refposeR3Callback(const geometry_msgs::PoseWithCovariance::ConstPtr& msg);
        void RolR1Callback(const std_msgs::String::ConstPtr& msg);
        void RolR2Callback(const std_msgs::String::ConstPtr& msg);
        void RolR3Callback(const std_msgs::String::ConstPtr& msg);
        void histogramR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void histogramR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void histogramR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void ICPxR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void ICPxR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void ICPxR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void ICPyR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void ICPyR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void ICPyR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void errorDistR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void errorDistR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void errorDistR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void errorAngR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void errorAngR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void errorAngR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void LastPosxR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void LastPosxR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void LastPosxR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void LastPosyR1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void LastPosyR2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void LastPosyR3Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void SisLocCallback(const std_msgs::Bool::ConstPtr& msg);
        void SisFormCallback(const std_msgs::Bool::ConstPtr& msg);
        void SisNavCallback(const std_msgs::Bool::ConstPtr& msg);
        void SisNavEndCallback(const std_msgs::Bool::ConstPtr& msg);

Q_SIGNALS:
        void loggingUpdated();
    void rosShutdown();
    void PosicionesUpdate(double xr1,double yr1,double zr1,double xr2,double yr2,double zr2,double xr3,double yr3,double zr3);
    void HistogramasUpdate();
    void ErrorDistUpdate();
    void ErrorAngUpdate();
    void PosUpdate();
    void SisLocFlag(bool FlagSisLoc);
    void SisFormFlag(bool FlagSisForm);
    void SisNavFlag(bool FlagSisNav);
    void SisNavEndFlag(bool FlagSisNavEnd);

private:
        int init_argc;
        char** init_argv;
        // topicos
        ros::Publisher chatter_publisher;
        ros::Publisher IniciarSistema_publisher;
        ros::Publisher DefinirTrayectoria_publisher;
        ros::Publisher DefinirPosicionX_publisher;
        ros::Publisher DefinirPosicionY_publisher;
        ros::Publisher TipoNavegacion_publisher;
        ros::Subscriber RefposeR1;
        ros::Subscriber RefposeR2;
        ros::Subscriber RefposeR3;
        ros::Subscriber RolR1;
        ros::Subscriber RolR2;
        ros::Subscriber RolR3;
        ros::Subscriber histogramaR1;
        ros::Subscriber histogramaR2;
        ros::Subscriber histogramaR3;
        ros::Subscriber ICPxR1;
        ros::Subscriber ICPxR2;
        ros::Subscriber ICPxR3;
        ros::Subscriber ICPyR1;
        ros::Subscriber ICPyR2;
        ros::Subscriber ICPyR3;
        ros::Subscriber ErrorDistR1;
        ros::Subscriber ErrorDistR2;
        ros::Subscriber ErrorDistR3;
        ros::Subscriber ErrorAngR1;
        ros::Subscriber ErrorAngR2;
        ros::Subscriber ErrorAngR3;
        ros::Subscriber LastPosxR1;
        ros::Subscriber LastPosxR2;
        ros::Subscriber LastPosxR3;
        ros::Subscriber LastPosyR1;
        ros::Subscriber LastPosyR2;
        ros::Subscriber LastPosyR3;
        ros::Subscriber SisLoc;
        ros::Subscriber SisForm;
        ros::Subscriber SisNav;
        ros::Subscriber SisNavEnd;

        // variables
        std_msgs::Bool Iniciar;
        std_msgs::Bool TipoNav;
        std_msgs::Int8 Trayectoria;
        std_msgs::Float32 PosicionX, PosicionY;



    QStringListModel logging_model;
};

}  // namespace gui

#endif /* gui_QNODE_HPP_ */
