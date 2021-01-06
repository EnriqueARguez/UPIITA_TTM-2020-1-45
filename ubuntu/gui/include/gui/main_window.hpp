/**
 * @file /include/gui/main_window.hpp
 *
 * @brief Qt based gui for gui.
 *
 * @date November 2010
 **/
#ifndef gui_MAIN_WINDOW_H
#define gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
        *******************************************/
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
        /******************************************
        ** Personales
        *******************************************/
        void on_BotonIniciar_clicked();
        void on_BotonDetener_clicked();
        void on_RBPuntos_clicked();
        void on_RBTrayectorias_clicked();
        void on_RBR1_clicked();
        void on_RBR2_clicked();
        void on_RBR3_clicked();
        void on_RBT1_clicked();
        void on_RBT2_clicked();
        void on_RBT3_clicked();
        void on_ValorPosX_valueChanged(double d);
        void on_ValorPosY_valueChanged(double d);
        void on_ActualizaH_clicked(bool check);
        void on_ActualizaED_clicked(bool check);
        void on_ActualizaEA_clicked(bool check);
        void on_ActualizaPos_clicked(bool check);
    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void ActualizaPosiciones(double xr1,double yr1,double zr1,double xr2,double yr2,double zr2,double xr3,double yr3,double zr3);
    void ActualizaHist();
    void ActualizaErrorD();
    void ActualizaErrorA();
    void ActualizaPos();
    void ActualizaFlagLoc(bool FlagSisLoc);
    void ActualizaFlagForm(bool FlagSisForm);
    void ActualizaFlagNav(bool FlagSisNav);
    void ActualizaFlagNavEnd(bool FlagSisNavEnd);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
        bool Iniciar;
        int Trayectoria;
        float PosicionX;
        float PosicionY;
        std::vector<double> xhist;
        std::vector<double> xt1,xt2,xt3;
        std::vector<double> yt1,yt2,yt3;
        double PuntoX, PuntoY, PosXL, PosYL;
        std::vector<double> xp;
        std::vector<double> yp;
        std::vector<double> xICPb;
        std::vector<double> yICPb;
        std::vector<double> xR1,xR2,xR3;
        std::vector<double> yR1,yR2,yR3;
        std::vector<double> xe;


};

}  // namespace gui

#endif // gui_MAIN_WINDOW_H
