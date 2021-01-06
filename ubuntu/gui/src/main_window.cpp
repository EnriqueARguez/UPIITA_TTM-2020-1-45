/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <qmessagebox.h>
#include "../include/gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    Iniciar=false;
    Trayectoria=0;
    ui.ActualizaH->setVisible(false);
    ui.ActualizaED->setVisible(false);
    ui.ActualizaEA->setVisible(false);
    ui.ActualizaPos->setVisible(false);
    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Señales actualizar
    **********************/

    QObject::connect(&qnode, SIGNAL(PosicionesUpdate(double,double,double,double,double,double,double,double,double)),this,SLOT(ActualizaPosiciones(double,double,double,double,double,double,double,double,double)));
    QObject::connect(&qnode, SIGNAL(HistogramasUpdate()),this,SLOT(ActualizaHist()));
    QObject::connect(&qnode,SIGNAL(ErrorDistUpdate()),this,SLOT(ActualizaErrorD()));
    QObject::connect(&qnode,SIGNAL(ErrorAngUpdate()),this,SLOT(ActualizaErrorA()));
    QObject::connect(&qnode,SIGNAL(PosUpdate()),this,SLOT(ActualizaPos()));
    QObject::connect(&qnode, SIGNAL(SisLocFlag(bool)),this,SLOT(ActualizaFlagLoc(bool)));
    QObject::connect(&qnode, SIGNAL(SisFormFlag(bool)),this,SLOT(ActualizaFlagForm(bool)));
    QObject::connect(&qnode, SIGNAL(SisNavFlag(bool)),this,SLOT(ActualizaFlagNav(bool)));
    QObject::connect(&qnode, SIGNAL(SisNavEndFlag(bool)),this,SLOT(ActualizaFlagNavEnd(bool)));
    /*********************
    ** Gráficas
    **********************/
    ui.GraficaTrayectoria->setAxisTitle(QwtPlot::xBottom, "x [m]");
    ui.GraficaTrayectoria->setAxisTitle(QwtPlot::yLeft, "y [m]");
    ui.GraficaTrayectoria->setAxisScale(QwtPlot::xBottom, -1.25,1.25);
    ui.GraficaTrayectoria->setAxisScale(QwtPlot::yLeft,-1,1);

    ui.GraficaTrayectoriaRobots->setAxisTitle(QwtPlot::xBottom, "x [m]");
    ui.GraficaTrayectoriaRobots->setAxisTitle(QwtPlot::yLeft, "y [m]");
    ui.GraficaTrayectoriaRobots->setAxisScale(QwtPlot::xBottom, -1.25,1.25);
    ui.GraficaTrayectoriaRobots->setAxisScale(QwtPlot::yLeft,-1,1);

    ui.GraficaHistograma->setAxisTitle(QwtPlot::xBottom, tr("Grados [ %1 ]").arg(QChar(0260)));
    ui.GraficaHistograma->setAxisTitle(QwtPlot::yLeft, "Distancias [m]");
    ui.GraficaHistograma->setAxisScale(QwtPlot::xBottom, 0,359);
    ui.GraficaHistograma->setAxisScale(QwtPlot::yLeft, 0,1);

    ui.GraficaICP->setAxisTitle(QwtPlot::xBottom, "x [m]");
    ui.GraficaICP->setAxisTitle(QwtPlot::yLeft, "y [m]");
    ui.GraficaICP->setAxisScale(QwtPlot::xBottom, -1.4,1.4);
    ui.GraficaICP->setAxisScale(QwtPlot::yLeft,-1.15,1.15);

    ui.GraficaAngular->setAxisTitle(QwtPlot::xBottom, "Tiempo [x0.1 s]");
    ui.GraficaAngular->setAxisTitle(QwtPlot::yLeft, tr("Angulo [rad]"));
    ui.GraficaAngular->setAxisScale(QwtPlot::xBottom, 0,100);
    ui.GraficaAngular->setAxisScale(QwtPlot::yLeft,-3.15,3.15);

    ui.GraficaLineal->setAxisTitle(QwtPlot::xBottom, "Tiempo [x0.1 s]");
    ui.GraficaLineal->setAxisTitle(QwtPlot::yLeft, "Distancia [m]");
    ui.GraficaLineal->setAxisScale(QwtPlot::xBottom, 0,100);
    ui.GraficaLineal->setAxisScale(QwtPlot::yLeft,0,3.2);

    for (int i=0;i<=359;i++)
    {
        xhist.push_back(i);
    }
    double x1[]={0.8, -0.7, -0.7, 0.8, 0.8};
    double y1[]={0.75, 0.75, -0.7, -0.7, 0.75};
    for (int i=0;i<=4;i++)
    {
        xt1.push_back(x1[i]);
        yt1.push_back(y1[i]);
    }
    double x2[]={0.8, -0.7, -0.7, 0.8, 0.8, -0.7};
    double y2[]={0.75, 0.75, 0.0, 0.0, -0.7, -0.7};
    for (int i=0;i<=5;i++)
    {
        xt2.push_back(x2[i]);
        yt2.push_back(y2[i]);
    }
    double x3[]={0.8, -0.7, -0.7, 0.8, 0.8, 0.0};
    double y3[]={0.75, 0.75, -0.7, -0.7, 0.0, 0.0};
    for (int i=0;i<=5;i++)
    {
        xt3.push_back(x3[i]);
        yt3.push_back(y3[i]);
    }
    double xb[]={1.20700002 ,1.20981575 ,1.20126780 ,1.20135128 ,1.21004525 ,1.22233089 ,1.17552492 ,1.22281685 ,1.22298108 ,1.23658578 ,1.22214648 ,1.21132791 ,1.23637862 ,1.22283443 ,1.20704784 ,1.21996433 ,1.21791853 ,1.22598265 ,1.22686287 ,1.23484725 ,1.21690190 ,1.21178737 ,1.23315457 ,1.22427149 ,1.21318832 ,1.22532813 ,1.22685388 ,1.25364613 ,1.22023355 ,1.24895692 ,1.23755032 ,1.21289170 ,1.22882169 ,1.23703911 ,1.24853062 ,1.23855786 ,1.26125751 ,1.24267685 ,1.21038453 ,1.13774168 ,1.10616818 ,1.04678216 ,1.03148503 ,1.02901462 ,0.96607339 ,0.95317998 ,0.89124668 ,0.86750188 ,0.85046502 ,0.82072988 ,0.80027058 ,0.75644312 ,0.74371903 ,0.70532724 ,0.69358657 ,0.66305437 ,0.63412472 ,0.61925456 ,0.58450097 ,0.57632758 ,0.54750001 ,0.52068553 ,0.50749875 ,0.48304590 ,0.45634437 ,0.43740989 ,0.42137917 ,0.41495647 ,0.38809244 ,0.36195163 ,0.35262277 ,0.32914942 ,0.31797850 ,0.29471068 ,0.27646427 ,0.25856023 ,0.24167998 ,0.22427620 ,0.20728795 ,0.19004576 ,0.17399547 ,0.15737308 ,0.13625046 ,0.11931008 ,0.10128808 ,0.08497685 ,0.06836135 ,0.05155092 ,0.03406191 ,0.01703355 ,0.00000000 ,-0.01706845 ,-0.03416661 ,-0.05139391 ,-0.06898915 ,-0.08767868 ,-0.10421488 ,-0.12150373 ,-0.13986897 ,-0.15659091 ,-0.17382183 ,-0.19290790 ,-0.21144618 ,-0.22922514 ,-0.24603456 ,-0.26399542 ,-0.28583592 ,-0.29851152 ,-0.32199573 ,-0.33533519 ,-0.35433287 ,-0.36732714 ,-0.39745759 ,-0.41730083 ,-0.43846208 ,-0.45980869 ,-0.48308500 ,-0.51210129 ,-0.52909444 ,-0.54686526 ,-0.56699997 ,-0.61444040 ,-0.61841581 ,-0.64485263 ,-0.67494584 ,-0.69288030 ,-0.73120483 ,-0.75347239 ,-0.77450215 ,-0.81182328 ,-0.85169361 ,-0.85812519 ,-0.89797328 ,-0.94524975 ,-0.95515526 ,-1.00621293 ,-0.98981160 ,-0.99244697 ,-1.00696125 ,-0.99923548 ,-0.98743131 ,-1.04137561 ,-1.08193878 ,-1.13406239 ,-1.16013038 ,-1.17138738 ,-1.16396879 ,-1.17497748 ,-1.13214424 ,-1.15803298 ,-1.16307216 ,-1.16586802 ,-1.16460788 ,-1.13959738 ,-1.16393825 ,-1.17004336 ,-1.15928923 ,-1.15615413 ,-1.14970799 ,-1.15950687 ,-1.17273641 ,-1.15920573 ,-1.16789744 ,-1.16764807 ,-1.17946810 ,-1.18325916 ,-1.15853310 ,-1.18580838 ,-1.18062417 ,-1.17108119 ,-1.16699713 ,-1.18028762 ,-1.17247741 ,-1.17120441 ,-1.17751394 ,-1.19244508 ,-1.20006950 ,-1.16440205 ,-1.20426599 ,-1.19781749 ,-1.19200003 ,-1.19981728 ,-1.17228542 ,-1.18537329 ,-1.18909638 ,-1.19642981 ,-1.18049753 ,-1.19006284 ,-1.18535090 ,-1.20201672 ,-1.19555660 ,-1.20445655 ,-1.21094676 ,-1.18385966 ,-1.20122614 ,-1.20740728 ,-1.21887984 ,-1.19538094 ,-1.18596746 ,-1.18568025 ,-1.18871115 ,-1.20618596 ,-1.18864965 ,-1.19665627 ,-1.20039874 ,-1.20720199 ,-1.21337198 ,-1.20553187 ,-1.19639401 ,-1.17636353 ,-1.19944518 ,-1.21117739 ,-1.23221384 ,-1.21774968 ,-1.21288202 ,-1.21398332 ,-1.22566073 ,-1.21951638 ,-1.20014038 ,-1.17815330 ,-1.14906666 ,-1.09131004 ,-1.09465236 ,-0.99171558 ,-0.96823134 ,-0.96237229 ,-0.92042237 ,-0.88727987 ,-0.84845761 ,-0.82335408 ,-0.81376913 ,-0.77469337 ,-0.75603232 ,-0.73180704 ,-0.70122778 ,-0.67509949 ,-0.65481489 ,-0.62306709 ,-0.60781737 ,-0.58302309 ,-0.55750000 ,-0.53232099 ,-0.51547980 ,-0.49530362 ,-0.47607105 ,-0.45220156 ,-0.43398801 ,-0.41339352 ,-0.39371153 ,-0.38417045 ,-0.35296477 ,-0.34607894 ,-0.31859653 ,-0.30085050 ,-0.28638722 ,-0.26270133 ,-0.24748610 ,-0.23102472 ,-0.21103036 ,-0.19157222 ,-0.17364818 ,-0.15784237 ,-0.13708551 ,-0.12004130 ,-0.10452846 ,-0.08750436 ,-0.07003550 ,-0.05212661 ,-0.03475990 ,-0.01748731 ,-0.00000000 ,0.01750476 ,0.03500420 ,0.05217895 ,0.06954720 ,0.08741721 ,0.10578280 ,0.12430673 ,0.14209574 ,0.15737308 ,0.17885762 ,0.19710569 ,0.21040662 ,0.23169958 ,0.24700226 ,0.26528952 ,0.29134870 ,0.30728266 ,0.32446783 ,0.34021871 ,0.36151531 ,0.37556960 ,0.39970525 ,0.42159889 ,0.43805537 ,0.46276701 ,0.48527686 ,0.50438342 ,0.53238073 ,0.55413740 ,0.57349998 ,0.60929003 ,0.60675755 ,0.64485263 ,0.68165614 ,0.68886529 ,0.72709035 ,0.76370330 ,0.77634914 ,0.80112486 ,0.82855325 ,0.86993426 ,0.89128198 ,0.91865179 ,0.95723925 ,0.99348501 ,1.05023614 ,1.08167210 ,1.12140550 ,1.16376221 ,1.20498787 ,1.20146766 ,1.20565643 ,1.19156417 ,1.20786240 ,1.21480251 ,1.17889147 ,1.18923485 ,1.20846850 ,1.20432003 ,1.22022983 ,1.22534223 ,1.20698935 ,1.19573076 ,1.20348522 ,1.18998209 ,1.20588005 ,1.20217933 ,1.19606714 ,1.18097923 ,1.19622871 ,1.20269959 ,1.20879286 ,1.20398763 ,1.21695734 ,1.21320288 ,1.21578057 ,1.21796258 ,1.19823083 ,1.21230959 ,1.22214648 ,1.21485668 ,1.19129244 ,1.0991374 ,1.20038794 ,1.20838423 ,1.21503307 ,1.21732938 ,1.19427209 ,1.21481498};
    double yb[]={0.00000000,0.02111741,0.04194920,0.06296015,0.08461461,0.10694010,0.12355265,0.15014303,0.17187878,0.19585595,0.21549740,0.23545829,0.26280039,0.28231357,0.30095083,0.32688846,0.34923252,0.37482051,0.39863191,0.42519201,0.44291607,0.46516159,0.49822679,0.51967242,0.54014624,0.57137989,0.59837662,0.63876461,0.64880969,0.69230812,0.71450001,0.72877886,0.76785301,0.80334259,0.84214454,0.86724755,0.91635722,0.93642417,0.94565603,0.92132504,0.92818531,0.90995385,0.92875329,0.95957166,0.93292623,0.95317998,0.92291296,0.93028188,0.94453709,0.94414172,0.95372534,0.93412946,0.95191695,0.93600086,0.95464001,0.94693978,0.94012856,0.95356841,0.93539709,0.95917017,0.94829784,0.93934157,0.95446632,0.94803095,0.93564461,0.93802853,0.94643311,0.97757619,0.96056249,0.94291622,0.96882311,0.95591932,0.97863720,0.96395521,0.96414550,0.96495991,0.96932544,0.97144693,0.97521314,0.97770067,0.98677734,0.99361451,0.96947241,0.97170266,0.96369170,0.97128985,0.97761279,0.98365011,0.97540546,0.97585136,0.97500002,0.97785103,0.97840359,0.98065420,0.98659087,1.00217191,0.99153831,0.98956849,0.99521940,0.98867608,0.98579261,0.99242512,0.99477607,0.99288315,0.98679071,0.98524432,0.99682832,0.97638718,0.99100094,0.97388411,0.97352157,0.95691991,0.98374206,0.98309915,0.98480196,0.98606293,0.99047104,1.00505538,0.99508191,0.98657105,0.98207276,1.02260055,0.98967218,0.99298597,1.00064836,0.98953562,1.00641711,0.99989164,0.99131754,1.00251826,1.01500892,0.98716011,0.99730036,1.01365626,0.98909223,1.00621293,0.95584995,0.92547177,0.90667198,0.86862216,0.82855325,0.84328935,0.84530322,0.85457731,0.84288406,0.82021427,0.78510686,0.76303930,0.70744224,0.69581641,0.67150003,0.64625120,0.61923299,0.58065387,0.56769062,0.54560018,0.51614882,0.49075831,0.46451218,0.44509299,0.42684115,0.39914654,0.37947288,0.35698584,0.33820704,0.31705334,0.28885474,0.27376543,0.25094941,0.22763512,0.20577308,0.18693919,0.16478095,0.14380582,0.12376170,0.10432543,0.08391703,0.06102373,0.04205390,0.02090798,0.00000000,-0.02094289,-0.04093711,-0.06212278,-0.08314972,-0.10467404,-0.12407529,-0.14612134,-0.16659021,-0.19038075,-0.21080889,-0.23412264,-0.25739468,-0.27331554,-0.29949932,-0.32352381,-0.34950817,-0.36546463,-0.38534419,-0.40826245,-0.43265548,-0.46301141,-0.48024563,-0.50795045,-0.53445195,-0.56292753,-0.59180106,-0.61424917,-0.63613398,-0.65206895,-0.69250000,-0.72774879,-0.76997266,-0.79081589,-0.81809925,-0.85004027,-0.89049465,-0.91897151,-0.93765243,-0.95404973,-0.96418141,-0.94866134,-0.98562942,-0.92478974,-0.93501014,-0.96237229,-0.95312527,-0.95149117,-0.94230764,-0.94716052,-0.96981229,-0.95666664,-0.96767724,-0.97114075,-0.96515724,-0.96414199,-0.97080300,-0.95943917,-0.97271113,-0.97031336,-0.96561833,-0.96033248,-0.96947650,-0.97208808,-0.97609030,-0.96974938,-0.97475303,-0.97389411,-0.97447023,-1.00079824,-0.96976273,-1.00508621,-0.98053929,-0.98403764,-0.99875094,-0.98041470,-0.99261253,-1.00067801,-0.99281980,-0.98555364,-0.98480775,-0.99657748,-0.97541406,-0.97765797,-0.99452190,-1.00017943,-1.00155426,-0.99463501,-0.99539326,-1.00184736,-1.00000000,-1.00284726,-1.00238902,-0.99563363,-0.99457134,-0.99918330,-1.00645612,-1.01239706,-1.01106373,-0.99361451,-1.01435196,-1.01402087,-0.98988534,-1.00360114,-0.99067196,-0.99007395,-1.01605365,-1.00507630,-0.99860930,-0.98806687,-0.99325514,-0.97839227,-0.98930520,-0.99322473,-0.98388848,-0.99240705,-0.99496500,-0.98990820,-1.00126252,-0.99969033,-0.99333110,-1.01402889,-0.97101505,-0.99298597,-1.01059679,-0.98380158,-1.00075401,-1.01346850,-0.99368159,-0.98930681,-0.98743131,-1.00074488,-0.98986892,-0.98513344,-0.99125026,-0.99348501,-1.01420125,-1.00867556,-1.00971805,-1.01164306,-1.01110488,-0.97292933,-0.94196204,-0.89790800,-0.87756340,-0.85061388,-0.79517234,-0.77229814,-0.75513493,-0.72362848,-0.70450002,-0.67921829,-0.64176762,-0.60925525,-0.58697896,-0.55489776,-0.53689239,-0.51029485,-0.48324249,-0.45333545,-0.43539164,-0.41412268,-0.39276061,-0.36809596,-0.34895690,-0.32507673,-0.30312814,-0.28118882,-0.25469183,-0.23564911,-0.21549740,-0.19241439,-0.16742523,-0.14855873,-0.12616586,-0.10571992,-0.08496339,-0.06379753,-0.04170490,-0.02120467};
    for (int i=0;i<=359;i++)
    {
        xICPb.push_back(xb[i]);
        yICPb.push_back(yb[i]);
    }
    for (int i=0;i<=99;i++)
    {
        xe.push_back(i);
    }
    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
            ui.tab_manager->setEnabled(true);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
            ui.tab_manager->setEnabled(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Funciones definidaas
*****************************************************************************/

void MainWindow::on_BotonIniciar_clicked() {
    if(ui.RBPuntos->isChecked()){
        qnode.DefinirTipoNavegacion(true);
        PosicionX=ui.ValorPosX->value();
        PosicionY=ui.ValorPosY->value();
        qnode.DefinirPunto(PosicionX,PosicionY);
        Iniciar=!Iniciar;
        qnode.InitSistema(Iniciar);
    }
    else if(ui.RBTrayectorias->isChecked()){
        ui.GraficaTrayectoriaRobots->detachItems(QwtPlotItem::Rtti_PlotCurve);
        if(ui.RBT1->isChecked()){
            qnode.DefinirTipoNavegacion(false);
            Trayectoria=1;
            qnode.DefinirTrayectoria(Trayectoria);
            Iniciar=!Iniciar;
            qnode.InitSistema(Iniciar);
        }
        else if(ui.RBT2->isChecked()){
            qnode.DefinirTipoNavegacion(false);
            Trayectoria=2;
            qnode.DefinirTrayectoria(Trayectoria);
            Iniciar=!Iniciar;
            qnode.InitSistema(Iniciar);
        }
        else if(ui.RBT3->isChecked()){
            qnode.DefinirTipoNavegacion(false);
            Trayectoria=3;
            qnode.DefinirTrayectoria(Trayectoria);
            Iniciar=!Iniciar;
            qnode.InitSistema(Iniciar);
        }
        else{
            QMessageBox msgBox;
            msgBox.setText("Error: No se ha seleccionado una trayectoria");
            msgBox.exec();
            Trayectoria=0;
        }
    }
    if(Iniciar){
        ui.BotonIniciar->setEnabled(false);
        ui.BotonDetener->setEnabled(true);

        if(ui.RBPuntos->isChecked()){
            ui.ValorPosX->setEnabled(false);
            ui.ValorPosY->setEnabled(false);
        }
    }
}

void MainWindow::on_BotonDetener_clicked() {
    Iniciar=!Iniciar;
    qnode.InitSistema(Iniciar);
    if(!Iniciar){
        ui.BotonIniciar->setEnabled(true);
        ui.BotonDetener->setEnabled(false);
        if(ui.RBPuntos->isChecked()){
            ui.ValorPosX->setEnabled(true);
            ui.ValorPosY->setEnabled(true);
        }
    }
}

void MainWindow::on_RBPuntos_clicked() {
    if(ui.RBPuntos->isChecked()){
        ui.BotonIniciar->setEnabled(true);
        ui.EtiquetaDefinirPuntos->setEnabled(true);
        ui.EtiquetaPosX->setEnabled(true);
        ui.EtiquetaPosY->setEnabled(true);
        ui.ValorPosX->setEnabled(true);
        ui.ValorPosY->setEnabled(true);
        ui.GrupoTrayectorias->setEnabled(false);
        ui.GraficaTrayectoria->detachItems(QwtPlotItem::Rtti_PlotCurve);
        xp.clear();
        yp.clear();
        PuntoX=ui.ValorPosX->value();
        PuntoY=ui.ValorPosY->value();
        if (qnode.rol1.compare("Lider")==0){
            PosXL=qnode.xr1;
            PosYL=qnode.yr1;
        }
        else if (qnode.rol2.compare("Lider")==0){
            PosXL=qnode.xr2;
            PosYL=qnode.yr2;
        }
        else if (qnode.rol3.compare("Lider")==0){
            PosXL=qnode.xr3;
            PosYL=qnode.yr3;
        }
        xp.push_back(PosXL);
        xp.push_back(PuntoX);
        yp.push_back(PosYL);
        yp.push_back(PuntoY);
        QwtPlotCurve *curvetrayectoria= new QwtPlotCurve(QString("Curve"));
        curvetrayectoria->setRenderHint( QwtPlotItem::RenderAntialiased );
        curvetrayectoria->setPen( Qt::yellow,3);
        curvetrayectoria->setSamples(xp.data(),yp.data(),(int) xp.size());
        curvetrayectoria->attach(ui.GraficaTrayectoria);
        ui.GraficaTrayectoria->replot();
        ui.GraficaTrayectoria->show();
    }
}

void MainWindow::on_ValorPosX_valueChanged(double d){
    ui.GraficaTrayectoria->detachItems(QwtPlotItem::Rtti_PlotCurve);
    xp.clear();
    yp.clear();
    PuntoX=ui.ValorPosX->value();
    PuntoY=ui.ValorPosY->value();
    if (qnode.rol1.compare("Lider")==0){
        PosXL=qnode.xr1;
        PosYL=qnode.yr1;
    }
    else if (qnode.rol2.compare("Lider")==0){
        PosXL=qnode.xr2;
        PosYL=qnode.yr2;
    }
    else if (qnode.rol3.compare("Lider")==0){
        PosXL=qnode.xr3;
        PosYL=qnode.yr3;
    }
    xp.push_back(PosXL);
    xp.push_back(PuntoX);
    yp.push_back(PosYL);
    yp.push_back(PuntoY);
    QwtPlotCurve *curvetrayectoria= new QwtPlotCurve(QString("Curve"));
    curvetrayectoria->setRenderHint( QwtPlotItem::RenderAntialiased );
    curvetrayectoria->setPen( Qt::yellow,3);
    curvetrayectoria->setSamples(xp.data(),yp.data(),(int) xp.size());
    curvetrayectoria->attach(ui.GraficaTrayectoria);
    ui.GraficaTrayectoria->replot();
    ui.GraficaTrayectoria->show();
}

void MainWindow::on_ValorPosY_valueChanged(double d){
    ui.GraficaTrayectoria->detachItems(QwtPlotItem::Rtti_PlotCurve);
    xp.clear();
    yp.clear();
    PuntoX=ui.ValorPosX->value();
    PuntoY=ui.ValorPosY->value();
    if (qnode.rol1.compare("Lider")==0){
        PosXL=qnode.xr1;
        PosYL=qnode.yr1;
    }
    else if (qnode.rol2.compare("Lider")==0){
        PosXL=qnode.xr2;
        PosYL=qnode.yr2;
    }
    else if (qnode.rol3.compare("Lider")==0){
        PosXL=qnode.xr3;
        PosYL=qnode.yr3;
    }
    xp.push_back(PosXL);
    xp.push_back(PuntoX);
    yp.push_back(PosYL);
    yp.push_back(PuntoY);
    QwtPlotCurve *curvetrayectoria= new QwtPlotCurve(QString("Curve"));
    curvetrayectoria->setRenderHint( QwtPlotItem::RenderAntialiased );
    curvetrayectoria->setPen( Qt::yellow,3);
    curvetrayectoria->setSamples(xp.data(),yp.data(),(int) xp.size());
    curvetrayectoria->attach(ui.GraficaTrayectoria);
    ui.GraficaTrayectoria->replot();
    ui.GraficaTrayectoria->show();
}

void MainWindow::on_RBTrayectorias_clicked() {
    if(ui.RBTrayectorias->isChecked()){
        ui.BotonIniciar->setEnabled(true);
        ui.GrupoTrayectorias->setEnabled(true);
        ui.EtiquetaDefinirPuntos->setEnabled(false);
        ui.EtiquetaPosX->setEnabled(false);
        ui.EtiquetaPosY->setEnabled(false);
        ui.ValorPosX->setEnabled(false);
        ui.ValorPosY->setEnabled(false);
        if(ui.RBT1->isChecked()){
            ui.GraficaTrayectoria->detachItems(QwtPlotItem::Rtti_PlotCurve);
            QwtPlotCurve *curvetrayectoria= new QwtPlotCurve(QString("Curve"));
            curvetrayectoria->setRenderHint( QwtPlotItem::RenderAntialiased );
            curvetrayectoria->setPen( Qt::darkCyan,3);
            curvetrayectoria->setSamples(xt1.data(),yt1.data(),(int) xt1.size());
            curvetrayectoria->attach(ui.GraficaTrayectoria);
            ui.GraficaTrayectoria->replot();
            ui.GraficaTrayectoria->show();
        }
        else if(ui.RBT2->isChecked()){
            ui.GraficaTrayectoria->detachItems(QwtPlotItem::Rtti_PlotCurve);
            QwtPlotCurve *curvetrayectoria= new QwtPlotCurve(QString("Curve"));
            curvetrayectoria->setRenderHint( QwtPlotItem::RenderAntialiased );
            curvetrayectoria->setPen(Qt::darkMagenta,3);
            curvetrayectoria->setSamples(xt2.data(),yt2.data(),(int) xt2.size());
            curvetrayectoria->attach(ui.GraficaTrayectoria);
            ui.GraficaTrayectoria->replot();
            ui.GraficaTrayectoria->show();
        }
        else if(ui.RBT3->isChecked()){
            ui.GraficaTrayectoria->detachItems(QwtPlotItem::Rtti_PlotCurve);
            QwtPlotCurve *curvetrayectoria= new QwtPlotCurve(QString("Curve"));
            curvetrayectoria->setRenderHint( QwtPlotItem::RenderAntialiased );
            curvetrayectoria->setPen(Qt::darkGray,3);
            curvetrayectoria->setSamples(xt3.data(),yt3.data(),(int) xt3.size());
            curvetrayectoria->attach(ui.GraficaTrayectoria);
            ui.GraficaTrayectoria->replot();
            ui.GraficaTrayectoria->show();
        }
    }
}

void MainWindow::on_RBT1_clicked(){
    if(ui.RBT1->isChecked()){
        ui.GraficaTrayectoria->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curvetrayectoria= new QwtPlotCurve(QString("Curve"));
        curvetrayectoria->setRenderHint( QwtPlotItem::RenderAntialiased );
        curvetrayectoria->setPen( Qt::darkCyan,3);
        curvetrayectoria->setSamples(xt1.data(),yt1.data(),(int) xt1.size());
        curvetrayectoria->attach(ui.GraficaTrayectoria);
        ui.GraficaTrayectoria->replot();
        ui.GraficaTrayectoria->show();
    }
}

void MainWindow::on_RBT2_clicked(){
    if(ui.RBT2->isChecked()){
        ui.GraficaTrayectoria->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curvetrayectoria= new QwtPlotCurve(QString("Curve"));
        curvetrayectoria->setRenderHint( QwtPlotItem::RenderAntialiased );
        curvetrayectoria->setPen(Qt::darkMagenta,3);
        curvetrayectoria->setSamples(xt2.data(),yt2.data(),(int) xt2.size());
        curvetrayectoria->attach(ui.GraficaTrayectoria);
        ui.GraficaTrayectoria->replot();
        ui.GraficaTrayectoria->show();
    }
}

void MainWindow::on_RBT3_clicked(){
    if(ui.RBT3->isChecked()){
        ui.GraficaTrayectoria->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curvetrayectoria= new QwtPlotCurve(QString("Curve"));
        curvetrayectoria->setRenderHint( QwtPlotItem::RenderAntialiased );
        curvetrayectoria->setPen(Qt::darkGray,3);
        curvetrayectoria->setSamples(xt3.data(),yt3.data(),(int) xt3.size());
        curvetrayectoria->attach(ui.GraficaTrayectoria);
        ui.GraficaTrayectoria->replot();
        ui.GraficaTrayectoria->show();
    }
}

void MainWindow::on_RBR1_clicked() {
    if(ui.RBR1->isChecked()){
        ui.EtiquetaRol->setEnabled(true);
        ui.EtiquetaRolAsinado->setEnabled(true);
        ui.EtiquetaPosicion->setEnabled(true);
        ui.EtiquetaPosicionX->setEnabled(true);
        ui.EtiquetaPosicionY->setEnabled(true);
        ui.EtiquetaX->setEnabled(true);
        ui.EtiquetaY->setEnabled(true);
        ui.EtiquetaZ->setEnabled(true);
        ui.EtiquetaOrientacion->setEnabled(true);
        ui.EtiquetaOrientacionZ->setEnabled(true);
        ui.EtiquetaM->setEnabled(true);
        ui.EtiquetaM1->setEnabled(true);
        ui.EtiquetaRad->setEnabled(true);
        ui.EtiquetaRolAsinado->setText(QString::fromStdString(qnode.rol1));

        ui.GraficaICP->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curveICPBase= new QwtPlotCurve(QString("Curve"));
        curveICPBase->setRenderHint( QwtPlotItem::RenderAntialiased );
        curveICPBase->setPen( Qt::black,2);
        curveICPBase->setSamples(xICPb.data(),yICPb.data(),(int) xICPb.size());

        QwtPlotCurve *curveICP= new QwtPlotCurve(QString("Curve"));
        curveICP->setRenderHint( QwtPlotItem::RenderAntialiased );
        curveICP->setPen( Qt::blue, 2);
        curveICP->setSamples(qnode.MapxR1.data(),qnode.MapyR1.data(),(int) qnode.MapxR1.size());
        curveICPBase->attach(ui.GraficaICP);
        curveICP->attach(ui.GraficaICP);
        ui.GraficaICP->replot();
        ui.GraficaICP->show();

        on_ActualizaED_clicked(true);
        on_ActualizaEA_clicked(true);
    }
}

void MainWindow::on_RBR2_clicked() {
    if(ui.RBR2->isChecked()){
        ui.EtiquetaRol->setEnabled(true);
        ui.EtiquetaRolAsinado->setEnabled(true);
        ui.EtiquetaPosicion->setEnabled(true);
        ui.EtiquetaPosicionX->setEnabled(true);
        ui.EtiquetaPosicionY->setEnabled(true);
        ui.EtiquetaX->setEnabled(true);
        ui.EtiquetaY->setEnabled(true);
        ui.EtiquetaZ->setEnabled(true);
        ui.EtiquetaOrientacion->setEnabled(true);
        ui.EtiquetaOrientacionZ->setEnabled(true);
        ui.EtiquetaM->setEnabled(true);
        ui.EtiquetaM1->setEnabled(true);
        ui.EtiquetaRad->setEnabled(true);
        ui.EtiquetaRolAsinado->setText(QString::fromStdString(qnode.rol2));

        ui.GraficaICP->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curveICPBase= new QwtPlotCurve(QString("Curve"));
        curveICPBase->setRenderHint( QwtPlotItem::RenderAntialiased );
        curveICPBase->setPen( Qt::black,2);
        curveICPBase->setSamples(xICPb.data(),yICPb.data(),(int) xICPb.size());

        QwtPlotCurve *curveICP= new QwtPlotCurve(QString("Curve"));
        curveICP->setRenderHint( QwtPlotItem::RenderAntialiased );
        curveICP->setPen( Qt::green, 2);
        curveICP->setSamples(qnode.MapxR2.data(),qnode.MapyR2.data(),(int) qnode.MapxR2.size());
        curveICPBase->attach(ui.GraficaICP);
        curveICP->attach(ui.GraficaICP);
        ui.GraficaICP->replot();
        ui.GraficaICP->show();

        on_ActualizaED_clicked(true);
        on_ActualizaEA_clicked(true);

    }
}

void MainWindow::on_RBR3_clicked() {
    if(ui.RBR3->isChecked()){
        ui.EtiquetaRol->setEnabled(true);
        ui.EtiquetaRolAsinado->setEnabled(true);
        ui.EtiquetaPosicion->setEnabled(true);
        ui.EtiquetaPosicionX->setEnabled(true);
        ui.EtiquetaPosicionY->setEnabled(true);
        ui.EtiquetaX->setEnabled(true);
        ui.EtiquetaY->setEnabled(true);
        ui.EtiquetaZ->setEnabled(true);
        ui.EtiquetaOrientacion->setEnabled(true);
        ui.EtiquetaOrientacionZ->setEnabled(true);
        ui.EtiquetaM->setEnabled(true);
        ui.EtiquetaM1->setEnabled(true);
        ui.EtiquetaRad->setEnabled(true);
        ui.EtiquetaRolAsinado->setText(QString::fromStdString(qnode.rol3));

        ui.GraficaICP->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curveICPBase= new QwtPlotCurve(QString("Curve"));
        curveICPBase->setRenderHint( QwtPlotItem::RenderAntialiased );
        curveICPBase->setPen( Qt::black,2);
        curveICPBase->setSamples(xICPb.data(),yICPb.data(),(int) xICPb.size());

        QwtPlotCurve *curveICP= new QwtPlotCurve(QString("Curve"));
        curveICP->setRenderHint( QwtPlotItem::RenderAntialiased );
        curveICP->setPen( Qt::red, 2);
        curveICP->setSamples(qnode.MapxR3.data(),qnode.MapyR3.data(),(int) qnode.MapxR3.size());
        curveICPBase->attach(ui.GraficaICP);
        curveICP->attach(ui.GraficaICP);
        ui.GraficaICP->replot();
        ui.GraficaICP->show();

        on_ActualizaED_clicked(true);
        on_ActualizaEA_clicked(true);
    }
}

void MainWindow::on_ActualizaH_clicked(bool check){
    if(ui.RBR1->isChecked()){
        ui.GraficaHistograma->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curvehist= new QwtPlotCurve(QString("Curve"));
        curvehist->setRenderHint( QwtPlotItem::RenderAntialiased );
        curvehist->setPen( Qt::blue, 2);
        curvehist->setSamples(xhist.data(),qnode.distanciasR1.data(),(int) xhist.size());
        curvehist->attach(ui.GraficaHistograma);
        ui.GraficaHistograma->replot();
        ui.GraficaHistograma->show();
    }
    else if(ui.RBR2->isChecked()){
        ui.GraficaHistograma->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curvehist= new QwtPlotCurve(QString("Curve"));
        curvehist->setRenderHint( QwtPlotItem::RenderAntialiased );
        curvehist->setPen( Qt::green, 2);
        curvehist->setSamples(xhist.data(),qnode.distanciasR2.data(),(int) xhist.size());
        curvehist->attach(ui.GraficaHistograma);
        ui.GraficaHistograma->replot();
        ui.GraficaHistograma->show();
    }
    else if(ui.RBR3->isChecked()){
        ui.GraficaHistograma->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curvehist= new QwtPlotCurve(QString("Curve"));
        curvehist->setRenderHint( QwtPlotItem::RenderAntialiased );
        curvehist->setPen( Qt::red, 2);
        curvehist->setSamples(xhist.data(),qnode.distanciasR3.data(),(int) xhist.size());
        curvehist->attach(ui.GraficaHistograma);
        ui.GraficaHistograma->replot();
        ui.GraficaHistograma->show();
    }
}

void MainWindow::on_ActualizaED_clicked(bool check){
    if(ui.RBR1->isChecked()){
        ui.GraficaLineal->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curveerrord= new QwtPlotCurve(QString("Curve"));
        curveerrord->setRenderHint( QwtPlotItem::RenderAntialiased );
        curveerrord->setPen( Qt::blue, 2);
        curveerrord->setSamples(xe.data(),qnode.errordistR1.data(),(int) xe.size());
        curveerrord->attach(ui.GraficaLineal);
        ui.GraficaLineal->replot();
        ui.GraficaLineal->show();
    }
    else if(ui.RBR2->isChecked()){
        ui.GraficaLineal->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curveerrord= new QwtPlotCurve(QString("Curve"));
        curveerrord->setRenderHint( QwtPlotItem::RenderAntialiased );
        curveerrord->setPen( Qt::green, 2);
        curveerrord->setSamples(xe.data(),qnode.errordistR2.data(),(int) xe.size());
        curveerrord->attach(ui.GraficaLineal);
        ui.GraficaLineal->replot();
        ui.GraficaLineal->show();
    }
    else if(ui.RBR3->isChecked()){
        ui.GraficaLineal->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curveerrord= new QwtPlotCurve(QString("Curve"));
        curveerrord->setRenderHint( QwtPlotItem::RenderAntialiased );
        curveerrord->setPen( Qt::red, 2);
        curveerrord->setSamples(xe.data(),qnode.errordistR3.data(),(int) xe.size());
        curveerrord->attach(ui.GraficaLineal);
        ui.GraficaLineal->replot();
        ui.GraficaLineal->show();
    }
}

void MainWindow::on_ActualizaEA_clicked(bool check){
    if(ui.RBR1->isChecked()){
        ui.GraficaAngular->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curveerrora= new QwtPlotCurve(QString("Curve"));
        curveerrora->setRenderHint( QwtPlotItem::RenderAntialiased );
        curveerrora->setPen( Qt::blue, 2);
        curveerrora->setSamples(xe.data(),qnode.errorangR1.data(),(int) xe.size());
        curveerrora->attach(ui.GraficaAngular);
        ui.GraficaAngular->replot();
        ui.GraficaAngular->show();
    }
    else if(ui.RBR2->isChecked()){
        ui.GraficaAngular->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curveerrora= new QwtPlotCurve(QString("Curve"));
        curveerrora->setRenderHint( QwtPlotItem::RenderAntialiased );
        curveerrora->setPen( Qt::green, 2);
        curveerrora->setSamples(xe.data(),qnode.errorangR2.data(),(int) xe.size());
        curveerrora->attach(ui.GraficaAngular);
        ui.GraficaAngular->replot();
        ui.GraficaAngular->show();
    }
    else if(ui.RBR3->isChecked()){
        ui.GraficaAngular->detachItems(QwtPlotItem::Rtti_PlotCurve);
        QwtPlotCurve *curveerrord= new QwtPlotCurve(QString("Curve"));
        curveerrord->setRenderHint( QwtPlotItem::RenderAntialiased );
        curveerrord->setPen( Qt::red, 2);
        curveerrord->setSamples(xe.data(),qnode.errorangR3.data(),(int) xe.size());
        curveerrord->attach(ui.GraficaAngular);
        ui.GraficaAngular->replot();
        ui.GraficaAngular->show();
    }
}

void MainWindow::on_ActualizaPos_clicked(bool check){
    QwtPlotCurve *curvetr= new QwtPlotCurve(QString("Curve"));
    curvetr->setRenderHint( QwtPlotItem::RenderAntialiased );
    curvetr->setPen( Qt::gray, 2);
    if (qnode.rol1.compare("Lider")==0){
        curvetr->setSamples(qnode.lastpxR1.data(),qnode.lastpyR1.data(),(int) qnode.lastpxR1.size());
    }
    else if (qnode.rol2.compare("Lider")==0){
        curvetr->setSamples(qnode.lastpxR2.data(),qnode.lastpyR2.data(),(int) qnode.lastpxR2.size());
    }
    else if (qnode.rol3.compare("Lider")==0){
        curvetr->setSamples(qnode.lastpxR3.data(),qnode.lastpyR3.data(),(int) qnode.lastpxR3.size());
    }
    curvetr->attach(ui.GraficaTrayectoriaRobots);
    ui.GraficaTrayectoriaRobots->replot();
    ui.GraficaTrayectoriaRobots->show();
}

void MainWindow::ActualizaPosiciones(double xr1,double yr1,double zr1,double xr2,double yr2,double zr2,double xr3,double yr3,double zr3){
    if(ui.RBR1->isChecked()){
        ui.EtiquetaPosicionX->setText(QString::number(xr1));
        ui.EtiquetaPosicionY->setText(QString::number(yr1));
        ui.EtiquetaOrientacionZ->setText(QString::number(zr1));
        if (qnode.rol1.compare("Lider")==0){
            PosXL=xr1;
            PosYL=yr1;
        }
    }
    else if (ui.RBR2->isChecked()){
        ui.EtiquetaPosicionX->setText(QString::number(xr2));
        ui.EtiquetaPosicionY->setText(QString::number(yr2));
        ui.EtiquetaOrientacionZ->setText(QString::number(zr2));
        if (qnode.rol2.compare("Lider")==0){
            PosXL=xr2;
            PosYL=yr2;
        }
    }
    else if (ui.RBR3->isChecked()){
        ui.EtiquetaPosicionX->setText(QString::number(xr3));
        ui.EtiquetaPosicionY->setText(QString::number(yr3));
        ui.EtiquetaOrientacionZ->setText(QString::number(zr3));
        if (qnode.rol3.compare("Lider")==0){
            PosXL=xr3;
            PosYL=yr3;
        }
    }
}

void MainWindow::ActualizaHist(){
    on_ActualizaH_clicked(true);
}

void MainWindow::ActualizaErrorD(){
    on_ActualizaED_clicked(true);
}

void MainWindow::ActualizaErrorA(){
    on_ActualizaEA_clicked(true);
}

void MainWindow::ActualizaPos(){
    on_ActualizaPos_clicked(true);
}

void MainWindow::ActualizaFlagLoc(bool FlagSisLoc){
    if(FlagSisLoc==true){
        ui.ImSisLoc->setStyleSheet("background-color:rgb(113, 255, 25);");
    }
    else{
        ui.ImSisLoc->setStyleSheet("background-color:rgb(255, 0, 0);");
    }

}

void MainWindow::ActualizaFlagForm(bool FlagSisForm){
    if(FlagSisForm==true){
        ui.ImSisFor->setStyleSheet("background-color:rgb(113, 255, 25);");
    }
    else{
        ui.ImSisFor->setStyleSheet("background-color:rgb(255, 0, 0);");
    }

}

void MainWindow::ActualizaFlagNav(bool FlagSisNav){
    if(FlagSisNav==true){
        ui.ImSisNav->setStyleSheet("background-color:rgb(113, 255, 25);");
    }
    else{
        ui.ImSisNav->setStyleSheet("background-color:rgb(255, 0, 0);");
    }
}

void MainWindow::ActualizaFlagNavEnd(bool FlagSisNavEnd){
    if(FlagSisNavEnd==true){
        Iniciar=false;
        qnode.InitSistema(Iniciar);
        ui.BotonIniciar->setEnabled(true);
        ui.BotonDetener->setEnabled(false);
        if(ui.RBPuntos->isChecked()){
            ui.ValorPosX->setEnabled(true);
            ui.ValorPosY->setEnabled(true);
        }
    }
}
/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "gui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace gui

