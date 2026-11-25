#include "robot.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kTickToMeter = 0.000085292090497737556558; // Kobuki: dlzka jedneho tiku enkodera [m]
constexpr double kWheelBase = 0.230;                        // Kobuki: vzdialenost kolies [m]
constexpr double kBatteryEmpty = 13.2;                      // [V] vybita bateria
constexpr double kBatteryFull = 16.7;                       // [V] plne nabita bateria

double normalizeAngle(double a)
{
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}
}

#if DEMO_HAS_AMCL
#include <QCoreApplication>
#include <iostream>
#endif

robot::robot(QObject *parent) : QObject(parent)
{
    /// pociatocne hodnoty - bez nich maju premenne nahodny obsah (MSVC Debug: 0xCDCDCDCD)
    /// a processThisCamera() by zapisoval mimo pola frame[3]
    x = y = fi = 0;
    forwardspeed = rotationspeed = 0;
    datacounter = 0;
    useDirectCommands = 1;
#ifndef DISABLE_OPENCV
    useCamera1 = false;
    actIndex = -1;
#endif
#ifndef DISABLE_SKELETON
    updateSkeletonPicture = 0;
#endif
    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
    #ifndef DISABLE_OPENCV
    qRegisterMetaType<cv::Mat>("cv::Mat");
#endif
#ifndef DISABLE_SKELETON
qRegisterMetaType<skeleton>("skeleton");
#endif
}

void robot::initAndStartRobot(std::string ipaddress)
{

    forwardspeed=0;
    rotationspeed=0;
    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robotCom.setLaserParameters([this](const std::vector<LaserData>& dat)->int{return processThisLidar(dat);},ipaddress);
    robotCom.setRobotParameters([this](const TKobukiData& dat)->int{return processThisRobot(dat);},ipaddress);
#if DEMO_HAS_AMCL
    const std::filesystem::path mapPath =
        std::filesystem::path(
            QCoreApplication::applicationDirPath().toLocal8Bit().constData()) /
        "mapa.txt";
    std::string amclError;
    amclReady = robotCom.setAMCLParameters(
        mapPath, 1500, 0.01, 2.0,
        [this](float x, float y, float theta) {
          return processThisAMCLPosition(x, y, theta);
        },
        &amclError);
    if (!amclReady) {
      std::cerr << "AMCL configuration failed for " << mapPath << ": "
                << amclError << std::endl;
    }
#endif
  #ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(std::bind(&robot::processThisCamera,this,std::placeholders::_1),"http://"+ipaddress+":8000/stream.mjpg");
#endif
   #ifndef DISABLE_SKELETON
      robotCom.setSkeletonParameters(std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robotCom.robotStart();


}

void robot::setSpeedVal(double forw, double rots)
{
    forwardspeed=forw;
    rotationspeed=rots;
    useDirectCommands=0;
}

void robot::setSpeed(double forw, double rots)
{
    /// oblúk: polomer = v / omega [mm]. Kobuki ho berie ako 16-bitové číslo, preto takmer rovná jazda
    /// (polomer nad 30 m) ide ako priamy pohyb a polomer pod 10 mm ako otáčanie na mieste
    if(forw==0 && rots==0)
        robotCom.setTranslationSpeed(0);
    else if(forw==0 || std::abs(forw/rots)<10.0)
        robotCom.setRotationSpeed(rots);
    else if(rots==0 || std::abs(forw/rots)>30000.0)
        robotCom.setTranslationSpeed(forw);
    else
        robotCom.setArcSpeed(forw,static_cast<int>(std::lround(forw/rots)));
    useDirectCommands=1;
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{


    ///tu mozete robit s datami z robota
    emit telemetryReceived();
    updateOdometry(robotdata);
    updateBattery(robotdata);
    /// odometria ide do HMI pri kazdom pakete (nie kazdy piaty) - HMI podla nej plynulo posuva robota
    /// medzi AMCL polohami (~7 Hz) a prekreslovanie mapy si obmedzuje samo
    emit publishPosition(x, y, fi);




///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    ///kazdy piaty krat, aby to ui moc nepreblikavalo..
    if(datacounter%5==0)
    {

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        /// (publishPosition sa posiela vyssie, pri kazdom pakete)
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    if(useDirectCommands==0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robotCom.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }
    datacounter++;

    return 0;

}

/// Odometria: draha z enkoderov, uhol z gyra (ak gyro neposiela data, tak z enkoderov).
/// Rozdiely enkoderov sa pocitaju v 16 bitoch, takze ich pretecenie nevadi.
void robot::updateOdometry(const TKobukiData &robotdata)
{
    if (!odomInit_) {
        prevEncLeft_ = robotdata.EncoderLeft;
        prevEncRight_ = robotdata.EncoderRight;
        prevGyro_ = robotdata.GyroAngle;
        odomInit_ = true;
        return;
    }
    const short dTicksLeft = static_cast<short>(robotdata.EncoderLeft - prevEncLeft_);
    const short dTicksRight = static_cast<short>(robotdata.EncoderRight - prevEncRight_);
    int dGyro = robotdata.GyroAngle - prevGyro_;   // GyroAngle je -18000..18000 (stotiny stupna)
    if (dGyro > 18000) dGyro -= 36000;             // prechod cez +-180 st. (inak skok o ~295 st.)
    else if (dGyro < -18000) dGyro += 36000;
    prevEncLeft_ = robotdata.EncoderLeft;
    prevEncRight_ = robotdata.EncoderRight;
    prevGyro_ = robotdata.GyroAngle;

    const double dLeft = dTicksLeft * kTickToMeter;
    const double dRight = dTicksRight * kTickToMeter;
    const double ds = (dLeft + dRight) / 2.0;
    if (dGyro != 0) gyroValid_ = true;
    const double dfi = gyroValid_ ? dGyro / 100.0 * kPi / 180.0
                                  : (dRight - dLeft) / kWheelBase;

    // integracia so stredovym uhlom (presnejsie pri pohybe po obluku)
    x += ds * std::cos(fi + dfi / 2.0);
    y += ds * std::sin(fi + dfi / 2.0);
    fi = normalizeAngle(fi + dfi);
}

/// Bateria Kobuki: napatie v 0.1 V, prepocet na % medzi vybitou a plnou baterou.
void robot::updateBattery(const TKobukiData &robotdata)
{
    if (robotdata.Battery == 0) return;   // robot (simulator) napatie neposiela
    const double volts = robotdata.Battery / 10.0;
    const int percent = static_cast<int>(std::lround(
        std::clamp((volts - kBatteryEmpty) / (kBatteryFull - kBatteryEmpty), 0.0, 1.0) * 100.0));
    if (percent == lastBatteryPercent_) return;
    lastBatteryPercent_ = percent;
    emit publishBattery(percent);
}

#if DEMO_HAS_AMCL
int robot::processThisAMCLPosition(float x, float y, float theta)
{
    emit publishAMCLPosition(x, y, theta);
    return 0;
}
#endif

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru
int robot::processThisLidar(const std::vector<LaserData>& laserData)
{

    copyOfLaserData=laserData;

    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    emit publishLidar(copyOfLaserData);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

  #ifndef DISABLE_OPENCV
///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int robot::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka

    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

  #ifndef DISABLE_SKELETON
/// vola sa ked dojdu nove data z trackera
int robot::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
