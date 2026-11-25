#ifndef ROBOT_H
#define ROBOT_H
#include "librobot/librobot.h"
#include <QObject>
#include <QWidget>

#if defined(LIBROBOT_HAS_AMCL) && LIBROBOT_HAS_AMCL &&                    \
    !defined(DISABLE_AMCL)
#define DEMO_HAS_AMCL 1
#else
#define DEMO_HAS_AMCL 0
#endif

#ifndef DISABLE_OPENCV
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

Q_DECLARE_METATYPE(cv::Mat)
#endif
#ifndef DISABLE_SKELETON
Q_DECLARE_METATYPE(skeleton)
#endif
Q_DECLARE_METATYPE(std::vector<LaserData>)
class robot : public QObject {
  Q_OBJECT
public:
  explicit robot(QObject *parent = nullptr);

  void initAndStartRobot(std::string ipaddress);

  // tato funkcia len nastavuje hodnoty.. posielaju sa v callbacku(dobre, kvoli
  // asynchronnosti a zabezpeceniu,ze sa poslu len raz pri viacero prepisoch
  // vramci callu)
  void setSpeedVal(double forw, double rots);
  // tato funkcia fyzicky posiela hodnoty do robota
  void setSpeed(double forw, double rots);
#if DEMO_HAS_AMCL
  bool isAmclReady() const { return amclReady; }
  Particle getBestParticle() const { return robotCom.getBestParticle(); }
  const GridMap &getAmclMap() const { return robotCom.getAmclMap(); }
  void getGridCoordinates(double realX, double realY, int &gridX,
                          int &gridY) const {
    robotCom.getGridCoordinates(realX, realY, gridX, gridY);
  }
#endif
signals:
  void telemetryReceived(); // emitted on every Kobuki packet (HMI connection watchdog)
  // Odometria v suradnicovom systeme startu robota: x, y [m], theta [rad] proti smeru hodinovych ruciciek.
  void publishPosition(double x, double y, double theta);
  // Stav baterie v % (posiela sa len pri zmene a ked robot hlasi platne napatie).
  void publishBattery(int percent);
  void publishLidar(const std::vector<LaserData> &lidata);
#if DEMO_HAS_AMCL
  void publishAMCLPosition(double x, double y, double theta);
#endif
#ifndef DISABLE_OPENCV
  void publishCamera(const cv::Mat &camframe);
#endif
#ifndef DISABLE_SKELETON
  void publishSkeleton(const skeleton &skeledata);
#endif
private:
  /// toto su vase premenne na vasu odometriu
  double x;   // [m]
  double y;   // [m]
  double fi;  // [rad]
  bool odomInit_ = false;          // prvy paket iba ulozi pociatocne hodnoty enkoderov/gyra
  unsigned short prevEncLeft_ = 0;
  unsigned short prevEncRight_ = 0;
  short prevGyro_ = 0;
  bool gyroValid_ = false;         // gyro sa aspon raz zmenilo -> uhol z gyra, inak z enkoderov
  int lastBatteryPercent_ = -1;
  void updateOdometry(const TKobukiData &robotdata);
  void updateBattery(const TKobukiData &robotdata);
  ///-----------------------------
  /// toto su rychlosti ktore sa nastavuju setSpeedVal a posielaju v
  /// processThisRobot
  double forwardspeed;  // mm/s
  double rotationspeed; // omega/s

  /// toto su callbacky co sa sa volaju s novymi datami
  int processThisLidar(const std::vector<LaserData> &laserData);
  int processThisRobot(const TKobukiData &robotdata);
#if DEMO_HAS_AMCL
  int processThisAMCLPosition(float x, float y, float theta);
#endif
#ifndef DISABLE_OPENCV
  int processThisCamera(cv::Mat cameraData);
#endif

  /// pomocne strukutry aby ste si trosku nerobili race conditions
  std::vector<LaserData> copyOfLaserData;
#ifndef DISABLE_OPENCV
  cv::Mat frame[3];
#endif
  /// classa ktora riesi komunikaciu s robotom
  libRobot robotCom;

  /// pomocne premenne... moc nerieste naco su
  int datacounter;
#ifndef DISABLE_OPENCV
  bool useCamera1;
  int actIndex;
#endif

#ifndef DISABLE_SKELETON
  int processThisSkeleton(skeleton skeledata);
  int updateSkeletonPicture;
  skeleton skeleJoints;
#endif
  int useDirectCommands;
#if DEMO_HAS_AMCL
  bool amclReady{false};
#endif
};

#endif // ROBOT_H
