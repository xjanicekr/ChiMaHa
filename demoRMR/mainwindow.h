#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
#include <QMainWindow>
#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QElapsedTimer>
#include <QPointF>
#include <QRectF>
#include <QVector>
#include <QFile>
#include <QTextStream>
#include <QJsonObject>
#include <memory>

class robot;
class QLabel;
namespace Ui { class MainWindow; }

// Supervisory-control HMI: mission editor on a known map, mission execution,
// alarms, recording/playback and object search during the mission.
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    enum class MissionState { Idle, Running, Paused, Done, Aborted };
    enum class Mode { Planning, Records };
    enum class LogLevel { Info, Ok, Warning, Danger, Detection };

    struct Pose { double x = 0.0, y = 0.0, th = 0.0; };          // map frame [m, rad], th CCW
    struct MissionPoint {
        bool operation = true;   // true = operačný bod, false = prejazdový bod
        QPointF pos;             // map frame [m]
        QString task = "photo";  // photo | rotate | wait | search (operation only)
    };
    struct Detection {
        qint64 t = 0;            // ms from mission start
        QString name;
        double confidence = 0.0;
        QPointF pos;             // map frame [m]
        QString file;            // saved photo
        QRectF box;              // normalized bounding box in the camera image
    };

    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

public slots:
    void setSafetyStopActive(bool active);
    void setConnectionState(bool connected, const QString &message = QString());
    void setCameraImage(const QImage &image);
    // Backend inputs. Connect these to AMCL / odometry / battery of the project backend.
    void setRobotPose(double x, double y, double theta);
    void setOdometryPose(double x, double y, double theta);
    void setBatteryLevel(int percent);

signals:
    void safetyStopChanged(bool active);
    void connectRequested(const QString &ipAddress, bool simulation);
    void disconnectRequested();
    void stopRequested();
    // Mission commands for a backend navigation stack (the HMI also has a simple built-in go-to controller).
    void goalRequested(double x, double y);
    void rotateRequested(double angle);
    void missionStarted();
    void missionPaused(const QString &reason);
    void missionFinished(const QString &recordingDir);

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    // --- robot session (kept from task 1) ---
    void toggleConnection();
    void startRobotSession(const QString &address, bool simulation);
    void endRobotSession(const QString &message = QString());
    void checkRobotSession();
    void handleScan(const QVector<QPointF> &robotPointsMm, const QVector<QPointF> &rawAngleDist);
    void handleCameraFrame(const QImage &rgb);
    void sendSpeed(double forward, double rotation);

    // --- map ---
    bool loadMap(const QString &path);   // AMCL map file (mapa.txt)
    QRectF mapRect() const;
    QPointF worldToWidget(const QPointF &m) const;
    QPointF widgetToWorld(const QPointF &w) const;
    bool cellOf(const QPointF &m, int &cx, int &cy) const;
    bool isBlocked(const QPointF &m) const;
    bool segmentBlocked(const QPointF &a, const QPointF &b) const;
    void paintMap();
    void paintCamera();
    void paintTimelineMarks();

    // --- mission editor ---
    bool editable() const;
    int pointAt(const QPointF &widgetPos) const;
    void showAddPointMenu(const QPointF &world, const QPoint &globalPos);
    void showPointMenu(int index, const QPoint &globalPos);
    void insertPoint(const MissionPoint &p);
    void refreshPointsTable();
    void selectPoint(int index);
    void movePoint(int delta);
    void deleteSelectedPoint();
    void clearMission();
    void saveMission();
    void loadMission();
    QJsonObject missionToJson() const;
    QString validationError() const;
    QString runBlocker() const;
    void updateValidation();

    // --- mission execution ---
    void startMission();
    void pauseMission(const QString &reason, LogLevel level = LogLevel::Warning);
    void resumeMission();
    void abortMission();
    void finishMission(MissionState state);
    void missionTick();
    void sendCurrentGoal();
    void startTask();
    void advanceMission();
    void driveToGoal(const QPointF &goal);
    void updateMissionUi();
    void savePhoto(const QString &subdir, const QString &name);

    // --- alarms, log, toast ---
    void refreshAlarms();
    void addLog(const QString &text, LogLevel level = LogLevel::Info);
    void showToast(const QString &text, LogLevel level = LogLevel::Info);
    void layoutOverlays();
    void setMode(Mode mode);
    void swapViews();                    // map <-> camera between the main area and the side slot
    void updateViewTitles();

    // --- recording ---
    void startRecording();
    void stopRecording();
    void recordTick();
    void recordEvent(const QString &type, const QString &text);
    qint64 missionTimeMs() const;

    // --- playback ---
    struct RecSample { qint64 t; Pose amcl, odom; };
    struct RecScan { qint64 t; QVector<QPointF> points; };   // robot frame mm
    struct RecFrame { qint64 t; QString file; };
    struct RecEvent { qint64 t; QString type, text; };
    struct Recording {
        QString dir;
        QVector<MissionPoint> points;
        QVector<RecSample> poses;
        QVector<RecScan> scans;
        QVector<RecFrame> frames;
        QVector<RecEvent> events;
        QVector<Detection> detections;
        qint64 duration = 0;
    };
    void refreshRecords();
    bool loadRecording(const QString &dir);
    void deleteSelectedRecord();
    void togglePlayback();
    void seekPlayback(qint64 t);
    void playbackTick();
    void showReplayFrame();

    // --- object detection ---
    bool detectTarget(const QImage &rgb, QRectF &box, double &confidence) const;
    void handleDetection(const QRectF &box, double confidence, const QImage &frame);
    void addFoundItem(const Detection &d, const QImage &thumb);

    Ui::MainWindow *ui;
    QLabel *toastLabel_ = nullptr;
    QLabel *cursorLabel_ = nullptr;
    QLabel *replayBadge_ = nullptr;
    QTimer toastTimer_;
    Mode mode_ = Mode::Planning;
    bool viewsSwapped_ = false;          // true = camera in the main area, map in the side slot

    // session
    robot *robot_ = nullptr;
    unsigned sessionId_ = 0;
    QTimer sessionTimer_;
    QElapsedTimer telemetryAge_;
    QElapsedTimer imageAge_;
    bool connected_ = false;
    bool connectionPending_ = false;
    bool safetyStopActive_ = false;
    QImage cameraImage_;
    QVector<QPointF> lidarPoints_;       // robot frame, mm: x forward, y right
    QVector<QPointF> lidarRaw_;          // (angle deg clockwise, distance mm) for recording
    unsigned scanSerial_ = 0;
    double lastForward_ = 1e9, lastRotation_ = 1e9;

    // robot state
    Pose pose_, odom_;
    bool hasPose_ = false, hasOdom_ = false;
    int battery_ = -1;                   // -1 = unknown
    bool obstacleAhead_ = false;
    double locQuality_ = 1.0;
    QElapsedTimer badLocalization_;
    bool offMission_ = false;

    // map
    bool mapLoaded_ = false;
    QString mapFile_;
    QImage mapImage_;                    // rendered occupancy grid
    int mapW_ = 0, mapH_ = 0;
    double mapRes_ = 0.05;
    QPointF mapOrigin_;
    QVector<quint8> occupancy_;          // 0 free, 1 occupied, 2 unknown
    QVector<quint8> blocked_;            // inflated by robot radius
    QVector<quint8> nearOccupied_;       // occupied dilated by 2 cells (localization check)

    // mission
    QVector<MissionPoint> points_;
    int selected_ = -1;
    int dragIndex_ = -1;
    QPointF dragOrigin_;
    bool dragMoved_ = false;
    bool hasPending_ = false;
    QPointF pendingPos_;
    bool hasHover_ = false;
    QPointF hoverPos_;
    bool tableUpdating_ = false;

    MissionState missionState_ = MissionState::Idle;
    int currentIndex_ = 0;
    bool inTask_ = false;
    QElapsedTimer taskTimer_;
    qint64 taskDurationMs_ = 0;
    QTimer missionTimer_;
    QElapsedTimer missionClock_;
    qint64 missionPausedMs_ = 0;
    QElapsedTimer pauseClock_;
    QVector<QPointF> trail_;
    QVector<Detection> detections_;
    int focusedDetection_ = -1;
    QRectF liveDetBox_;
    QElapsedTimer liveDetAge_;
    QString liveDetLabel_;
    unsigned frameCounter_ = 0;

    // recording
    bool recording_ = false;
    QString recDir_;
    QTimer recordTimer_;
    std::unique_ptr<QFile> posesFile_, scanFile_, cameraFile_, eventsFile_, detFile_;
    QTextStream posesOut_, scanOut_, cameraOut_, eventsOut_, detOut_;
    unsigned lastRecordedScan_ = 0;
    QElapsedTimer cameraRecordAge_;
    int cameraFrameNo_ = 0;
    int photoNo_ = 0;

    // playback
    Recording rec_;
    bool recLoaded_ = false;
    qint64 playPos_ = 0;
    QTimer playbackTimer_;
    QElapsedTimer playbackClock_;
    double playSpeed_ = 1.0;
    Pose replayPose_, replayOdom_;
    QVector<QPointF> replayScan_;
    QImage replayImage_;
    QString replayImageFile_;
    bool sliderUpdating_ = false;
};
#endif
