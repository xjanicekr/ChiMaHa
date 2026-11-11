#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "robot.h"
#include <QCoreApplication>
#include <QComboBox>
#include <QCursor>
#include <QDateTime>
#include <QDir>
#include <QDirIterator>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QHash>
#include <QHeaderView>
#include <QIcon>
#include <QJsonArray>
#include <QJsonDocument>
#include <QListWidgetItem>
#include <QMenu>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QRegularExpression>
#include <QSignalBlocker>
#include <algorithm>
#include <functional>
#include <cmath>
#ifndef DISABLE_OPENCV
#include <opencv2/imgproc.hpp>
#endif

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRobotRadius = 0.177;     // Kobuki base radius [m]
constexpr double kClearance = 0.05;        // extra margin for collision checks [m]
constexpr double kGoalTolerance = 0.15;    // point reached [m]
constexpr double kObstacleMm = 450.0;      // obstacle in front of the robot [mm]
constexpr double kCameraFovDeg = 60.0;     // horizontal camera field of view
constexpr double kLocThreshold = 0.5;      // scan/map match below this = robot mimo misie
constexpr int kLocTimeoutMs = 2000;
constexpr double kRotateSpeed = 0.8;       // rad/s for rotate/search tasks
constexpr int kTelemetryTimeoutMs = 2000;
const char *kTargetName = "Červená lopta";

const QColor kOpColor("#00008B");
const QColor kPassColor("#2375bd");
const QColor kRouteColor("#2375bd");
const QColor kCollisionColor("#dc2626");
const QColor kTrailColor("#0f766e");
const QColor kOdomColor("#d97706");
const QColor kDetColor("#6d28d9");
const QColor kRobotColor("#14243b");

struct TaskDef { const char *id; const char *label; };
const TaskDef kTasks[] = {
    {"photo", "odfotiť okolie"},
    {"rotate", "otočiť sa o 360°"},
    {"wait", "počkať 3 s"},
    {"search", "hľadať objekt (sken 360°)"},
};

// Mission recordings (and photos taken outside a mission) live in zadanie2/zaznamy.
// HMI_RECORDS_DIR is set by CMake; without it, fall back to a folder next to the executable.
QString recordsRoot()
{
#ifdef HMI_RECORDS_DIR
    return QString::fromUtf8(HMI_RECORDS_DIR);
#else
    return QDir(QCoreApplication::applicationDirPath()).filePath("zaznamy");
#endif
}

QString taskLabel(const QString &id)
{
    for (const TaskDef &t : kTasks) if (id == t.id) return QString::fromUtf8(t.label);
    return id;
}

double normAngle(double a)
{
    while (a > kPi) a -= 2 * kPi;
    while (a < -kPi) a += 2 * kPi;
    return a;
}

QString fmtMs(qint64 ms)
{
    const qint64 m = ms / 60000;
    const double s = (ms % 60000) / 1000.0;
    return QString("%1:%2").arg(m, 2, 10, QChar('0')).arg(s, 4, 'f', 1, QChar('0'));
}

QString poseText(const MainWindow::Pose &p)
{
    return QString("x %1  y %2  θ %3°").arg(p.x, 0, 'f', 2).arg(p.y, 0, 'f', 2)
            .arg(qRound(p.th * 180.0 / kPi));
}

QPointF evPos(QMouseEvent *e)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    return e->position();
#else
    return e->localPos();
#endif
}

// Marks every cell within `radius` cells of a source cell. Only boundary source cells are stamped.
QVector<quint8> dilate(const QVector<quint8> &grid, int w, int h, int radius,
                       const std::function<bool(quint8)> &isSource)
{
    QVector<quint8> out(grid.size(), 0);
    QVector<QPoint> disc;
    for (int dy = -radius; dy <= radius; ++dy)
        for (int dx = -radius; dx <= radius; ++dx)
            if (dx * dx + dy * dy <= radius * radius) disc.append(QPoint(dx, dy));
    auto src = [&](int x, int y) { return x < 0 || y < 0 || x >= w || y >= h || isSource(grid[y * w + x]); };
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            if (!isSource(grid[y * w + x])) continue;
            out[y * w + x] = 1;
            if (src(x - 1, y) && src(x + 1, y) && src(x, y - 1) && src(x, y + 1)) continue;
            for (const QPoint &d : disc) {
                const int nx = x + d.x(), ny = y + d.y();
                if (nx >= 0 && ny >= 0 && nx < w && ny < h) out[ny * w + nx] = 1;
            }
        }
    return out;
}
} // namespace

// ============================================================================
// Construction
// ============================================================================

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->mapView->installEventFilter(this);
    ui->cameraLabel->installEventFilter(this);
    ui->timelineMarks->installEventFilter(this);

    // Overlays inside the map view.
    toastLabel_ = new QLabel(ui->mapView);
    toastLabel_->setAlignment(Qt::AlignCenter);
    toastLabel_->hide();
    cursorLabel_ = new QLabel(ui->mapView);
    cursorLabel_->setStyleSheet("color: #ccd5df; font-size: 13px;");
    replayBadge_ = new QLabel(QString::fromUtf8("PREHRÁVANIE ZÁZNAMU – nie sú to živé dáta"), ui->mapView);
    replayBadge_->setStyleSheet("background: #1e3a5f; border: 1px solid #60a5fa; color: #dbeafe;"
                                "font-size: 13px; font-weight: bold; padding: 4px 10px; border-radius: 3px;");
    replayBadge_->adjustSize();
    replayBadge_->hide();
    toastTimer_.setSingleShot(true);
    connect(&toastTimer_, &QTimer::timeout, toastLabel_, &QLabel::hide);

    ui->legendLabel->setText(QString::fromUtf8(
        "<span style='color:#00008B'>■</span> Operačný bod &nbsp;&nbsp;"
        "<span style='color:#2375bd'>●</span> Prejazdový bod &nbsp;&nbsp;"
        "<span style='color:#2375bd'>━</span> Trasa &nbsp;&nbsp;"
        "<span style='color:#dc2626'>╍</span> Kolízia &nbsp;&nbsp;"
        "<span style='color:#0f766e'>•••</span> Prejdená dráha &nbsp;&nbsp;"
        "<span style='color:#6d28d9'>◆</span> Nájdený objekt &nbsp;&nbsp;"
        "<span style='color:#0284c7'>···</span> Lidar"));

    ui->pointsTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    ui->pointsTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->pointsTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    ui->pointsTable->verticalHeader()->setDefaultSectionSize(32);
    ui->foundList->setIconSize(QSize(96, 64));

    // Header
    connect(ui->pushButton_9, &QPushButton::clicked, this, &MainWindow::toggleConnection);
    connect(ui->planModeButton, &QPushButton::clicked, this, [this]{ setMode(Mode::Planning); });
    connect(ui->recordsModeButton, &QPushButton::clicked, this, [this]{ setMode(Mode::Records); });
    connect(ui->safetyStopButton, &QPushButton::toggled, this, &MainWindow::setSafetyStopActive);

    // Planning
    connect(ui->pointUpButton, &QPushButton::clicked, this, [this]{ movePoint(-1); });
    connect(ui->pointDownButton, &QPushButton::clicked, this, [this]{ movePoint(1); });
    connect(ui->pointDeleteButton, &QPushButton::clicked, this, &MainWindow::deleteSelectedPoint);
    connect(ui->clearMissionButton, &QPushButton::clicked, this, &MainWindow::clearMission);
    connect(ui->pointsTable, &QTableWidget::currentCellChanged, this, [this](int row){
        if (!tableUpdating_) selectPoint(row);
    });
    connect(ui->startMissionButton, &QPushButton::clicked, this, &MainWindow::startMission);
    connect(ui->pauseMissionButton, &QPushButton::clicked, this, [this]{
        pauseMission(QString::fromUtf8("Misia pozastavená operátorom"));
    });
    connect(ui->resumeMissionButton, &QPushButton::clicked, this, &MainWindow::resumeMission);
    connect(ui->abortMissionButton, &QPushButton::clicked, this, &MainWindow::abortMission);
    connect(ui->foundList, &QListWidget::itemClicked, this, [this](QListWidgetItem *item){
        const int i = item->data(Qt::UserRole).toInt();
        focusedDetection_ = focusedDetection_ == i ? -1 : i;
        ui->mapView->update();
    });

    // Records / playback
    connect(ui->refreshRecordsButton, &QPushButton::clicked, this, &MainWindow::refreshRecords);
    connect(ui->deleteRecordButton, &QPushButton::clicked, this, &MainWindow::deleteSelectedRecord);
    connect(ui->recordsList, &QListWidget::currentItemChanged, this, [this](QListWidgetItem *item){
        if (item) loadRecording(item->data(Qt::UserRole).toString());
    });
    connect(ui->playButton, &QPushButton::clicked, this, &MainWindow::togglePlayback);
    connect(ui->toStartButton, &QPushButton::clicked, this, [this]{ seekPlayback(0); });
    connect(ui->back5Button, &QPushButton::clicked, this, [this]{ seekPlayback(playPos_ - 5000); });
    connect(ui->fwd5Button, &QPushButton::clicked, this, [this]{ seekPlayback(playPos_ + 5000); });
    connect(ui->timelineSlider, &QSlider::valueChanged, this, [this](int v){
        if (!sliderUpdating_) seekPlayback(v);
    });
    connect(ui->speedComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int i){
        static const double speeds[] = {0.5, 1.0, 2.0, 4.0};
        playSpeed_ = speeds[qBound(0, i, 3)];
    });
    for (QCheckBox *c : {ui->layerAmclCheck, ui->layerOdomCheck, ui->layerLidarCheck, ui->layerCameraCheck})
        connect(c, &QCheckBox::toggled, this, [this]{ ui->mapView->update(); ui->cameraLabel->update(); });

    // Adapter connections: replace these when the project backend is ready.
    connect(this, &MainWindow::connectRequested, this, &MainWindow::startRobotSession);
    connect(this, &MainWindow::disconnectRequested, this, [this]{ endRobotSession(); });
    connect(this, &MainWindow::stopRequested, this, [this]{
        lastForward_ = lastRotation_ = 0.0;
        if (robot_ && connected_) robot_->setSpeed(0, 0);
    });

    connect(&sessionTimer_, &QTimer::timeout, this, &MainWindow::checkRobotSession);
    sessionTimer_.setInterval(200);
    connect(&missionTimer_, &QTimer::timeout, this, &MainWindow::missionTick);
    missionTimer_.setInterval(100);
    connect(&recordTimer_, &QTimer::timeout, this, &MainWindow::recordTick);
    recordTimer_.setInterval(100);
    connect(&playbackTimer_, &QTimer::timeout, this, &MainWindow::playbackTick);
    playbackTimer_.setInterval(50);

    setConnectionState(false);
    setSafetyStopActive(false);
    setMode(Mode::Planning);
    ui->cameraLabel->setCursor(Qt::PointingHandCursor);
    ui->cameraLabel->setToolTip(QString::fromUtf8("Kliknutím prehodíte s hlavným zobrazením"));
    addLog(QString::fromUtf8("Aplikácia spustená – pripojte robota"));
    // Same occupancy map the AMCL localization loads (copied next to the executable by CMake).
    const QString mapPath = QCoreApplication::applicationDirPath() + "/mapa.txt";
    if (!loadMap(mapPath))
        addLog(QString::fromUtf8("Mapu %1 sa nepodarilo načítať").arg(mapPath), LogLevel::Danger);
    refreshPointsTable();
    updateValidation();
    updateMissionUi();
    refreshAlarms();
}

MainWindow::~MainWindow()
{
    if (missionState_ == MissionState::Running || missionState_ == MissionState::Paused)
        finishMission(MissionState::Aborted);
    stopRecording();
    endRobotSession();
    delete ui;
}

// ============================================================================
// Robot session (protocol, ports and camera URL as in task 1)
// ============================================================================

void MainWindow::toggleConnection()
{
    if (connectionPending_) return;
    if (connected_) {
        connectionPending_ = true;
        ui->pushButton_9->setEnabled(false);
        ui->connectionStatusLabel->setText(QString::fromUtf8("Odpájam…"));
        emit stopRequested();
        emit disconnectRequested();
        return;
    }
    const QString address = ui->lineEdit->text().trimmed();
    const QStringList octets = address.split('.');
    bool valid = octets.size() == 4;
    for (const QString &octet : octets) {
        bool ok = false;
        const int value = octet.toInt(&ok);
        valid = valid && ok && value >= 0 && value <= 255
                && QRegularExpression("^[0-9]{1,3}$").match(octet).hasMatch();
    }
    if (!valid) {
        QMessageBox::warning(this, "IP adresa", QString::fromUtf8("Zadajte IPv4 adresu, napríklad 127.0.0.1."));
        return;
    }
    connectionPending_ = true;
    ui->lineEdit->setReadOnly(true);
    ui->pushButton_9->setEnabled(false);
    ui->connectionStatusLabel->setText(QString::fromUtf8("Pripájam…"));
    emit connectRequested(address, address.startsWith("127."));
}

void MainWindow::setConnectionState(bool connected, const QString &message)
{
    const bool changed = connected_ != connected;
    connected_ = connected;
    connectionPending_ = false;
    ui->pushButton_9->setEnabled(true);
    ui->pushButton_9->setText(connected ? QString::fromUtf8("Odpojiť") : QString::fromUtf8("Pripojiť"));
    ui->lineEdit->setReadOnly(connected);
    ui->connectionStatusLabel->setText(message.isEmpty()
            ? (connected ? QString::fromUtf8("Pripojený") : QString::fromUtf8("Odpojený")) : message);
    ui->connectionDotLabel->setStyleSheet(connected ? "color: #16a34a; font-size: 20px;" : "color: #808890; font-size: 20px;");
    if (connected && safetyStopActive_) emit stopRequested();
    if (!connected) {
        setCameraImage(QImage());
        lidarPoints_.clear();
        lidarRaw_.clear();
        obstacleAhead_ = false;
        offMission_ = false;
        badLocalization_.invalidate();
    }
    if (changed)
        addLog(connected ? QString::fromUtf8("Spojenie s robotom nadviazané")
                         : QString::fromUtf8("Spojenie s robotom ukončené"),
               connected ? LogLevel::Ok : LogLevel::Danger);
    refreshAlarms();
    updateValidation();
    ui->mapView->update();
}

void MainWindow::startRobotSession(const QString &address, bool simulation)
{
    Q_UNUSED(simulation);
    if (robot_) return;
    robot_ = new robot();
    const unsigned id = ++sessionId_;
    telemetryAge_.start();
    imageAge_.invalidate();
    connect(robot_, &robot::telemetryReceived, this, [this, id]{
        if (id != sessionId_ || !robot_) return;
        telemetryAge_.restart();
        if (!connected_) setConnectionState(true);
    }, Qt::QueuedConnection);
#ifndef DISABLE_OPENCV
    connect(robot_, &robot::publishCamera, this, [this, id](const cv::Mat &frame){
        if (id != sessionId_ || !robot_ || frame.empty() || frame.depth() != CV_8U) return;
        cv::Mat rgb;
        if (frame.channels() == 3) cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
        else if (frame.channels() == 4) cv::cvtColor(frame, rgb, cv::COLOR_BGRA2RGB);
        else if (frame.channels() == 1) cv::cvtColor(frame, rgb, cv::COLOR_GRAY2RGB);
        else return;
        const QImage image(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888);
        handleCameraFrame(image.copy());
        imageAge_.restart();
    }, Qt::QueuedConnection);
#endif
    connect(robot_, &robot::publishLidar, this, [this, id](const std::vector<LaserData> &scan){
        if (id != sessionId_ || !robot_) return;
        // RPLidar: angle in degrees clockwise from forward, distance in mm.
        QVector<QPointF> points, raw;
        points.reserve(int(scan.size()));
        raw.reserve(int(scan.size()));
        for (const LaserData &d : scan) {
            if (!std::isfinite(d.scanDistance) || !std::isfinite(d.scanAngle) || d.scanDistance <= 0.0f)
                continue;
            const double angle = d.scanAngle * kPi / 180.0;
            points.append(QPointF(d.scanDistance * std::cos(angle), d.scanDistance * std::sin(angle)));
            raw.append(QPointF(d.scanAngle, d.scanDistance));
        }
        handleScan(points, raw);
    }, Qt::QueuedConnection);
#if DEMO_HAS_AMCL
    // AMCL pose: millimetres in the grid frame of mapa.txt (file row 0 at y = 0, y grows down the
    // file, theta in rad measured towards +y). The HMI map frame is metres with y up (file row 0
    // drawn at the top), so y is mirrored about the map height and theta changes sign.
    connect(robot_, &robot::publishAMCLPosition, this, [this, id](double x, double y, double theta){
        if (id != sessionId_ || !robot_ || !mapLoaded_) return;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(theta)) return;
        const double mx = x / 1000.0;
        const double my = 2.0 * mapOrigin_.y() + mapH_ * mapRes_ - y / 1000.0;
        setRobotPose(mx, my, normAngle(-theta));
    }, Qt::QueuedConnection);
#endif
    robot_->initAndStartRobot(address.toStdString());
    sessionTimer_.start();
}

void MainWindow::endRobotSession(const QString &message)
{
    sessionTimer_.stop();
    ++sessionId_; // Ignore queued frames/telemetry from the old session.
    if (robot_) {
        if (connected_) robot_->setSpeed(0, 0);
        delete robot_; // ~libRobot() stops the communication threads
        robot_ = nullptr;
    }
    imageAge_.invalidate();
    lastForward_ = lastRotation_ = 1e9;
    if (ui) setConnectionState(false, message);
}

void MainWindow::checkRobotSession()
{
    if (!robot_) return;
    const int timeout = connected_ ? kTelemetryTimeoutMs : 5000;
    if (telemetryAge_.elapsed() > timeout) {
        endRobotSession(connected_ ? QString::fromUtf8("Spojenie stratené")
                                   : QString::fromUtf8("Bez dát – skúste znova"));
        return;
    }
    if (imageAge_.isValid() && imageAge_.elapsed() > 3000) {
        setCameraImage(QImage());
        imageAge_.invalidate();
    }
}

void MainWindow::sendSpeed(double forward, double rotation)
{
    if (!robot_ || !connected_) return;
    if (safetyStopActive_) { forward = 0; rotation = 0; }
    if (forward == lastForward_ && rotation == lastRotation_) return;
    lastForward_ = forward;
    lastRotation_ = rotation;
    robot_->setSpeed(forward, rotation);
}

void MainWindow::handleScan(const QVector<QPointF> &robotPointsMm, const QVector<QPointF> &rawAngleDist)
{
    lidarPoints_ = robotPointsMm;
    lidarRaw_ = rawAngleDist;
    ++scanSerial_;

    // Obstacle in the driving corridor in front of the robot.
    const double halfWidth = kRobotRadius * 1000.0 + 40.0;
    bool obstacle = false;
    for (const QPointF &p : lidarPoints_)
        if (p.x() > 0 && p.x() < kObstacleMm && std::abs(p.y()) < halfWidth) { obstacle = true; break; }
    if (obstacle != obstacleAhead_) {
        obstacleAhead_ = obstacle;
        if (missionState_ == MissionState::Running && !inTask_)
            addLog(obstacle ? QString::fromUtf8("Prekážka na trase – robot zastavil")
                            : QString::fromUtf8("Prekážka odstránená – robot pokračuje"),
                   obstacle ? LogLevel::Warning : LogLevel::Ok);
        refreshAlarms();
    }

    // Localization check: share of scan points that lie on known walls of the map.
    if (mapLoaded_ && hasPose_) {
        int valid = 0, hits = 0;
        const double c = std::cos(pose_.th), s = std::sin(pose_.th);
        for (const QPointF &p : lidarPoints_) {
            if (std::hypot(p.x(), p.y()) > 6000.0) continue;
            const double px = p.x() / 1000.0, py = p.y() / 1000.0;
            const QPointF w(pose_.x + px * c + py * s, pose_.y + px * s - py * c);
            int cx, cy;
            if (!cellOf(w, cx, cy)) continue;
            ++valid;
            if (nearOccupied_[cy * mapW_ + cx]) ++hits;
        }
        if (valid >= 20) {
            locQuality_ = double(hits) / valid;
            const bool bad = locQuality_ < kLocThreshold;
            if (bad && !badLocalization_.isValid()) badLocalization_.start();
            if (!bad) badLocalization_.invalidate();
            const bool off = bad && badLocalization_.elapsed() > kLocTimeoutMs;
            if (off != offMission_) {
                offMission_ = off;
                addLog(off ? QString::fromUtf8("Robot mimo misie – sken nesedí s mapou (%1 %)").arg(qRound(locQuality_ * 100))
                           : QString::fromUtf8("Lokalizácia obnovená"),
                       off ? LogLevel::Danger : LogLevel::Ok);
                refreshAlarms();
                updateValidation();
            }
        }
    }
    if (mode_ == Mode::Planning) ui->mapView->update();
}

void MainWindow::handleCameraFrame(const QImage &rgb)
{
    setCameraImage(rgb);
    if (recording_ && (!cameraRecordAge_.isValid() || cameraRecordAge_.elapsed() >= 200)) {
        cameraRecordAge_.restart();
        const QString file = QString("camera/%1.jpg").arg(cameraFrameNo_++, 6, 10, QChar('0'));
        if (rgb.save(QDir(recDir_).filePath(file), "JPG", 80))
            cameraOut_ << missionTimeMs() << ',' << file << '\n';
    }
    if (missionState_ == MissionState::Running && (++frameCounter_ % 3) == 0) {
        QRectF box;
        double confidence = 0.0;
        if (detectTarget(rgb, box, confidence)) handleDetection(box, confidence, rgb);
    }
}

void MainWindow::setCameraImage(const QImage &image)
{
    cameraImage_ = image.copy();
    ui->cameraLabel->update();
}

void MainWindow::setRobotPose(double x, double y, double theta)
{
    pose_ = {x, y, theta};
    const bool first = !hasPose_;
    hasPose_ = true;
    if (!hasOdom_) odom_ = pose_;
    if (missionState_ == MissionState::Running
            && (trail_.isEmpty() || std::hypot(trail_.last().x() - x, trail_.last().y() - y) > 0.05))
        trail_.append(QPointF(x, y));
    if (first) updateValidation();
    if (mode_ == Mode::Planning) ui->mapView->update();
}

void MainWindow::setOdometryPose(double x, double y, double theta)
{
    odom_ = {x, y, theta};
    hasOdom_ = true;
}

void MainWindow::setBatteryLevel(int percent)
{
    const int old = battery_;
    battery_ = qBound(0, percent, 100);
    const auto level = [](int b) { return b < 0 ? 0 : b < 10 ? 3 : b < 20 ? 2 : 1; };
    if (level(old) != level(battery_)) {
        if (battery_ < 10) addLog(QString::fromUtf8("Kritická batéria %1 %").arg(battery_), LogLevel::Danger);
        else if (battery_ < 20) addLog(QString::fromUtf8("Nízka batéria %1 %").arg(battery_), LogLevel::Warning);
        refreshAlarms();
        updateValidation();
    } else {
        refreshAlarms();
    }
}

// Latched UI stop: switching OFF only permits future commands; it never resumes the mission.
void MainWindow::setSafetyStopActive(bool active)
{
    const bool changed = safetyStopActive_ != active;
    safetyStopActive_ = active;
    {
        const QSignalBlocker blocker(ui->safetyStopButton);
        ui->safetyStopButton->setChecked(active);
        ui->safetyStopButton->setText(active ? QString::fromUtf8("SAFETY STOP\nZAPNUTÝ")
                                             : QString::fromUtf8("SAFETY STOP\nVYPNUTÝ"));
    }
    if (active) {
        emit stopRequested();
        if (missionState_ == MissionState::Running)
            pauseMission(QString::fromUtf8("SAFETY STOP – misia pozastavená"), LogLevel::Danger);
    }
    if (changed) {
        addLog(active ? QString::fromUtf8("SAFETY STOP aktivovaný")
                      : QString::fromUtf8("SAFETY STOP uvoľnený – misiu obnovte tlačidlom Pokračovať"),
               active ? LogLevel::Danger : LogLevel::Warning);
        emit safetyStopChanged(active);
    }
    refreshAlarms();
    updateValidation();
    updateMissionUi();
}

// ============================================================================
// Map
// ============================================================================

// AMCL map format (mapa.txt), the same file librobot's AMCL loads:
//   header "offsetX offsetY resX resY", then one text row per grid row, '1' = wall, '0' = free.
// AMCL works in millimetres and res is in cells per mm (0.01 -> one cell = 100 mm),
// consistent with the demo that adds lidar scanDistance [mm] to the AMCL pose.
// Row 0 of the file is drawn at the top, as in the original demo.
bool MainWindow::loadMap(const QString &path)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
    QTextStream in(&f);
    double offX = 0, offY = 0, resX = 0, resY = 0;
    in >> offX >> offY >> resX >> resY;
    if (in.status() != QTextStream::Ok || resX <= 0) return false;
    in.readLine(); // rest of the header line

    QStringList rows;
    int width = 0;
    while (!in.atEnd()) {
        const QString line = in.readLine().trimmed();
        if (line.isEmpty()) continue;
        rows.append(line);
        width = std::max(width, int(line.size()));
    }
    if (rows.isEmpty() || width == 0) return false;

    const double res = 0.001 / resX;   // metres per cell
    mapW_ = width;
    mapH_ = rows.size();
    mapRes_ = res;
    mapOrigin_ = QPointF(offX / 1000.0, offY / 1000.0);
    occupancy_.fill(0, mapW_ * mapH_);
    mapImage_ = QImage(mapW_, mapH_, QImage::Format_RGB32);
    for (int y = 0; y < mapH_; ++y) {
        const QString &row = rows[y];
        for (int x = 0; x < mapW_; ++x) {
            const quint8 v = (x < row.size() && row[x] == QLatin1Char('1')) ? 1 : 0; // short rows = free
            occupancy_[y * mapW_ + x] = v;
            mapImage_.setPixel(x, y, v == 1 ? qRgb(0x2b, 0x34, 0x40) : qRgb(0xf5, 0xf7, 0xfa));
        }
    }
    const int inflate = int(std::ceil((kRobotRadius + kClearance) / res));
    blocked_ = dilate(occupancy_, mapW_, mapH_, inflate, [](quint8 v) { return v != 0; });
    nearOccupied_ = dilate(occupancy_, mapW_, mapH_, 2, [](quint8 v) { return v == 1; });

    mapLoaded_ = true;
    mapFile_ = path;
    addLog(QString::fromUtf8("Načítaná mapa %1 (%2 × %3 m, bunka %4 m)")
           .arg(QFileInfo(path).fileName())
           .arg(mapW_ * res, 0, 'f', 1).arg(mapH_ * res, 0, 'f', 1).arg(res));
    refreshPointsTable();
    updateValidation();
    ui->mapView->update();
    return true;
}

QRectF MainWindow::mapRect() const
{
    if (!mapLoaded_ || mapW_ <= 0 || mapH_ <= 0) return QRectF();
    const QRectF area = QRectF(ui->mapView->rect()).adjusted(16, 40, -16, -16);
    if (area.width() <= 0 || area.height() <= 0) return QRectF();
    const double s = std::min(area.width() / mapW_, area.height() / mapH_);
    const QSizeF size(mapW_ * s, mapH_ * s);
    return QRectF(area.center() - QPointF(size.width() / 2, size.height() / 2), size);
}

QPointF MainWindow::worldToWidget(const QPointF &m) const
{
    const QRectF r = mapRect();
    const double s = r.width() / mapW_;
    const double cx = (m.x() - mapOrigin_.x()) / mapRes_;
    const double cy = mapH_ - (m.y() - mapOrigin_.y()) / mapRes_;
    return r.topLeft() + QPointF(cx * s, cy * s);
}

QPointF MainWindow::widgetToWorld(const QPointF &w) const
{
    const QRectF r = mapRect();
    const double s = r.width() / mapW_;
    const double cx = (w.x() - r.left()) / s;
    const double cy = (w.y() - r.top()) / s;
    return QPointF(mapOrigin_.x() + cx * mapRes_, mapOrigin_.y() + (mapH_ - cy) * mapRes_);
}

bool MainWindow::cellOf(const QPointF &m, int &cx, int &cy) const
{
    if (!mapLoaded_) return false;
    cx = int(std::floor((m.x() - mapOrigin_.x()) / mapRes_));
    cy = mapH_ - 1 - int(std::floor((m.y() - mapOrigin_.y()) / mapRes_));
    return cx >= 0 && cy >= 0 && cx < mapW_ && cy < mapH_;
}

bool MainWindow::isBlocked(const QPointF &m) const
{
    int cx, cy;
    if (!cellOf(m, cx, cy)) return true;
    return blocked_[cy * mapW_ + cx] != 0;
}

bool MainWindow::segmentBlocked(const QPointF &a, const QPointF &b) const
{
    const double len = std::hypot(b.x() - a.x(), b.y() - a.y());
    const int n = std::max(1, int(std::ceil(len / (mapRes_ * 0.5))));
    for (int i = 0; i <= n; ++i) {
        const double t = double(i) / n;
        if (isBlocked(a + (b - a) * t)) return true;
    }
    return false;
}

void MainWindow::paintMap()
{
    QWidget *view = ui->mapView;
    QPainter p(view);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(view->rect(), QColor("#242d38"));

    if (!mapLoaded_) {
        p.setPen(QColor("#ccd5df"));
        p.drawText(view->rect(), Qt::AlignCenter, QString::fromUtf8("Mapa nie je načítaná\nSúbor mapa.txt chýba pri aplikácii"));
        return;
    }
    const QRectF r = mapRect();
    if (r.isEmpty()) return;
    const double s = r.width() / mapW_;
    const double pxPerM = s / mapRes_;
    const bool replay = mode_ == Mode::Records && recLoaded_;

    // Map and 1 m grid
    p.drawImage(r, mapImage_);
    p.save();
    p.setClipRect(r);
    p.setPen(QPen(QColor(120, 134, 152, 50), 1));
    const double x0 = std::ceil(mapOrigin_.x()), y0 = std::ceil(mapOrigin_.y());
    for (double x = x0; x < mapOrigin_.x() + mapW_ * mapRes_; x += 1.0) {
        const double wx = worldToWidget(QPointF(x, 0)).x();
        p.drawLine(QPointF(wx, r.top()), QPointF(wx, r.bottom()));
    }
    for (double y = y0; y < mapOrigin_.y() + mapH_ * mapRes_; y += 1.0) {
        const double wy = worldToWidget(QPointF(0, y)).y();
        p.drawLine(QPointF(r.left(), wy), QPointF(r.right(), wy));
    }
    p.restore();

    const QVector<MissionPoint> &pts = replay ? rec_.points : points_;
    const bool live = !replay;
    const bool runningView = live && (missionState_ == MissionState::Running || missionState_ == MissionState::Paused);

    // Route: from the robot (or the recording start) through all points.
    QVector<QPointF> chain;
    if (replay) { if (!rec_.poses.isEmpty()) chain.append(QPointF(rec_.poses.first().amcl.x, rec_.poses.first().amcl.y)); }
    else if (hasPose_) chain.append(runningView && !trail_.isEmpty() ? trail_.first() : QPointF(pose_.x, pose_.y));
    for (const MissionPoint &mp : pts) chain.append(mp.pos);
    for (int i = 1; i < chain.size(); ++i) {
        const QPointF a = worldToWidget(chain[i - 1]), b = worldToWidget(chain[i]);
        const bool collision = live && !runningView && segmentBlocked(chain[i - 1], chain[i]);
        if (collision) {
            p.setPen(QPen(kCollisionColor, 3, Qt::DashLine, Qt::FlatCap));
            p.drawLine(a, b);
            const QPointF mid = (a + b) / 2;
            p.setPen(Qt::NoPen);
            p.setBrush(kCollisionColor);
            p.drawEllipse(mid, 10, 10);
            p.setPen(QPen(Qt::white, 2));
            p.drawLine(mid + QPointF(-4, -4), mid + QPointF(4, 4));
            p.drawLine(mid + QPointF(-4, 4), mid + QPointF(4, -4));
        } else {
            QColor c = kRouteColor;
            if (runningView || replay) c.setAlpha(110);
            p.setPen(QPen(c, 3, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(a, b);
        }
    }

    // Trails
    auto drawTrail = [&](const QVector<QPointF> &trail, const QColor &color, Qt::PenStyle style) {
        if (trail.size() < 2) return;
        QPolygonF poly;
        for (const QPointF &t : trail) poly << worldToWidget(t);
        p.setPen(QPen(color, 2.5, style, Qt::RoundCap, Qt::RoundJoin));
        p.setBrush(Qt::NoBrush);
        p.drawPolyline(poly);
    };
    int replayIndex = -1;
    if (replay && !rec_.poses.isEmpty()) {
        auto it = std::upper_bound(rec_.poses.begin(), rec_.poses.end(), playPos_,
                                   [](qint64 t, const RecSample &s) { return t < s.t; });
        replayIndex = std::max(0, int(it - rec_.poses.begin()) - 1);
        QVector<QPointF> amcl, odom;
        for (int i = 0; i <= replayIndex; i += 2) {
            amcl << QPointF(rec_.poses[i].amcl.x, rec_.poses[i].amcl.y);
            odom << QPointF(rec_.poses[i].odom.x, rec_.poses[i].odom.y);
        }
        if (ui->layerOdomCheck->isChecked()) drawTrail(odom, kOdomColor, Qt::DashLine);
        if (ui->layerAmclCheck->isChecked()) drawTrail(amcl, kTrailColor, Qt::SolidLine);
    } else if (runningView || missionState_ == MissionState::Done || missionState_ == MissionState::Aborted) {
        drawTrail(trail_, kTrailColor, Qt::SolidLine);
    }

    // Lidar scan in the map frame
    const Pose scanPose = replay ? replayPose_ : pose_;
    const QVector<QPointF> &scan = replay ? replayScan_ : lidarPoints_;
    const bool showScan = replay ? ui->layerLidarCheck->isChecked() : (connected_ && hasPose_);
    if (showScan) {
        const double c = std::cos(scanPose.th), sn = std::sin(scanPose.th);
        p.setPen(Qt::NoPen);
        for (const QPointF &pt : scan) {
            const double d = std::hypot(pt.x(), pt.y());
            if (d > 8000.0) continue;
            const double px = pt.x() / 1000.0, py = pt.y() / 1000.0;
            const QPointF w(scanPose.x + px * c + py * sn, scanPose.y + px * sn - py * c);
            p.setBrush(d < 1500.0 ? QColor("#0c4a6e") : QColor("#0284c7"));
            p.drawEllipse(worldToWidget(w), 1.8, 1.8);
        }
    }

    // Mission points
    QFont numberFont = p.font();
    numberFont.setPixelSize(12);
    numberFont.setBold(true);
    p.setFont(numberFont);
    for (int i = 0; i < pts.size(); ++i) {
        const MissionPoint &mp = pts[i];
        const QPointF c = worldToWidget(mp.pos);
        const bool bad = live && !runningView && isBlocked(mp.pos);
        const bool sel = live && i == selected_;
        const bool current = runningView && i == currentIndex_;
        QColor fill = bad ? kCollisionColor : (mp.operation ? kOpColor : kPassColor);
        p.setPen(QPen(sel ? QColor("#facc15") : (current ? QColor("#5eead4") : Qt::white), sel || current ? 3 : 2));
        p.setBrush(fill);
        if (mp.operation) p.drawRoundedRect(QRectF(c - QPointF(13, 13), QSizeF(26, 26)), 4, 4);
        else p.drawEllipse(c, 11, 11);
        p.setPen(Qt::white);
        p.drawText(QRectF(c - QPointF(13, 13), QSizeF(26, 26)), Qt::AlignCenter, QString::number(i + 1));
    }
    if (hasPending_ && live) {
        const QPointF c = worldToWidget(pendingPos_);
        p.setPen(QPen(kOpColor, 2, Qt::DashLine));
        p.setBrush(QColor(255, 255, 255, 120));
        p.drawEllipse(c, 12, 12);
    }

    // Detected objects
    const QVector<Detection> &dets = replay ? rec_.detections : detections_;
    QFont labelFont = p.font();
    labelFont.setPixelSize(11);
    p.setFont(labelFont);
    for (int i = 0; i < dets.size(); ++i) {
        const Detection &d = dets[i];
        if (replay && d.t > playPos_) continue;
        const QPointF c = worldToWidget(d.pos);
        if (live && i == focusedDetection_) {
            p.setPen(QPen(kDetColor, 2));
            p.setBrush(Qt::NoBrush);
            p.drawEllipse(c, 18, 18);
        }
        QPolygonF diamond;
        diamond << c + QPointF(0, -10) << c + QPointF(10, 0) << c + QPointF(0, 10) << c + QPointF(-10, 0);
        p.setPen(QPen(Qt::white, 2));
        p.setBrush(kDetColor);
        p.drawPolygon(diamond);
        const QString label = QString("%1 %2 %").arg(d.name).arg(qRound(d.confidence * 100));
        const QRectF lr(c + QPointF(14, -10), QSizeF(p.fontMetrics().horizontalAdvance(label) + 12, 20));
        p.setPen(Qt::NoPen);
        p.drawRoundedRect(lr, 2, 2);
        p.setPen(Qt::white);
        p.drawText(lr, Qt::AlignCenter, label);
    }

    // Obstacle ahead of the robot
    if (live && obstacleAhead_ && hasPose_ && connected_) {
        const QPointF o(pose_.x + 0.4 * std::cos(pose_.th), pose_.y + 0.4 * std::sin(pose_.th));
        p.setPen(QPen(Qt::white, 2));
        p.setBrush(kCollisionColor);
        p.drawRoundedRect(QRectF(worldToWidget(o) - QPointF(9, 9), QSizeF(18, 18)), 4, 4);
    }

    // Robots: odometry ghost (replay) and AMCL pose
    const double robotPx = std::max(8.0, kRobotRadius * pxPerM);
    auto drawRobot = [&](const Pose &rp, bool ghost) {
        const QPointF c = worldToWidget(QPointF(rp.x, rp.y));
        const QPointF head = c + QPointF(std::cos(rp.th), -std::sin(rp.th)) * robotPx;
        if (ghost) {
            p.setPen(QPen(kOdomColor, 2, Qt::DashLine));
            p.setBrush(Qt::NoBrush);
            p.drawEllipse(c, robotPx, robotPx);
            p.drawLine(c, head);
        } else {
            p.setPen(QPen(Qt::white, 2));
            p.setBrush(kRobotColor);
            p.drawEllipse(c, robotPx, robotPx);
            p.setPen(QPen(QColor("#5eead4"), 2.5, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(c, head);
        }
    };
    if (replay && replayIndex >= 0) {
        if (ui->layerOdomCheck->isChecked()) drawRobot(replayOdom_, true);
        if (ui->layerAmclCheck->isChecked()) drawRobot(replayPose_, false);
    } else if (live && hasPose_) {
        drawRobot(pose_, false);
    }

    // Scale bar
    const QPointF sb = r.bottomLeft() + QPointF(12, -14);
    p.setPen(QPen(QColor("#14243b"), 2));
    p.drawLine(sb, sb + QPointF(pxPerM, 0));
    p.drawLine(sb, sb + QPointF(0, -5));
    p.drawLine(sb + QPointF(pxPerM, 0), sb + QPointF(pxPerM, -5));
    p.drawText(QRectF(sb + QPointF(0, -22), QSizeF(std::max(40.0, pxPerM), 14)), Qt::AlignLeft, "1 m");
}

// ============================================================================
// Mission editor – points are added only by clicking into the map
// ============================================================================

bool MainWindow::editable() const
{
    return mode_ == Mode::Planning && mapLoaded_
            && missionState_ != MissionState::Running && missionState_ != MissionState::Paused;
}

int MainWindow::pointAt(const QPointF &widgetPos) const
{
    for (int i = points_.size() - 1; i >= 0; --i) {
        const QPointF c = worldToWidget(points_[i].pos);
        if (std::hypot(c.x() - widgetPos.x(), c.y() - widgetPos.y()) <= 14.0) return i;
    }
    return -1;
}

void MainWindow::showAddPointMenu(const QPointF &world, const QPoint &globalPos)
{
    hasPending_ = true;
    pendingPos_ = world;
    ui->mapView->update();

    QMenu menu(this);
    QAction *title = menu.addAction(QString::fromUtf8("Bod x %1 m, y %2 m").arg(world.x(), 0, 'f', 2).arg(world.y(), 0, 'f', 2));
    title->setEnabled(false);
    menu.addSeparator();
    for (const TaskDef &t : kTasks) {
        QAction *a = menu.addAction(QString::fromUtf8("Operačný bod – %1").arg(QString::fromUtf8(t.label)));
        a->setData(QString::fromUtf8(t.id));
    }
    menu.addSeparator();
    QAction *pass = menu.addAction(QString::fromUtf8("Prejazdový bod"));
    pass->setData(QString("pass"));
    menu.addSeparator();
    menu.addAction(QString::fromUtf8("Zrušiť"));

    QAction *chosen = menu.exec(globalPos);
    hasPending_ = false;
    if (chosen && chosen->data().isValid()) {
        MissionPoint mp;
        mp.pos = world;
        const QString id = chosen->data().toString();
        mp.operation = id != "pass";
        if (mp.operation) mp.task = id;
        insertPoint(mp);
    }
    ui->mapView->update();
}

void MainWindow::showPointMenu(int index, const QPoint &globalPos)
{
    QMenu menu(this);
    MissionPoint &mp = points_[index];
    QAction *title = menu.addAction(QString::fromUtf8("Bod %1").arg(index + 1));
    title->setEnabled(false);
    menu.addSeparator();
    for (const TaskDef &t : kTasks) {
        QAction *a = menu.addAction(QString::fromUtf8("Operačný bod – %1").arg(QString::fromUtf8(t.label)));
        a->setCheckable(true);
        a->setChecked(mp.operation && mp.task == t.id);
        a->setData(QString::fromUtf8(t.id));
    }
    QAction *pass = menu.addAction(QString::fromUtf8("Prejazdový bod"));
    pass->setCheckable(true);
    pass->setChecked(!mp.operation);
    pass->setData(QString("pass"));
    menu.addSeparator();
    QAction *del = menu.addAction(QString::fromUtf8("Zmazať bod"));
    QAction *chosen = menu.exec(globalPos);
    if (!chosen) return;
    if (chosen == del) { deleteSelectedPoint(); return; }
    if (!chosen->data().isValid()) return;
    const QString id = chosen->data().toString();
    mp.operation = id != "pass";
    if (mp.operation) mp.task = id;
    refreshPointsTable();
    updateValidation();
    ui->mapView->update();
}

void MainWindow::insertPoint(const MissionPoint &p)
{
    // A new point goes after the selected one, otherwise at the end of the mission.
    const int at = (selected_ >= 0 && selected_ < points_.size()) ? selected_ + 1 : points_.size();
    points_.insert(at, p);
    selected_ = at;
    if (missionState_ == MissionState::Done || missionState_ == MissionState::Aborted) {
        missionState_ = MissionState::Idle;
        trail_.clear();
        updateMissionUi();
    }
    refreshPointsTable();
    updateValidation();
    ui->mapView->update();
}

void MainWindow::refreshPointsTable()
{
    tableUpdating_ = true;
    QTableWidget *t = ui->pointsTable;
    t->setRowCount(points_.size());
    const bool canEdit = editable();
    for (int i = 0; i < points_.size(); ++i) {
        const MissionPoint &mp = points_[i];
        auto *num = new QTableWidgetItem(QString::number(i + 1));
        num->setTextAlignment(Qt::AlignCenter);
        const bool bad = mapLoaded_ && isBlocked(mp.pos);
        num->setForeground(bad ? kCollisionColor : (mp.operation ? kOpColor : kPassColor));
        QFont f = num->font();
        f.setBold(true);
        num->setFont(f);
        t->setItem(i, 0, num);

        // The combo of a row is created once and then only updated: recreating it here would destroy
        // an open popup (clicking the combo selects its row, which refreshes this table).
        auto *combo = qobject_cast<QComboBox *>(t->cellWidget(i, 1));
        if (!combo) {
            combo = new QComboBox;
            for (const TaskDef &td : kTasks)
                combo->addItem(QString::fromUtf8("Operačný – %1").arg(QString::fromUtf8(td.label)), QString::fromUtf8(td.id));
            combo->addItem(QString::fromUtf8("Prejazdový bod"), QString("pass"));
            connect(combo, QOverload<int>::of(&QComboBox::activated), this, [this, i, combo](int){
                if (i >= points_.size() || !editable()) return;
                const QString id = combo->currentData().toString();
                points_[i].operation = id != "pass";
                if (points_[i].operation) points_[i].task = id;
                QTimer::singleShot(0, this, [this]{ refreshPointsTable(); updateValidation(); ui->mapView->update(); });
            });
            t->setCellWidget(i, 1, combo);
        }
        {
            const QSignalBlocker block(combo);
            combo->setCurrentIndex(combo->findData(mp.operation ? mp.task : QString("pass")));
        }
        combo->setEnabled(canEdit);

        auto *coords = new QTableWidgetItem(QString("%1, %2").arg(mp.pos.x(), 0, 'f', 2).arg(mp.pos.y(), 0, 'f', 2));
        if (bad) coords->setToolTip(QString::fromUtf8("Bod leží v prekážke"));
        t->setItem(i, 2, coords);
    }
    if (selected_ >= 0 && selected_ < points_.size()) t->selectRow(selected_);
    else t->clearSelection();
    ui->pointsGroup->setTitle(QString::fromUtf8("Body misie (%1)").arg(points_.size()));
    const bool hasSel = selected_ >= 0 && selected_ < points_.size();
    ui->pointUpButton->setEnabled(canEdit && hasSel && selected_ > 0);
    ui->pointDownButton->setEnabled(canEdit && hasSel && selected_ < points_.size() - 1);
    ui->pointDeleteButton->setEnabled(canEdit && hasSel);
    ui->clearMissionButton->setEnabled(canEdit && !points_.isEmpty());
    tableUpdating_ = false;
}

void MainWindow::selectPoint(int index)
{
    selected_ = (index >= 0 && index < points_.size()) ? index : -1;
    refreshPointsTable();
    ui->mapView->update();
}

void MainWindow::movePoint(int delta)
{
    const int j = selected_ + delta;
    if (!editable() || selected_ < 0 || j < 0 || j >= points_.size()) return;
    std::swap(points_[selected_], points_[j]);
    selected_ = j;
    refreshPointsTable();
    updateValidation();
    ui->mapView->update();
}

void MainWindow::deleteSelectedPoint()
{
    if (!editable() || selected_ < 0 || selected_ >= points_.size()) return;
    points_.remove(selected_);
    selected_ = -1;
    refreshPointsTable();
    updateValidation();
    ui->mapView->update();
}

void MainWindow::clearMission()
{
    if (!editable() || points_.isEmpty()) return;
    if (QMessageBox::question(this, QString::fromUtf8("Vymazať misiu"),
                              QString::fromUtf8("Vymazať všetky body naplánovanej misie?")) != QMessageBox::Yes)
        return;
    points_.clear();
    selected_ = -1;
    missionState_ = MissionState::Idle;
    trail_.clear();
    addLog(QString::fromUtf8("Naplánovaná misia vymazaná"), LogLevel::Warning);
    refreshPointsTable();
    updateValidation();
    updateMissionUi();
    ui->mapView->update();
}

QJsonObject MainWindow::missionToJson() const
{
    QJsonArray arr;
    for (const MissionPoint &mp : points_) {
        QJsonObject o{{"type", mp.operation ? "op" : "pass"}, {"x", mp.pos.x()}, {"y", mp.pos.y()}};
        if (mp.operation) o["task"] = mp.task;
        arr.append(o);
    }
    return QJsonObject{{"map", QFileInfo(mapFile_).fileName()}, {"target", "ball"}, {"points", arr}};
}

void MainWindow::saveMission()
{
    if (points_.isEmpty()) { showToast(QString::fromUtf8("Misia nemá žiadne body."), LogLevel::Warning); return; }
    const QString path = QFileDialog::getSaveFileName(this, QString::fromUtf8("Uložiť misiu"), "misia.json", "Misia (*.json)");
    if (path.isEmpty()) return;
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) { showToast(QString::fromUtf8("Misiu sa nepodarilo uložiť."), LogLevel::Danger); return; }
    f.write(QJsonDocument(missionToJson()).toJson());
    addLog(QString::fromUtf8("Misia uložená: %1").arg(QFileInfo(path).fileName()));
    showToast(QString::fromUtf8("Misia uložená"));
}

void MainWindow::loadMission()
{
    const QString path = QFileDialog::getOpenFileName(this, QString::fromUtf8("Načítať misiu"), QString(), "Misia (*.json)");
    if (path.isEmpty()) return;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return;
    const QJsonObject root = QJsonDocument::fromJson(f.readAll()).object();
    QVector<MissionPoint> loaded;
    for (const QJsonValue &v : root["points"].toArray()) {
        const QJsonObject o = v.toObject();
        MissionPoint mp;
        mp.operation = o["type"].toString() != "pass";
        mp.pos = QPointF(o["x"].toDouble(), o["y"].toDouble());
        mp.task = o["task"].toString("photo");
        loaded.append(mp);
    }
    points_ = loaded;
    selected_ = -1;
    missionState_ = MissionState::Idle;
    trail_.clear();
    addLog(QString::fromUtf8("Načítaná misia %1 (%2 bodov)").arg(QFileInfo(path).fileName()).arg(points_.size()));
    refreshPointsTable();
    updateValidation();    // a loaded mission is validated again
    updateMissionUi();
    ui->mapView->update();
}

// Conditions that stop a running mission (and block a new one).
QString MainWindow::runBlocker() const
{
    if (!connected_) return QString::fromUtf8("Robot nie je pripojený.");
    if (safetyStopActive_) return QString::fromUtf8("Aktívny SAFETY STOP.");
    if (!hasPose_) return QString::fromUtf8("Poloha robota nie je známa (čaká sa na AMCL).");
    if (offMission_) return QString::fromUtf8("Robot je mimo misie – sken nesedí s mapou.");
    return QString();
}

QString MainWindow::validationError() const
{
    if (!mapLoaded_) return QString::fromUtf8("Načítajte mapu.");
    if (std::none_of(points_.begin(), points_.end(), [](const MissionPoint &p) { return p.operation; }))
        return QString::fromUtf8("Pridajte aspoň jeden operačný bod (kliknutím do mapy).");
    for (int i = 0; i < points_.size(); ++i)
        if (isBlocked(points_[i].pos)) return QString::fromUtf8("Bod %1 leží v prekážke.").arg(i + 1);
    if (hasPose_) {
        QPointF prev(pose_.x, pose_.y);
        for (int i = 0; i < points_.size(); ++i) {
            if (segmentBlocked(prev, points_[i].pos))
                return QString::fromUtf8("Úsek %1 → %2 prechádza prekážkou. Misiu nie je možné vykonať.")
                        .arg(i == 0 ? QString::fromUtf8("štart") : QString::number(i)).arg(i + 1);
            prev = points_[i].pos;
        }
    }
    if (battery_ >= 0 && battery_ < 10) return QString::fromUtf8("Kritická batéria – misiu nie je možné spustiť.");
    return runBlocker();
}

void MainWindow::updateValidation()
{
    const bool active = missionState_ == MissionState::Running || missionState_ == MissionState::Paused;
    if (active) {
        ui->validationLabel->setText(QString::fromUtf8("Misia prebieha – body nie je možné upravovať."));
        ui->validationLabel->setStyleSheet("background: #eaf3fc; border: 1px solid #93c5fd; color: #1e3a5f; padding: 6px; border-radius: 3px; font-weight: bold;");
        ui->startMissionButton->setEnabled(false);
        return;
    }
    const QString err = validationError();
    const bool ok = err.isEmpty();
    ui->validationLabel->setText(ok ? QString::fromUtf8("✓ Trasa je priechodná – misiu je možné spustiť.")
                                    : QString::fromUtf8("✕ ") + err);
    ui->validationLabel->setStyleSheet(ok
            ? "background: #ecfdf5; border: 1px solid #86c9a6; color: #166534; padding: 6px; border-radius: 3px; font-weight: bold;"
            : "background: #fef2f2; border: 1px solid #fca5a5; color: #991b1b; padding: 6px; border-radius: 3px; font-weight: bold;");
    ui->startMissionButton->setEnabled(ok);
}

// ============================================================================
// Mission execution
// ============================================================================

void MainWindow::startMission()
{
    const QString err = validationError();
    if (!err.isEmpty()) { showToast(err, LogLevel::Danger); return; }
    missionState_ = MissionState::Running;
    currentIndex_ = 0;
    inTask_ = false;
    trail_.clear();
    trail_.append(QPointF(pose_.x, pose_.y));
    detections_.clear();
    ui->foundList->clear();
    focusedDetection_ = -1;
    selected_ = -1;
    missionClock_.start();
    startRecording();      // every mission is recorded automatically
    addLog(QString::fromUtf8("Misia spustená (%1 bodov) – nahrávanie zapnuté").arg(points_.size()), LogLevel::Ok);
    emit missionStarted();
    sendCurrentGoal();
    missionTimer_.start();
    refreshPointsTable();
    updateValidation();
    updateMissionUi();
}

void MainWindow::pauseMission(const QString &reason, LogLevel level)
{
    if (missionState_ != MissionState::Running) return;
    missionState_ = MissionState::Paused;
    emit stopRequested();
    addLog(reason, level);
    emit missionPaused(reason);
    updateMissionUi();
}

void MainWindow::resumeMission()
{
    if (missionState_ != MissionState::Paused) return;
    const QString b = runBlocker();
    if (!b.isEmpty()) { showToast(QString::fromUtf8("Nie je možné pokračovať: ") + b, LogLevel::Danger); return; }
    missionState_ = MissionState::Running;
    if (inTask_) taskTimer_.restart(); else sendCurrentGoal();
    addLog(QString::fromUtf8("Misia obnovená operátorom"), LogLevel::Ok);
    updateMissionUi();
}

void MainWindow::abortMission()
{
    if (missionState_ != MissionState::Running && missionState_ != MissionState::Paused) return;
    if (QMessageBox::question(this, QString::fromUtf8("Prerušiť misiu"),
                              QString::fromUtf8("Prerušiť misiu? Čiastočný záznam zostane uložený.")) != QMessageBox::Yes)
        return;
    finishMission(MissionState::Aborted);
}

void MainWindow::finishMission(MissionState state)
{
    missionTimer_.stop();
    emit stopRequested();
    missionState_ = state;
    inTask_ = false;
    const QString text = state == MissionState::Done ? QString::fromUtf8("Misia dokončená")
                                                     : QString::fromUtf8("Misia prerušená");
    recordEvent("end", text);
    const QString dir = recDir_;
    stopRecording();
    if (ui) {
        addLog(text + QString::fromUtf8(" – záznam uložený (%1)").arg(QFileInfo(dir).fileName()),
               state == MissionState::Done ? LogLevel::Ok : LogLevel::Warning);
        showToast(text + QString::fromUtf8(", záznam uložený"), state == MissionState::Done ? LogLevel::Ok : LogLevel::Warning);
        refreshPointsTable();
        updateValidation();
        updateMissionUi();
    }
    emit missionFinished(dir);
}

void MainWindow::sendCurrentGoal()
{
    if (currentIndex_ < 0 || currentIndex_ >= points_.size()) return;
    const QPointF g = points_[currentIndex_].pos;
    emit goalRequested(g.x(), g.y());
}

void MainWindow::missionTick()
{
    if (missionState_ != MissionState::Running) { updateMissionUi(); return; }
    const QString blocker = runBlocker();
    if (!blocker.isEmpty()) { pauseMission(blocker + QString::fromUtf8(" Misia pozastavená."), LogLevel::Danger); return; }
    if (currentIndex_ >= points_.size()) { finishMission(MissionState::Done); return; }

    const MissionPoint &mp = points_[currentIndex_];
    if (!inTask_) {
        const double dist = std::hypot(mp.pos.x() - pose_.x, mp.pos.y() - pose_.y);
        if (dist < kGoalTolerance) {
            sendSpeed(0, 0);
            if (mp.operation) startTask();
            else { addLog(QString::fromUtf8("Prejazdový bod %1 dosiahnutý").arg(currentIndex_ + 1)); advanceMission(); }
        } else {
            driveToGoal(mp.pos);
        }
    } else {
        const bool rotating = mp.task == "rotate" || mp.task == "search";
        sendSpeed(0, rotating ? kRotateSpeed : 0.0);
        if (taskTimer_.elapsed() >= taskDurationMs_) {
            sendSpeed(0, 0);
            inTask_ = false;
            advanceMission();
        }
    }
    updateMissionUi();
}

void MainWindow::startTask()
{
    const MissionPoint &mp = points_[currentIndex_];
    inTask_ = true;
    taskTimer_.start();
    if (mp.task == "photo") {
        taskDurationMs_ = 1500;
        savePhoto("foto", QString("bod_%1.jpg").arg(currentIndex_ + 1, 2, 10, QChar('0')));
    } else if (mp.task == "wait") {
        taskDurationMs_ = 3000;
    } else {
        taskDurationMs_ = qint64(2 * kPi / kRotateSpeed * 1000.0);
        emit rotateRequested(2 * kPi);
    }
    const QString text = QString::fromUtf8("Bod %1: %2").arg(currentIndex_ + 1).arg(taskLabel(mp.task));
    addLog(text);
    recordEvent("op", text);
}

void MainWindow::advanceMission()
{
    ++currentIndex_;
    if (currentIndex_ >= points_.size()) finishMission(MissionState::Done);
    else sendCurrentGoal();
}

// Simple built-in go-to controller (rotate in place, then drive straight). The robot stops in front of
// obstacles; replace with the backend navigation by connecting goalRequested() and removing this call.
void MainWindow::driveToGoal(const QPointF &goal)
{
    if (obstacleAhead_) { sendSpeed(0, 0); return; }
    const double dx = goal.x() - pose_.x, dy = goal.y() - pose_.y;
    const double dist = std::hypot(dx, dy);
    const double err = normAngle(std::atan2(dy, dx) - pose_.th);
    if (std::abs(err) > 0.25) sendSpeed(0, err > 0 ? 0.7 : -0.7);
    else sendSpeed(qBound(80.0, dist * 600.0, 300.0), 0);
}

void MainWindow::updateMissionUi()
{
    const MissionState st = missionState_;
    const bool active = st == MissionState::Running || st == MissionState::Paused;
    ui->pauseMissionButton->setEnabled(st == MissionState::Running);
    ui->resumeMissionButton->setEnabled(st == MissionState::Paused);
    ui->abortMissionButton->setEnabled(active);
    ui->recordsModeButton->setEnabled(!active);
    ui->recIndicatorLabel->setVisible(recording_);

    QString status = QString::fromUtf8("PRIPRAVENÉ"), color = "#647184", activity;
    const int n = points_.size();
    if (st == MissionState::Running && obstacleAhead_ && !inTask_) {
        status = QString::fromUtf8("STOJÍ"); color = "#b45309";
        activity = QString::fromUtf8("Prekážka na trase – čaká na uvoľnenie");
    } else if (st == MissionState::Running) {
        status = QString::fromUtf8("BEŽÍ"); color = "#0f766e";
        activity = inTask_ ? QString::fromUtf8("Bod %1 / %2 – vykonáva: %3").arg(currentIndex_ + 1).arg(n).arg(taskLabel(points_[currentIndex_].task))
                           : QString::fromUtf8("Presun k bodu %1 / %2").arg(currentIndex_ + 1).arg(n);
    } else if (st == MissionState::Paused) {
        status = QString::fromUtf8("POZASTAVENÉ"); color = "#b91c1c";
        activity = QString::fromUtf8("Misia čaká na operátora – stlačte Pokračovať");
    } else if (st == MissionState::Done) {
        status = QString::fromUtf8("DOKONČENÉ"); color = "#2375bd";
        activity = QString::fromUtf8("Všetky body misie vykonané");
    } else if (st == MissionState::Aborted) {
        status = QString::fromUtf8("PRERUŠENÉ"); color = "#596579";
        activity = QString::fromUtf8("Misia prerušená operátorom");
    } else {
        activity = QString::fromUtf8("Kliknite do mapy a vyberte akciu bodu.");
    }
    ui->missionStatusLabel->setText(status);
    ui->missionStatusLabel->setStyleSheet(QString("background: %1; color: white; font-weight: bold; padding: 2px 10px; border-radius: 3px;").arg(color));
    ui->missionActivityLabel->setText(activity);
    if (mode_ == Mode::Planning) ui->mapView->update();
}

void MainWindow::savePhoto(const QString &subdir, const QString &name)
{
    if (cameraImage_.isNull()) { addLog(QString::fromUtf8("Fotografia neuložená – bez obrazu z kamery"), LogLevel::Warning); return; }
    const QString base = recording_ ? recDir_ : QDir(recordsRoot()).filePath("foto");
    QDir().mkpath(QDir(base).filePath(subdir));
    const QString path = QDir(QDir(base).filePath(subdir)).filePath(name);
    if (cameraImage_.save(path, "JPG", 90)) addLog(QString::fromUtf8("Fotografia uložená: %1/%2").arg(subdir, name));
}

// ============================================================================
// Alarms, log, toast, mode
// ============================================================================

void MainWindow::refreshAlarms()
{
    struct Alarm { bool danger; QString title, text; };
    QVector<Alarm> alarms;
    if (!connected_) alarms.push_back(Alarm{true, QString::fromUtf8("SPOJENIE"), QString::fromUtf8("Robot nie je pripojený alebo neposiela dáta.")});
    if (safetyStopActive_) alarms.push_back(Alarm{true, "SAFETY STOP", QString::fromUtf8("Pohyb je zablokovaný. Po uvoľnení obnovte misiu tlačidlom Pokračovať.")});
    if (battery_ >= 0 && battery_ < 10) alarms.push_back(Alarm{true, QString::fromUtf8("BATÉRIA"), QString::fromUtf8("Kritická úroveň %1 %. Spustenie misie je zablokované.").arg(battery_)});
    else if (battery_ >= 0 && battery_ < 20) alarms.push_back(Alarm{false, QString::fromUtf8("BATÉRIA"), QString::fromUtf8("Nízka úroveň %1 %. Odporúčaný návrat do nabíjacej stanice.").arg(battery_)});
    if (obstacleAhead_ && connected_) alarms.push_back(Alarm{false, QString::fromUtf8("PREKÁŽKA V MISII"), QString::fromUtf8("Robot zastavil pred prekážkou na trase. Pokračuje po jej odstránení.")});
    if (offMission_) alarms.push_back(Alarm{true, QString::fromUtf8("ROBOT MIMO MISIE"), QString::fromUtf8("Sken lidaru nesedí s mapou (zhoda %1 %). Možná zmena prostredia alebo strata lokalizácie.").arg(qRound(locQuality_ * 100))});

    QString key;
    for (const Alarm &a : alarms) key += a.title + a.text;
    const QString oldKey = ui->alarmContainer->property("alarmKey").toString();
    if (key != oldKey) {
        ui->alarmContainer->setProperty("alarmKey", key);
        QLayoutItem *item;
        while ((item = ui->alarmLayout->takeAt(0))) { delete item->widget(); delete item; }
        for (const Alarm &a : alarms) {
            auto *label = new QLabel(QString("<b>!&nbsp;&nbsp;%1</b>&nbsp;&nbsp;&nbsp;%2").arg(a.title.toHtmlEscaped(), a.text.toHtmlEscaped()));
            label->setTextFormat(Qt::RichText);
            label->setWordWrap(true);
            label->setStyleSheet(a.danger
                    ? "background: rgba(157,27,30,245); border: 1px solid #ff4545; color: white; padding: 8px 12px; border-radius: 3px;"
                    : "background: rgba(87,62,5,245); border: 1px solid #e3a81b; color: #ffcf55; padding: 8px 12px; border-radius: 3px;");
            ui->alarmLayout->addWidget(label);
        }
        ui->alarmContainer->setVisible(!alarms.isEmpty());
    }
    ui->alarmCountLabel->setText(QString::fromUtf8("Alarmy: %1").arg(alarms.size()));
    ui->alarmCountLabel->setStyleSheet(alarms.isEmpty()
            ? "background: #e6eaf0; border: 1px solid #aeb8c6; color: #3c4a5e; padding: 6px 10px; border-radius: 3px; font-weight: bold;"
            : "background: #fee2e2; border: 1px solid #dc2626; color: #991b1b; padding: 6px 10px; border-radius: 3px; font-weight: bold;");
    const QString batText = battery_ < 0 ? QString::fromUtf8("Batéria –") : QString::fromUtf8("Batéria %1 %").arg(battery_);
    ui->batteryLabel->setText(batText);
    ui->batteryLabel->setStyleSheet(battery_ < 0
            ? "background: #e6eaf0; border: 1px solid #aeb8c6; color: #3c4a5e; padding: 6px 10px; border-radius: 3px; font-weight: bold;"
            : battery_ < 10 ? "background: #fee2e2; border: 1px solid #dc2626; color: #991b1b; padding: 6px 10px; border-radius: 3px; font-weight: bold;"
            : battery_ < 20 ? "background: #fef3c7; border: 1px solid #d97706; color: #92400e; padding: 6px 10px; border-radius: 3px; font-weight: bold;"
            : "background: #ecfdf5; border: 1px solid #86c9a6; color: #166534; padding: 6px 10px; border-radius: 3px; font-weight: bold;");
}

void MainWindow::addLog(const QString &text, LogLevel level)
{
    static const QHash<int, QColor> colors = {
        {int(LogLevel::Info), QColor("#3c4a5e")}, {int(LogLevel::Ok), QColor("#166534")},
        {int(LogLevel::Warning), QColor("#92400e")}, {int(LogLevel::Danger), QColor("#b91c1c")},
        {int(LogLevel::Detection), QColor("#6d28d9")}};
    auto *item = new QListWidgetItem(QTime::currentTime().toString("HH:mm:ss") + "  " + text);
    item->setForeground(colors.value(int(level)));
    ui->logList->insertItem(0, item);
    while (ui->logList->count() > 200) delete ui->logList->takeItem(ui->logList->count() - 1);
    if (recording_ && level != LogLevel::Info) recordEvent(level == LogLevel::Detection ? "det" : "alarm", text);
}

void MainWindow::showToast(const QString &text, LogLevel level)
{
    const char *bg = level == LogLevel::Danger ? "#b91c1c" : level == LogLevel::Detection ? "#6d28d9"
                   : level == LogLevel::Warning ? "#92400e" : "#14243b";
    toastLabel_->setStyleSheet(QString("background: %1; color: white; border: 1px solid white; border-radius: 3px;"
                                       "padding: 8px 14px; font-weight: bold;").arg(QString::fromLatin1(bg)));
    toastLabel_->setText(text);
    toastLabel_->adjustSize();
    layoutOverlays();
    toastLabel_->show();
    toastLabel_->raise();
    toastTimer_.start(3500);
}

void MainWindow::layoutOverlays()
{
    const QRect r = ui->mapView->rect();
    toastLabel_->move(r.center().x() - toastLabel_->width() / 2, r.bottom() - toastLabel_->height() - 16);
    cursorLabel_->adjustSize();
    cursorLabel_->move(r.right() - cursorLabel_->width() - 12, 10);
    replayBadge_->move(r.center().x() - replayBadge_->width() / 2, 8);
}

void MainWindow::setMode(Mode mode)
{
    if (mode == Mode::Records && (missionState_ == MissionState::Running || missionState_ == MissionState::Paused)) {
        showToast(QString::fromUtf8("Počas misie nie je možné prepnúť na záznamy."), LogLevel::Warning);
        mode = Mode::Planning;
    }
    mode_ = mode;
    {
        const QSignalBlocker b1(ui->planModeButton), b2(ui->recordsModeButton);
        ui->planModeButton->setChecked(mode == Mode::Planning);
        ui->recordsModeButton->setChecked(mode == Mode::Records);
    }
    ui->sideStack->setCurrentIndex(mode == Mode::Planning ? 0 : 1);
    ui->bottomStack->setCurrentIndex(mode == Mode::Planning ? 0 : 1);
    updateViewTitles();
    if (mode == Mode::Planning) {
        playbackTimer_.stop();
        ui->playButton->setText(QString::fromUtf8("Prehrať"));
        replayBadge_->hide();
    } else {
        refreshRecords();
    }
    replayBadge_->setVisible(mode == Mode::Records && recLoaded_);
    refreshPointsTable();
    ui->mapView->update();
    ui->cameraLabel->update();
}

// The group box in the side panel is titled after whatever view it currently holds.
void MainWindow::updateViewTitles()
{
    if (viewsSwapped_) ui->cameraGroup->setTitle(QString::fromUtf8("Mapa"));
    else if (mode_ == Mode::Records) ui->cameraGroup->setTitle(QString::fromUtf8("Kamera – zo záznamu"));
    else ui->cameraGroup->setTitle(QString::fromUtf8("Kamera"));
}

// Swaps the map and the camera between the main area and the side slot. The widgets are moved
// physically, so painting and map editing keep working on ui->mapView wherever it is.
void MainWindow::swapViews()
{
    QWidget *big = viewsSwapped_ ? static_cast<QWidget *>(ui->cameraLabel) : ui->mapView;
    QWidget *small = viewsSwapped_ ? static_cast<QWidget *>(ui->mapView) : ui->cameraLabel;
    dragIndex_ = -1;
    hasHover_ = false;
    cursorLabel_->clear();

    QWidget placeholder;
    delete ui->cameraLayout->replaceWidget(small, &placeholder);
    delete ui->mapColumnLayout->replaceWidget(big, small);
    delete ui->cameraLayout->replaceWidget(&placeholder, big);
    viewsSwapped_ = !viewsSwapped_;

    // small -> main area
    small->setMinimumSize(640, 420);
    small->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    small->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    small->unsetCursor();
    small->setToolTip(QString());
    // big -> side slot
    big->setMinimumSize(0, 190);
    big->setMaximumSize(QWIDGETSIZE_MAX, 190);
    big->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    big->setCursor(Qt::PointingHandCursor);
    big->setToolTip(QString::fromUtf8("Kliknutím prehodíte s hlavným zobrazením"));

    updateViewTitles();
    layoutOverlays();
    ui->mapView->update();
    ui->cameraLabel->update();
}

// ============================================================================
// Recording – every mission is recorded automatically
// ============================================================================

qint64 MainWindow::missionTimeMs() const
{
    return missionClock_.isValid() ? missionClock_.elapsed() : 0;
}

void MainWindow::startRecording()
{
    stopRecording();
    const QDir root(recordsRoot());
    recDir_ = root.filePath("misia_" + QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss"));
    QDir().mkpath(QDir(recDir_).filePath("camera"));
    QDir().mkpath(QDir(recDir_).filePath("detekcie"));

    QJsonObject mission = missionToJson();
    mission["started"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    QFile mf(QDir(recDir_).filePath("mission.json"));
    if (mf.open(QIODevice::WriteOnly)) mf.write(QJsonDocument(mission).toJson());

    auto open = [this](std::unique_ptr<QFile> &file, QTextStream &out, const char *name, const char *header) {
        file = std::make_unique<QFile>(QDir(recDir_).filePath(name));
        if (!file->open(QIODevice::WriteOnly | QIODevice::Text)) return;
        out.setDevice(file.get());
        out << header << '\n';
    };
    open(posesFile_, posesOut_, "poses.csv", "t_ms,amcl_x,amcl_y,amcl_th,odom_x,odom_y,odom_th");
    open(scanFile_, scanOut_, "scan.csv", "t_ms,n,angle_deg_1,dist_mm_1,...");
    open(cameraFile_, cameraOut_, "camera.csv", "t_ms,file");
    open(eventsFile_, eventsOut_, "events.csv", "t_ms,type,text");
    open(detFile_, detOut_, "detections.csv", "t_ms,name,confidence,x,y,file,box_x,box_y,box_w,box_h");
    recording_ = true;
    lastRecordedScan_ = scanSerial_;
    cameraRecordAge_.invalidate();
    cameraFrameNo_ = 0;
    photoNo_ = 0;
    recordTimer_.start();
    ui->recIndicatorLabel->setVisible(true);
}

void MainWindow::stopRecording()
{
    if (!recording_) return;
    recordTick();
    recordTimer_.stop();
    recording_ = false;
    for (QTextStream *s : {&posesOut_, &scanOut_, &cameraOut_, &eventsOut_, &detOut_}) { s->flush(); s->setDevice(nullptr); }
    for (std::unique_ptr<QFile> *f : {&posesFile_, &scanFile_, &cameraFile_, &eventsFile_, &detFile_}) f->reset();
    if (ui) ui->recIndicatorLabel->setVisible(false);
}

// 10 Hz: poses (AMCL + odometry) and the newest lidar scan on the common mission time axis.
void MainWindow::recordTick()
{
    if (!recording_) return;
    const qint64 t = missionTimeMs();
    posesOut_ << t << ',' << pose_.x << ',' << pose_.y << ',' << pose_.th << ','
              << odom_.x << ',' << odom_.y << ',' << odom_.th << '\n';
    if (scanSerial_ != lastRecordedScan_) {
        lastRecordedScan_ = scanSerial_;
        scanOut_ << t << ',' << lidarRaw_.size();
        for (const QPointF &a : lidarRaw_) scanOut_ << ',' << QString::number(a.x(), 'f', 2) << ',' << QString::number(a.y(), 'f', 0);
        scanOut_ << '\n';
    }
}

void MainWindow::recordEvent(const QString &type, const QString &text)
{
    if (!recording_) return;
    QString clean = text;
    clean.replace(',', ';').replace('\n', ' ');
    eventsOut_ << missionTimeMs() << ',' << type << ',' << clean << '\n';
}

// ============================================================================
// Playback
// ============================================================================

void MainWindow::refreshRecords()
{
    const QString current = recLoaded_ ? rec_.dir : QString();
    const QSignalBlocker blocker(ui->recordsList);
    ui->recordsList->clear();
    QDir root(recordsRoot());
    const QStringList dirs = root.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name | QDir::Reversed);
    for (const QString &name : dirs) {
        const QString dir = root.filePath(name);
        if (!QFileInfo::exists(QDir(dir).filePath("poses.csv"))) continue;
        qint64 bytes = 0, durationMs = 0;
        QDirIterator it(dir, QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext()) { it.next(); bytes += it.fileInfo().size(); }
        QFile poses(QDir(dir).filePath("poses.csv"));
        if (poses.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QString last;
            while (!poses.atEnd()) { const QString l = QString::fromUtf8(poses.readLine()).trimmed(); if (!l.isEmpty()) last = l; }
            durationMs = last.section(',', 0, 0).toLongLong();
        }
        int detCount = 0;
        QFile dets(QDir(dir).filePath("detections.csv"));
        if (dets.open(QIODevice::ReadOnly | QIODevice::Text)) detCount = std::max(0, int(dets.readAll().count('\n')) - 1);
        auto *item = new QListWidgetItem(QString::fromUtf8("%1\n%2 · %3 detekcií · %4 MB")
                .arg(name, fmtMs(durationMs)).arg(detCount).arg(bytes / 1048576.0, 0, 'f', 1));
        item->setData(Qt::UserRole, dir);
        ui->recordsList->addItem(item);
        if (dir == current) ui->recordsList->setCurrentItem(item);
    }
    if (ui->recordsList->count() == 0) {
        auto *item = new QListWidgetItem(QString::fromUtf8("Žiadne záznamy. Záznam vznikne automaticky pri každej misii."));
        item->setFlags(Qt::NoItemFlags);
        ui->recordsList->addItem(item);
        recLoaded_ = false;
    } else if (!ui->recordsList->currentItem()) {
        ui->recordsList->setCurrentRow(0);
        loadRecording(ui->recordsList->item(0)->data(Qt::UserRole).toString());
    }
    ui->deleteRecordButton->setEnabled(recLoaded_);
    replayBadge_->setVisible(mode_ == Mode::Records && recLoaded_);
}

bool MainWindow::loadRecording(const QString &dir)
{
    playbackTimer_.stop();
    ui->playButton->setText(QString::fromUtf8("Prehrať"));
    Recording r;
    r.dir = dir;
    auto lines = [&](const char *name) {
        QStringList out;
        QFile f(QDir(dir).filePath(name));
        if (f.open(QIODevice::ReadOnly | QIODevice::Text)) {
            f.readLine(); // header
            while (!f.atEnd()) { const QString l = QString::fromUtf8(f.readLine()).trimmed(); if (!l.isEmpty()) out << l; }
        }
        return out;
    };
    for (const QString &l : lines("poses.csv")) {
        const QStringList v = l.split(',');
        if (v.size() < 7) continue;
        r.poses.append(RecSample{v[0].toLongLong(), {v[1].toDouble(), v[2].toDouble(), v[3].toDouble()},
                        {v[4].toDouble(), v[5].toDouble(), v[6].toDouble()}});
    }
    for (const QString &l : lines("scan.csv")) {
        const QStringList v = l.split(',');
        if (v.size() < 2) continue;
        RecScan s{v[0].toLongLong(), {}};
        for (int i = 2; i + 1 < v.size(); i += 2) {
            const double a = v[i].toDouble() * kPi / 180.0, d = v[i + 1].toDouble();
            s.points.append(QPointF(d * std::cos(a), d * std::sin(a)));
        }
        r.scans.append(s);
    }
    for (const QString &l : lines("camera.csv")) {
        const QStringList v = l.split(',');
        if (v.size() >= 2) r.frames.append(RecFrame{v[0].toLongLong(), QDir(dir).filePath(v[1])});
    }
    for (const QString &l : lines("events.csv")) {
        const QStringList v = l.split(',');
        if (v.size() >= 3) r.events.append(RecEvent{v[0].toLongLong(), v[1], v.mid(2).join(',')});
    }
    for (const QString &l : lines("detections.csv")) {
        const QStringList v = l.split(',');
        if (v.size() < 6) continue;
        Detection d;
        d.t = v[0].toLongLong(); d.name = v[1]; d.confidence = v[2].toDouble();
        d.pos = QPointF(v[3].toDouble(), v[4].toDouble()); d.file = QDir(dir).filePath(v[5]);
        if (v.size() >= 10) d.box = QRectF(v[6].toDouble(), v[7].toDouble(), v[8].toDouble(), v[9].toDouble());
        r.detections.append(d);
    }
    QFile mf(QDir(dir).filePath("mission.json"));
    if (mf.open(QIODevice::ReadOnly)) {
        for (const QJsonValue &v : QJsonDocument::fromJson(mf.readAll()).object()["points"].toArray()) {
            const QJsonObject o = v.toObject();
            MissionPoint mp;
            mp.operation = o["type"].toString() != "pass";
            mp.pos = QPointF(o["x"].toDouble(), o["y"].toDouble());
            mp.task = o["task"].toString("photo");
            r.points.append(mp);
        }
    }
    if (r.poses.isEmpty()) { recLoaded_ = false; return false; }
    r.duration = r.poses.last().t;
    rec_ = r;
    recLoaded_ = true;
    replayImageFile_.clear();
    {
        const QSignalBlocker b(ui->timelineSlider);
        ui->timelineSlider->setRange(0, int(rec_.duration));
    }
    ui->deleteRecordButton->setEnabled(true);
    replayBadge_->setVisible(mode_ == Mode::Records);
    seekPlayback(0);
    ui->timelineMarks->update();
    return true;
}

void MainWindow::deleteSelectedRecord()
{
    QListWidgetItem *item = ui->recordsList->currentItem();
    if (!item || !item->data(Qt::UserRole).isValid()) return;
    const QString dir = item->data(Qt::UserRole).toString();
    if (QMessageBox::warning(this, QString::fromUtf8("Vymazať záznam"),
            QString::fromUtf8("Vymazať záznam %1 aj s fotografiami detekcií?\nAkciu nie je možné vrátiť.")
                    .arg(QFileInfo(dir).fileName()),
            QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
        return;
    playbackTimer_.stop();
    if (rec_.dir == dir) { recLoaded_ = false; rec_ = Recording(); replayImage_ = QImage(); }
    if (QDir(dir).removeRecursively()) addLog(QString::fromUtf8("Záznam vymazaný: %1").arg(QFileInfo(dir).fileName()), LogLevel::Warning);
    else showToast(QString::fromUtf8("Záznam sa nepodarilo vymazať."), LogLevel::Danger);
    refreshRecords();
    ui->mapView->update();
    ui->cameraLabel->update();
}

void MainWindow::togglePlayback()
{
    if (!recLoaded_) return;
    if (playbackTimer_.isActive()) {
        playbackTimer_.stop();
        ui->playButton->setText(QString::fromUtf8("Prehrať"));
    } else {
        if (playPos_ >= rec_.duration) seekPlayback(0);
        playbackClock_.start();
        playbackTimer_.start();
        ui->playButton->setText(QString::fromUtf8("Pauza"));
    }
}

void MainWindow::seekPlayback(qint64 t)
{
    if (!recLoaded_) return;
    playPos_ = qBound<qint64>(0, t, rec_.duration);
    showReplayFrame();
}

void MainWindow::playbackTick()
{
    const qint64 dt = playbackClock_.restart();
    playPos_ += qint64(dt * playSpeed_);
    if (playPos_ >= rec_.duration) {
        playPos_ = rec_.duration;
        playbackTimer_.stop();
        ui->playButton->setText(QString::fromUtf8("Prehrať"));
    }
    showReplayFrame();
}

void MainWindow::showReplayFrame()
{
    if (!recLoaded_) return;
    auto pit = std::upper_bound(rec_.poses.begin(), rec_.poses.end(), playPos_,
                                [](qint64 t, const RecSample &s) { return t < s.t; });
    const RecSample &s = rec_.poses[std::max(0, int(pit - rec_.poses.begin()) - 1)];
    replayPose_ = s.amcl;
    replayOdom_ = s.odom;

    replayScan_.clear();
    auto sit = std::upper_bound(rec_.scans.begin(), rec_.scans.end(), playPos_,
                                [](qint64 t, const RecScan &x) { return t < x.t; });
    if (sit != rec_.scans.begin()) replayScan_ = (sit - 1)->points;

    auto fit = std::upper_bound(rec_.frames.begin(), rec_.frames.end(), playPos_,
                                [](qint64 t, const RecFrame &x) { return t < x.t; });
    if (fit != rec_.frames.begin()) {
        const QString file = (fit - 1)->file;
        if (file != replayImageFile_) { replayImageFile_ = file; replayImage_.load(file); }
    } else {
        replayImage_ = QImage();
        replayImageFile_.clear();
    }

    {
        sliderUpdating_ = true;
        ui->timelineSlider->setValue(int(playPos_));
        sliderUpdating_ = false;
    }
    ui->timeLabel->setText(fmtMs(playPos_) + " / " + fmtMs(rec_.duration));
    ui->replayInfoGroup->setTitle(QString::fromUtf8("Stav v čase %1").arg(fmtMs(playPos_)));
    ui->replayAmclLabel->setText("AMCL: " + poseText(replayPose_));
    ui->replayOdomLabel->setText(QString::fromUtf8("Odometria: ") + poseText(replayOdom_));
    ui->replayDriftLabel->setText(QString::fromUtf8("Odchýlka odometrie od AMCL: %1 m")
            .arg(std::hypot(replayOdom_.x - replayPose_.x, replayOdom_.y - replayPose_.y), 0, 'f', 2));
    QString event = QString::fromUtf8("–");
    for (const RecEvent &e : rec_.events) if (e.t <= playPos_) event = fmtMs(e.t) + "  " + e.text;
    ui->replayEventLabel->setText(QString::fromUtf8("Udalosť: ") + event);
    ui->mapView->update();
    ui->cameraLabel->update();
}

void MainWindow::paintTimelineMarks()
{
    QPainter p(ui->timelineMarks);
    p.setRenderHint(QPainter::Antialiasing);
    if (!recLoaded_ || rec_.duration <= 0) return;
    const double w = ui->timelineMarks->width() - 12.0;
    for (const RecEvent &e : rec_.events) {
        if (e.type != "op" && e.type != "det") continue;
        const double x = 6.0 + w * e.t / rec_.duration;
        p.setPen(Qt::NoPen);
        p.setBrush(e.type == "det" ? kDetColor : kOpColor);
        p.drawRoundedRect(QRectF(x - 3, 1, 6, 12), 2, 2);
    }
}

// ============================================================================
// Camera view and object detection during the mission
// ============================================================================

void MainWindow::paintCamera()
{
    QLabel *label = ui->cameraLabel;
    QPainter p(label);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(label->rect(), QColor("#2a333e"));
    const bool replay = mode_ == Mode::Records && recLoaded_;
    if (replay && !ui->layerCameraCheck->isChecked()) {
        p.setPen(QColor("#ccd5df"));
        p.drawText(label->rect(), Qt::AlignCenter, QString::fromUtf8("Vrstva kamery je skrytá"));
        return;
    }
    const QImage &img = replay ? replayImage_ : cameraImage_;
    if (img.isNull()) {
        p.setPen(QColor("#ccd5df"));
        p.drawText(label->rect(), Qt::AlignCenter, replay ? QString::fromUtf8("Bez snímky v tomto čase")
                                                         : (connected_ ? QString::fromUtf8("Bez obrazu") : QString::fromUtf8("Bez obrazu – robot odpojený")));
        return;
    }
    const QSize size = img.size().scaled(label->size(), Qt::KeepAspectRatio);
    const QRectF target(QPointF((label->width() - size.width()) / 2.0, (label->height() - size.height()) / 2.0), size);
    p.setRenderHint(QPainter::SmoothPixmapTransform);
    p.drawImage(target, img);

    QRectF box;
    QString text;
    if (replay) {
        for (const Detection &d : rec_.detections)
            if (std::abs(d.t - playPos_) < 2000 && !d.box.isEmpty()) { box = d.box; text = QString("%1 %2 %").arg(d.name).arg(qRound(d.confidence * 100)); }
    } else if (liveDetAge_.isValid() && liveDetAge_.elapsed() < 3000) {
        box = liveDetBox_;
        text = liveDetLabel_;
    }
    if (!box.isEmpty()) {
        const QRectF b(target.left() + box.x() * target.width(), target.top() + box.y() * target.height(),
                       box.width() * target.width(), box.height() * target.height());
        p.setPen(QPen(QColor("#a78bfa"), 2));
        p.setBrush(Qt::NoBrush);
        p.drawRect(b);
        QFont f = p.font();
        f.setPixelSize(11);
        f.setBold(true);
        p.setFont(f);
        const QRectF tr(b.left(), b.top() - 18, p.fontMetrics().horizontalAdvance(text) + 10, 17);
        p.fillRect(tr, kDetColor);
        p.setPen(Qt::white);
        p.drawText(tr, Qt::AlignCenter, text);
    }
    if (!replay && recording_) {
        p.setPen(Qt::NoPen);
        p.setBrush(QColor("#ef4444"));
        p.drawEllipse(QPointF(label->width() - 44, 12), 4, 4);
        p.setPen(QColor("#fecaca"));
        p.drawText(QRectF(label->width() - 38, 4, 34, 16), Qt::AlignLeft | Qt::AlignVCenter, "REC");
    }
}

// Red object (ball) by HSV thresholding; returns the largest blob as a normalized box.
bool MainWindow::detectTarget(const QImage &rgb, QRectF &box, double &confidence) const
{
#ifndef DISABLE_OPENCV
    if (rgb.isNull()) return false;
    const QImage img = rgb.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(img.height(), img.width(), CV_8UC3, const_cast<uchar *>(img.constBits()), size_t(img.bytesPerLine()));
    cv::Mat hsv, low, high, mask;
    cv::cvtColor(mat, hsv, cv::COLOR_RGB2HSV);
    cv::inRange(hsv, cv::Scalar(0, 120, 80), cv::Scalar(10, 255, 255), low);
    cv::inRange(hsv, cv::Scalar(170, 120, 80), cv::Scalar(180, 255, 255), high);
    mask = low | high;
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double bestArea = 0;
    int best = -1;
    for (int i = 0; i < int(contours.size()); ++i) {
        const double a = cv::contourArea(contours[i]);
        if (a > bestArea) { bestArea = a; best = i; }
    }
    if (best < 0 || bestArea < 0.002 * img.width() * img.height()) return false;
    const double perimeter = cv::arcLength(contours[best], true);
    const double circularity = perimeter > 0 ? 4 * kPi * bestArea / (perimeter * perimeter) : 0;
    if (circularity < 0.5) return false;
    const cv::Rect r = cv::boundingRect(contours[best]);
    box = QRectF(double(r.x) / img.width(), double(r.y) / img.height(),
                 double(r.width) / img.width(), double(r.height) / img.height());
    confidence = std::min(0.99, 0.5 + 0.5 * circularity);
    return true;
#else
    Q_UNUSED(rgb); Q_UNUSED(box); Q_UNUSED(confidence);
    return false;
#endif
}

// Object position = robot pose + camera bearing + lidar range in that direction.
void MainWindow::handleDetection(const QRectF &box, double confidence, const QImage &frame)
{
    liveDetBox_ = box;
    liveDetAge_.restart();
    liveDetLabel_ = QString("%1 %2 %").arg(QString::fromUtf8(kTargetName)).arg(qRound(confidence * 100));
    ui->cameraLabel->update();
    if (!hasPose_) return;

    const double bearing = (0.5 - box.center().x()) * kCameraFovDeg * kPi / 180.0;   // left positive
    double range = -1;
    for (const QPointF &p : lidarPoints_) {
        const double a = std::atan2(-p.y(), p.x());   // robot frame: y right -> left positive angle
        if (std::abs(normAngle(a - bearing)) < 3.0 * kPi / 180.0) {
            const double d = std::hypot(p.x(), p.y()) / 1000.0;
            if (range < 0 || d < range) range = d;
        }
    }
    if (range <= 0 || range > 6.0) return;
    const QPointF pos(pose_.x + range * std::cos(pose_.th + bearing), pose_.y + range * std::sin(pose_.th + bearing));
    for (const Detection &d : detections_)
        if (std::hypot(d.pos.x() - pos.x(), d.pos.y() - pos.y()) < 0.7) return;   // already found

    Detection d;
    d.t = missionTimeMs();
    d.name = QString::fromUtf8(kTargetName);
    d.confidence = confidence;
    d.pos = pos;
    d.box = box;
    QImage annotated = frame.convertToFormat(QImage::Format_RGB32);
    {
        QPainter p(&annotated);
        const QRectF b(box.x() * annotated.width(), box.y() * annotated.height(),
                       box.width() * annotated.width(), box.height() * annotated.height());
        p.setPen(QPen(QColor("#a78bfa"), 3));
        p.drawRect(b);
        p.setPen(Qt::white);
        p.drawText(b.topLeft() + QPointF(2, -6), liveDetLabel_);
    }
    const QString base = recording_ ? recDir_ : QDir(recordsRoot()).filePath("foto");
    QDir().mkpath(QDir(base).filePath("detekcie"));
    const QString rel = QString("detekcie/obj_%1.jpg").arg(detections_.size() + 1, 3, 10, QChar('0'));
    annotated.save(QDir(base).filePath(rel), "JPG", 90);
    d.file = QDir(base).filePath(rel);
    detections_.append(d);
    if (recording_) {
        detOut_ << d.t << ',' << d.name << ',' << QString::number(d.confidence, 'f', 2) << ','
                << QString::number(pos.x(), 'f', 3) << ',' << QString::number(pos.y(), 'f', 3) << ',' << rel << ','
                << box.x() << ',' << box.y() << ',' << box.width() << ',' << box.height() << '\n';
        detOut_.flush();
    }
    addFoundItem(d, annotated);
    addLog(QString::fromUtf8("Nájdený objekt: %1 (x %2, y %3 m) – fotografia uložená")
            .arg(d.name).arg(pos.x(), 0, 'f', 2).arg(pos.y(), 0, 'f', 2), LogLevel::Detection);
    showToast(QString::fromUtf8("Nájdený objekt: %1 – poloha vyznačená v mape").arg(d.name), LogLevel::Detection);
    ui->mapView->update();
}

void MainWindow::addFoundItem(const Detection &d, const QImage &thumb)
{
    auto *item = new QListWidgetItem(QIcon(QPixmap::fromImage(thumb.scaled(96, 64, Qt::KeepAspectRatio, Qt::SmoothTransformation))),
            QString::fromUtf8("%1 · %2 %\nx %3 m, y %4 m · čas %5")
                    .arg(d.name).arg(qRound(d.confidence * 100))
                    .arg(d.pos.x(), 0, 'f', 2).arg(d.pos.y(), 0, 'f', 2).arg(fmtMs(d.t)));
    item->setData(Qt::UserRole, detections_.size() - 1);
    item->setToolTip(d.file);
    ui->foundList->addItem(item);
}

// ============================================================================
// Event filter: painting and mouse interaction of the custom-drawn widgets
// ============================================================================

bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    // Whichever view sits in the side slot is only a preview: a left click swaps it into the main area,
    // other clicks are swallowed so the small map cannot be edited by accident.
    QObject *sideView = viewsSwapped_ ? static_cast<QObject *>(ui->mapView) : ui->cameraLabel;
    if (watched == sideView && (event->type() == QEvent::MouseButtonPress
                                || event->type() == QEvent::MouseButtonDblClick
                                || event->type() == QEvent::MouseButtonRelease)) {
        if (event->type() == QEvent::MouseButtonPress
            && static_cast<QMouseEvent *>(event)->button() == Qt::LeftButton)
            swapViews();
        return true;
    }
    if (watched == ui->cameraLabel && event->type() == QEvent::Paint) { paintCamera(); return true; }
    if (watched == ui->timelineMarks && event->type() == QEvent::Paint) { paintTimelineMarks(); return true; }
    if (watched != ui->mapView) return QMainWindow::eventFilter(watched, event);

    switch (event->type()) {
    case QEvent::Paint:
        paintMap();
        return true;
    case QEvent::Resize:
        layoutOverlays();
        break;
    case QEvent::Leave:
        hasHover_ = false;
        cursorLabel_->clear();
        break;
    case QEvent::MouseMove: {
        auto *me = static_cast<QMouseEvent *>(event);
        if (!mapLoaded_) break;
        const QPointF w = widgetToWorld(evPos(me));
        hasHover_ = mapRect().contains(evPos(me));
        hoverPos_ = w;
        cursorLabel_->setText(hasHover_ ? QString::fromUtf8("x %1 m · y %2 m%3").arg(w.x(), 0, 'f', 2).arg(w.y(), 0, 'f', 2)
                                                  .arg(mode_ == Mode::Planning && isBlocked(w) ? QString::fromUtf8(" · prekážka") : QString())
                                        : QString());
        layoutOverlays();
        if (dragIndex_ >= 0 && (me->buttons() & Qt::LeftButton)) {
            dragMoved_ = true;
            points_[dragIndex_].pos = w;
            ui->mapView->update();
        }
        break;
    }
    case QEvent::MouseButtonPress: {
        auto *me = static_cast<QMouseEvent *>(event);
        if (mode_ != Mode::Planning || !mapLoaded_) break;
        if (!editable()) { showToast(QString::fromUtf8("Počas misie nie je možné upravovať body."), LogLevel::Warning); break; }
        const QPointF pos = evPos(me);
        const int index = pointAt(pos);
        if (me->button() == Qt::LeftButton) {
            if (index >= 0) {
                selectPoint(index);
                dragIndex_ = index;
                dragOrigin_ = points_[index].pos;
                dragMoved_ = false;
            } else if (mapRect().contains(pos)) {
                const QPointF world = widgetToWorld(pos);
                if (isBlocked(world)) {
                    showToast(QString::fromUtf8("Bod nie je možné vložiť do steny ani prekážky."), LogLevel::Danger);
                } else {
                    showAddPointMenu(world, QCursor::pos());
                }
            }
        } else if (me->button() == Qt::RightButton && index >= 0) {
            selectPoint(index);
            showPointMenu(index, QCursor::pos());
        }
        return true;
    }
    case QEvent::MouseButtonRelease: {
        if (dragIndex_ < 0) break;
        const int i = dragIndex_;
        dragIndex_ = -1;
        if (dragMoved_ && isBlocked(points_[i].pos)) {
            points_[i].pos = dragOrigin_;
            showToast(QString::fromUtf8("Bod nemôže byť v kolízii – vrátený na pôvodné miesto."), LogLevel::Danger);
        }
        refreshPointsTable();
        updateValidation();
        ui->mapView->update();
        return true;
    }
    default:
        break;
    }
    return QMainWindow::eventFilter(watched, event);
}
