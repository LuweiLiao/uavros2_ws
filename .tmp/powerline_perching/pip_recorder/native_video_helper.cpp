#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>

#include <QCoreApplication>
#include <QFileInfo>
#include <QGuiApplication>
#include <QMetaObject>
#include <QObject>
#include <QString>
#include <QTimer>
#include <QWindow>

#include <gz/gui/Application.hh>
#include <tinyxml2.h>

namespace
{
QString outputPath;
QString controlPath;
bool loadRequested = false;

QObject *find_video_recorder(QObject *root)
{
  if (!root)
    return nullptr;
  for (QObject *child : root->children())
  {
    if (child && child->metaObject() &&
        QString::fromLatin1(child->metaObject()->className())
            .contains(QStringLiteral("VideoRecorder")))
      return child;
    if (QObject *found = find_video_recorder(child))
      return found;
  }
  return nullptr;
}

QObject *find_video_recorder_anywhere(QCoreApplication *app)
{
  if (auto *gzapp = gz::gui::App())
  {
    if (QObject *found = find_video_recorder(gzapp))
      return found;
  }
  if (QObject *found = find_video_recorder(app))
    return found;
  for (QWindow *window : QGuiApplication::allWindows())
  {
    if (QObject *found = find_video_recorder(window))
      return found;
  }
  return nullptr;
}

void start_capture(QObject *app)
{
  if (!QFileInfo::exists(controlPath + ".start"))
  {
    QTimer::singleShot(250, app, [app]() { start_capture(app); });
    return;
  }
  QObject *recorder = find_video_recorder_anywhere(
      static_cast<QCoreApplication *>(app));
  if (!recorder)
  {
    if (!loadRequested && gz::gui::App())
    {
      loadRequested = true;
      if (std::getenv("UAVROS_GZ_WIDE"))
        for (QWindow *window : QGuiApplication::allWindows())
          if (window->isVisible() && window->width() > 500)
            window->resize(1500, 900);
      tinyxml2::XMLDocument config;
      config.Parse("<plugin filename='VideoRecorder'><record_video>"
          "<use_sim_time>true</use_sim_time><lockstep>false</lockstep>"
          "<bitrate>2070000</bitrate></record_video></plugin>");
      const char *plugin = std::getenv("PERCHING_VIDEO_PLUGIN");
      const bool loaded = gz::gui::App()->LoadPlugin(
          plugin ? plugin : "VideoRecorder", config.RootElement());
      std::fprintf(stderr, "[capture-helper] native LoadPlugin returned %d; sim_time=true\n", loaded);
    }
    std::fprintf(stderr, "[capture-helper] VideoRecorder not found yet\n");
    QTimer::singleShot(500, app, [app]() { start_capture(app); });
    return;
  }

  std::fprintf(stderr, "[capture-helper] found VideoRecorder=%p, starting\n",
      static_cast<void *>(recorder));
  const bool started = QMetaObject::invokeMethod(recorder, "OnStart", Qt::DirectConnection,
      Q_ARG(QString, QStringLiteral("mp4")));
  std::fprintf(stderr, "[capture-helper] OnStart returned %d\n", started);

  // The test requests start/stop; the built-in VideoRecorder owns all image
  // acquisition and encoding. This helper only invokes its normal GUI slots.
  auto *stopTimer = new QTimer(recorder);
  QObject::connect(stopTimer, &QTimer::timeout, recorder, [recorder, stopTimer]() {
    if (!QFileInfo::exists(controlPath + ".stop"))
      return;
    stopTimer->stop();
    std::fprintf(stderr, "[capture-helper] stopping\n");
    const bool stopped = QMetaObject::invokeMethod(recorder, "OnStop", Qt::DirectConnection);
    std::fprintf(stderr, "[capture-helper] OnStop returned %d\n", stopped);
    QTimer::singleShot(1500, recorder, [recorder]() {
      std::fprintf(stderr, "[capture-helper] saving\n");
      const bool saved = QMetaObject::invokeMethod(recorder, "OnSave", Qt::DirectConnection,
          Q_ARG(QString, QStringLiteral("file://") + outputPath));
      std::fprintf(stderr, "[capture-helper] OnSave returned %d\n", saved);
    });
  });
  stopTimer->start(250);
}
}

extern "C" __attribute__((constructor)) void gz_builtin_video_helper_init()
{
  const char *path = std::getenv("UAVROS_GZ_VIDEO_OUTPUT");
  if (!path)
    return;
  // The process may load this helper before C++ global initialization.
  // Defer all QString use until the Qt application exists.
  std::fprintf(stderr, "[capture-helper] constructor\n");
  std::thread([]() {
    QCoreApplication *app = nullptr;
    while (!app)
    {
      app = QCoreApplication::instance();
      if (!app)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::fprintf(stderr, "[capture-helper] app=%p, scheduling\n",
        static_cast<void *>(app));
    outputPath = QString::fromLocal8Bit(std::getenv("UAVROS_GZ_VIDEO_OUTPUT"));
    controlPath = QString::fromLocal8Bit(std::getenv("UAVROS_GZ_VIDEO_CONTROL"));
    QMetaObject::invokeMethod(app, [app]() { start_capture(app); },
        Qt::QueuedConnection);
  }).detach();
}
