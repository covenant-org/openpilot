#include "selfdrive/ui/qt/window.h"

#include "common/swaglog.h"
#include <QFontDatabase>
#include <cassert>

#include "system/hardware/hw.h"
#include "window.h"
#include <ctime>
#include <errno.h>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <assert.h>

MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
  main_layout = new QStackedLayout(this);
  main_layout->setMargin(0);
  screenshots_path = "/data/media/0/openpilot_screenshots";
  std::stat buf;
  if (lstat(screenshots_path.c_str(), &buf) != 0) {
    if (errno == ENOENT) {
      mkdir(screenshots_path.c_str(), S_IRWXU);
    } else {
      assert(0);
    }
  }
  time_t timestamp = time(&timestamp);
  tm datetime = *localtime(&timestamp);
  std::string date = std::to_string(datetime.tm_year + 1900) + "-" +
                     std::to_string(datetime.tm_mon + 1) + "-" +
                     std::to_string(datetime.tm_mday);
  std::string time = std::to_string(datetime.tm_hour) +
                     std::to_string(datetime.tm_min) +
                     std::to_string(datetime.tm_sec);
  screenshots_path = screenshots_path + "/" + date + "_" + time;
  if (lstat(screenshots_path.c_str(), &buf) != 0) {
    if (errno == ENOENT) {
      mkdir(screenshots_path.c_str(), S_IRWXU);
    } else {
      assert(0);
    }
  }

  homeWindow = new HomeWindow(this);
  main_layout->addWidget(homeWindow);
  QObject::connect(homeWindow, &HomeWindow::openSettings, this,
                   &MainWindow::openSettings);
  QObject::connect(homeWindow, &HomeWindow::closeSettings, this,
                   &MainWindow::closeSettings);

  settingsWindow = new SettingsWindow(this);
  main_layout->addWidget(settingsWindow);
  QObject::connect(settingsWindow, &SettingsWindow::closeSettings, this,
                   &MainWindow::closeSettings);
  QObject::connect(settingsWindow, &SettingsWindow::reviewTrainingGuide, [=]() {
    onboardingWindow->showTrainingGuide();
    main_layout->setCurrentWidget(onboardingWindow);
  });
  QObject::connect(settingsWindow, &SettingsWindow::showDriverView,
                   [=] { homeWindow->showDriverView(true); });

  screenshotTimer = new QTimer(this);
  QObject::connect(screenshotTimer, &QTimer::timeout, this,
                   &MainWindow::takeScreenshot);
  screenshotTimer->start(300);

  onboardingWindow = new OnboardingWindow(this);
  main_layout->addWidget(onboardingWindow);
  QObject::connect(onboardingWindow, &OnboardingWindow::onboardingDone,
                   [=]() { main_layout->setCurrentWidget(homeWindow); });
  if (!onboardingWindow->completed()) {
    main_layout->setCurrentWidget(onboardingWindow);
  }

  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    if (!offroad) {
      closeSettings();
    }
  });
  QObject::connect(device(), &Device::interactiveTimeout, [=]() {
    if (main_layout->currentWidget() == settingsWindow) {
      closeSettings();
    }
  });

  // load fonts
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-Black.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-Bold.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-ExtraBold.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-ExtraLight.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-Medium.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-Regular.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-SemiBold.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-Thin.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/JetBrainsMono-Medium.ttf");

  // no outline to prevent the focus rectangle
  setStyleSheet(R"(
    * {
      font-family: Inter;
      outline: none;
    }
  )");
  setAttribute(Qt::WA_NoSystemBackground);
}

void MainWindow::openSettings(int index, const QString &param) {
  main_layout->setCurrentWidget(settingsWindow);
  settingsWindow->setCurrentPanel(index, param);
}

void MainWindow::closeSettings() {
  main_layout->setCurrentWidget(homeWindow);

  if (uiState()->scene.started) {
    homeWindow->showSidebar(false);
  }
}

void MainWindow::takeScreenshot() {
  std::string filename =
      this->screenshots_path + std::to_string(this->nextScreenshot) + ".png";
  QString filePath = filename.c_str();
  this->nextScreenshot++;
  auto image = this->grab();
  std::thread t([image, filePath] { image.save(filePath, "PNG"); });
  t.detach();
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event) {
  bool ignore = false;
  switch (event->type()) {
  case QEvent::TouchBegin:
  case QEvent::TouchUpdate:
  case QEvent::TouchEnd:
  case QEvent::MouseButtonPress:
  case QEvent::MouseMove: {
    // ignore events when device is awakened by resetInteractiveTimeout
    ignore = !device()->isAwake();
    device()->resetInteractiveTimeout();
    break;
  }
  default:
    break;
  }
  return ignore;
}
