#include <QApplication>
#include <QScreen>
#include <QPixmap>
#include <QGuiApplication>
#include <QImage>
#include <iostream>

QImage captureScreen() {
  QScreen *screen = QGuiApplication::primaryScreen();
  if (!screen) return QImage();
  return screen->grabWindow(0).toImage(); // Grab entire screen
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    QImage image = captureScreen();
    if (image.isNull()) {
      std::cerr << "Failed to capture screen!" << std::endl;
      return -1;
    }

    QString filePath = "processed_screenshot.png";
    if (!image.save(filePath, "PNG")) {
      std::cerr << "Failed to save image!" << std::endl;
      return -1;
    }

    std::cout << "Screenshot saved to " << filePath.toStdString() << std::endl;
    return 0;
}
