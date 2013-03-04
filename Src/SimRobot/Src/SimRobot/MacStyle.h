#pragma once

#include <QMacStyle>

class MainWindow;

class MacStyle : public QMacStyle
{
public:
  int pixelMetric(PixelMetric metric, const QStyleOption* option = 0, const QWidget* widget = 0 ) const
  {
    int s = QMacStyle::pixelMetric(metric, option, widget);
    if (metric == QStyle::PM_SmallIconSize)
    {
      s = 20;
    }
    return s;
  }

  static void addFullscreen(MainWindow *window);
};
