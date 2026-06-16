#pragma once

#include <QWidget>

class LidarWidget : public QWidget {
    Q_OBJECT

public:
    explicit LidarWidget(QWidget* parent = nullptr);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QPointF radarToScreen(float x, float y) const;
};
