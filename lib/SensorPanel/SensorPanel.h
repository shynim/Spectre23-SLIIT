//
// Created by lakshith on 8/8/22.
//
#include <QTRSensors.h>
#include <Variables.h>

#ifndef ROBOFEST22_SENSORPANEL_H
#define ROBOFEST22_SENSORPANEL_H

class SensorPanel {
public:
    char pattern = 0;
    uint16_t position = 0;
    int error = 0;
    uint16_t panelReading[16];
    uint16_t rawReadings[16];
    bool isMiddle;
    bool isEnd;
    bool junc;
    bool left;
    bool right;

    SensorPanel(uint8_t *sensorPins);

    void calibrate();

    void read();

private:
    QTRSensors qtr;
    const uint8_t SensorCount = 16;

    uint16_t readLine(uint16_t *sensorValues);

    void updatePattern();
};

#endif //ROBOFEST22_SENSORPANEL_H

