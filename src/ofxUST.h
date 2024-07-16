#pragma once

#include "Urg_driver.h"
#include "ofMain.h"

using namespace qrk;

// ofUST
//----------------------------------------
class ofxUST
{
public:
    ofxUST(std::string deviceIp = "192.168.0.10", int port = 10940,float checkInterval = 1.0);

    enum Direction
    {
        DIRECTION_RIGHT = 0,
        DIRECTION_DOWN,
        DIRECTION_LEFT,
        DIRECTION_UP,
        DIRECTION_SIZE
    };

    void open();

    void setDirection(Direction _dir);
    void setMirror(bool _b);

    void setScanningParameterBySteps(int _minStep, int _maxStep, int _skipStep);
    void setScanningParameterByAngles(float _minAngle, float _maxAngle, int _skipStep);

    Direction getDirection();

    int getMinDistance();
    int getMaxDistance();

    int getMinStep();
    int getMaxStep();

    bool isConnected();

    void startMeasurement();
    void stopMeasurement();

    void update();

    void close();

    const std::vector<ofVec2f>& getCoordinates();




private:
    Urg_driver urg;
    bool bConnected;
    std::vector<ofVec2f> coordinates;
    std::vector<long> data;


    int minStep, maxStep;
    float minAngle, maxAngle;
    int skip;

    Direction direction;
    bool bMirror;

    float time = 0.0;
    const float checkInterval;
    const string deviceIp;
    const int port;
    float lastCheckTime = 0.0;
    
};
