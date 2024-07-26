#pragma once

#include "ofMain.h"
#include "ofxUst.h"
#include <atomic>

class ThreadedObject : public ofThread
{
public:
	~ThreadedObject()
	{
		stop();
		waitForThread(false);
	}

	void setup()
	{
		ust = std::make_unique<ofxUST>("192.168.1.20");


		bool isConnectionFailed = !(ust->open());
		if (isConnectionFailed)
		{
			ofLog() << "Connection failed!";
			return;
		}

		ust->setDirection(ofxUST::DIRECTION_RIGHT);
		ust->setScanningParameterByAngles(-135, 135, 1);
		ust->startMeasurement();

		mesh.setMode(OF_PRIMITIVE_LINES);
		startThread();
	}

	void stop()
	{
		std::unique_lock<std::mutex> lck(mutex);
		stopThread();
		condition.notify_all();
	}

	void threadedFunction()
	{
		while (isThreadRunning())
		{
			ust->update();
			std::unique_lock<std::mutex> lock(mutex);
			coordinates = ust->getCoordinates();
			condition.wait(lock);
		}
	}

	void update()
	{
		std::unique_lock<std::mutex> lock(mutex);
		mesh.clear();
		for (int i = 0; i < coordinates.size(); ++i)
		{
			mesh.addVertex(glm::vec3(0.0, 0.0, 0.0));
			mesh.addVertex(glm::vec3(coordinates.at(i).x, coordinates.at(i).y, 0.0));
		}
		condition.notify_all();
	}

	void draw()
	{
		ofPushMatrix();
		{
			ofTranslate(ofGetWidth() / 2, ofGetHeight() / 2);
			ofScale(scale, scale);

			ofSetColor(120);
			mesh.draw();
		}
		ofPopMatrix();
		ofSetColor(255);
		int temp_y = 0;
		ofDrawBitmapString("Direction [arrow] : " + ofToString((int)ust->getDirection()), 20, temp_y += 15);
		ofDrawBitmapString("FPS               : " + ofToString(ofGetFrameRate(), 1), 20, temp_y += 15);
		ofDrawBitmapString("line amt       : " + ofToString(ust->getCoordinates().size()), 20, temp_y += 15);
	}

private:
	std::unique_ptr<ofxUST> ust;
	float scale = 0.15;
	ofVboMesh mesh;
	std::vector<ofVec2f> coordinates;

protected:
	std::condition_variable condition;
};
