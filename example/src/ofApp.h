#pragma once

#include "ofMain.h"
#include "ThreadedObject.h"

class ofApp : public ofBaseApp
{
public:
	void setup();
	void update();
	void draw();

	ThreadedObject threadedObject;
};
