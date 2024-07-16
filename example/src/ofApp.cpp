#include "ofApp.h"

// setup
//----------------------------------------
void ofApp::setup()
{
    ofBackground(0);
    threadedObject.setup();
}

// update
//----------------------------------------
void ofApp::update()
{
    threadedObject.update();
}

// draw
//----------------------------------------
void ofApp::draw()
{
    threadedObject.draw();
}

