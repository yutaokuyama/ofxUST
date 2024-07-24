#pragma once

#include "ofMain.h"
#include "ThreadedObject.h"
#include "ofxCv.h"
#include "ofxGui.h"


class ofApp : public ofBaseApp
{
public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void mousePressed(int x, int y, int button);

	void drawSrcAnchor();

private:


	enum AnchorID{
		TOP_LEFT,
		TOP_RIGHT,
		BOTTOM_LEFT,
		BOTTOM_RIGHT,
		INVALID,
	};


	int settingSrcAnchorId = INVALID;






	void initializeAnchors() {
		dstAnchors = std::vector<ofVec2f>(4);
		dstAnchors[AnchorID::TOP_LEFT] = ofVec2f(0.0, 0.0);
		dstAnchors[AnchorID::TOP_RIGHT] = ofVec2f(ofGetWidth(), 0.0);
		dstAnchors[AnchorID::BOTTOM_LEFT] = ofVec2f(0.0,ofGetHeight());
		dstAnchors[AnchorID::BOTTOM_RIGHT] = ofVec2f(ofGetWidth(), ofGetHeight());


		srcAnchors = std::vector<ofVec2f>(4);
	}

	void moveSrcAnchor();


	std::vector<ofVec2f> srcAnchors,dstAnchors;



	ThreadedObject threadedObject;
	cv::Mat homography;
	ofImage left, right, warpedColor;


	void guiSetup();
	ofxPanel gui;
	ofxToggle setLeftTopAnchor;
	ofxToggle setRightTopAnchor;
	ofxToggle setLeftBottomAnchor;
	ofxToggle setRightBottomAnchor;
	ofxToggle isCalib;

	ofxVec2Slider thresh;
	ofxToggle gauss;

	void disableAllToggle() {
		setLeftTopAnchor = false;
		setRightTopAnchor = false;
		setLeftBottomAnchor = false;
		setRightBottomAnchor = false;
	}

	ofVboMesh mesh;
	glm::mat3 tmpMat;




	

};
