#include "ofApp.h"



// setup
//----------------------------------------
void ofApp::setup()
{
	ofBackground(0);
	ofSetVerticalSync(true);

	threadedObject.setup();
	initializeAnchors();

	left.load("GLvPlPVaIAAlchb.jfif");
	guiSetup();

}

void ofApp::guiSetup() {
	gui.setup();
	gui.add(isCalib.setup("calib", true));
	gui.add(setLeftTopAnchor.setup("SetLeftTopAnchor", false));
	gui.add(setRightTopAnchor.setup("SetRightTopAnchor", false));
	gui.add(setLeftBottomAnchor.setup("SetLeftBottomAnchor", false));
	gui.add(setRightBottomAnchor.setup("setRightBottomAnchor", false));
	gui.add(thresh.setup("thresh", glm::vec2(100, 10000), glm::vec2(0), glm::vec2(10000)));
	gui.add(gauss.setup("gauss", false));

}

// update
//----------------------------------------
void ofApp::update()
{
	threadedObject.getCoordinate();

	vector<cv::Point2f> srcPoints, dstPoints;
	for (int i = 0; i < 4; i++) {
		srcPoints.push_back(cv::Point2f(srcAnchors[i].x, srcAnchors[i].y));
		dstPoints.push_back(cv::Point2f(dstAnchors[i].x, dstAnchors[i].y));
	}
	homography = findHomography(cv::Mat(srcPoints), cv::Mat(dstPoints));

	threadedObject.update(isCalib,homography, srcAnchors[TOP_LEFT], srcAnchors[TOP_RIGHT], srcAnchors[BOTTOM_RIGHT], srcAnchors[BOTTOM_LEFT]);



	//ofxCv::warpPerspective(left, warpedColor, homography, CV_INTER_LINEAR);


	

}

// draw
//----------------------------------------
void ofApp::draw()
{
	threadedObject.draw();

	ofSetColor(255);

	mesh.draw();
	moveSrcAnchor();
	drawSrcAnchor();

	gui.draw();

}

void ofApp::keyPressed(int key) {
	disableAllToggle();
	switch (key) {
	case '1':
		settingSrcAnchorId = TOP_LEFT;
		setLeftTopAnchor = true;
		return;
	case '2':
		settingSrcAnchorId = TOP_RIGHT;
		setRightTopAnchor = true;
		return;

	case '3':
		settingSrcAnchorId = BOTTOM_LEFT;
		setLeftBottomAnchor = true;
		return;

	case '4':
		settingSrcAnchorId = BOTTOM_RIGHT;
		setRightBottomAnchor = true;
		return;
	case '0':
		isCalib = !isCalib;
		settingSrcAnchorId = INVALID;
		return;
	default:
		settingSrcAnchorId = INVALID;
	}
}

void ofApp::drawSrcAnchor() {
	for (int i = 0; i < 4; i++) {
		ofColor color;
		switch (i) {
		case TOP_LEFT:
			color = ofColor(255, 0,0);
			break;
		case TOP_RIGHT:
			color = ofColor(0, 255,0);
			break;

		case BOTTOM_LEFT:
			color = ofColor(0, 0,255);
			break;

		case BOTTOM_RIGHT:
			color = ofColor(255, 255, 0);
			break;

		}
		ofSetColor(color);
		ofDrawCircle(srcAnchors[i].x, srcAnchors[i].y, 3);
		ofSetColor(255);

	}
}


void ofApp::moveSrcAnchor() {
	if (settingSrcAnchorId == INVALID) {
		return;
	}
	srcAnchors[settingSrcAnchorId] = ofVec2f(mouseX, mouseY);


}

void ofApp::mousePressed(int x, int y, int button) {
	if (settingSrcAnchorId == INVALID) {
		return;
	}
	srcAnchors[settingSrcAnchorId] = ofVec2f(mouseX, mouseY);
	settingSrcAnchorId = AnchorID::INVALID;

	disableAllToggle();
}