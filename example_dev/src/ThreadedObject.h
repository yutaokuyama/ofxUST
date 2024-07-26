#pragma once

#include "ofMain.h"
#include "ofxUst.h"
#include "ofxCv.h"
#include <atomic>
#include "USTPostProcessing.h"

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

		ust->setDirection(ofxUST::DIRECTION_UP);
		ust->setScanningParameterByAngles(-135, 135, 1);
		ust->startMeasurement();

		mesh.setMode(OF_PRIMITIVE_POINTS);
		clusterMesh.setMode(OF_PRIMITIVE_POINTS);
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
			fetchSensorData();
			condition.wait(lock);
		}
	}

	void fetchSensorData()
	{
		coordinates = ust->getCoordinates();
		distances = ust->getDistances();
	}

	void applyGauss()
	{
		int kernelSize = 5;
		double sigma = 2.0;
		std::vector<double> gaussianKernel = UST::postProcessing::createGaussianKernel(kernelSize, sigma);
		std::unique_lock<std::mutex> lock(mutex);
		filteredCoordinates = UST::postProcessing::applyGaussianFilter(coordinates, gaussianKernel);
		condition.notify_all();
	}

	void update(bool isCaribMode, cv::Mat homography, glm::vec2 topLeft, glm::vec2 topRight, glm::vec2 bottomRight, glm::vec2 bottomLeft, bool gauss = false)
	{
		applyGauss();
		mesh.clear();
		clusterMesh.clear();
		float searchRadius = 5.0f;
		std::vector<glm::vec2> validPoints;
		for (int i = 0; i < filteredCoordinates.size(); i++)
		{
			glm::vec2 deviceCoordinate = UST::postProcessing::scaleToDeviceCoordinate(filteredCoordinates[i], scale);
			const bool isPointInside = UST::postProcessing::isPointInRectangle(deviceCoordinate, topLeft, topRight, bottomRight, bottomLeft);
			if (!isPointInside)
			{
				continue;
			}

			if (isCaribMode) {
				mesh.addVertex(glm::vec3(deviceCoordinate.x, deviceCoordinate.y, 0.0));
				mesh.addColor(ofColor(255, 255, isPointInside ? 255 : 0));
				validPoints.push_back(glm::vec3(deviceCoordinate.x, deviceCoordinate.y, 0.0));

			}
			else {
				cv::Mat urgPosMat = (cv::Mat_<double>(3, 1) << (double)deviceCoordinate.x, (double)deviceCoordinate.y, 1.0);
				cv::Mat unityPosMat = homography * urgPosMat;
				glm::vec3 projectedPoint = glm::vec3((unityPosMat.at<double>(0)), unityPosMat.at<double>(1), 0.0);
				mesh.addVertex(projectedPoint);
				mesh.addColor(ofColor(255, 255, isPointInside ? 255 : 0));
				validPoints.push_back(glm::vec3(projectedPoint.x, projectedPoint.y, 0.0));

			}


		}
	
		if (validPoints.size() > 50)
		{
			return;
		}
		std::vector<std::vector<glm::vec2>> clusters = UST::postProcessing::clusterPoints(validPoints, searchRadius);

		for (const auto &cluster : clusters)
		{
			if (cluster.size() < 6)
			{
				continue;
			}
			std::cout << cluster.size() << std::endl;


			for (const auto &point : cluster)
			{
				clusterMesh.addVertex(glm::vec3(point, 1.0));

				//if (isCaribMode) {
				//}
				//else {
				//	cv::Mat urgPosMat = (cv::Mat_<double>(3, 1) << (double)point.x, (double)point.y, 1.0);
				//	cv::Mat unityPosMat = homography * urgPosMat;
				//	clusterMesh.addVertex(glm::vec3((unityPosMat.at<double>(0)), unityPosMat.at<double>(1), 0.0));
				//}
				clusterMesh.addColor(ofColor(0, 0, 255));
			}
		}
		// Gaussian�J�[�l���̐���
		// float searchRadius = 1.0f; // ���a1.0�ŃN���X�^�����O
		//// �N���X�^�����O���ʂ̕\��
	}



	void draw()
	{
		{
			ofSetColor(120);
			mesh.draw();
			clusterMesh.draw();
		}
		drawDebugInfo();
	}

	void drawDebugInfo()
	{
		ofSetColor(255);
		int temp_y = 0;
		ofDrawBitmapString("Direction [arrow] : " + ofToString((int)ust->getDirection()), 20, temp_y += 15);
		ofDrawBitmapString("FPS               : " + ofToString(ofGetFrameRate(), 1), 20, temp_y += 15);
		ofDrawBitmapString("line amt       : " + ofToString(ust->getCoordinates().size()), 20, temp_y += 15);
		ofDrawBitmapString("line amt       : " + ofToString(ofGetMouseX())+" "+ ofToString(ofGetMouseY()), 20, temp_y += 15);

	}

	std::vector<glm::vec2> getCoordinate()
	{

		return coordinates;
	}

private:
	std::unique_ptr<ofxUST> ust;
	float scale = 0.15;
	ofVboMesh mesh;
	ofVboMesh clusterMesh;

	std::vector<glm::vec2> coordinates;
	std::vector<glm::vec2> filteredCoordinates;
	std::vector<float> distances;

protected:
	std::condition_variable condition;
};
