#pragma once

#include "ofMain.h"
#include "ofxUst.h"
#include "ofxCv.h"
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

		ust->setDirection(ofxUST::DIRECTION_UP);
		ust->setScanningParameterByAngles(-135, 135, 1);
		ust->startMeasurement();

		mesh.setMode(OF_PRIMITIVE_POINTS);
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
			distances = ust->getDistances();
			condition.wait(lock);
		}
	}

	void update(glm::vec2 thresh, bool gauss = false)
	{
		std::unique_lock<std::mutex> lock(mutex);
		mesh.clear();
		// Gaussian�J�[�l���̐���
		int kernelSize = 5;
		double sigma = 2.0;
		std::vector<double> gaussianKernel = createGaussianKernel(kernelSize, sigma);

		// ���W�f�[�^�̃t�B���^�����O
		std::vector<glm::vec2> filteredCoordinates = applyGaussianFilter(coordinates, gaussianKernel);

		// float searchRadius = 1.0f; // ���a1.0�ŃN���X�^�����O
		// std::vector<std::vector<glm::vec2>> clusters = clusterPoints(filteredCoordinates, searchRadius);

		//// �N���X�^�����O���ʂ̕\��
		// for (const auto& cluster : clusters) {
		//	std::cout << "Cluster:\n";
		//	for (const auto& point : cluster) {
		//		std::cout << "(" << point.x << ", " << point.y << ")\n";
		//	}
		//	std::cout << std::endl;
		// }

		for (int i = 0; i < coordinates.size(); ++i)
		{
			glm::vec3 scaledPositoin;
			if (gauss)
			{
				scaledPositoin = glm::vec3(filteredCoordinates.at(i).x, filteredCoordinates.at(i).y, 0.0) * scale;
			}
			else
			{

				scaledPositoin = glm::vec3(coordinates.at(i).x, coordinates.at(i).y, 1.0) * scale;
			}

			glm::vec3 ofsettedPosition = glm::vec3(scaledPositoin.x + (ofGetWidth() / 2), scaledPositoin.y + (ofGetHeight() / 2), 0.0);

			mesh.addVertex(ofsettedPosition);
		}
		condition.notify_all();
	}

	void update(glm::vec2 thresh, cv::Mat mat)
	{
		std::unique_lock<std::mutex> lock(mutex);
		mesh.clear();

		for (int i = 0; i < coordinates.size(); ++i)
		{
			glm::vec3 scaledPositoin = glm::vec3(coordinates.at(i).x, coordinates.at(i).y, 1.0) * scale;

			glm::vec3 ofsettedPosition = glm::vec3(scaledPositoin.x + (ofGetWidth() / 2), scaledPositoin.y + (ofGetHeight() / 2), 0.0);
			cv::Mat urgPosMat = (cv::Mat_<double>(3, 1) << (double)ofsettedPosition.x, (double)ofsettedPosition.y, 1.0);

			cv::Mat unityPosMat = mat * urgPosMat;

			mesh.addVertex(glm::vec3((unityPosMat.at<double>(0)), unityPosMat.at<double>(1), 0.0));
		}
		condition.notify_all();
	}
	void updateWithMat()
	{
	}

	void draw()
	{
		{
			ofSetColor(120);
			mesh.draw();
		}
	}

	void drawDebugInfo()
	{
		ofSetColor(255);
		int temp_y = 0;
		ofDrawBitmapString("Direction [arrow] : " + ofToString((int)ust->getDirection()), 20, temp_y += 15);
		ofDrawBitmapString("FPS               : " + ofToString(ofGetFrameRate(), 1), 20, temp_y += 15);
		ofDrawBitmapString("line amt       : " + ofToString(ust->getCoordinates().size()), 20, temp_y += 15);
	}

	std::vector<glm::vec2> getCoordinate()
	{

		return coordinates;
	}

	std::vector<double> createGaussianKernel(int kernelSize, double sigma)
	{
		std::vector<double> kernel(kernelSize);
		double sum = 0.0;
		int halfSize = kernelSize / 2;

		for (int i = -halfSize; i <= halfSize; ++i)
		{
			kernel[i + halfSize] = std::exp(-0.5 * std::pow(i / sigma, 2)) / (sigma * std::sqrt(2 * 3.14));
			sum += kernel[i + halfSize];
		}

		// �J�[�l���̐��K��
		for (double &value : kernel)
		{
			value /= sum;
		}

		return kernel;
	}

	// ���W�f�[�^�̃t�B���^�����O
	std::vector<glm::vec2> applyGaussianFilter(std::vector<glm::vec2> &coordinates, std::vector<double> &kernel)
	{
		int dataSize = coordinates.size();
		int kernelSize = kernel.size();
		int halfKernelSize = kernelSize / 2;
		std::vector<glm::vec2> filteredData(dataSize, glm::vec2(0.0));

		for (int i = 0; i < dataSize; ++i)
		{
			double sumX = 0.0;
			double sumY = 0.0;

			for (int j = -halfKernelSize; j <= halfKernelSize; ++j)
			{
				int dataIndex = i + j;

				// ���E�����̏���
				if (dataIndex < 0)
				{
					dataIndex = 0; // �[���p�f�B���O
				}
				else if (dataIndex >= dataSize)
				{
					dataIndex = dataSize - 1; // �[���p�f�B���O
				}

				sumX += coordinates[dataIndex].x * kernel[j + halfKernelSize];
				sumY += coordinates[dataIndex].y * kernel[j + halfKernelSize];
			}

			filteredData[i] = glm::vec2(sumX, sumY);
		}

		return filteredData;
	}

	float calculateDistance(const glm::vec2 &p1, const glm::vec2 &p2)
	{
		return glm::length(p1 - p2);
	}

	// �ŋߖT������p�����_�̃O���[�v��
	std::vector<std::vector<glm::vec2>> clusterPoints(const std::vector<glm::vec2> &points, float radius)
	{
		std::vector<bool> visited(points.size(), false);
		std::vector<std::vector<glm::vec2>> clusters;

		for (size_t i = 0; i < points.size(); ++i)
		{
			if (visited[i])
				continue;

			std::vector<glm::vec2> cluster;
			std::vector<size_t> toVisit;
			toVisit.push_back(i);

			while (!toVisit.empty())
			{
				size_t index = toVisit.back();
				toVisit.pop_back();
				if (visited[index])
					continue;

				visited[index] = true;
				cluster.push_back(points[index]);

				// ���a���̋ߖT�_��T��
				for (size_t j = 0; j < points.size(); ++j)
				{
					if (!visited[j] && calculateDistance(points[index], points[j]) <= radius)
					{
						toVisit.push_back(j);
					}
				}
			}

			clusters.push_back(cluster);
		}

		return clusters;
	}

private:
	std::unique_ptr<ofxUST> ust;
	float scale = 0.15;
	ofVboMesh mesh;
	std::vector<glm::vec2> coordinates;
	std::vector<glm::vec2> filteredCoordinates;
	std::vector<float> distances;

protected:
	std::condition_variable condition;
};
