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
			coordinates = ust->getCoordinates();
			distances = ust->getDistances();
			condition.wait(lock);
		}
	}

	void filterGauss()
	{
		int kernelSize = 5;
		double sigma = 2.0;
		std::vector<double> gaussianKernel = createGaussianKernel(kernelSize, sigma);

		std::unique_lock<std::mutex> lock(mutex);
		filteredCoordinates = applyGaussianFilter(coordinates, gaussianKernel);
		condition.notify_all();
	}

	glm::vec2 scaleToDeviceCoordinate(glm::vec2 &coordinates)
	{
		glm::vec2 scaledPosition = glm::vec2(coordinates.x, coordinates.y) * scale;
		glm::vec2 offsettedPosition = glm::vec2(scaledPosition.x + (ofGetWidth() / 2), scaledPosition.y + (ofGetHeight() / 2));
		return glm::vec2(ofGetWidth() - offsettedPosition.x, offsettedPosition.y);
	}

	void update(glm::vec2 topLeft, glm::vec2 topRight, glm::vec2 bottomRight, glm::vec2 bottomLeft, bool gauss = false)
	{
		filterGauss();
		mesh.clear();
		clusterMesh.clear();

		float searchRadius = 3.0f;

		std::vector<glm::vec2> validPoints;
		for (int i = 0; i < filteredCoordinates.size(); i++)
		{
			glm::vec2 deviceCoordinate = scaleToDeviceCoordinate(filteredCoordinates[i]);
			const bool isPointInside = isPointInRectangle(deviceCoordinate, topLeft, topRight, bottomRight, bottomLeft);
			if (!isPointInside)
			{
				continue;
			}

			validPoints.push_back(glm::vec3(deviceCoordinate.x, deviceCoordinate.y, 0.0));
			mesh.addVertex(glm::vec3(deviceCoordinate.x, deviceCoordinate.y, 0.0));
			mesh.addColor(ofColor(255, 255, isPointInside ? 255 : 0));
			// validPoints.push_back(deviceCoordinate);
		}
		if (validPoints.size() > 300)
		{
			return;
		}
		std::vector<std::vector<glm::vec2>> clusters = clusterPoints(validPoints, searchRadius);

		for (const auto &cluster : clusters)
		{
			std::cout << cluster.size() << std::endl;
			if (cluster.size() < 6)
			{
				continue;
			}

			for (const auto &point : cluster)
			{
				clusterMesh.addVertex(glm::vec3(point, 1.0));
				clusterMesh.addColor(ofColor(0, 0, 255));
			}
		}
		std::cout << "momo" << std::endl;

		// Gaussian�J�[�l���̐���

		// float searchRadius = 1.0f; // ���a1.0�ŃN���X�^�����O

		//// �N���X�^�����O���ʂ̕\��

		for (int i = 0; i < coordinates.size(); ++i)
		{
		}
	}

	void update(glm::vec2 thresh, cv::Mat mat)
	{
		filterGauss();

		mesh.clear();

		for (int i = 0; i < coordinates.size(); ++i)
		{
			glm::vec3 scaledPosition = glm::vec3(coordinates.at(i).x, coordinates.at(i).y, 1.0) * scale;
			// XXX
			glm::vec3 offsettedPosition = glm::vec3(scaledPosition.x + (ofGetWidth() / 2), scaledPosition.y + (ofGetHeight() / 2), 0.0);
			cv::Mat urgPosMat = (cv::Mat_<double>(3, 1) << (double)offsettedPosition.x, (double)offsettedPosition.y, 1.0);
			cv::Mat unityPosMat = mat * urgPosMat;

			mesh.addVertex(glm::vec3((unityPosMat.at<double>(0)), unityPosMat.at<double>(1), 0.0));
		}
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

	// ベクトルのクロスプロダクトを計算
	float crossProduct(const glm::vec2 &a, const glm::vec2 &b)
	{
		return a.x * b.y - a.y * b.x;
	}

	// 点が矩形の内側にあるかを判定
	bool isPointInRectangle(const glm::vec2 &p, const glm::vec2 &topLeft, const glm::vec2 &topRight, const glm::vec2 &bottomRight, const glm::vec2 &bottomLeft)
	{
		std::vector<glm::vec2> rect = {topLeft, topRight, bottomRight, bottomLeft};

		for (size_t i = 0; i < 4; ++i)
		{
			glm::vec2 v1 = rect[i];
			glm::vec2 v2 = rect[(i + 1) % 4];
			glm::vec2 edge = v2 - v1;
			glm::vec2 vp = p - v1;

			if (crossProduct(edge, vp) < 0)
			{
				return false;
			}
		}
		return true;
	}

	// 矩形の内側にある点をフィルタリング
	std::vector<glm::vec2> filterPointsInsideRectangle(const std::vector<glm::vec2> &points, const glm::vec2 &topLeft, const glm::vec2 &topRight, const glm::vec2 &bottomRight, const glm::vec2 &bottomLeft)
	{
		std::vector<glm::vec2> filteredPoints;

		for (const auto &point : points)
		{
			if (isPointInRectangle(point, topLeft, topRight, bottomRight, bottomLeft))
			{
				filteredPoints.push_back(point);
			}
		}

		return filteredPoints;
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
