#pragma once

#include "ofMain.h"
namespace UST
{

	// ofUST
	//----------------------------------------
	class postProcessing
	{
	public:
		static const std::vector<double> createGaussianKernel(int kernelSize, double sigma)
		{
			std::vector<double> kernel(kernelSize);
			double sum = 0.0;
			int halfSize = kernelSize / 2;

			for (int i = -halfSize; i <= halfSize; ++i)
			{
				kernel[i + halfSize] = std::exp(-0.5 * std::pow(i / sigma, 2)) / (sigma * std::sqrt(2 * 3.14));
				sum += kernel[i + halfSize];
			}

			for (double &value : kernel)
			{
				value /= sum;
			}

			return kernel;
		}

		static const std::vector<glm::vec2> applyGaussianFilter(std::vector<glm::vec2> &coordinates, std::vector<double> &kernel)
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

					if (dataIndex < 0)
					{
						dataIndex = 0;
					}
					else if (dataIndex >= dataSize)
					{
						dataIndex = dataSize - 1;
					}

					sumX += coordinates[dataIndex].x * kernel[j + halfKernelSize];
					sumY += coordinates[dataIndex].y * kernel[j + halfKernelSize];
				}

				filteredData[i] = glm::vec2(sumX, sumY);
			}

			return filteredData;
		}

		static const float calculateDistance(const glm::vec2 &p1, const glm::vec2 &p2)
		{
			return glm::length(p1 - p2);
		}

		// ベクトルのクロスプロダクトを計算
		static const float crossProduct(const glm::vec2 &a, const glm::vec2 &b)
		{
			return a.x * b.y - a.y * b.x;
		}

		static std::vector<std::vector<glm::vec2>> clusterPoints(const std::vector<glm::vec2> &points, float radius)
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

		// 点が矩形の内側にあるかを判定
		static bool isPointInRectangle(const glm::vec2 &p, const glm::vec2 &topLeft, const glm::vec2 &topRight, const glm::vec2 &bottomRight, const glm::vec2 &bottomLeft)
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
		static std::vector<glm::vec2> filterPointsInsideRectangle(const std::vector<glm::vec2> &points, const glm::vec2 &topLeft, const glm::vec2 &topRight, const glm::vec2 &bottomRight, const glm::vec2 &bottomLeft)
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

		static glm::vec2 scaleToDeviceCoordinate(glm::vec2 &coordinates, float scale)
		{
			glm::vec2 scaledPosition = glm::vec2(coordinates.x, coordinates.y) * scale;
			glm::vec2 offsettedPosition = glm::vec2(scaledPosition.x + (ofGetWidth() / 2), scaledPosition.y + (ofGetHeight() / 2));
			return glm::vec2(ofGetWidth() - offsettedPosition.x, offsettedPosition.y);
		}
	};
};