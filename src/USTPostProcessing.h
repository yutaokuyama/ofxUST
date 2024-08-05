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
			//std::cout <<"p1: " << p1 << "p2:"<<p2 << std::endl;

			return glm::length(p1 - p2);
		}


		// 点が指定された範囲内にあるかをチェックする関数
		static bool isPointInRange(const glm::vec2& point, const glm::vec4& range) {
			//std::cout << "x: "<< range.x << "w: " << range.z << std::endl;
			//std::cout << "(point.x >= range.x) "<<(point.x >= range.x) << std::endl;
			//std::cout << " point.x <= range.x + range.z  " << (point.x)<<"asd" << range.x + range.z <<(point.x <= range.x + range.z )<< std::endl;
			//std::cout << "point.y >= range.y  " << (point.y >= range.y )<< std::endl;
			//std::cout << "  point.y <= range.y + range.w " << (point.y <= range.y + range.w)<< std::endl;
			return (point.x >= range.x && point.x <= range.x + range.z &&
				point.y >= range.y && point.y <= range.y + range.w);
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

		static glm::vec2 calculateClusterMedian(const std::vector<glm::vec2>& cluster) {
			if (cluster.empty()) {
				throw std::runtime_error("Cluster is empty");
			}

			std::vector<float> xValues, yValues;
			for (const auto& point : cluster) {
				xValues.push_back(point.x);
				yValues.push_back(point.y);
			}

			std::sort(xValues.begin(), xValues.end());
			std::sort(yValues.begin(), yValues.end());

			size_t midIndex = xValues.size() / 2;
			float medianX = xValues[midIndex];
			float medianY = yValues[midIndex];

			return glm::vec2(medianX, medianY);
		}


		static std::vector<std::vector<glm::vec2>> clusterPointsWithRange(const std::vector<glm::vec2>& points,const glm::vec4& xywh, float radius)
		{
			std::vector<bool> visited(points.size(), false);
			std::vector<std::vector<glm::vec2>> clusters;

			for (size_t i = 0; i < points.size(); ++i)
			{

				if (visited[i] || !isPointInRange(points[i], glm::vec4(ofGetWidth() * (xywh.x), 0.0, xywh.z * ofGetWidth(), ofGetHeight()))) {
					continue;
				}
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


		static glm::vec2 projectPoint(const cv::Mat& homography, const glm::vec2& point) {

				const cv::Mat originalPos = (cv::Mat_<double>(3, 1) << (double)point.x, (double)point.y, 1.0);
				const cv::Mat projectedPoint = homography * originalPos;
				const glm::vec2 translatedPoint(projectedPoint.at<double>(0), projectedPoint.at<double>(1));
			
			return translatedPoint;
		}

		static std::vector<glm::vec2> projectPoints(cv::Mat& homography ,const std::vector<glm::vec2>& points){
			
				std::vector<glm::vec2> projectedPoints;
				for (auto point :points){
					const cv::Mat originalPos = (cv::Mat_<double>(3, 1) << (double)point.x, (double)point.y, 1.0);
					const cv::Mat projectedPoint = homography * originalPos;
					const glm::vec2 translatedPoint(projectedPoint.at<double>(0), projectedPoint.at<double>(1));
					projectedPoints.push_back(translatedPoint);
				}
				return projectedPoints;
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


		// クラスタの平均点をマージする関数
		static const std::vector<glm::vec2> mergeClusters(const std::vector<glm::vec2>& clusterMeans, float threshold) {
			std::vector<glm::vec2> mergedClusters;
			std::vector<bool> merged(clusterMeans.size(), false);

			for (size_t i = 0; i < clusterMeans.size(); ++i) {
				if (merged[i]) continue;

				glm::vec2 sum = clusterMeans[i];
				int count = 1;
				merged[i] = true;

				for (size_t j = i + 1; j < clusterMeans.size(); ++j) {
					if (!merged[j] && calculateDistance(clusterMeans[i], clusterMeans[j]) <= threshold) {
						sum += clusterMeans[j];
						count++;
						merged[j] = true;
					}
				}

				mergedClusters.push_back(sum / static_cast<float>(count));
			}

			return mergedClusters;
		}

		static std::vector<glm::vec2> mergeClustersWithMedian(const std::vector<std::vector<glm::vec2>>& clusters, float threshold, size_t minClusterSize) {
			std::vector<glm::vec2> mergedClusters;
			std::vector<bool> merged(clusters.size(), false);

			for (size_t i = 0; i < clusters.size(); ++i) {
				if (merged[i] || clusters[i].size() < minClusterSize || clusters[i].empty()) continue;

				glm::vec2 median = calculateClusterMedian(clusters[i]);
				std::vector<glm::vec2> toMerge = { median };
				merged[i] = true;

				for (size_t j = i + 1; j < clusters.size(); ++j) {
					if (!merged[j] && clusters[j].size() >= minClusterSize && !clusters[j].empty()) {
						glm::vec2 otherMedian = calculateClusterMedian(clusters[j]);
						if (calculateDistance(median, otherMedian) <= threshold) {
							toMerge.push_back(otherMedian);
							merged[j] = true;
						}
					}
				}

				// 新しいクラスタの代表点として、中央値の中央値を計算
				mergedClusters.push_back(calculateClusterMedian(toMerge));
			}

			return mergedClusters;
		}





		// 特定の点が除外対象の領域内にあるかをチェックする関数
		static bool isPointInExclusionZones(const glm::vec2& point, const std::vector<glm::vec2>& exclusionCenters, float exclusionRadius) {
			for (const auto& center : exclusionCenters) {
				const float dist = calculateDistance(point, center);
				if (dist <= exclusionRadius) {
					return true;
				}
			}
			return false;
		}

		// 中央値を用いたクラスタの代表点を計算する関数
		static glm::vec2 calculateClusterMedian(const std::vector<glm::vec2>& cluster, const std::vector<glm::vec2>& exclusionCenters, float exclusionRadius) {
			std::vector<float> xValues, yValues;

			// クラスタ内の各点に対して、除外領域に含まれる点を無視
			for (const auto& point : cluster) {
				if (!isPointInExclusionZones(point, exclusionCenters, exclusionRadius)) {
					xValues.push_back(point.x);
					yValues.push_back(point.y);
				}
			}

			if (xValues.empty() || yValues.empty()) {
				throw std::runtime_error("No valid points in cluster to calculate median");
			}

			std::sort(xValues.begin(), xValues.end());
			std::sort(yValues.begin(), yValues.end());

			size_t midIndex = xValues.size() / 2;
			float medianX = xValues[midIndex];
			float medianY = yValues[midIndex];

			return glm::vec2(medianX, medianY);
		}

		// 最近傍検索を用いた点のグループ化
		static std::vector<std::vector<glm::vec2>> clusterPointss(const std::vector<glm::vec2>& points, float radius, const std::vector<glm::vec2>& exclusionCenters, float exclusionRadius) {
			std::vector<bool> visited(points.size(), false);
			std::vector<std::vector<glm::vec2>> clusters;

			for (size_t i = 0; i < points.size(); ++i) {
				if (visited[i] || isPointInExclusionZones(points[i], exclusionCenters, exclusionRadius)) continue;

				std::vector<glm::vec2> cluster;
				std::vector<size_t> toVisit;
				toVisit.push_back(i);

				while (!toVisit.empty()) {
					size_t index = toVisit.back();
					toVisit.pop_back();
					if (visited[index] || isPointInExclusionZones(points[index], exclusionCenters, exclusionRadius)) continue;

					visited[index] = true;
					cluster.push_back(points[index]);

					// 半径内の近傍点を探索
					for (size_t j = 0; j < points.size(); ++j) {
						if (!visited[j] && !isPointInExclusionZones(points[j], exclusionCenters, exclusionRadius) && calculateDistance(points[index], points[j]) <= radius) {
							toVisit.push_back(j);
						}
					}
				}

				clusters.push_back(cluster);
			}

			return clusters;
		}

		// クラスタの中央値をマージする関数（最小サイズの条件付き）
		static std::vector<glm::vec2> mergeClustersWithMedian(const std::vector<std::vector<glm::vec2>>& clusters, float threshold, size_t minClusterSize, const std::vector<glm::vec2>& exclusionCenters, float exclusionRadius) {
			std::vector<glm::vec2> mergedClusters;
			std::vector<bool> merged(clusters.size(), false);

			for (size_t i = 0; i < clusters.size(); ++i) {
				if (merged[i] || clusters[i].size() < minClusterSize || clusters[i].empty()) continue;

				glm::vec2 median = calculateClusterMedian(clusters[i], exclusionCenters, exclusionRadius);
				std::vector<glm::vec2> toMerge = { median };
				merged[i] = true;

				for (size_t j = i + 1; j < clusters.size(); ++j) {
					if (!merged[j] && clusters[j].size() >= minClusterSize && !clusters[j].empty()) {
						glm::vec2 otherMedian = calculateClusterMedian(clusters[j], exclusionCenters, exclusionRadius);
						if (calculateDistance(median, otherMedian) <= threshold) {
							toMerge.push_back(otherMedian);
							merged[j] = true;
						}
					}
				}

				// 新しいクラスタの代表点として、中央値の中央値を計算
				mergedClusters.push_back(calculateClusterMedian(toMerge, {}, 0.0f)); // 除外領域はなしで計算
			}

			return mergedClusters;
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

		// クラスタをID付けして返す関数
		static std::vector<std::pair<int, glm::vec2>> assignClusterIDs(std::vector<glm::vec2>& clusterMedians) {
			// X座標でソート
			std::sort(clusterMedians.begin(), clusterMedians.end(), [](const glm::vec2& a, const glm::vec2& b) {
				return a.x < b.x;
				});

			// IDを付与
			std::vector<std::pair<int, glm::vec2>> clustersWithIDs;
			for (size_t i = 0; i < clusterMedians.size(); ++i) {
				clustersWithIDs.push_back({ static_cast<int>(i), clusterMedians[i] });
			}

			return clustersWithIDs;
		}

		// クラスタの代表点をX座標でソートして返す関数
		static std::vector<glm::vec2> sortClustersByX(const std::vector<glm::vec2>& clusterMedians) {
			std::vector<glm::vec2> sortedClusters = clusterMedians;

			// X座標でソート
			std::sort(sortedClusters.begin(), sortedClusters.end(), [](const glm::vec2& a, const glm::vec2& b) {
				return a.x < b.x;
				});

			return sortedClusters;
		}
	};
};