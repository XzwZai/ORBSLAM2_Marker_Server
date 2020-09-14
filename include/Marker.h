#ifndef MARKER_H
#define MARKER_H


#include<Tools.h>

namespace ORB_SLAM2
{

	class Marker
	{
	public:
		int id;
		cv::Mat mTmc;
		std::vector<cv::Point2f> points;
		std::vector<cv::Point2f> pointsUn;
		Transformation transformation;
		bool needEstimate = true;
		std::vector<cv::Point3f> m_markerCorners3d;

		Marker() : id(-1) {}

		void SetSize(float size);

		void estimatePose(float fx, float fy, float cx, float cy);

		void draw(cv::Mat& dstImg, cv::Scalar color);

		static cv::Mat rotate(cv::Mat in)
		{
			cv::Mat out;
			in.copyTo(out);
			for (int i = 0; i < in.rows; i++)
			{
				for (int j = 0; j < in.cols; j++)
				{
					out.at<uchar>(i, j) = in.at<uchar>(in.cols - j - 1, i);
				}
			}
			return out;
		}

		static int mat2id(const cv::Mat &bits)
		{
			int val = 0;
			for (int y = 0; y < 5; y++)
			{
				val <<= 1;
				if (bits.at<uchar>(y, 1)) val |= 1;
				val <<= 1;
				if (bits.at<uchar>(y, 3)) val |= 1;
			}
			return val;
		}

		static int hammDistMarker(cv::Mat bits)
		{
			int ids[4][5] =
			{
			  {1,0,0,0,0},
			  {1,0,1,1,1},
			  {0,1,0,0,1},
			  {0,1,1,1,0}
			};

			int dist = 0;

			for (int y = 0; y < 5; y++)
			{
				int minSum = 1e5; //hamming distance to each possible word

				for (int p = 0; p < 4; p++)
				{
					int sum = 0;
					//now, count
					for (int x = 0; x < 5; x++)
					{
						sum += bits.at<uchar>(y, x) == ids[p][x] ? 0 : 1;
					}

					if (minSum > sum)
						minSum = sum;
				}

				//do the and
				dist += minSum;
			}

			return dist;
		}

		static int getMarkerId(cv::Mat &in, int &nRotations)
		{
			assert(in.rows == in.cols);
			assert(in.type() == CV_8UC1);
			cv::Mat grey = in;
			cv::threshold(grey, grey, 125, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
			int cellSize = grey.rows / 7;
			cv::Mat allBits = cv::Mat::zeros(cv::Size(7, 7), CV_8UC1);
			for (int y = 0; y < 7; y++)
			{
				for (int x = 0; x < 7; x++)
				{
					int xOff = x * cellSize;
					int yOff = y * cellSize;
					cv::Mat cell = grey(cv::Rect(xOff, yOff, cellSize, cellSize));
					int nonZero = cv::countNonZero(cell);
					allBits.at<uchar>(y, x) = nonZero > cellSize * cellSize / 2 ? 1 : 0;
				}
			}
			cv::Mat bits = allBits(cv::Rect(1, 1, 5, 5));
			int allNonZero = cv::countNonZero(allBits);
			int nonZero = cv::countNonZero(bits);
			if (allNonZero != nonZero) return -1;
			cv::Mat rotations[4];
			int distances[4];

			rotations[0] = bits;
			distances[0] = hammDistMarker(rotations[0]);

			std::pair<int, int> minDist(distances[0], 0);

			for (int i = 1; i < 4; i++)
			{
				//get the hamming distance to the nearest possible word
				rotations[i] = rotate(rotations[i - 1]);
				distances[i] = hammDistMarker(rotations[i]);

				if (distances[i] < minDist.first)
				{
					minDist.first = distances[i];
					minDist.second = i;
				}
			}

			nRotations = minDist.second;
			if (minDist.first == 0)
			{
				//std::cout << mat2id(rotations[minDist.second]) << std::endl;
				return mat2id(rotations[minDist.second]);
			}
			return -1;

		}
	};

}

#endif