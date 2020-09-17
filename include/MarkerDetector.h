#ifndef MARKERDETECTOR_H
#define MARKERDETECTOR_H

#include<Marker.h>
#define SPEEDMODE 1
#define ACCURATEMODE 2


namespace ORB_SLAM2
{

	class MarkerDetector
	{
	public:
		//Track
		std::vector<Marker> lastFrameMarkers;
		cv::Mat velocity;
		int frameCount = 0;
		std::vector<Marker> keyFrameMarkers;

		float lastMinMarkerArea = 2000;
		float minMarkerAreaAllowed = 2000;

		float m_minContourLengthAllowed;
		cv::Size markerSize;

		cv::Mat sourceImage;
		cv::Mat m_grayscaleImage;
		cv::Mat m_thresholdImg;
		cv::Mat canonicalMarkerImage;

		std::vector<Marker> markers;
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Point2f> m_markerCorners2d;
		std::vector<cv::Point3f> m_markerCorners3d;
		cv::Mat camMatrix;
		cv::Mat distCoeff;
		int cannyThreshold;

		cv::Mat kernel;
		int mode = ACCURATEMODE;
		MarkerDetector(float f, float dx, float dy, cv::Mat distortion);
		bool getMarkersPos2(cv::Mat& sourceImg, std::vector<Marker>& detectedMarkers);

		void myFindContours(const cv::Mat& thresholdImg, std::vector<std::vector<cv::Point>>& contours, int minPointsAllowed);
		void findCandidates(const std::vector<std::vector<cv::Point>>& contours, std::vector<Marker>& detectedMarkers);
		void recognizeMarkers(const cv::Mat& grayscale, std::vector<Marker>& detectedMarkers);
		void estimatePosition(std::vector<Marker>& detectedMarkers);
		float calPerimeter(const std::vector<cv::Point2f> &a);

		bool getMarkersPos(cv::Mat& sourceImg, std::vector<Marker>& detectedMarkers);
		void findMarkers(cv::Mat& img, std::vector<Marker>& markers, bool part);
		bool rectIn(cv::Size size, cv::Rect& rect);
	};
}


#endif