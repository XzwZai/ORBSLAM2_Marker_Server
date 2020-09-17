#ifndef MAPMARKER_H
#define MAPMARKER_H

#include<KeyFrame.h>
#include<Frame.h>
#include<Map.h>
#include<vector>
#include<mutex>
namespace ORB_SLAM2
{
	class KeyFrame;
	class Frame;
	class Map;

	class MapMarker
	{
	public:
		MapMarker(cv::Mat &Tmw, int markerId, float scale, KeyFrame* pRefKF, Map* pMap);

		KeyFrame* mpRefKF;
		const long int mnFirstKFid;
		const long int mnFirstFrame;
		int mMarkerId;
		int nObs;
		float mScale;
		std::vector<cv::Mat> mvLocalCornors;
		std::vector<cv::Mat> mvWorldCornors;
		std::vector<cv::Mat> mvCalCornors;

		long unsigned int mnBALocalForKF;

		void AddObservations(KeyFrame* pKF, size_t idx);
		std::map<KeyFrame*, size_t> GetObservations();
		void SetCalCornor(std::vector<cv::Mat> vCalCornors);
		
		cv::Mat GetPose();
		cv::Mat GetPoseInverse();

		void SetPose(cv::Mat Tmw);

	protected:
		cv::Mat mTmw;
		cv::Mat mTwm;
		cv::Mat mOw;
		cv::Mat mRmw;
		cv::Mat mRwm;
		cv::Mat mtmw;
		
		std::map <KeyFrame*, size_t> mObservations;

		Map* mpMap;
		std::mutex mMutexFeatures;

		void UpdatePoseMatrices();
	};
}




#endif // ! MAPMARKER_H
