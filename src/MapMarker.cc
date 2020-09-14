#include<MapMarker.h>

namespace ORB_SLAM2
{
	MapMarker::MapMarker(cv::Mat& Tmw, int markerId, float scale, KeyFrame* pRefKF, Map* pMap) :
		mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), mpRefKF(pRefKF), mpMap(pMap), mMarkerId(markerId), mScale(scale), nObs(0)
	{
		Tmw.copyTo(mTmw);
		UpdatePoseMatrices();
	}

	void MapMarker::SetPose(cv::Mat Tmw)
	{
		Tmw.copyTo(mTmw);
		UpdatePoseMatrices();
	}

	cv::Mat MapMarker::GetPose()
	{
		unique_lock<mutex> lock(mMutexFeatures);
		return mTmw;
	}

	cv::Mat MapMarker::GetPoseInverse()
	{
		unique_lock<mutex> lock(mMutexFeatures);
		return mTwm;
	}



	void MapMarker::UpdatePoseMatrices()
	{
		/*mTwm = mTmw.inv();*/
		mRmw = mTmw.rowRange(0, 3).colRange(0, 3);
		mRwm = mRmw.t();
		mtmw = mTmw.rowRange(0, 3).col(3);
		mOw = -mRmw.t()*mtmw;
		
		mTwm = cv::Mat::eye(4, 4, mTmw.type());
		mRwm.copyTo(mTwm.rowRange(0, 3).colRange(0, 3));
		mOw.copyTo(mTwm.rowRange(0, 3).col(3));

		mvLocalCornors.clear();
		mvLocalCornors.push_back((cv::Mat_<float>(3, 1) << -0.5f * mScale, -0.5f * mScale, 0));
		mvLocalCornors.push_back((cv::Mat_<float>(3, 1) << +0.5f * mScale, -0.5f * mScale, 0));
		mvLocalCornors.push_back((cv::Mat_<float>(3, 1) << +0.5f * mScale, +0.5f * mScale, 0));
		mvLocalCornors.push_back((cv::Mat_<float>(3, 1) << -0.5f * mScale, +0.5f * mScale, 0));

		mvWorldCornors.clear();
		for (int i = 0; i < 4; i++)
		{
			mvWorldCornors.push_back(mRwm * mvLocalCornors[i] + mOw);
		}
		cout << "Tmw :" << mTmw << endl;
	}

	void MapMarker::AddObservations(KeyFrame* pKF, size_t idx)
	{
		unique_lock<mutex> lock(mMutexFeatures);
		if (mObservations.count(pKF))
			return;
		mObservations[pKF] = idx;
		nObs++;
	}

	std::map<KeyFrame*, size_t> MapMarker::GetObservations()
	{
		unique_lock<mutex> lock(mMutexFeatures);
		return mObservations;
	}

	void MapMarker::SetCalCornor(std::vector<cv::Mat> vCalCornors)
	{
		mvCalCornors = vCalCornors;
	}


}