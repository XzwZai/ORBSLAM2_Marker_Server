#include<Tools.h>
#include<Marker.h>
namespace ORB_SLAM2
{

	void Marker::draw(cv::Mat& dstImg, cv::Scalar color = cv::Scalar(255, 255, 0))
	{
		float thickness = 1;
		cv::line(dstImg, points[0], points[1], color, thickness, CV_AA);
		cv::line(dstImg, points[1], points[2], color, thickness, CV_AA);
		cv::line(dstImg, points[2], points[3], color, thickness, CV_AA);
		cv::line(dstImg, points[3], points[0], color, thickness, CV_AA);
	}

	void Marker::SetSize(float size)
	{
		m_markerCorners3d.clear();
		m_markerCorners3d.push_back(cv::Point3f(-size, -size, 0));
		m_markerCorners3d.push_back(cv::Point3f(+size, -size, 0));
		m_markerCorners3d.push_back(cv::Point3f(+size, +size, 0));
		m_markerCorners3d.push_back(cv::Point3f(-size, +size, 0));
		
	}

	void Marker::estimatePose(float fx,float fy,float cx,float cy)
	{
		cv::Mat camMatrix;
		float m_intrinsic[3][3];
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				m_intrinsic[i][j] = 0;
		m_intrinsic[0][0] = fx;
		m_intrinsic[1][1] = fy;
		m_intrinsic[0][2] = cx;
		m_intrinsic[1][2] = cy;
		m_intrinsic[2][2] = 1;
		cv::Mat(3, 3, CV_32F, const_cast<float*>(&m_intrinsic[0][0])).copyTo(camMatrix);

		cv::Mat Rvec;
		cv::Mat_<float> Tvec;
		cv::Mat raux, taux;
		cv::solvePnP(m_markerCorners3d, pointsUn, camMatrix, cv::Mat(), raux, taux);
		raux.convertTo(Rvec, CV_32F);
		taux.convertTo(Tvec, CV_32F);

		cv::Mat_<float> _Rvec;
		raux.convertTo(_Rvec, CV_32F);


		cv::Mat_<float> rotMat(3, 3);
		cv::Rodrigues(Rvec, rotMat);

		cv::Mat Tmc = cv::Mat::eye(4, 4, CV_32F);
		// Copy to transformation matrix
		for (int col = 0; col < 3; col++)
		{
			for (int row = 0; row < 3; row++)
			{
				Tmc.at<float>(row, col) = rotMat(row, col);
				transformation.r().mat[row][col] = rotMat(row, col); // Copy rotation component
			}			
			transformation.t().data[col] = Tvec(col); // Copy translation component
		}
		for (int i = 0; i < 3; i++)
		{
			Tmc.at<float>(i, 3) = Tvec(i);
		}
		mTmc = Tmc.inv();
		//Tmw.copyTo(mTmw);
		// Since solvePnP finds camera location, w.r.t to marker pose, to get marker pose w.r.t to the camera we invert it.
		//m.transformation = m.transformation.getInverted();
		
	}

}