#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

struct Matrix44
{
	union
	{
		float data[16];
		float mat[4][4];
	};

	Matrix44 getTransposed() const
	{
		Matrix44 t;
		
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				t.mat[i][j] = mat[j][i];

		return t;
	}

	static Matrix44 identity()
	{
		Matrix44 eye;

		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				eye.mat[i][j] = i == j ? 1 : 0;

		return eye;
	}

	Matrix44 getInvertedRT() const
	{
		Matrix44 t = identity();

		for (int col = 0; col < 3; col++)
		{
			for (int row = 0; row < 3; row++)
			{
				// Transpose rotation component (inversion)
				t.mat[row][col] = mat[col][row];
			}

			// Inverse translation component
			t.mat[3][col] = -mat[3][col];
		}
		return t;
	}
};

struct Matrix33
{
	union
	{
		float data[9];
		float mat[3][3];
	};
	
	static Matrix33 identity()
	{
		Matrix33 eye;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				eye.mat[i][j] = i == j ? 1 : 0;
			}
		}
		return eye;
	}
	Matrix33 getTransposed() const
	{
		Matrix33 t;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				t.mat[i][j] = mat[j][i];
		return t;
	}
};

struct Vector4
{
	float data[4];
};

struct Vector3
{
	float data[3];

	static Vector3 zero()
	{
		Vector3 v;
		for (int i = 0; i < 3; i++)
		{
			v.data[i] = 0;
		}
		return v;
	}
	Vector3 operator-() const
	{
		Vector3 v;
		for (int i = 0; i < 3; i++)
		{
			v.data[i] = -data[i];
		}
		return v;
	}
};

class Transformation
{
public:
	Transformation() {}
	Transformation(Matrix33& r, Vector3& t) : m_rotation(r), m_translation(t) {}
	Matrix33& r() { return m_rotation; }
	Vector3& t() { return m_translation; }
	friend std::ostream &operator <<(std::ostream &out, Transformation &transformation)
	{		
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				out << (transformation.m_rotation.mat[i][j]);
				out << ' ';
				if (j == 2)
				{
					out << transformation.m_translation.data[i] << '\n';
				}				
			}
		}
		return out;
	}
	Matrix44 getMat44() const
	{
		Matrix44 res = Matrix44::identity();

		for (int col = 0; col < 3; col++)
		{
			for (int row = 0; row < 3; row++)
			{
				// Copy rotation component
				res.mat[row][col] = m_rotation.mat[row][col];
			}

			// Copy translation component
			res.mat[3][col] = m_translation.data[col];
		}
		return res;
	}

	cv::Mat getMat()
	{
		cv::Mat t = cv::Mat::eye(4, 4, CV_32F);
		Matrix44 m44 = getMat44();
		for (int i = 0; i < 4; i++)
		{
			float* data = t.ptr<float>(i);
			for (int j = 0; j < 4; j++)
			{
				data[j] = m44.mat[i][j];
			}
		}
		return t;
	}

	Transformation getInverted() const
	{
		Vector3 t = -m_translation;
		Matrix33 r = m_rotation.getTransposed();
		return Transformation(r, t);		
	}
private:
	Matrix33 m_rotation;
	Vector3 m_translation;
};