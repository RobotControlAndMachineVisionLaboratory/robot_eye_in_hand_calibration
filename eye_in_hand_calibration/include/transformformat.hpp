#ifndef _TRANSFORMFORMAT_H_
#define _TRANSFORMFORMAT_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

Eigen::AngleAxisd Vector3d2AngleAxisd(Eigen::Vector3d vec)
{
	Eigen::AngleAxisd rotation_vector(vec.norm(), vec.normalized());
	return rotation_vector;
}

Eigen::Matrix4d Vector3ds2Matrix4d_RPY(Eigen::Vector3d trans_vector, double translation_scale, Eigen::Vector3d RPY_vector, double rotation_scale)
{
	Eigen::Translation3d translation_vector(trans_vector * translation_scale);

	Eigen::Vector3d current_rpy = RPY_vector * rotation_scale;
	Eigen::AngleAxisd rotation_vector;
	rotation_vector =
	Eigen::AngleAxisd(current_rpy[2], Eigen::Vector3d::UnitZ()) *
	Eigen::AngleAxisd(current_rpy[1], Eigen::Vector3d::UnitY()) *
	Eigen::AngleAxisd(current_rpy[0], Eigen::Vector3d::UnitX());

	Eigen::Matrix4d res = (translation_vector * rotation_vector).matrix ();
	return res;
}

Eigen::Matrix4d Vector3ds2Matrix4d_AngleAxis(Eigen::Vector3d trans_vector, double translation_scale, Eigen::Vector3d rot_vector, double rotation_scale)
{
	Eigen::Translation3d translation_vector(trans_vector * translation_scale);

	rot_vector = rot_vector * rotation_scale;

	Eigen::AngleAxisd rotation_vector;
	rotation_vector = Eigen::AngleAxisd(rot_vector.norm(), rot_vector.normalized());

	Eigen::Matrix4d res = (translation_vector * rotation_vector).matrix ();
	return res;
}

Eigen::Matrix4f create_affine_matrixf(float a, float b, float c, Eigen::Vector3f trans)
{
	Eigen::Transform<float, 3, Eigen::Affine> t;
	t = Translation<float, 3>(trans);
	t.rotate(Eigen::AngleAxis<float>(a, Eigen::Vector3f::UnitX()));
	t.rotate(Eigen::AngleAxis<float>(b, Eigen::Vector3f::UnitY()));
	t.rotate(Eigen::AngleAxis<float>(c, Eigen::Vector3f::UnitZ()));
	return t.matrix();
}

Eigen::Matrix4d combineRT(Eigen::Matrix3d R, Eigen::Vector3d T)
{
	// Your Transformation Matrix
	Eigen::Matrix4d Trans;
	// Set to Identity to make bottom row of Matrix 0,0,0,1
	Trans.setIdentity();
	Trans.block<3,3>(0,0) = R;
	Trans.block<3,1>(0,3) = T;
	return Trans;
}
#endif