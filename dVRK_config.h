#include "Eigen\Dense"
#include <cmath>
using namespace Eigen;
// MTM parameters
const float l_arm = 0.2794F;
const float l_forearm1 = 0.3048F;
const float l_forearm2 = 0.0597F;
const float l_forearm = l_forearm1 + l_forearm2;
const float h = 0.1506F;
#define MTM_frame_num 7
#define PSM_frame_num 7
// PSM parameters
const float l_RCC = 0.4318F;
const float l_tool = 0.4152F;
const float l_Pitch2Yaw = 0.0091F;
const float l_Yaw2CtrlPnt = 0.0102F;

MatrixXf dVRK_MTMinit()
{
	MatrixXf MTM_DH(7, 5);
//	Joint MTM_jt = revolute;
	MTM_DH.row(0) << 1, 0, 0, 0, -pi / 2;
	MTM_DH.row(1) << 1, pi / 2, 0, 0, -pi / 2;
	MTM_DH.row(2) << 1, 0, l_arm, 0, pi / 2;
	MTM_DH.row(3) << 1, -pi / 2, l_forearm, h, 0;
	MTM_DH.row(4) << 1, pi / 2, 0, 0, 0;
	MTM_DH.row(5) << 1, -pi / 2, 0, 0, -pi / 2;
	MTM_DH.row(6) << 1,  pi / 2,  0,  0,  pi / 2;

	return MTM_DH;
}

MatrixXf dVRK_PSMinit()
{
	MatrixXf PSM_DH(7,5);
	PSM_DH.row(0) << 1, pi / 2, 0, 0, pi / 2;
	PSM_DH.row(1) << 1, -pi / 2, 0, 0, -pi / 2;
	PSM_DH.row(2) << 2, pi / 2, 0 - l_RCC, 0;
	PSM_DH.row(3) << 1, 0, 0, l_tool, 0;
	PSM_DH.row(4) << 1, -pi / 2, 0, 0, -pi / 2;
	PSM_DH.row(5) << 1, -pi / 2, l_Pitch2Yaw, 0, -pi / 2;
	PSM_DH.row(6) << 1, -pi / 2, 0, l_Yaw2CtrlPnt, pi / 2;

 	return PSM_DH;
}

DH MTM_Model(MatrixXf mtm_q)
{
	DH MTM(PSM_frame_num);
	if (!(mtm_q.rows() == 7 && mtm_q.cols() == 1))
	{
		cerr << "Invalid MTM joint position input!\n";
		return  MTM;
	}

	MatrixXf MTM_DH = dVRK_MTMinit();
	int row = 0, j_type = 1;
	for ( ; row < MTM_DH.rows(); row++);
	{
		j_type = (int)MTM_DH(row, 1);
		if (j_type == 1)
			MTM_DH(row, 5) += mtm_q(row, 1);
		else
			MTM_DH(row, 4) += mtm_q(row, 1);
	}
	MTM.DH_set(MTM_DH);
	return MTM;
}

MatrixXf MTM_FK(DH MTM)
{
	Matrix4f T = MatrixXf::Identity(4,4),
			 Ti = MatrixXf::Identity(4,4);
	MatrixXf MTM_DH = MTM.DH_get();
	int frame_num = MTM_DH.rows(), row = 0;
	float alp, a, d, the;

	for (; row < frame_num; ++row)
	{
		alp = MTM_DH(row, 2);
		a   = MTM_DH(row, 3);
		d   = MTM_DH(row, 4);
		the = MTM_DH(row, 5);
		Ti.row(0) << cos(the),  -sin(the),  0,  a;
		Ti.row(1) << sin(the)*cos(alp), cos(the)*cos(alp), -sin(alp),  -sin(alp)*d;
		Ti.row(2) << sin(the)*sin(alp), cos(the)*sin(alp), cos(alp), cos(alp)*d;
		Ti.row(3) << 0, 0, 0, 1;
		
		T *= Ti;
	}
	return T;
}

DH PSM_Model(MatrixXf psm_q)
{
	DH PSM(MTM_frame_num);
	if (!(psm_q.rows() == 6 && psm_q.cols() == 1))     // Joint 7 of PSM is fixed as 0. psm_q only has 6 joint angles.
	{
		cerr << "Invalid PSM joint position input!\n";
		return  PSM;
	}

	MatrixXf PSM_DH = dVRK_PSMinit();
	int row = 0, j_type = 1;
	for (; row < PSM_DH.rows() - 1; row++);		
	{
		j_type = (int)PSM_DH(row, 0);
		if (j_type == 1)
			PSM_DH(row, 5) += psm_q(row, 1);
		else
			PSM_DH(row, 4) += psm_q(row, 1);
	}
	PSM.DH_set(PSM_DH);
	return PSM;
}

Kinematics_info PSM_FK(DH PSM)
{
	int frame_num = PSM.DH_row(), row = 0, j_type = 1;
	Kinematics_info psm_fwadk_info(frame_num);
	Matrix4f T = MatrixXf::Identity(4, 4),
			 Ti = MatrixXf::Identity(4, 4);
	MatrixXf J = MatrixXf::Zero(6, frame_num - 1),
			 Z = MatrixXf::Zero(3, frame_num),
			 P = MatrixXf::Zero(3, frame_num);
	MatrixXf PSM_DH = PSM.DH_get();

	float alp, a, d, the;

	for (; row < frame_num; ++row)
	{
		alp = PSM_DH(row, 2);
		a = PSM_DH(row, 3);
		d = PSM_DH(row, 4);
		the = PSM_DH(row, 5);
		Ti.row(0) << cos(the), -sin(the), 0, a;
		Ti.row(1) << sin(the)*cos(alp), cos(the)*cos(alp), -sin(alp), -sin(alp)*d;
		Ti.row(2) << sin(the)*sin(alp), cos(the)*sin(alp), cos(alp), cos(alp)*d;
		Ti.row(3) << 0, 0, 0, 1;

		T *= Ti;
		Z.block(0, row, 3, 1) = T.block(0, 2, 3, 1);  // T.block<3, 1>(0, 2) The former for block syntax is more robust.
		P.block(0, row, 3, 1) = T.block(0, 3, 3, 1);  // As it latter doesn't allow variable in argument.
	}                                                 // Z.col(row) = T.block(0, 2, 3, 1) will causes error
	                                                  // since the complier doesn't know the dimension matches or not.
	                                                  // thus, just use block on both sides.
	Vector3f z, pd;
	for (row = 0; row < frame_num; row++)
	{
		j_type = int(PSM_DH(row, 0));
		if (j_type == 1)
		{
			z.block(0, 0, 3, 1) = Z.col(row);		 // When cos() is on the right side, bolck and col work well.
			pd.block(0, 0, 3, 1) = P.col(frame_num - 1) - P.col(row);
			J.block(0, row, 3, 1) = z.cross(pd);
			J.block(3, row, 3, 1) = Z.col(row);
		}
		else
			J.block(0, row, 3, 1) = Z.col(row);
	}

	psm_fwadk_info.tip_set(T);
	psm_fwadk_info.origin_pos_set(P);
	psm_fwadk_info.J_set(J);

	return psm_fwadk_info;
}