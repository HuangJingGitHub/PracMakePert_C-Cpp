#include "Eigen\Dense"
using namespace Eigen;
// MTM model
const float l_arm = 0.2794;
const float l_forearm1 = 0.3048;
const float l_forearm2 = 0.0597;
const float l_forearm = l_forearm1 + l_forearm2;
const float h = 0.1506;

void dVRK_init()
{
	static MatrixXf MTM_DH(7, 5);
	MTM_DH.row(0) << 1, 0, 0, 0, -pi / 2,
	MTM_DH.row(1) << 1, pi / 2, 0, 0, -pi / 2,
	MTM_DH.row(2) << 1, 0, l_arm, 0, pi / 2,
	MTM_DH.row(3) << 1, -pi / 2, l_forearm, h, 0,
	MTM_DH.row(4) << 1, pi / 2, 0, 0, 0,
	MTM_DH.row(5) << 1, -pi / 2, 0, 0, -pi / 2,
	MTM_DH.row(6) << 1, pi / 2, 0, 0, pi / 2;
}
