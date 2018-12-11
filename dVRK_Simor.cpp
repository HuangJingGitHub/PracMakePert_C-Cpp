#include <fstream>
#include "Robotics_config.h"
#include "dVRK_config.h"

int main()
{
	MatrixXf mtm_q_test = MatrixXf::Zero(MTM_frame_num, 1);
	cout << mtm_q_test << endl;
	DH MTM_test(7);
	DH MTM_test1 = MTM_Model(mtm_q_test);
}