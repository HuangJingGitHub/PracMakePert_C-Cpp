#include <iostream>
#include "Eigen/Dense"
#include "Robotics_config.h"
#include "dVRK_config.h"
using namespace std;
using namespace Eigen;


/*int main()
{
	enum test { t1, t2 };
	test T1 = t1;
	MatrixXd m(2, 2);
	m(0, 0) = T1;
	m(0, 1) = 2;
	m(1, 0) = 3;
	m(1, 1) = 4;
	m.row(1) << 12, 34;
	cout << m << endl;
	cout << "First test on Eigen library finished!\n";

	float temp1 = (float)9;
	int temp2 = 5;
	cout << "temp2 = " << temp2  << pi << endl;

	MatrixXf temp3 = dVRK_MTMinit();
	cout << temp3 << endl;

	Matrix4f temp4, temp5, temp6;
	temp6 = temp4 * temp6;
	temp5 = MatrixXf::Zero(4, 4);
	temp4.row(0) << 1, 3, 5, 7;
	temp4.row(1) << 3, 5, 5, 6;
	Matrix4f T = MatrixXf::Identity(4, 4);
	cout << temp5 << endl << T << endl << temp4 << endl, cout << "hj";

	//MatrixXf temp7(3,1), temp8(3,1);
	//temp7.cross(temp8);


}