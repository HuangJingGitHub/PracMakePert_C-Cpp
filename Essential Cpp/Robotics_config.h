#include <iostream>
#include "Eigen\Dense"

using namespace std;
using namespace Eigen;

enum JointType {revolute = 1, prismatic};
typedef JointType Joint;

class DH {
public:
	DH(int frame_num);

	bool DH_set(int row, int col, float val);
	void DH_read() const;
	ostream& operator<<(ostream& os) const;

private:
	int _frame_num;
	MatrixXf _DH;
};

DH::DH(int frame_num)
{
	_frame_num = frame_num;
	MatrixXf temp_mat(frame_num, 5);
	_DH = temp_mat;
}

bool DH::DH_set(int row, int col, float val)
{
	if (row < 0 || row > _frame_num - 1 || col < 0 || col > 4)
	{
		cerr << "Invalid index used for DH table !!!\n" << endl;
		return false;
	}
	else
		_DH(row, col) = val;
	return true;
}

void DH::DH_read() const
{
	cout << _DH << endl;
}

ostream& DH::operator<<(ostream& os) const
{
	os << _DH << endl;
}