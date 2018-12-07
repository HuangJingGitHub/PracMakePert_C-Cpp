#include <iostream>
#include "Eigen\Dense"

using namespace std;
using namespace Eigen;

const float pi = 3.141592F;
enum JointType {revolute = 1, prismatic};
typedef JointType Joint;

class DH {
public:
	DH(int frame_num);
	DH(MatrixXf input_DH)
	{
		if (input_DH.cols() != 5)
			cerr << "Not valid DH table size inputed! NOTE: DH table need to have 5 cols.";
		else
		{
			_frame_num = input_DH.rows();
			_DH = input_DH;
		}
	}
	bool DH_set(int row, int col, float val);
	bool DH_set(MatrixXf DH_table);
	void DH_read() const;
	MatrixXf DH_get() const;
	MatrixXi DH_size() const;
	int DH_row() const
	{
		return _frame_num;
	}
	ostream& operator<<(ostream& os) const;

private:
	int _frame_num;
	MatrixXf _DH;
};

DH::DH(int frame_num)
{
	_frame_num = frame_num;
	_DH = MatrixXf::Zero(frame_num, 5);
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

bool DH::DH_set(MatrixXf DH_table)
{
	_DH = DH_table;
	return true;
}

void DH::DH_read() const
{
	cout << _DH << endl;
}

MatrixXf DH::DH_get() const
{
	return _DH;
}
MatrixXi DH::DH_size() const
{
	MatrixXi DH_size(2, 1);
	DH_size(1, 1) = _frame_num;
	DH_size(2, 1) = 5;
	return DH_size;
}

ostream& DH::operator<<(ostream& os) const
{
	os << _DH << endl;
	return os;
}


//*** Defination of class Kinematics_info ***
class Kinematics_info
{
public:
	Kinematics_info(int frame_num)
	{
		_frame_num = frame_num;
		_T_tip = MatrixXf::Zero(4, 4);
		_T_tip(3, 3) = 1;
		_origin_pos = MatrixXf::Zero(3, frame_num);
		_Jacobian = MatrixXf::Zero(6, frame_num - 1);
	}
	MatrixXf tip_get() const
	{	return _T_tip;	}
    void  tip_set(Matrix4f tip) 
	{  _T_tip = tip;}

	MatrixXf origin_pos_get() const
	{	return _origin_pos;	}
	void origin_pos_set(MatrixXf pos)
	{	_origin_pos = pos;	}

	MatrixXf J_get() const
	{	return _Jacobian; }
	void J_set(MatrixXf j)
	{	_Jacobian = j;	}
private:
	int _frame_num;
	Matrix4f _T_tip;
	MatrixXf _origin_pos;
	MatrixXf _Jacobian;
};