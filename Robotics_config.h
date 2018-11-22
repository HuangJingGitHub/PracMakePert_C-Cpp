#include <vector>
using namespace std;

enum JointType {revolute = 1, prismatic};
typedef JointType Joint;

class DH {
public:
	DH(int frame_num);
	//void DH_set(vector<Joint>, vector<float>, vector<float>, vector<float>, vector<float>);
	//void DH_read();

private:
	int _frame_num;
	vector<Joint> _DH_type;
	vector<float> _DH_twist;
	vector<float> _DH_length;
	vector<float> _DH_offset;
	vector<float> _DH_ang;
};

DH::DH(int frame_num)
{
	_frame_num = frame_num;
	vector<Joint>::iterator type = _DH_type.begin();
	vector<float>::iterator twist = _DH_twist.begin();
	vector<float>::iterator length = _DH_length.begin();
	vector<float>::iterator offset = _DH_offset.begin();
	vector<float>::iterator ang = _DH_ang.begin();

	for (int ix = 0; ix <= frame_num; ++ix)
	{
		_DH_type.push_back(revolute);
		_DH_twist.push_back(0.0);
		_DH_length.push_back(0.0);
		_DH_offset.push_back(0.0);
		_DH_ang.push_back(0.0);
	}

}