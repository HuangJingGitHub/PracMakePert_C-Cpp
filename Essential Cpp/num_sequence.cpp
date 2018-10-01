#include <iostream>

using namespace std;

bool num_sequence::check_integrity(int pos) const
{
	if (pos <= 0 || pos > _max_elems)
	{
		cerr << "!! Invalid position: " << pos << " Cannot honor request\n";
		return false;
	}
	return true;
}

ostream& operator<<(ostream &os, const num_sequence &ns)
{
	return ns.print(os);
}
