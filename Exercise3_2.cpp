#include <iostream>
#include <string>
#include <vector>
#include <fstream> 
#include <algorithm>
using namespace std;

class LessThan
{
	public:
		bool operator()(const string &s1, string &s2)
		{
			return s1.size() < s2.size();
		}
};

template <typename elemType>
void display_vector(const vector<elemType> &vec, ostream &os=cout, int len = 8);

int main()
{
	ifstream ifile("Exercise3_1_in.txt");
	ofstream ofile("Exercise3_1_soeted.txt");
	
	if (!ifile || !ofile)
	{
		cerr << "Unable to open file -- bailing out! \n";
		return -1;
	}
	
	vector<string> text;
	string word;
	while (ifile >> word)
		text.push_back(word);
	
	sort(text.begin(), text.end(), LessThan());
	display_vector(text, ofile);
		
}

template <typename elemType>
void display_vector(const vector<elemType> &vec, ostream &os=cout, int len = 8)
{
	typename vector<elemType>::const_iterator iter = vec.begin(),  // Very important to add 
									          end_it = vec.end();  // typename in the beginning
	int elem_cnt = 1;                                              // different form Stanley's book.
	while (iter != end_it)
		os << *iter++
			<< (!(elem_cnt++ % len) ? '\n' : ' ');          // Tricky output 8 words every line.
	os << endl;												 
}
