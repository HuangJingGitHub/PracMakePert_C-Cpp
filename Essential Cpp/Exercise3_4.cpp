#include <iostream>
#include <iterator>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <sstream>
using namespace std;

class even_elem
{
	public:
		bool operator()(int elem)
		{
			return elem%2 ? false : true;
		}
 };
 
void input_process(string &s_put, vector<int> &i_put);
 
 int main()
 {
 	vector<int> input;
 	string s_input;

	getline(cin,s_input);
	//cout << s_input;
	input_process(s_input, input);
	
	vector<int>::const_iterator it = input.begin();
	for( ;it != input.end(); ++it)
	cout << *it << "***";
	
 	ofstream even_file("Exercise3_4_even.txt"),
 			 odd_file("Exercise3_4_odd.txt");
 	if (!even_file || !odd_file)
 	{
 		cerr << "Sorry, unable to open the files. Bailing out!\n";
 		return -1;
	 }
	
	vector<int>::iterator division = partition(input.begin(), 
											   input.end(), even_elem());
 	istream_iterator<int> in(cin), endf;
 	ostream_iterator<int> even_iter(even_file, "\n"),
 						  odd_iter(odd_file, " ");
 	
 	copy(input.begin(), division, even_iter);
 	copy(division, input.end(), odd_iter);
 	//copy(in, endf, back_inserter(input));
 	
 	
 }
 
void input_process(string &s_put, vector<int> &i_put)
{
	string snum;
	int inum;
	string::size_type pos = 0, prev_pos = 0,
 					  s_input_size = s_put.size();
 	stringstream ss;
 	while ((pos = s_put.find_first_of(' ', pos)) != string::npos)
 	{
 		string::size_type end_pos = pos - prev_pos;
 		snum = s_put.substr(prev_pos, end_pos);
 		//cout << "-->" << snum << endl;
 		//ss << snum;
 		//ss >> inum;
 		//cout << "<--" << inum << endl;
		i_put.push_back(stoi(snum)); 
		prev_pos = ++pos;   // Two operations completed.(++pos, =) 
	 }
	 
	 // Handle the last int
	 snum = s_put.substr(prev_pos, s_put.size());
	 //ss << snum;
	 //ss >> inum;
	 i_put.push_back(stoi(snum));
}
