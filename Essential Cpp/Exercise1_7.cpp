#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <vector>
#include <direct.h>

#define MAX_PATH 256
using namespace std;

int main()
{
	/*char buffer1[MAX_PATH], buffer2[MAX_PATH];
	const char ifname[] = "\\Exercise1_7text.txt";   // 注意单斜杠的转义 
	const char ofname[] = "\\Exercise1_7textsort.txt";
	getcwd(buffer1, MAX_PATH);
	getcwd(buffer2, MAX_PATH);
	
	strcat(buffer1, ifname);
	cout << buffer1 << endl;
    //string str1 = buffer1; 
	//cout << str1; */
	
	ifstream in_file("Exercise1_7text.txt");
	
	if (! in_file)
	{
		cerr << "opps! unable to open input file\n";
		return -1;
	}
	
	ofstream out_file("Exercise1_7textsort.txt");
	if (! out_file)
	{
		cerr << "opps! unable to open output file\n";
		return -2;
	}
	
	string word;
	vector<string> text;
	while (in_file >> word)
		text.push_back(word);
	
	int ix;
	cout << "unsorted text: \n";
	for (ix = 0; ix < text.size(); ++ix)
		cout << text[ix] << ' ';
	cout << endl;
	
	sort(text.begin(), text.end());
	
	out_file << "sorted text: \n";
	for (ix = 0; ix < text.size(); ++ix)
		out_file << text[ix] << ' ';
	out_file << endl;
	
	return 0;
}
