#include <iostream>
#include <vector>
#include <string>
#include <map> 
#include <fstream>
using namespace std;

typedef vector<string> vstring;

void populate_map(ifstream &nameFile, map<string,vstring> &families);
void query_map(const string &family, const map<string,vstring> &families);
void display_map(const map<string, vstring> &families, ostream &os);

int main()
{
	map<string, vstring> families;
	ifstream nameFile("Exercise3_3.txt");
	
	if (!nameFile)
	{
		cerr << "Unable to find the txt file. Bailing out! \n";
		return -1;
	}
	
	populate_map(nameFile, families);
	
	string family_name;
	while(1)
	{
		cout << "Please enter a family name or q to quit: \n";
		cin >> family_name;
		if (family_name == "q")
			break;
		query_map(family_name, families);
	}
	display_map(families, cout);
}

void populate_map(ifstream &nameFile, map<string, vstring> &families)
{
	string textline;
	while (getline(nameFile, textline))
	{
		string fam_name;
		vector<string> child;
		string::size_type pos = 0, prev_pos = 0, text_size = textline.size();
		
		while ((pos = textline.find_first_of(' ', pos)) != string::npos)
		{
			string::size_type end_pos = pos - prev_pos;
			if (!prev_pos)
				fam_name = textline.substr(prev_pos, end_pos);
			else
				child.push_back(textline.substr(prev_pos, end_pos));
			prev_pos = ++pos;
		}
	
	// Now handle the last child
	if (prev_pos < text_size)
		child.push_back(textline.substr(prev_pos, pos-prev_pos));
	
	if (! families.count(fam_name))
		families[fam_name] = child;
	else 
		cerr << "Opps! We already have a " << fam_name << " family in our map! \n";
	}
}

void query_map(const string &family, const map<string, vstring> &families)
{
	map<string,vstring>::const_iterator it = families.find(family);
	
	if (it == families.end())
	{
		cout << "Sorry, the " << family << " is not currently entered.\n";
		return;
	}
	
	cout << "The " << family;
	if (! it->second.size())
		cout << " has no child.\n";
	else
	{
		cout << " has " << it->second.size() 
			 << (it->second.size() == 1 ? " child.\n" : " children.\n");
		vector<string>::const_iterator itchd = it->second.begin(),
									  end_itchd = it->second.end();
		while (itchd != end_itchd)
		{	
			cout << *itchd << " ";
			++itchd;	
		} 
		cout << endl;
	}
}

void display_map(const map<string,vstring> &families, ostream &os)
{
	map<string, vstring>::const_iterator it = families.begin(),
									 end_it = families.end();
	while (it != end_it)
	{
		if (it->second.empty())
		{
			os << "The family " << it->first << " has no child.\n";
		    ++it;
		 } 
		else
		{
			os << "The " << it->first << " has " << it->second.size() 
			   << ( it->second.size() == 1 ? " child:\n" : " children:\n" );
			vector<string>::const_iterator it_chd = it->second.begin(),
										   end_chd = it->second.end();
			for ( ;it_chd != end_chd; it_chd++)
				os << *it_chd << " ";
			os << endl;
		    ++it;
		}
	}
}
