#include <iostream>
#include <string>
#include <vector> 
#include <cstdlib>
#include <time.h>
#include <fstream>
using namespace std;

int main()
{
	string user_name;
	cout << "Please enter your name: ";
	getline(cin, user_name);
	//cin >> user_name;
	cout << "Hello,"
	     << user_name
	     << '\n';    // New line, use of '\n' or "\n" is OK. Different from C. 
	                 // cout << '\n' is equal to printf("%c", '\n') in C.
	                 // cout << will not start a new line.
    
    string word;
    const int min_size = 4;
    while (cin >> word)
    {  if (word.size() <= min_size) 
       {  cout << word.size()
               << '\n';
          continue;
	   }
	   else
	   { cout << "Get a word of size over 4 !";
	     break; 
	   }
    }

	const int seq_size = 18; 
	int pell_seq1[seq_size] = { 1, 2, 3, 5};
    vector<int> pell_seq2(pell_seq1, pell_seq1 + seq_size);  
					// Vector class doesn't support an explicit initialization list. 
					// But we can address to realize similar function.
    
    const int max_seq = 6;
    string seq_names[max_seq] = {
	    "Fibonacci", "Lucas", "Pell",
	    "Triangular", "Square", "Pentagonal"
	};
	cout << '\n'  
	     << seq_names[4]
	     << '\n';
	
	int ival = 1024;
	int *pi = &ival;  //Only when first initialization this operation is valid to assign an adress to pi.
	if (! pi)         // Typical use of pointer to avoid null pointer. 
	  *pi = 1024*2;
	if (pi && *pi != 1024)
	  *pi = 1024;
	  cout << *pi << "  "
	       << pi;
	
	int seq_index;
	srand(time(NULL));
    seq_index = rand() % 6;
    cout <<'\n' <<"The random sequence number is " 
	     << seq_index
	     << '\n';
	 
	     
	ofstream outfile("cpp1.txt", ios_base::app);
	if (! outfile)
	  cerr << "Opps! Unable to save data.\n";
	else outfile << '\n'
	             << user_name << ' '
	             << endl;
		outfile.close(); 
		
    ifstream infile("cpp1.txt");
    int counter = 0;
    if (! infile)
      cerr << "Opps£¡Unable to read data.\n";
	else
	{  string name;
	   //char name;
	   while (! infile.eof())
       { infile >> name;
         cout << name;
         name = '\0';     // Actually, there is a small detail (may) needs attention. We want to output all the contents
         ++counter;       // in this file. But the logical structure without name = '\0' will result in twice output
	   }                  // of the last string. The cause is when EOF is assigned to name, name is intact. 
	}
	cout << '\n'  << counter;
	
	struct Stage{ 
	       int a : 1;
	       int b : 1;
	};
 } 
