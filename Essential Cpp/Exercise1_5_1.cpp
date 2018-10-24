#include <iostream>
#include <string>
using namespace std;

int main()
{
	string user_name;
	
	cout << "Please enter your name: ";
	cin >> user_name;
	
	switch (user_name.size())
	{
		case 0:
			cout << "Ah, the user with no name. "
				 << "Well, OK, hi, user with no name\n";
			break;
		case 1:
			cout << " A 1-character name? Hmm, have you read Kafka?"
				 << "Hello, " << user_name
				 << " -- happy to make your acquaintance!\n";
			break; 
			
		default:
			cout << "Hello, " << user_name		
				 << " -- happy to make your acquaintance!\n";
			break;
	}
	return 0;
}
