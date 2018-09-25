#include <iostream>
#include <cstdlib>

using namespace std;

/* void Account::PrintBalance() const
{
	cerr << "Error. Balance not available for base type.\n";
 } */

void CheckingAccount::PrintBalance() const
{
	cout << "Checking account balance: " << GetBalance() << endl;
}

void SavingsAccount::PrintBalance() const
{
	cout << "Checking account balance: " << GetBalance() << endl;
}
