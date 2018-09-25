#include <iostream>
#include "Account.h"
#include "Account.cpp"

using namespace std;

int main()
{
	CheckingAccount* checking = new CheckingAccount(100.00);
	SavingsAccount* savings = new SavingsAccount(1000.00);
	
	Account* account = checking;
	account->PrintBalance();

	account = savings;
	account->PrintBalance();	
	
	delete checking;
	delete savings;
	return 0;
}
