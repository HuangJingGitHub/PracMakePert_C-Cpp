#include <iostream>
#include <iomanip>

#include "random.h"

using namespace std;

const int highest_price = 200;
const int lowest_price  = 100;
const int guess_count   = 6;

static int InitializeBout();
static bool PlayBout(int actual_price, int lower_price, int higher_price, int chance_left);
static bool Again();
static int GetPrice(int lower_price, int higher_price, int chance_left);
static int CheckPrice(int lower_price, int higher_price, int guess_price);
static int JudgePrice(int actual_price, int guess_price);

void PrintWelcomeInfo()
{
	cout << "It's known that the price of the item id between "
		 << lowest_price << " and " << highest_price << " RMB yuan.\n"
		 << "If your guess of the price is right. You win the Game.\n"
		 << "You have " << guess_count << " chances to guess.\n"
		 << "Good Luck.\n";
 } 
 
void InitializeGame()
{
	Randomize();
}

double PlayGame()
{
	int actual_price,
	    lower_price  = lowest_price,
	    higher_price = highest_price;
	int chance_left  = guess_count,
		bout_count   = 0,
		prevailed_bout_count = 0;
	while (true)
	{
		cout << endl;
		actual_price = InitializeBout(); 
		if (PlayBout(actual_price, lower_price, higher_price, chance_left))
			prevailed_bout_count++;
		bout_count++;
		if (!Again()) 
			break;
	}
	return (double)prevailed_bout_count / (double)bout_count; 
}


static int InitializeBout()
{
	return GenerateRandomNumber(lowest_price, highest_price);
 } 


static bool PlayBout(int actual_price, int lower_price, int higher_price, int chance_left)
{
	int guess_price,
		judge_result;
	
	while (chance_left > 0)
	{
		guess_price = GetPrice(lower_price, higher_price, chance_left);
		guess_price = CheckPrice(lower_price, higher_price, guess_price);
		chance_left--;
		judge_result = JudgePrice(actual_price, guess_price);
		
		switch (judge_result)
		{
			case 1:
				if (chance_left > 0)
				{
					cout << "Your guess is higher.\n";
					higher_price = guess_price - 1;
				}
				else
				{
					cout << "You lose the game and the actual price is " 
						 << actual_price << " .\n";
					return false;
				}
				break;
			case -1:
				if (chance_left > 0)
				{
					cout << "Your guess is lower.\n";
					lower_price = guess_price + 1;
				}
				else
				{
					cout << "You lose the game and the actual price is " 
						 << actual_price << " .\n";
					return false;
				}
				break;
		    default:
		    	cout << "Congts! You winnnnnnn!\n";
		    	return true;		
		}
	}
}


static GetPrice(int lower_price, int higher_price, int chance_left)
{
	int t;
	cout << "The actual price is in [" << lower_price << ", " << higher_price << "].\n"
	     << "Chances left: " << chance_left << ".\n Please input your guess:\n";
	cin >> t;
	return t;
}


static CheckPrice(int lower_price, int higher_price, int guess_price)
{
	int t = guess_price;
	while (t < lower_price || t > higher_price)
	{
		cout << "Your guess price " << t << " is out of range.\n"
			 << "Please choose one in the range of [ " << lower_price << " , " 
			 << higher_price << " ]. Try again:\n";
		cin >> t;
	}
	return t;
}

static int JudgePrice(int actual_price, int guess_price)
{
	int t = guess_price - actual_price;
	if (t > 0)
		return 1;
	else if (t < 0)
		return -1;
	else
		return 0; 
}

static bool Again()
{
	int t;
	cout << "\n>>Stop the game. (Input 0)"
		 << "\n>>Play again? (Input a nonzero number)\n";
	cin >> t;
	return t!=0;
}

void PrintGameOverInfo(double prevailed_ratio)
{
	cout << "\nPrevailedy Ratio: " << setw(3) << prevailed_ratio*100 << "%.\n";
	if (prevailed_ratio >= 0.75)
		cout << "Amazing play!\n\n";
	else if (prevailed_ratio >= 0.50)
		cout << "Good play!\n\n";
	else
		cout << "You can do it better!\n\n";
}
