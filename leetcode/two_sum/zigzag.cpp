#include <iostream>
#include <vector>
#include <string>

using namespace std;

class Solution
{
public:
	string convert(string s, int numRows)
	{
		if (numRows == 1)
			return s;

		vector<vector<char>> zigzagString(numRows);
		bool goingDown = false;
		int currentRow = 0;

		for (int i = 0; i < s.size(); ++i)
		{
			zigzagString[currentRow].push_back(s[i]);

			if (currentRow == numRows - 1 || currentRow == 0)
				goingDown = !goingDown;

			if (goingDown)
				++currentRow;
			else
				--currentRow;
		}

		string resultStr;
		for (int i = 0; i < numRows; ++i)
			for (int j = 0; j < zigzagString[i].size(); j++)
				resultStr.push_back(zigzagString[i][j]);
		return resultStr;
	}
};

int main()
{
	string testStr = "fdasfdsafdsafds";
	int testRows = 3;
	Solution sol;
	cout << sol.convert(testStr, testRows) << endl;
}
