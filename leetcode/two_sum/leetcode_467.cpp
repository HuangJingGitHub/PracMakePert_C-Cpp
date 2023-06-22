class Solution {
public:
    int findSubstringInWraproundString(string s) {
			int res = 0, subStrNum = 1;
			vector<int> uniqueSubStrNum(26, 0);
			uniqueSubStrNum[s[0] - 'a'] = 1;

			for (int i = 1; i < s.size(); i++) {
				if (s[i] - s[i - 1] == 1 || s[i] - s[i - 1] == -25) // s[i] - s[i - 1] == -25 <-> s[i] = a, s[i - 1] = z
					subStrNum++;
				else 
					subStrNum = 1;
				uniqueSubStrNum[s[i] - 'a'] = max(subStrNum, uniqueSubStrNum[s[i] - 'a']);
			}

			for (int& num : uniqueSubStrNum)
				res += num;
			return res;
    }
};
