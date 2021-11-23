class Solution {
public:
    vector<vector<int>> palindromePairs(vector<string>& words) {
        vector<vector<int>> res;
        
        string wordSum;
        for (int i = 0; i < words.size() - 1; i++)
            for (int j = i + 1; j < words.size(); j++) {
                wordSum = words[i] + words[j];
                if (isPalindrome(wordSum))
                    res.push_back(vector<int>{i, j});
                
                wordSum = words[j] + words[i];
                if (isPalindrome(wordSum))
                    res.push_back(vector<int>{j, i});
            }
        return res;
    }

    bool isPalindrome(string& wordSum) {
        int left = 0, right = wordSum.size() - 1;
        while (left < right) {
            if (wordSum[left] != wordSum[right])
                return false;
            left++;
            right--;
        }
        return true;
    }
};
