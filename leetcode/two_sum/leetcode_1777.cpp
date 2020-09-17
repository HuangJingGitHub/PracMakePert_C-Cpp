// dynamic programming
class Solution {
public:
    int respace(vector<string>& dictionary, string sentence) {
        int len = sentence.size();
        vector<int> dp(len+1, 0);
        string word;

        for (int i = 0; i < len; i++){
            dp[i+1] = dp[i] + 1;
            for (int j = 0; j < dictionary.size(); j++){
                word = dictionary[j];
                if (i+1 >= word.size() && sentence.substr(i-word.size()+1, word.size()) == word){
                    dp[i+1] = min(dp[i+1], dp[i+1-dictionary[j].size()]);  // cool to track back to last word position
                }
            }
        }
        return dp.back();
    }
};
