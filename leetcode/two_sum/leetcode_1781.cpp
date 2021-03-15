class Solution {
public:
    int beautySum(string s) {
        int res = 0;
        vector<vector<int>> freqLog(s.size(), vector<int>(26));

        for (int i = 0; i < s.size(); i++) {
            if (i == 0)
                freqLog[i][s[i] - 'a']++;
            else {
                freqLog[i] = freqLog[i - 1];
                freqLog[i][s[i] - 'a']++;
            }
        }

        vector<int> freq;
        for (int i = 2; i < s.size(); i++)
            for (int j = 0; j <= i - 2; j++) {
                if (j == 0){
                    for (int num : freqLog[i])
                        if (num != 0)
                            freq.push_back(num);
                }
                else
                    for (int k = 0; k < 26; k++) {
                        if (freqLog[i][k] - freqLog[j - 1][k] != 0)   // exclude those char appearing 0 time
                            freq.push_back(freqLog[i][k] - freqLog[j - 1][k]);
                }
                int maxFreq = *max_element(freq.begin(), freq.end()), minFreq = *min_element(freq.begin(), freq.end());
                res += maxFreq - minFreq;
                freq.clear();
            }
        return res;
    }
};
