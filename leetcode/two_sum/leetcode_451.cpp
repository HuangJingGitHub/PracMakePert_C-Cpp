class Solution {
public:
    string frequencySort(string s) {
        unordered_map<char, int> frequencyLog;
        for (char& c : s)
            frequencyLog[c]++;
        
        vector<pair<char, int>> pair_vec(frequencyLog.begin(), frequencyLog.end());
        sort(pair_vec.begin(), pair_vec.end(), [](auto& a, auto& b) {
            return a.second > b.second;
        });

        string res;
        for (const auto [a, b] : pair_vec) {
            for (int i = 0; i < b; i++)
                res += a;
        }
        return res;
    }
};
