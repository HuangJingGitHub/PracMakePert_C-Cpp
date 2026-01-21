class Solution {
public:
    vector<string> topKFrequent(vector<string>& words, int k) {
        map<string, int> word_fre;
        map<int, vector<string>, std::greater<int>> fre_words;

        for (auto& w : words)
            word_fre[w]++;
        
        for (auto& pair : word_fre)
            fre_words[pair.second].push_back(pair.first);

        vector<string> res;
        for (auto& pair : fre_words) {
            vector<string> cur_words = pair.second;
            sort(cur_words.begin(), cur_words.end());
            for (auto& w : cur_words) {
                res.push_back(w);
                k--;
                if (k <= 0)
                    break;
            }
            if (k <= 0)
                break;
        }
        return res;
    }
};
