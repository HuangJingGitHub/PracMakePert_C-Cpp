class Solution {
public:
    vector<string> findRelativeRanks(vector<int>& score) {
        vector<pair<int, int>> score_idx_vec;
        vector<string> res(score.size());

        for (int i = 0; i < score.size(); i++) 
            score_idx_vec.push_back(make_pair(score[i], i));
        
        sort(score_idx_vec.rbegin(), score_idx_vec.rend());

        res[score_idx_vec[0].second] = "Gold Medal";
        if (score_idx_vec.size() >= 2)
            res[score_idx_vec[1].second] = "Silver Medal";
        if (score_idx_vec.size() >= 3)
            res[score_idx_vec[2].second] = "Bronze Medal";
        for (int i = 3; i < score_idx_vec.size(); i++)
            res[score_idx_vec[i].second] = to_string(i + 1);
        
        return res;
    }
};
