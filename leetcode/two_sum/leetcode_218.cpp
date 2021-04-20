class Solution {
public:
    vector<vector<int>> getSkyline(vector<vector<int>>& buildings) {
        multiset<pair<int, int>> all;
        vector<vector<int>> res;
        
        for (auto& e : buildings) {
            all.insert(make_pair(e[0], -e[2]));
            all.insert(make_pair(e[1], e[2]));
        }

        multiset<int> heights({0});
        vector<int> last = {0, 0};

        for (auto& p : all) {
            if (p.second < 0)
                heights.insert(-p.second);
            else
                heights.erase(heights.find(p.second));
            int maxHeight = *heights.rbegin();
            if (last[1] != maxHeight) {
                last[0] = p.first;
                last[1] = maxHeight;
                res.push_back(last);
            }
        }

        return res;
    }
};
