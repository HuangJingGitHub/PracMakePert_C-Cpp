class Solution {
public:
    vector<vector<int>> kSmallestPairs(vector<int>& nums1, vector<int>& nums2, int k) {
        auto cmp = [&nums1, &nums2] (const pair<int, int>& a, const pair<int, int>& b) {
            return nums1[a.first] + nums2[a.second] > nums1[b.first] + nums2[b.second];
        };

        int len1 = nums1.size(), len2 = nums2.size();
        vector<vector<int>> res;
        priority_queue<pair<int, int>, vector<pair<int, int>>, decltype(cmp)> pq(cmp);

        for (int i = 0; i < len1; i++)
            pq.emplace(make_pair(i, 0));

        while (k-- > 0 && pq.empty() == false) {
            auto [x, y] = pq.top();
            pq.pop();
            res.push_back({nums1[x], nums2[y]});
            if (y < len2 - 1)
                pq.push(make_pair(x, y + 1));
        }

        return res;
    }
};
