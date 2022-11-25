class Solution {
public:
    string removeKdigits(string num, int k) {
        if (k >= num.size())
            return "0";

        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> num_idx_queue;
        for (int i = 0; i < k; i++)   
            num_idx_queue.push(make_pair(num[i] - '0', i));
        
        vector<int> keep_idx;
        int keep_pos = 0;
        for (int i = k; i < num.size(); i++) { 
            num_idx_queue.push(make_pair(num[i] - '0', i));
            keep_pos = num_idx_queue.top().second;
            keep_idx.push_back(keep_pos);

            while (num_idx_queue.empty() == false && num_idx_queue.top().second <= keep_pos)
                num_idx_queue.pop();
        }

        string res;
        int i = 0;
        while (i < keep_idx.size() && num[keep_idx[i]] == '0')
            i++;
        if (i == keep_idx.size())
            return "0";

        for (; i < keep_idx.size(); i++)
                res += num[keep_idx[i]];
        return res;
    }
};
