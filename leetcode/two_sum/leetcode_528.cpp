class Solution {
    vector<int> sum_vec;

public:
    Solution(vector<int>& w) {
        for (int i = 1; i < w.size(); i++)
            w[i] += w[i - 1];
        sum_vec = w;
    }
    
    int pickIndex() {
        if (sum_vec.size() <= 1)
            return 0;

        int rand_num = rand() % sum_vec.back() + 1;
        int left = 0, right = sum_vec.size() - 1, mid;
        while (left < right) {
            mid = left + (right - left) / 2;
            if (mid == 0) {
                if (rand_num <= sum_vec[0])
                    return 0;
                else
                    left = 1;
            }
            else {
                if (rand_num > sum_vec[mid])
                    left = mid + 1;
                else if (rand_num <= sum_vec[mid - 1])
                    right = mid - 1;
                else
                    return mid;
            }
        }
        return left;
    }
};

/**
 * Your Solution object will be instantiated and called as such:
 * Solution* obj = new Solution(w);
 * int param_1 = obj->pickIndex();
 */
