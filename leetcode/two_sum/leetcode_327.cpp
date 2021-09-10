class Solution {
public:
    int countRangeSum(vector<int>& nums, int lower, int upper) {
        long sum = 0;
        vector<long> preSum{0};
        for (auto& num : nums) {
            sum += num;
            preSum.push_back(sum);
        }

        return countRangeSumRecursive(preSum, lower, upper, 0, preSum.size() - 1);
    }

    int countRangeSumRecursive(vector<long>& preSum, int lower, int upper, int left, int right) {
        if (left == right)
            return 0;
        else {
            int mid = left + (right - left) / 2;
            int n1 = countRangeSumRecursive(preSum, lower, upper, left, mid);
            int n2 = countRangeSumRecursive(preSum, lower, upper, mid + 1, right);
            int res = n1 + n2;

            int i = left;
            int l = mid + 1, r = mid + 1;
            while (i <= mid) {
                while (l <= right && preSum[l] - preSum[i] < lower) l++;
                while (r <= right && preSum[r] - preSum[i] <= upper) r++;
                res += (r - l);
                i++;
            } 

            vector<long> sorted(right - left + 1);
            int p1 = left, p2 = mid + 1;
            int p = 0;
            while (p1 <= mid || p2 <= right) {
                if (p1 > mid)
                    sorted[p++] = preSum[p2++];
                else if (p2 > right)
                    sorted[p++] = preSum[p1++];
                else {
                    if (preSum[p1] < preSum[p2])
                        sorted[p++] = preSum[p1++];
                    else
                        sorted[p++] = preSum[p2++];
                }
            }
            for (int i = 0; i < sorted.size(); i++)
                preSum[left + i] = sorted[i];
            return res;
        }
    }
};
