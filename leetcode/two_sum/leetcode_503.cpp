class Solution {
public:
    vector<int> nextGreaterElements(vector<int>& nums) {
        vector<int> res(nums.size(), -1);
        stack<int> dreasingIdxStk;

        int len = nums.size();
        for (int i = 0; i < len; i++)
            nums.push_back(nums[i]);
        
        dreasingIdxStk.push(0);
        for (int i = 1; i < nums.size(); i++) {
            int topIdx = dreasingIdxStk.top();
            if (nums[topIdx] > nums[i] && i < len) {
                dreasingIdxStk.push(i);
                continue;
            }
            else  {
                while (dreasingIdxStk.empty() == false) {
                    int topIdx = dreasingIdxStk.top();
                    if (nums[topIdx] < nums[i]) {
                        res[topIdx] = nums[i];
                        dreasingIdxStk.pop();
                    }
                    else
                        break;
                }
                if (i < len)
                    dreasingIdxStk.push(i);
            }
        }

        return res;
    }
};
