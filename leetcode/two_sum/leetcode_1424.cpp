// O(max(nums.size(), nums[i].size())^2) in the worst case instead of simple linear traversal.
// Optimal if no use of hash, but time exceeds the limits in extreme cases.
class Solution {
public:
    vector<int> findDiagonalOrder(vector<vector<int>>& nums) {
        vector<int> res;
        int idxSum = 0, maxIdxSum = 0, row = 0, col = 0, curLargestRow = nums.size() - 1;
        for (int i = 0; i < nums.size(); i++)
            maxIdxSum = max(maxIdxSum, i + (int)nums[i].size() - 1);

        while (idxSum <= maxIdxSum) {
            if (idxSum < nums.size()) {
                row = idxSum;
                col = 0;
            }
            else {
                for (row = curLargestRow, col = idxSum - row; row + col < idxSum; ) {
                    if (col >= nums[row].size()) {
                        row--;
                        col++;
                        curLargestRow = row;
                    }
                }
            }

            while (row >= 0) {
                if (col >= nums[row].size()) {
                    row--;
                    col++;
                }
                else {
                    res.push_back(nums[row][col]);
                    row--;
                    col++;
                }
            }
            idxSum++;
        }
        return res;
    }
};
