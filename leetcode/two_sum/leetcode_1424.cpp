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


// Use hash table to store info
class Solution {
public:
    vector<int> findDiagonalOrder(vector<vector<int>>& nums) {
        vector<int> res;
        unordered_map<int, vector<int>> idxHash;
        int maxIdxSum = 0;
        for (int i = 0; i < nums.size(); i++) 
            for (int j = 0; j < nums[i].size(); j++) {
                idxHash[i + j].push_back(nums[i][j]);  // Row and col idx sum is the criterion.
                maxIdxSum = max(maxIdxSum, i + j);
            }
        for (int i = 0; i <= maxIdxSum; i++) {
            reverse(idxHash[i].begin(), idxHash[i].end());  // reverse the order
            res.insert(res.end(), idxHash[i].begin(), idxHash[i].end());
        }
        return res;
    }
};
