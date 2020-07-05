// The commented parts is a feasible but not simple enough solution. The uncommented part is simper and better as for speed and memory cost.
class Solution {
public:
    /*void placeEntry(vector<int> &nums, int num, int &start, int end)
    {
        for ( ;start <= end; start++){
            if (nums[start] >= num){
                break;
            }
        }
        
        vector<int>::iterator it = nums.begin() + start;
        nums.insert(it, num);
    }*/

    void merge(vector<int>& nums1, int m, vector<int>& nums2, int n) {
       /* int start = 0, end = m-1;
        for (int i = 0; i < n; i++){
            placeEntry(nums1, nums2[i], start, end);
            end++;
        }
        nums1.erase(nums1.begin()+m+n, nums1.end());*/
        int resIdx = m-- + --n;
        while (n>=0){
            nums1[resIdx--] = (m >= 0 && nums1[m] > nums2[n]) ? nums1[m--] : nums2[n--];
        }
    }
};
