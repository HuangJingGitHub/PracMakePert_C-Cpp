// dp version
class Solution {
public:
    int numTrees(int n) {
        vector<int> nums(n+1, 0);
        nums[0] = 1;
         
        for (int i = 1; i < n+1; i++)
            for (int j = 0; j < i; j++)
                nums[i] += nums[j] * nums[i-1-j];
        
        return nums[n];
    }
}; 

// recursion version (will exceed the time limit without opeimizaiton to avoid repeated computation)
/*class Solution{
public: 
    int numTrees(int n)
    {
        if (n == 0 || n == 1)
            return 1;
        int res = 0;
        
        for (int i = 0; i < n; i++)
            res += numTrees(i) * numTrees(n-i-1);
        
        return res;
    }
}; */
