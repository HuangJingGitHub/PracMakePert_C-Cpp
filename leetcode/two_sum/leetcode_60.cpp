// See the discussion and favorite solution. It is interesting.
class Solution {
public:
    static const vector<int> fac;
    string getPermutation(int n, int k) {
        string res;
        string s = string("123456789").substr(0, n);
        k--;
        while (k > 0){
            size_t i = k/fac[n-1];
            res.push_back(s[i]);
            s.erase(s.begin() + i);
            k %= fac[n-1];
            n--;
        }
        return res + s;
    }
};

const vector<int> Solution::fac = {0,1,2,6,24,120,720,5040,40320,362880,3628800};


// This is the true cool solution. See this https://leetcode-cn.com/problems/permutation-sequence/solution/zhe-ti-ru-guo-di-gui-jiu-zuo-fu-za-liao-qi-shi-shi/
class Solution {
public:
    string getPermutation(int n, int k) {
        string nums, res;
        for (int i = 1; i <= n; i++)
            nums.append(1, char('0' + i));
        int interval, loop = n;
        
        for (int i = 0; i < loop; i++){
            n--;
            interval = (k - 1) / factorial(n);
            res.push_back(nums[interval]);
            nums.erase(nums.begin() + interval);
            k -= interval * factorial(n);
        }
        return res;
    }

    int factorial(int n)
    {
        int res = 1;
        if (n <= 0)
            return res;
        else 
            res = n * factorial(n-1);

        return res;
    }
};
