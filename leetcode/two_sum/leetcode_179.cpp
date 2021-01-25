class Solution {
public:
    static bool cmp(const int &a, const int &b) {
        int lenA = 1, lenB = 1;
        int temp = a / 10;
        while (temp > 0) {
            temp /= 10;
            lenA++;
        }
        temp = b / 10;
        while (temp > 0) {
            temp /= 10;
            lenB++;
        }
        return a * pow(10, lenB) + b > b * pow(10, lenA) + a;
    }

    string largestNumber(vector<int>& nums) {
        string res;
        sort(nums.begin(), nums.end(), cmp);
        for (int num : nums) 
            res += to_string(num);
        if (res[0] == '0')
            return "0";
        return res;
    }
};
