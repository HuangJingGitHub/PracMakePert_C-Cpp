class Solution {
public:
    // The point is to sort string, e.g., "30" + "3" < "3" + "30", so "30" < "3".
    // Note in comparator, no "=", as it requires comp(a, a) returns false.
    static bool comparator(const string& a, const string& b) {
        return a + b < b + a;
    }

    string minNumber(vector<int>& nums) {
        vector<string> numStr;
        string res;
        for (auto num : nums)
            numStr.push_back(to_string(num));
        sort(numStr.begin(), numStr.end(), comparator);
        for (string str : numStr)
            res += str;
        return res;
    }
};
