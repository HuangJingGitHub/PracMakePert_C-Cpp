class Solution {
public:
    int characterReplacement(string s, int k) {
        unordered_map<char, int> charFrequency;
        int left = 0, right = 0, maxRepeat = 0;

        while (right < s.size()) {
            charFrequency[s[right]]++;
            maxRepeat = max(maxRepeat, charFrequency[s[right]]);
            if (right - left + 1 - maxRepeat > k) {
                charFrequency[s[left]]--;
                left++;
            }
            right++;
        }

        return right - left;
    }
};
