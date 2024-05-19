class Solution {
public:
    string reverseWords(string s) {
        int start_idx = 0, end_idx = 0;
        while (end_idx < s.size()) {
            while (end_idx < s.size() && s[end_idx] != ' ')
                end_idx++;
            
            int temp = end_idx;
            end_idx--;
            while (start_idx < end_idx)
                std::swap(s[start_idx++], s[end_idx--]);
            
            start_idx = temp + 1;
            end_idx =temp + 1;
        }

        return s;
    }
};
