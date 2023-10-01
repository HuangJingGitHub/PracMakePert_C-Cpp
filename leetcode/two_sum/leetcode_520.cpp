class Solution {
public:
    bool detectCapitalUse(string word) {
        if (word.size() <= 1)
            return true;
        
        if (97 <= word[1] && word[1] <= 122) {
            for (int i = 2; i < word.size(); i++)
                if (97 > word[i] || word[i] > 122)
                    return false;
            return true;
        }
        else {
            if (97 <= word[0] && word[0] <= 122)
                return false;
            for (int i = 2; i < word.size(); i++)
                if (97 <= word[i] && word[i] <= 122)
                    return false;
            return true;
        }
    }
};
