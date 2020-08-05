class Solution {
public:
    int uniqueMorseRepresentations(vector<string>& words) {
        vector<string> dic{".-","-...","-.-.","-..",".","..-.","--.",
                            "....","..",".---","-.-",".-..","--","-.",
                            "---",".--.","--.-",".-.","...","-","..-",
                            "...-",".--","-..-","-.--","--.."};
        unordered_set<string> uniqueMorseResp;

        for (auto w:words){
            string temp = "";
            for (auto c:w)
                temp += dic[c - 'a'];
            uniqueMorseResp.emplace(temp);
        }

        return uniqueMorseResp.size();
    }
};
