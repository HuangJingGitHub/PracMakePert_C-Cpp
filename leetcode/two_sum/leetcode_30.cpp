class Solution {
public:
    vector<int> findSubstring(string s, vector<string>& words) {
        vector<int> ret;

        if (words.size() == 0)
            return ret;

        int wordNum = words.size(), wordLength = words[0].size(), charNum = words.size() * words[0].size();

        if (s.size() < charNum)
            return ret;

        for (int i = 0, j = 0; i + charNum <= s.size(); i++){
            string firstStr = s.substr(i, wordLength), lastStr = s.substr(i+charNum-wordLength, wordLength);
            vector<string> currentWords = words;
            if (words.size() == 1){
                if (!isInWords(firstStr, currentWords))
                    continue;
            }
            else if (!isInWords(firstStr, currentWords) || !isInWords(lastStr, currentWords))
                continue;
            else{
                for (j = i + wordLength; j < i + charNum - wordLength; j += wordLength){
                    string currentStr = s.substr(j, wordLength);
                    if (!isInWords(currentStr, currentWords))
                        break;
                }
            }
            if ( currentWords.size() == 0)
                ret.push_back(i);
        }
        return ret;
    }
        bool isInWords(string s, vector<string>& words)
        {
            for (int i = 0; i < words.size(); i++)
                if (words[i] == s){
                    words.erase(words.begin() + i);
                    return true;
                }
            return false;
        }     

};
