// This solution is feasible. But the speed is slow because of the manipulation of the vector like copy and erase. 
// To deal with this, hashmap is better.
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
            string firstStr = s.substr(i, wordLength);
            vector<string> currentWords = words;
                for (j = i; j <= i + charNum - wordLength; j += wordLength){
                    string currentStr = s.substr(j, wordLength);
                    if (!isInWords(currentStr, currentWords))
                        break;
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

// This solution can pass the test with the usage of hashmap to deal with the words check. 
// But the speed is not optimized to the best.
class Solution {
public:
    vector<int> findSubstring(string s, vector<string>& words) {
        vector<int> ret;

        if (words.size() == 0)
            return ret;

        int wordNum = words.size(), wordLength = words[0].size(), charNum = words.size() * words[0].size();

        if (s.size() < charNum)
            return ret;
        unordered_map<string, int> hmap1, hmap2;
        for (int i = 0; i < wordNum; i++)
            hmap1[words[i]]++;

        for (int i = 0, j = 0; i + charNum <= s.size(); i++){
            for(j = i; j <= i + charNum - wordLength; j += wordLength){
                    string currentStr = s.substr(j, wordLength);
                    if (hmap1[currentStr] == 0)
                        break;
                    else{
                        hmap2[currentStr]++;
                        if (hmap1[currentStr] < hmap2[currentStr])
                            break;
                    }
                }
            if (j == i + charNum)
                ret.push_back(i);
            hmap2.clear();
        }
        return ret;
        } 
};
