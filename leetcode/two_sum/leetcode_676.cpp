class MagicDictionary {
public:
    /** Initialize your data structure here. */
    vector<vector<string>> dict;

    MagicDictionary() {
        dict = vector<vector<string>>(101);
    }
    
    void buildDict(vector<string> dictionary) {
        for (int i = 0; i < dictionary.size(); i++)
            dict[dictionary[i].size()].push_back(dictionary[i]);
    }
    
    bool search(string searchWord) {
        int wordLen = searchWord.size();

        for (int i = 0; i < dict[wordLen].size(); i++){
            int charDifNum = 0;
            for (int j = 0; j < wordLen; j++)
                if (searchWord[j] != dict[wordLen][i][j]){
                    charDifNum++;
                    if (charDifNum > 1)
                        break;
                }
            if (charDifNum == 1)
                return true;
        }
        return false;
    }
};

/**
 * Your MagicDictionary object will be instantiated and called as such:
 * MagicDictionary* obj = new MagicDictionary();
 * obj->buildDict(dictionary);
 * bool param_2 = obj->search(searchWord);
 */
