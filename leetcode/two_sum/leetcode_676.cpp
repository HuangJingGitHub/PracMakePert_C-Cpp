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


struct TrieNode {
    bool isEnd;
    vector<TrieNode*> children;
    TrieNode(): isEnd(false), children(vector<TrieNode*>(26, NULL)) {}
};

class MagicDictionary {
public:
    TrieNode* root;

    MagicDictionary() {
        root = new TrieNode();    
    }
    
    void insert(string& str) {
        TrieNode* curNode = root;
        for (char ch : str) {
            int chIdx = ch - 'a';
            if (!curNode->children[chIdx])
                curNode->children[chIdx] = new TrieNode();

            curNode = curNode->children[chIdx];
        }
        curNode->isEnd = true;
    }

    void buildDict(vector<string> dictionary) {
        for (string& str : dictionary)
            insert(str);
    }
    
    bool dfs(TrieNode* node, string& searchWord, int idx, bool diff) {
        if (idx == searchWord.size())
            return diff && node->isEnd;
        
        int chIdx = searchWord[idx] - 'a';
        if (node->children[chIdx])
            if (dfs(node->children[chIdx], searchWord, idx + 1, diff))
                return true;
        
        if (!diff) {
            for (int j = 0; j < 26; j++) {
                if (j != chIdx && node->children[j])
                    if (dfs(node->children[j], searchWord, idx + 1, true))
                        return true;
            }
        }
        return false;
    }

    bool search(string searchWord) {
        return dfs(root, searchWord, 0, false);
    }
};

/**
 * Your MagicDictionary object will be instantiated and called as such:
 * MagicDictionary* obj = new MagicDictionary();
 * obj->buildDict(dictionary);
 * bool param_2 = obj->search(searchWord);
 */
