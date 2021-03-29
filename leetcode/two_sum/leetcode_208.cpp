struct TrieNode {
    bool isEnd;
    vector<TrieNode*> children;
    TrieNode(): isEnd(false), children(vector<TrieNode*>(26, NULL)) {}
};

class Trie {
public:
    TrieNode *root;
    /** Initialize your data structure here. */
    Trie() {
        root = new TrieNode();
    }
    
    /** Inserts a word into the trie. */
    void insert(string word) {
        TrieNode *curNode = root;
        for (char ch : word) {
            int chIdx = ch - 'a';
            if (curNode->children[chIdx]) 
                curNode = curNode->children[chIdx];
            else {
                curNode->children[chIdx] = new TrieNode();
                curNode = curNode->children[chIdx];
            }
        }
        curNode->isEnd = true;
    }
    
    /** Returns if the word is in the trie. */
    bool search(string word) {
        TrieNode *curNode = root;
        for (char ch : word) {
            int chIdx = ch - 'a';
            if (curNode->children[chIdx])
                curNode = curNode->children[chIdx];
            else    
                return false;
        }
        return curNode->isEnd;
    }
    
    /** Returns if there is any word in the trie that starts with the given prefix. */
    bool startsWith(string prefix) {
        TrieNode *curNode = root;
        for (char ch : prefix) {
            int chIdx = ch - 'a';
            if (curNode->children[chIdx])
                curNode = curNode->children[chIdx];
            else    
                return false;
        }
        return true;        
    }
};
