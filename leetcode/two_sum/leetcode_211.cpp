struct Node {
    bool isEnd;
    vector<Node*> children;
    Node(): isEnd(false), children(vector<Node*>(26, NULL)) {}
};


class WordDictionary {
public:
    Node* root;
    /** Initialize your data structure here. */
    WordDictionary() {
        root = new Node();
    }
    
    void addWord(string word) {
        Node* curNode = root;
        for (char ch : word) {
            int chIdx = ch - 'a';
            if (curNode->children[chIdx])
                curNode = curNode->children[chIdx];
            else {
                curNode->children[chIdx] = new Node();
                curNode = curNode->children[chIdx];
            }
        }
        curNode->isEnd = true;
    }
    
    // bfs
    bool search(string word) {
         queue<Node*> levelNodes;
         levelNodes.push(root);

        for (char ch : word) {
            if (ch == '.') {
                int curLen = levelNodes.size();
                for (int i = 0; i < curLen; i++) {
                    Node* curNode = levelNodes.front();
                    levelNodes.pop();
                    for (Node* ptr : curNode->children) {
                        if (ptr)
                            levelNodes.push(ptr);
                    }
                }
                if (levelNodes.empty())
                    return false;
            }
            else {
                int chIdx = ch - 'a', curLen = levelNodes.size();
                for (int i = 0; i < curLen; i++) {
                    Node* curNode = levelNodes.front();
                    levelNodes.pop();
                    if (curNode->children[chIdx])
                        levelNodes.push(curNode->children[chIdx]);
                }
                if (levelNodes.empty())
                    return false;
            }
        }
        
        while (!levelNodes.empty()) {
            Node* curNode = levelNodes.front();
            levelNodes.pop();
            if (curNode->isEnd)
                return true;
        }
        return false;
    }
};
