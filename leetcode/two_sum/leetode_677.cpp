struct TrieNode {
    bool isEnd;
    int val = 0;
    vector<TrieNode*> children;
    TrieNode(): isEnd(false), children(vector<TrieNode*>(26, NULL)) {}
};

class MapSum {
public:
    TrieNode* root;

    MapSum() {
        root = new TrieNode();
    }
    
    void insert(string key, int val) {
         TrieNode *curNode = root;
        for (char ch : key) {
            int chIdx = ch - 'a';
            if (curNode->children[chIdx]) 
                curNode = curNode->children[chIdx];
            else {
                curNode->children[chIdx] = new TrieNode();
                curNode = curNode->children[chIdx];
            }
        }
        curNode->isEnd = true;   
        curNode->val = val;    
    }
    
    int sum(string prefix) {
        int res = 0;

        TrieNode *curNode = root;
        for (char ch : prefix) {
            int chIdx = ch - 'a';
            if (curNode->children[chIdx])
                curNode = curNode->children[chIdx];
            else    
                return 0;
        }

        queue<TrieNode*> nodeQueue;
        nodeQueue.push(curNode);
        while (!nodeQueue.empty()) {
            TrieNode* node = nodeQueue.front();
            nodeQueue.pop();
            if (node->isEnd)
                res += node->val;
            for (auto child : node->children) {
                if (child)
                    nodeQueue.push(child);
            }
        }

        return res;
    }
};

/**
 * Your MapSum object will be instantiated and called as such:
 * MapSum* obj = new MapSum();
 * obj->insert(key,val);
 * int param_2 = obj->sum(prefix);
 */
