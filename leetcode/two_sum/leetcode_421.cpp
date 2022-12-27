class Trie {
private:
    Trie* next[2] = {nullptr};

public:
    Trie() {}

    void insert(int x) {
        Trie* root = this;
        for (int i = 30; i >= 0; i--) {
            int u = x >> i & 1;
            if (root->next[u] == nullptr)
                root->next[u] = new Trie();
            root = root->next[u];
        }
    }
    
    int search(int x) {
        Trie* root = this;
        int y = 0;

        for (int i = 30; i >= 0; i--) {
            int u = x >> i & 1;
            if (root->next[!u] != nullptr) {
                root = root->next[!u];
                y = y * 2 + !u;
            }
            else {
                root = root->next[u];
                y = y * 2 + u;
            }
        }
        return x^y;
    }
};


class Solution {
public:
    int findMaximumXOR(vector<int>& nums) {
        int res = 0;
        Trie* root = new Trie();

        for (int num : nums)
            root->insert(num);
        for (int num : nums)
            res = max(res, root->search(num));
        return res;
    }
};
