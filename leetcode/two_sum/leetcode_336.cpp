// brute force
class Solution {
public:
    vector<vector<int>> palindromePairs(vector<string>& words) {
        vector<vector<int>> res;
        
        string wordSum;
        for (int i = 0; i < words.size() - 1; i++)
            for (int j = i + 1; j < words.size(); j++) {
                wordSum = words[i] + words[j];
                if (isPalindrome(wordSum))
                    res.push_back(vector<int>{i, j});
                
                wordSum = words[j] + words[i];
                if (isPalindrome(wordSum))
                    res.push_back(vector<int>{j, i});
            }
        return res;
    }

    bool isPalindrome(string& wordSum) {
        int left = 0, right = wordSum.size() - 1;
        while (left < right) {
            if (wordSum[left] != wordSum[right])
                return false;
            left++;
            right--;
        }
        return true;
    }
};


// trie
class TrieNode {
private:
    bool isEnd;
    int index;
    vector<TrieNode*> children;
public:
    TrieNode(): index(-1), isEnd(false), children(26, nullptr) {}
    
    ~TrieNode() {
        for(int i = 0; i < 26; i++) {
            if( children[i]) {
                delete children[i];
                children[i] = nullptr;
            }
        }
    }
    int getIndex() { 
        return index;
    }
    void setIndex(int i) { 
        index = i;
    }
    bool isWordEnd() { 
        return isEnd;
    }
    void SetEnd() { 
        isEnd = true ;
    }

    TrieNode* insertNode(char c) {
        if (!( 'a' <= c <= 'z')) 
            return nullptr;
        int id = c - 'a';
        if(children[id] == nullptr) {
            children[id] = new TrieNode();
        }
        return children[id];
    }
    
    TrieNode* getNode(char c) {
         if(!( 'a' <= c <= 'z')) 
            return nullptr;
         int id = c - 'a';
         return children[id] ;
    }
};

class Trie {
private:
    TrieNode* root;
public:
    Trie(): root(new TrieNode()){}
    ~Trie() { 
        delete root;
    }

    void insert(string word,int index) {
        TrieNode *p = root;
        for(int i = 0; i < word.size(); i++) {
            p = p->insertNode(word[i]);
        }
        p->SetEnd();
        p->setIndex(index);
    }
    
    TrieNode* getNode(string word) {
        TrieNode * p = root;
        for(int i = 0; i < word.size(); i++ ) {
            p = p->getNode(word[i]) ;
            if (p == nullptr) 
                return nullptr;
        }
        return p;
    }
    
    bool search(string word, int& index) {
        TrieNode* p = getNode(word);
        if(p) {
            index = p->getIndex();
            return  p->isWordEnd();
        }
        return false;
    }
};

class Solution {
public:
    vector<vector<int>> palindromePairs(vector<string>& words) {
        vector<vector<int>> res;
        Trie* trieTree = new Trie();
        for(int i = 0; i < words.size(); i++) {
            trieTree->insert(words[i], i);
        }
        for(int i = 0; i < words.size(); i++) {
            for(int j = 0; j < words[i].size(); j++) {
                bool flag = check(words[i], 0, j);
                if(flag) {
                    string temp = words[i].substr(j + 1);
                    reverse(temp.begin(), temp.end());
                    int index = -1;
                    if(trieTree->search(temp, index)) {
                        if(i != index ) {
                            res.push_back({index, i});
                            if( temp == "") {
                                res.push_back({i, index});
                            }
                        }
                    }
                }
                flag = check(words[i], j + 1, words[i].size() - 1);
                if(flag) {
                    string temp = words[i].substr(0, j + 1);
                    reverse(temp.begin(), temp.end());
                    int index = -1;
                    if (trieTree->search(temp,index)) {
                            if(i != index)
                                res.push_back({i,index});
                    }
                }    
            }
        }
        return res;
    }
    bool check(string &vec,int left,int right) {
        int i = left;
        int j = right;
        while(i <= j) {
            if (vec[i] != vec[j]) 
                return false;
            i++;
            j--;
        }
        return true;
    }
};
