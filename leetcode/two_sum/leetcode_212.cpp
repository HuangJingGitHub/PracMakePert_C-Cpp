// will exceed the time limit for large number of words with similar looks.
class Solution {
public:
    vector<string> findWords(vector<vector<char>>& board, vector<string>& words) {
        if (board.empty() || board.front().empty())
            return {};

        vector<string> res;

        for (string curWord : words) {
            bool found = false;
            for (int row = 0; row < board.size(); row++) {
                for (int col = 0; col < board.front().size(); col++) {
                    if (backtrace(board, row, col, curWord, 0)) {
                        found = true;
                        res.push_back(curWord);
                        break;
                    }
                if (found)
                    bresak;
                }
            }
        }
        return res;
    }

    bool backtrace(vector<vector<char>>& board, int row, int col, string& word, int wordIdx) {
        if (board[row][col] != word[wordIdx])
            return false;
        if (wordIdx == word.size() - 1)
            return true;
        
        char originChar = board[row][col];
        board[row][col] = '*';
        if ((col < board.front().size() - 1 && backtrace(board, row, col + 1, word, wordIdx + 1)) ||
            (col > 0 && backtrace(board, row, col - 1, word, wordIdx + 1)) ||
            (row < board.size() - 1 && backtrace(board, row + 1, col, word, wordIdx + 1)) ||
            (row > 0 && backtrace(board, row - 1, col, word, wordIdx + 1))) {
                board[row][col] = originChar;
                return true;   
            }
        board[row][col] = originChar;
        return false;
    }
};


struct TireNode {
    bool isEnd;
    string word;
    vector<TireNode*> children;
    TireNode(): isEnd(false), children(vector<TireNode*>(26, NULL)) {}
};


// Trie tree + dfs to decrease invocation times of functions.
class Solution {
    TireNode* root;
public:
    vector<string> findWords(vector<vector<char>>& board, vector<string>& words) {
        root = new TireNode();

        for (string word : words) {
            TireNode* cur = root;
            for (char ch : word) {
                int idx = ch - 'a';
                if (!cur->children[idx])
                    cur->children[idx] = new TireNode();
                cur = cur->children[idx];
            }
            cur->isEnd = true;
            cur->word  = word;
        }

        vector<string> res;
        if (board.empty() || board[0].empty())
            return res;

        for (int i = 0; i < board.size(); i++)
            for (int j = 0; j < board[0].size(); j++) {         
                dfs(board, i, j, res, root);
            }
        return res;
    }


    void dfs(vector<vector<char>>& board, int row, int col, vector<string>& res, TireNode* curNode) {
        if (board[row][col] == '*')
            return;

        char curChar = board[row][col];
        int idx = curChar - 'a';
        if (!curNode->children[idx])
            return;
        else if (curNode->children[idx]->isEnd) {
            res.push_back(curNode->children[idx]->word);
            curNode->children[idx]->isEnd = false;
        }

        board[row][col] = '*';  // Important detail. This line cannot be palced ahead, otherwise '*' cannot be retrieved if current char is not in the tire.
        if (col < board[0].size() - 1)
            dfs(board, row, col + 1, res, curNode->children[idx]);
        if (col > 0)
            dfs(board, row, col - 1, res, curNode->children[idx]);
        if (row < board.size() - 1)
            dfs(board, row + 1, col, res, curNode->children[idx]);
        if (row > 0)
            dfs(board, row - 1, col, res, curNode->children[idx]);
        board[row][col] = curChar;
    }
};
