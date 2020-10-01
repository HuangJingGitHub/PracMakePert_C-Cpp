// BFS, will exceed the time limit.
class Solution {
public:
    vector<vector<string>> findLadders(string beginWord, string endWord, vector<string>& wordList) {
        vector<vector<string>> res;
        unordered_set<string> stringSet(wordList.begin(), wordList.end());
        
        if (stringSet.find(endWord) == stringSet.end())
            return res;
        
        unordered_set<string> visited;
        queue<vector<string>> queue;
        vector<string> list{beginWord};
        queue.push(list);
        visited.emplace(beginWord);

        bool flag = false;
        while (!queue.empty() && !flag){
            int size = queue.size();
            unordered_set<string> subVisited;

            for (int i = 0; i < size; i++){
                vector<string> path = queue.front();
                queue.pop();
                string word = path.back();
                for (int j = 0; j < word.size(); j++){
                    char temp = word[j];
                    for (char ch = 'a'; ch <= 'z'; ch++){
                        if (temp == ch)
                            continue;
                        string newStr = word;
                        newStr.replace(j, 1, 1, ch);
                        if (stringSet.find(newStr) != stringSet.end() && visited.find(newStr) == visited.end()){
                            vector<string> pathVec = path;
                            pathVec.push_back(newStr);
                            if (newStr == endWord){
                                flag = true;
                                res.push_back(pathVec);
                            }
                            queue.push(pathVec);
                            subVisited.emplace(newStr);
                        }
                    }
                }
            }
            visited.insert(subVisited.begin(), subVisited.end());
        }
        return res;
    }
};
