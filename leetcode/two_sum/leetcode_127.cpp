// BFS, speed is slow for large scale input.
class Solution {
public:
    int ladderLength(string beginWord, string endWord, vector<string>& wordList) {
        unordered_set<string> wordSet(wordList.begin(), wordList.end());
        if (wordSet.find(endWord) == wordSet.end())
            return 0;

        int res = 1;
        unordered_set<string> visited;
        queue<vector<string>> pathQueue;
        vector<string> path{beginWord};
        pathQueue.push(path);
        visited.emplace(beginWord);

        while (!pathQueue.empty()){
            int queueSize = pathQueue.size();
            unordered_set<string> subVisited;

            for (int i = 0; i < queueSize; i++){
                path = pathQueue.front();
                pathQueue.pop();
                string curEndWord = path.back();
                for (int j = 0; j < curEndWord.size(); j++){
                    for (char ch = 'a'; ch <= 'z'; ch++){
                        string newWord = curEndWord;
                        newWord.replace(j, 1, 1, ch);
                        if (newWord  == curEndWord)
                            continue;
                        if (wordSet.find(newWord) != wordSet.end() && visited.find(newWord) == visited.end()){
                            if (newWord == endWord)
                                return path.size() + 1;

                            vector<string> pathVec = path;
                            pathVec.push_back(newWord);
                            pathQueue.push(pathVec);
                            subVisited.emplace(newWord);
                        }            
                    }
                }
            }
            visited.insert(subVisited.begin(), subVisited.end());
        }
        return 0;
    }
};


// BFS, improved speed, but still exceed time limit.
class Solution {
public:
    int ladderLength(string beginWord, string endWord, vector<string>& wordList) {
        if (std::find(wordList.begin(), wordList.end(), endWord) == wordList.end() )
            return 0;

        vector<int> visited(wordList.size(), 0);
        queue<vector<string>> pathQueue;
        vector<string> path{beginWord};
        pathQueue.push(path);

        while (!pathQueue.empty()){
            int queueSize = pathQueue.size();
            vector<int> subVisitedIdx;

            for (int i = 0; i < queueSize; i++){
                path = pathQueue.front();
                pathQueue.pop();
                string curEndWord = path.back();
                
                for (int i = 0; i < wordList.size(); i++){
                    if (visited[i] == 1)
                        continue;

                    if (checkTrans(curEndWord, wordList, i)){
                        if (wordList[i] == endWord)
                            return path.size() + 1;
                        
                        vector<string> newPath = path;
                        newPath.push_back(wordList[i]);
                        pathQueue.push(newPath);
                        subVisitedIdx.push_back(i);
                    }

                }
            }
            for (int i : subVisitedIdx)
                visited[i] = 1;
        }
        return 0;
    }

    bool checkTrans(string &curWord, vector<string> &wordList, int wordIdx){
        int diffCount = 0;
        for (int i = 0; i < curWord.size(); i++){
            if (curWord[i] != wordList[wordIdx][i])
                diffCount++;
        }

        return diffCount == 1;
    }
};

// BFS, refer to the offical solution, AC, (heavy use of map, pair, set)
class Solution {
public:
    int ladderLength(string beginWord, string endWord, vector<string>& wordList) {
        if (std::find(wordList.begin(), wordList.end(), endWord) == wordList.end())
            return 0;

        int len = beginWord.size();
        map<string, vector<string>> allCombDict;

        for (int i = 0; i < wordList.size(); i++ ){
            string curWord = wordList[i];
            for (int j = 0; j < len; j++){
                string keyStr = curWord.substr(0, j) + "*" + curWord.substr(j+1);
                if (allCombDict.count(keyStr) == 0){
                    vector<string> valueStrVec{curWord};
                    allCombDict.insert(std::pair<string, vector<string>>(keyStr, valueStrVec));
                }
                else
                    allCombDict[keyStr].push_back(curWord);
            }
        }

        // BFS
        queue<std::pair<string, int>> stringQueue;
        stringQueue.push(std::pair<string, int>(beginWord, 1));

        unordered_set<string> visited;
        visited.insert(beginWord);

        while (!stringQueue.empty()){
            std::pair<string, int> front = stringQueue.front();
            stringQueue.pop();
            string word = front.first;
            int step = front.second;

            for (int i = 0; i < len; i++){
                string newKeyStr = word.substr(0, i) + "*" + word.substr(i+1);
                for (int j = 0; j < allCombDict[newKeyStr].size(); j++){
                    string transWord = allCombDict[newKeyStr][j];
                    if (transWord == endWord)
                        return step + 1;
                    if (visited.count(transWord) == 0){
                        visited.insert(transWord);
                        stringQueue.push(std::pair<string, int>(transWord, step + 1));
                    }
                }
            }
        }

        return 0;
    }
};


// Bidirectional BFS, refer to the offical solution, AC, faster
class Solution {
public:
    int ladderLength(string beginWord, string endWord, vector<string>& wordList) {
        if (std::find(wordList.begin(), wordList.end(), endWord) == wordList.end())
            return 0;

        int len = beginWord.size();
        map<string, vector<string>> allCombDict;

        for (int i = 0; i < wordList.size(); i++ ){
            string curWord = wordList[i];
            for (int j = 0; j < len; j++){
                string keyStr = curWord.substr(0, j) + "*" + curWord.substr(j+1);
                if (allCombDict.count(keyStr) == 0){
                    vector<string> valueStrVec{curWord};
                    allCombDict.insert(std::pair<string, vector<string>>(keyStr, valueStrVec));
                }
                else
                    allCombDict[keyStr].push_back(curWord);
            }
        }

        // Bidirectional BFS
        queue<std::pair<string, int>> beginQueue, endQueue;
        beginQueue.push(std::pair<string, int>(beginWord, 1));
        endQueue.push(std::pair<string, int>(endWord, 1));

        map<string, int> beginVisited, endVisited;
        beginVisited.insert(std::pair<string, int>(beginWord, 1));
        endVisited.insert(std::pair<string, int>(endWord, 1));

        while (!beginQueue.empty() && !endQueue.empty()){
            // one level forward from beginword
            std::pair<string, int> front = beginQueue.front();
            beginQueue.pop();
            string word = front.first;
            int level = front.second;

            for (int i = 0; i < len; i++){
                string newKeyStr = word.substr(0, i) + "*" + word.substr(i+1);
                for (int j = 0; j < allCombDict[newKeyStr].size(); j++){
                    string transWord = allCombDict[newKeyStr][j];
                    if (endVisited.count(transWord) != 0)
                        return level + endVisited[transWord];
                    if (beginVisited.count(transWord) == 0){
                        beginVisited.insert(std::pair<string, int>(transWord, level + 1));
                        beginQueue.push(std::pair<string, int>(transWord, level + 1));
                    }
                }
            }

            // one level forward from endword
            front = endQueue.front();
            endQueue.pop();
            word = front.first;
            level = front.second;

            for (int i = 0; i < len; i++){
                string newKeyStr = word.substr(0, i) + "*" + word.substr(i+1);
                for (int j = 0; j < allCombDict[newKeyStr].size(); j++){
                    string transWord = allCombDict[newKeyStr][j];
                    if (beginVisited.count(transWord) != 0)
                        return level + beginVisited[transWord];
                    if (endVisited.count(transWord) == 0){
                        endVisited.insert(std::pair<string, int>(transWord, level + 1));
                        endQueue.push(std::pair<string, int>(transWord, level + 1));
                    }
                }
            }
        }

        return 0;
    }
};
