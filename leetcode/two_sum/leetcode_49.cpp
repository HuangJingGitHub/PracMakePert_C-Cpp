// Personal solution which is correct but will exceed the time limit.
class Solution {
public:
    vector<vector<string>> groupAnagrams(vector<string>& strs) {
        vector<vector<string>> res;
        int n = strs.size();
        if (n == 0 || n == 1){
            res.push_back(strs);
            return res;
        }

        vector<string> vecStr{strs[0]};
        res.push_back(vecStr);
        for (int i = 1, j = 0; i < n; i++, j = 0){
            for(; j < res.size(); j++){
                if (sortedStr(res[j][0]) == sortedStr(strs[i])){
                    res[j].push_back(strs[i]);
                    break;
                }        
            }
            if (j == res.size()){
                vector<string> vecStr{strs[i]};
                res.push_back(vecStr);
            }
        }     
        return res;
    }
        string sortedStr(string str){
            string resStr = str;
            sort(resStr.begin(), resStr.end());
            return resStr;
        }
};

// Nice solution to use hashing table.
class Solution {
public:
    vector<vector<string>> groupAnagrams(vector<string>& strs) {
        vector<vector<string>> res;  
        unordered_map<string, int> strMap;
        int mapSize = 0;
       
        string temp;
        for (auto str:strs) {
            temp = str;
            sort(temp.begin(), temp.end());
            if (strMap.count(temp)){
                res[strMap[temp]].push_back(str);
            }
            else {
                vector<string> vec{str};
                res.push_back(vec);
                strMap[temp] = mapSize++;
            }
        }
        return res;
    }
};

class Solution {
public:
    vector<vector<string>> groupAnagrams(vector<string>& strs) {
        vector<vector<string>> res;
        map<string, vector<string>> ordedStrtoStr;

        for (auto& s : strs) {
            string copyStr = s;
            sort(copyStr.begin(), copyStr.end());
            ordedStrtoStr[copyStr].push_back(s);
        }

        for (auto itr = ordedStrtoStr.begin(); itr != ordedStrtoStr.end(); itr++)
            res.push_back(itr->second);

        return res;
    }
};
