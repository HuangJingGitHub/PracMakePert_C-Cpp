class Solution {
public:
    string findReplaceString(string S, vector<int>& indexes, vector<string>& sources, vector<string>& targets) {
        string res = S;
        int idxShift = 0;
        vector<pair<int, int>> idxInfo(indexes.size());  // Just make sure the replacement takes place from left to right instead of random order. If it occurs 
        for (int i = 0; i < indexes.size(); i++)         // in a random order, finding the corresponding index will be hard to solve. (need to matain a log) 
            idxInfo[i] = pair(indexes[i], i);            // Processing form back to front will be even easier as idxShift is not needed.
        sort(idxInfo.begin(), idxInfo.end());

        for (int i = 0; i < indexes.size(); i++){
            int mapIdx = idxInfo[i].second;
            if (sourceCheck(S, indexes[mapIdx], sources[mapIdx])){
                res.replace(indexes[mapIdx] + idxShift, sources[mapIdx].size(), targets[mapIdx]);  
                idxShift += (-sources[mapIdx].size() + targets[mapIdx].size());
            }
        }
        return res;
    }

    bool sourceCheck(string& S, int& index, string& source){
        for (int i = 0; i < source.size(); i++)
            if (index + i >= S.size() || S[index + i] != source[i])
                return false;
        return true;
    }
};
