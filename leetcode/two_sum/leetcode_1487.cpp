class Solution {
public:
    vector<string> getFolderNames(vector<string>& names) {
        unordered_map<string, int> strLog;

        for (auto itr = names.begin(); itr != names.end(); itr++){
            if (strLog.find(*itr) == strLog.end())
                strLog[*itr] = 0;
            else{
                int idx = strLog[*itr] + 1;
                string tempStr = *itr + "(" + to_string(idx) + ")";                
                while (strLog.find(tempStr) != strLog.end()){
                    idx++;
                    tempStr  = *itr + "(" + to_string(idx) + ")";
                }

                strLog[*itr] = idx;
                *itr = tempStr;
                strLog[*itr] = 0;
            }
        }
        return names;
    }
};
