class Solution {
public:
    vector<string> getFolderNames(vector<string>& names) {
        unordered_map<string, int> strLog;

        for (auto itr = names.begin(); itr != names.end(); itr++){
            if (strLog.find(*itr) == strLog.end())
                strLog[*itr] = 0;
            else{
                int order = strLog[*itr] + 1;
                string temp;
                for (temp = *itr + "(" + to_string(order) + ")"; strLog.find(temp) != strLog.end(); ){
                    order++;
                    temp  = *itr + "(" + to_string(order) + ")";
                }

                strLog[*itr] = order;
                *itr = temp;
                strLog[*itr] = 0;
            }
        }
        return names;
    }
};
