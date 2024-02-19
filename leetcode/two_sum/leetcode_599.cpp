class Solution {
public:
    vector<string> findRestaurant(vector<string>& list1, vector<string>& list2) {
        vector<string> res;
        int minimum_index_sum = INT_MAX;
        unordered_map<string, int> str_index_map;
        
        for (int i = 0; i < list1.size(); i++)
            str_index_map[list1[i]] = i;
        
        for (int i = 0; i < list2.size(); i++) {
            if (str_index_map.find(list2[i]) != str_index_map.end()) {
                if (str_index_map[list2[i]] + i < minimum_index_sum) {
                    minimum_index_sum = str_index_map[list2[i]] + i;
                    res.clear();
                    res.push_back(list2[i]);
                }
                else if (str_index_map[list2[i]] + i == minimum_index_sum)
                    res.push_back(list2[i]);
            }
        }
        return res;
    }
};
