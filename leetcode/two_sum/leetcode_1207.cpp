class Solution {
public:
    bool uniqueOccurrences(vector<int>& arr) {
        map<int, int> occurrencesMap;
        set<int> occurrencesSet;

        for (int& num : arr)
            occurrencesMap[num]++;
        for (auto it = occurrencesMap.begin(); it != occurrencesMap.end(); it++) {
            if (occurrencesSet.find(it->second) != occurrencesSet.end())
                return false;
            else
                occurrencesSet.insert(it->second);
        }
        return true;
    }
};


// or an more efficient way
class Solution {
public:
    bool uniqueOccurrences(vector<int>& arr) {
        map<int, int> occurrencesMap;
        set<int> occurrencesSet;

        for (int& num : arr)
            occurrencesMap[num]++;
        for (auto it = occurrencesMap.begin(); it != occurrencesMap.end(); it++)
            occurrencesSet.insert(it->second);
        return occurrencesMap.size() == occurrencesSet.size();
    }
};
