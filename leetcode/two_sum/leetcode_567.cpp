class Solution {
public:
    bool checkInclusion(string s1, string s2) {
        int len_1 = s1.size(), len_2 = s2.size();
        if (len_1 > len_2)
            return false;
        
        unordered_map<char, int> frequency_1, frequency_2;
        for (auto letter : s1)
            frequency_1[letter]++;
        for (int i = 0; i < len_1; i++)
            frequency_2[s2[i]]++;
        
        for (int idx = len_1 - 1; idx < len_2; idx++) {
            if (checkMapEquality(frequency_1, frequency_2) == true)
                return true;

            frequency_2[s2[idx + 1]]++;
            frequency_2[s2[idx - len_1 + 1]]--;
        }
        return false;
    }

    bool checkMapEquality(unordered_map<char, int>& map_1, unordered_map<char, int>& map_2) {
        for (auto itr = map_1.begin(); itr != map_1.end(); itr++) {
            char key_char = itr->first;
            int value = itr->second;
            if (map_2.find(key_char) == map_2.end())
                return false;
            if (map_2[key_char] != value)
                return false;
        }
        return true;
    }
};
