class Solution {
public:
    int minimumTeachings(int n, vector<vector<int>>& languages, vector<vector<int>>& friendships) {
        unordered_map<int, int> languagesUser;
        unordered_set<int> uncommunicablePeople;
        
        for (int i = 0; i < friendships.size(); i++) {
            int person_1 = friendships[i][0], person_2 = friendships[i][1];
            unordered_set<int> languagePool1(languages[person_1 - 1].begin(), languages[person_1 - 1].end());
            bool communicable = false;
            for (int lg : languages[person_2 - 1])
                if (languagePool1.find(lg) != languagePool1.end()) {
                    communicable = true;
                    break;
                }
            if (communicable)
                continue;
            else {
                uncommunicablePeople.insert(person_1);
                uncommunicablePeople.insert(person_2);
            }
        }
        
        for (int person : uncommunicablePeople) {
            for (int lg : languages[person - 1]) {
                languagesUser[lg]++;
            }
        }
        
        int maxUserNum = 0;
        for (auto it = languagesUser.begin(); it != languagesUser.end(); it++)
            maxUserNum = max(maxUserNum, it->second);
        
        return uncommunicablePeople.size() - maxUserNum;
    }
};
