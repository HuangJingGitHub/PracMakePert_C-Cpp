class Solution {
public:
    string smallestEquivalentString(string s1, string s2, string baseStr) {
        vector<set<char>> adjacency(26);
        for (int i = 0; i < s1.size(); i++) {
            int num_1 = s1[i] - 'a', num_2 = s2[i] - 'a';
            adjacency[num_1].insert(s2[i]);
            adjacency[num_2].insert(s1[i]);
        }

        vector<char> lex_smallest(26);
        for (int i = 0; i < 26; i++) 
            lex_smallest[i] = 'a' + i;

        for (int i = 1; i < 26; i++) {
            if (adjacency[i].size() == 0)
                continue;

            char smallest_eqv = 'a' + i;
            for (char c : adjacency[i]) 
                smallest_eqv = smallest_eqv < c ? smallest_eqv : c;
            if (smallest_eqv < 'a' + i) {
                lex_smallest[i] = lex_smallest[smallest_eqv - 'a'];
                continue;
            }

            queue<char> level;
            level.push(smallest_eqv);
            vector<bool> visited(26, false);
            while (!level.empty()) {
                char front_c = level.front();
                level.pop();
                visited[front_c - 'a'] = true;
                for (char c : adjacency[front_c - 'a']) {
                    smallest_eqv = smallest_eqv < c ? smallest_eqv : c;
                    if (!visited[c - 'a'])
                        level.push(c);
                }
            }
            lex_smallest[i] = smallest_eqv;
        }
        
        vector<char> base_eqv(baseStr.size());
        for (int i = 0; i < baseStr.size(); i++)
            base_eqv[i] = lex_smallest[baseStr[i] - 'a'];
        
        string res;
        for (char c : base_eqv)
            res += c;
        return res;
    }
};
