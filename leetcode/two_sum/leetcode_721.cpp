class UnionFind {
public:
    vector<int> parent;

    UnionFind(int n) {
        parent.resize(n);
        for (int i = 0; i < n; i++)
            parent[i] = i;
    }

    void unionSet(int idx_1, int idx_2) {
        parent[find(idx_2)] = find(idx_1);
    }

    int find(int idx) {
        if (parent[idx] != idx)
            parent[idx] = find(parent[idx]);
        return parent[idx];
    }
};

class Solution {
public:
    vector<vector<string>> accountsMerge(vector<vector<string>>& accounts) {
        map<string, int> email_to_idx;
        map<string, string> email_to_name;

        int email_cnt = 0;
        for (auto& account : accounts) {
            string& name = account[0];
            int size = account.size();
            for (int i = 1; i < size; i++) {
                string& email = account[i];
                if (email_to_idx.count(email) == 0) {
                    email_to_idx[email] = email_cnt++;
                    email_to_name[email] = name;
                }
            }
        }

        UnionFind uf(email_cnt);
        for (auto& account : accounts) {
            string& first_email = account[1];
            int first_idx = email_to_idx[first_email];
            int size = account.size();
            for (int i = 2; i < size; i++) {
                string& next_email = account[i];
                int next_idx = email_to_idx[next_email];
                uf.unionSet(first_idx, next_idx);
            }
        }

        map<int, vector<string>> idx_to_email;
        for (auto& [email, _] : email_to_idx) {
            int idx = uf.find(email_to_idx[email]);
            idx_to_email[idx].emplace_back(email);
        }

        vector<vector<string>> merged;
        for (auto& [_, email] : idx_to_email) {
            sort(email.begin(), email.end());
            string& name = email_to_name[email[0]];
            vector<string> account;
            account.emplace_back(name);
            for (auto& single_email : email) 
                account.emplace_back(single_email);
            merged.emplace_back(account);
        }
        return merged;
    }
};
