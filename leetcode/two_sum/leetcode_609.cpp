class Solution {
public:
    vector<vector<string>> findDuplicate(vector<string>& paths) {
        vector<vector<string>> res;
        map<string, vector<string>> contents_to_paths;
        
        for (string& path : paths) {
            int idx = 0;
            while (path[idx] != ' ') {
                idx++;
            }
            string directory = path.substr(0, idx) + "/";

            idx++;
            int file_start = idx, content_start = 0;
            while (idx < path.size()) {
                while (path[idx] != '(')
                    idx++;
                string file_name = path.substr(file_start, idx - file_start);
                content_start = idx + 1;
                while (path[idx] != ')')
                    idx++;
                string content = path.substr(content_start, idx - content_start);
                contents_to_paths[content].push_back(directory + file_name);

                idx += 2;
                file_start = idx;
            }
        } 
        for (auto it = contents_to_paths.begin(); it != contents_to_paths.end(); it++) {
            if (it->second.size() > 1)
                res.push_back(it->second);
        }

        return res;
    }
};
