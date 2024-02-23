class Solution {
public:
    unordered_map<int, string> dataBase;
    int id_ = 0;
    // Encodes a URL to a shortened URL.
    string encode(string longUrl) {
        id_++;
        dataBase[id_] = longUrl;
        return string("heep://tinyurl.com/") + to_string(id_);
    }

    // Decodes a shortened URL to its original URL.
    string decode(string shortUrl) {
        int p = shortUrl.rfind('/') + 1;
        int key = stoi(shortUrl.substr(p, int(shortUrl.size()) - p));
        return dataBase[key];
    }
};

// Your Solution object will be instantiated and called as such:
// Solution solution;
// solution.decode(solution.encode(url));
