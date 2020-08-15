class Solution {
public:
    int numDistinct(string s, string t) {
        vector<long long int> matched(t.size() + 1, 0);
        matched[0] = 1;
        for (int i = 0; i < s.size(); i++)
            for (int j = t.size(); j > 0; j--){
                if (s[i] == t[j-1])
                    matched[j] += matched[j-1];
            }
        return matched.back();
    }
};


// recursion with memeorization
class Solution {
public:
    int numDistinct(string s, string t) {
        map<string, int> interResMap;
        return subDistinct(s, t, 0, 0, interResMap);
    }

    int subDistinct(string& s, string& t, int sStart, int tStart, map<string, int>& intermediateRes){
        if (tStart == t.length())
            return 1;
        if (sStart == s.length())
            return 0;

        string key = to_string(sStart) + "_" + to_string(tStart);   // use to_string, a native way is to coast int to char, but that is NOT correct as sStart, tStart can easily exceed ASCII range!!!
        map<string, int>::iterator it = intermediateRes.find(key);
        if (it != intermediateRes.end()){
            return it->second;
        }

        int res;
        if (s[sStart] == t[tStart])
            res = subDistinct(s, t, sStart+1, tStart+1, intermediateRes) + subDistinct(s, t, sStart+1, tStart, intermediateRes);
        else
            res = subDistinct(s, t, sStart+1, tStart, intermediateRes);

        intermediateRes.emplace(key, res);
        return res;
    }
};
