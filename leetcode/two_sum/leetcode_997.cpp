class Solution {
public:
    int findJudge(int n, vector<vector<int>>& trust) {
        vector<int> trustPersonNum(n + 1, 0), beingTrustedNum(n + 1, 0);

        for (auto& trustRelation : trust) {
            int truster = trustRelation[0], trustee = trustRelation[1];
            trustPersonNum[truster]++;
            beingTrustedNum[trustee]++;
        }

        vector<int> judgeCandidate;
        for (int i = 1; i <= n; i++) {
            if (trustPersonNum[i] == 0 && beingTrustedNum[i] == n - 1)
                judgeCandidate.push_back(i);
        }

        return judgeCandidate.size() == 1 ? judgeCandidate[0] : -1;
    }
};
