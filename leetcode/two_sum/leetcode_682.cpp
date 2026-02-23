class Solution {
public:
    int calPoints(vector<string>& operations) {
        vector<int> score;
        for (string& op : operations) {
            if (op == "+") {
                int len = score.size();
                score.push_back(score[len - 1] + score[len - 2]);
            }
            else if (op == "D") {
                int len = score.size();
                score.push_back(score[len - 1] * 2);
            }          
            else if (op == "C") {
                score.pop_back();
            }    
            else {
                score.push_back(std::stoi(op));
            }              
        }

        int res = 0;
        for (int s : score)
            res += s;
        return res;
    }
};
