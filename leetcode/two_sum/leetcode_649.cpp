// greedy strategy, manipulation on string, slow.
class Solution {
public:
    string predictPartyVictory(string senate) {
        int R_BanNum = 0, D_BanNum = 0, R_Num, D_Num;
        bool victory = false;

        while (!victory){
            int sLen = senate.size();
            R_Num = 0;
            D_Num = 0;
            for (int i = 0, shift = 0; i < sLen-shift; ){
                if (senate[i] == 'R'){
                    if (D_BanNum > 0){
                        senate.erase(i, 1);
                        shift++;
                        D_BanNum--;
                    }
                    else{
                        R_BanNum++;
                        R_Num++;
                        i++;
                    }
                }
                else{
                    if (R_BanNum > 0){
                        senate.erase(i, 1);
                        shift++;
                        R_BanNum--;
                    }
                    else{
                        D_BanNum++;
                        D_Num++;
                        i++;
                    }
                }
            }
            if (R_Num == 0 || D_Num == 0)
                victory = true;
        }

        return R_Num == 0 ? "Dire" : "Radiant";
    }
};

// use flag vector instead of manipulating the original string for fast speed.
class Solution {
public:
    string predictPartyVictory(string senate) {
        int R_BanNum = 0, D_BanNum = 0, R_Num, D_Num;
        vector<int> validSentaor(senate.size(), 1);   // 1 stands for i-th senator is not banned.
        bool victory = false;

        while (!victory){
            R_Num = 0;
            D_Num = 0;
            for (int i = 0; i < senate.size(); i++){
                if (!validSentaor[i])
                    continue;

                if (senate[i] == 'R'){
                    if (D_BanNum > 0){
                        validSentaor[i] = 0;
                        D_BanNum--;
                    }
                    else{
                        R_BanNum++;   // Ban execucation from Radiant party senator.
                        R_Num++;
                    }
                }
                else{
                    if (R_BanNum > 0){
                        validSentaor[i] = 0;
                        R_BanNum--;
                    }
                    else{
                        D_BanNum++;
                        D_Num++;
                    }
                }
            }
            if (R_Num == 0 || D_Num == 0)
                victory = true;
        }

        return R_Num == 0 ? "Dire" : "Radiant";
    }
};
