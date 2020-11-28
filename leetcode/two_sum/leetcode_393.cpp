// Refer to the offical solution, handle details.
class Solution {
public:
    void getBinnaryRep(vector<int>& res, int num){
        vector<int> tempRes;
        while (num / 2 != 0){
            tempRes.push_back(num % 2);
            num /= 2;
        }
        tempRes.push_back(1);
        if (tempRes.size() >= 8)
            for (int i = 0; i < 8; i++)
                res[i] = tempRes[tempRes.size() - 1 - i];
        else
            for (int i = 0; i < tempRes.size(); i++)
                res[8 - tempRes.size() + i] = tempRes[tempRes.size() - 1 - i];
    }

    bool validUtf8(vector<int>& data) {
        int bytesToProcessNum = 0;

        for (int i = 0; i < data.size(); i++){
            vector<int> binRep(8);
            getBinnaryRep(binRep, data[i]);

            if (bytesToProcessNum == 0){
                for (int i = 0; i < 8; i++){
                    if (binRep[i] == 0)
                        break;
                    bytesToProcessNum++;
                }

                if (bytesToProcessNum == 0)
                    continue;
                if (bytesToProcessNum > 4 || bytesToProcessNum == 1)
                    return false; 
            }
            else{
                if (!(binRep[0] == 1 && binRep[1] == 0))
                    return false;
            }
            bytesToProcessNum--;
        }
        return bytesToProcessNum == 0;
    }
};
