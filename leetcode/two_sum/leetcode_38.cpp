class Solution {
public:
    string countAndSay(int n) {
    vector<char> seqChar;
    vector<int> seqTimes;
    string sequence = "1";
    if (n==1)
        return "1";

    for (int i = 1; i < n; i++){
        for (int j = 0; j < sequence.size(); j++){
            if (j == 0){
                seqChar.push_back(sequence[j]);
                seqTimes.push_back(1);
            }
            else{
                if (sequence[j] == sequence[j-1]){
                    seqTimes.back()++;
                }
                else{
                    seqChar.push_back(sequence[j]);
                    seqTimes.push_back(1);
                }
            }

        }
        sequence = "";
        for (int k = 0; k < seqChar.size(); k++){
            sequence += to_string(seqTimes[k]);
            sequence += seqChar[k];            
        }
        seqChar.clear();
        seqTimes.clear();
    }
    return sequence;
    }
};
