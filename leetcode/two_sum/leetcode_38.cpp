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

// Recurrence solution
class Solution {
public:
    string countAndSay(int n) {
        if(n==1) 
            return "1";

        int count = 1;
        string res="", str=countAndSay(n-1);
        for(int i=0; i<str.size(); i++){
            if(str[i] == str[i+1]){     // If pos str[pos] is equal to the string length,                               
                count++;                // the function returns a reference to the null character that follows
                continue;               // the last character in the string (which should not be modified).
            }
            else{
                res += to_string(count)+str[i];
                count = 1;
            }
        }
       return res;
    }
};

// SImilar thought as above
class Solution {
public:
    string countAndSay(int n) {
        vector<string> log(n);
        log[0] = "1";
        
        for (int i = 1; i < n; i++) {
            string lastStr = log[i - 1], curStr;
            int count = 1;
            for (int i = 1; i < lastStr.size(); i++) {
                if (lastStr[i] == lastStr[i-1])
                    count++;
                else {
                    curStr += to_string(count) + lastStr[i - 1];
                    count = 1;
                }
            }
            curStr += to_string(count) + lastStr.back();
            log[i] = curStr;
        }
        return log.back();
    }
};
