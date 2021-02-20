// Consider feasible mapped string length. Quite cumbersome anyway.
class Solution {
public:
    bool patternMatching(string pattern, string value) {
        if (pattern.empty() && value.empty())
            return true;
        else if (pattern.empty() && !value.empty())
            return false;
        else if (value.empty()) {
            for (int i = 1; i < pattern.size(); i++)
                if (pattern[i] != pattern[i - 1])
                    return false;
            return true;
        }
        
        int patternLen = pattern.size(), valueLen = value.size(), 
            aNum = 0, bNum = 0;
        vector<int> aStrLen;
        for (char c : pattern) {
            if (c == 'a')
                aNum++;
            else
                bNum++;
        }

        if (aNum == 0) {
            if (valueLen % bNum != 0)
                return false;
            else
                aStrLen.push_back(0);
        }
        else if (bNum == 0) {
            if (valueLen % aNum != 0)
                return false;
            else
                aStrLen.push_back(valueLen / aNum);
        }
        else {
            for (int i = 0; i * aNum <= valueLen; i++) {
                if ((valueLen - i * aNum) % bNum == 0)
                    aStrLen.push_back(i);
            }
        }


        for (int aCurStrLen : aStrLen) {
            int bCurStrLen = 0;
            if (bNum != 0)
                bCurStrLen = (valueLen - aCurStrLen * aNum) / bNum;
            string aStr, bStr;
            if (pattern[0] == 'a') {
                aStr = value.substr(0, aCurStrLen);
                int i = 1;
                while (pattern[i] == 'a')
                    i++;
                bStr = value.substr(i * aCurStrLen, bCurStrLen);
            }
            else {
                bStr = value.substr(0, bCurStrLen);
                int i = 1;
                while (pattern[i] == 'b')
                    i++;
                aStr = value.substr(i * bCurStrLen, aCurStrLen);               
            }

            int idx = 0;
            string curStr;
            for (char c : pattern) {
                if (c == 'a') {
                    curStr = value.substr(idx, aCurStrLen);
                    if (curStr != aStr)
                        break;
                    idx += aCurStrLen;
                }
                else {
                    curStr = value.substr(idx, bCurStrLen);
                    if (curStr != bStr)
                        break;
                    idx += bCurStrLen;
                }
            }
            if (idx == valueLen)
                return true;
        }
        return false;
    }
};
