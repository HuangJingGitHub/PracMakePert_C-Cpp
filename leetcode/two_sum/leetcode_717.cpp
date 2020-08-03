// see how many continuous ones there are from back 
class Solution {
public:
    bool isOneBitCharacter(vector<int>& bits) {
        int len = bits.size(), oneCount = 0, idx = len - 2;
        if (len == 0)
            return false;
        else if (len == 1)
            return true;
        else if (bits[len-2] == 0)
            return true;
        
        while(idx >= 0 && bits[idx] == 0)
            idx--;
        for ( ; idx >= 0 && bits[idx] == 1; idx--){
            oneCount++;
        }

        return oneCount % 2 == 0;
    }
};

// More elegent using decoding
class Solution {
public:
    bool isOneBitCharacter(vector<int>& bits) {
        int start = 0, len = bits.size();
        while (start <= len-2){
            if (bits[start] == 0)
                start++;
            else
                start += 2;
        }
        return start != len;
    }
};
