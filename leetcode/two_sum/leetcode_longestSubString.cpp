class Solution {
public:
    int lengthOfLongestSubstring(string s) {
        int globalMax = 1, localMax = 1, stringLength = s.length(), subStringLength;
        string subString;
        char buffer[100];
        
        for (int index = 0; index < stringLength; index++)
        {   
            for (int subIndex = index + 1; subIndex < stringLength; subIndex++)
            {
                subStringLength = s.copy(buffer, subIndex - index, index);
                buffer[subStringLength] = '\0';
                subString = buffer;
              //  std::cout << "subSting: " << subString << std::endl;
                if (subString.find(s[subIndex]) == std::string::npos)
                    localMax++;
                else
                    break;
            }
            
            if (localMax > globalMax)
                globalMax = localMax;
           // std::cout << s[index] << ": " << localMax << std::endl;
            localMax = 1;
        }
        if (stringLength == 0)
            return 0;
        return globalMax;
    }
};
