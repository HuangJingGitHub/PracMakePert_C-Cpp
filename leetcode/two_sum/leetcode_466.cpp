class Solution {
public:
	int getMaxRepetitions(string s1, int n1, string s2, int n2) {
		 unordered_map<int, pair<int, int>> hash;
			int res = 0;
			int len1 = s1.size();
			int len2 = s2.size();
			int index1 = 0;
			int index2 = 0;
			while (index1 / len1 < n1) {
				if (index1%len1 == len1 - 1) {
					if (hash.count(index2%len2)) {
						int cycleLen = index1 - hash[index2%len2].first;
						int cycNum2 = (index2 - hash[index2%len2].second) / len2;  //每个循环有几个s2
						int cycNum1 = cycleLen / len1;  //每个循环有几个S1
						int cycle = (n1 - 1 - index1 / len1) / cycNum1;
						res += cycle * cycNum2;
						index1 += cycle * cycleLen;
					}
					else {
						hash[index2%len2].first = index1;
						hash[index2%len2].second = index2;
					}
			}
			
			if (s1[index1%len1] == s2[index2%len2]) {
				if (index2 % len2 == len2 - 1)
					res++;
				index2++;
			}
			index1++;
		}
		return res / n2;
	}
};
