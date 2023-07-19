class Solution {
public:
    vector<int> constructRectangle(int area) {
        int refW = sqrt(area);

        while (area % refW != 0)
            refW--;
        return {area / refW, refW};
    }
};
