class Solution {
public:
    int computeArea(int A, int B, int C, int D, int E, int F, int G, int H) {
        int xMin = min(A, E), xMax = max(C, G),
            yMin = min(B, F), yMax = max(D, H),
            xTotal = C - A + G - E,
            yTotal = D - B + H - F,
            xRange = xMax - xMin,
            yRange = yMax - yMin;
        bool isOverlapped = (xRange < xTotal && yRange < yTotal);

        if (isOverlapped) {
            int xOverlap = xTotal - xRange, yOverlap = yTotal - yRange;
            return (C - A) * (D - B) + (G - E) * (H - F) - xOverlap * yOverlap;
        }
        return (C - A) * (D - B) + (G - E) * (H - F);
    }
};
