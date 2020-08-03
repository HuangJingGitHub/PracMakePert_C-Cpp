class Solution {
public:
    vector<int> findClosestElements(vector<int>& arr, int k, int x) {
        // Cases where x is not between two elements in arr.
        if (x <= arr[0])
            return vector<int>(arr.begin(), arr.begin() + k);
        else if (x >= arr.back())
            return vector<int>(arr.end() - k, arr.end());

        int left = 0, right = arr.size() - 1, mid;
        for (; left <= right; ){
            mid = left + (right - left) / 2;
            if (arr[mid] == x){
                left = mid;
                right = mid;
                break;
            }
            else if (x < arr[mid])
                right = mid - 1;
            else 
                left = mid + 1;
        }
        // Make the closest value is between left and right
        left = min(left, right);
        left = abs(arr[left] - x) <= abs(arr[left+1] - x) ? left : left + 1;
        right = left + 1;
        left--;

        for (int i = 1; i < k; i++){
            if (left < 0)
                right++;
            else if (right >= arr.size())
                left--;
            else if (abs(arr[left] - x) <= abs(arr[right] - x))
                left--;
            else
                right++;
        }
        return vector<int>(arr.begin() + left + 1, arr.begin() + right);
    }
};
