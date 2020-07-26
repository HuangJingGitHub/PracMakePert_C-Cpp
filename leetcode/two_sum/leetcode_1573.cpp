// Very elegent and compact code. For postorder traversal array,
// check recursively: values in the left branch < root value && values in the right branch > root value
class Solution {
public:
    bool verifyPostorder(vector<int>& postorder) {
        if (postorder.size() < 3)
            return true;

        return checkPostBST(postorder, 0, postorder.size()-1);
    }

    bool checkPostBST(vector<int>& postorder, int start, int end)
    {
        if (start >= end)
            return true;
        
        int left = start;
        while (postorder[left] < postorder[end] && left < end){  // find the boundary of left and right tree values
            left++;
        }

        for(int i = left+1;i < end; i++){
            if (postorder[i] < postorder[end])
                return false;
        }

        return checkPostBST(postorder, start, left-1) && checkPostBST(postorder, left, end-1);
    }
};
