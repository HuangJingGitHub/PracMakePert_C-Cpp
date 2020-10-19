/*
// Definition for a Node.
class Node {
public:
    int val;
    vector<Node*> neighbors;
    
    Node() {
        val = 0;
        neighbors = vector<Node*>();
    }
    
    Node(int _val) {
        val = _val;
        neighbors = vector<Node*>();
    }
    
    Node(int _val, vector<Node*> _neighbors) {
        val = _val;
        neighbors = _neighbors;
    }
};
*/

class Solution {
public:
    Node* cloneGraph(Node* node) {
        unordered_set<int> valBuilt;
        Node* res;
        if (node == NULL)
            return res;
        
        res = new Node(node->val, node->neighbors);
        valBuilt.insert(node->val);

        DFS_build(res, valBuilt);
        return res;
    }

    void DFS_build(Node* source, unordered_set<int>& built){
        for (Node* nodePt : source->neighbors){
            if (built.find(nodePt->val) == built.end()){
                built.insert(nodePt->val);
                cout << nodePt->val << " OK\n";
                nodePt = new Node(nodePt->val, nodePt->neighbors);
                DFS_build(nodePt, built);
            }
        }
    }
};
