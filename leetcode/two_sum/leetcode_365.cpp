// use BFS, interesting. Refer to the discussion section
class Solution {
public:
    pair<int, int> op(int type, const pair<int, int>& state, int x, int y){
        switch(type){
            case 0: 
                return pair(x, state.second);  // fill x-jug
            case 1: 
                return pair(state.first, y);  // fill y-jug
            case 2: 
                return pair(0, state.second);  // pour away x-jug
            case 3:
                return pair(state.first, 0);  // pour away y-jug
            case 4:{
                int move = min(state.first, y - state.second);  // move x-jug water to y-jug as much as possible
                return pair(state.first - move, state.second + move);
            } 
            case 5:{
                int move = min(x - state.first, state.second);  // mvoe y-jug water to x-jug as much as possible
                return pair(state.first + move, state.second - move);
            }
        }
        return pair(0, 0);
    }

    bool canMeasureWater(int x, int y, int z) {
        if (x + y < z)
            return false;
        
        unordered_set<string> visited;
        queue<pair<int, int>> stateQueue;
        stateQueue.push(pair(0, 0));

        while (!stateQueue.empty()){
            pair<int, int> curState = stateQueue.front();
            stateQueue.pop();
            if (curState.first + curState.second == z)
                return true;
            
            for (int i = 0; i < 6; i++){
                pair<int, int> nextState = op(i, curState, x, y);
                string stateStr = to_string(nextState.first) + "_" + to_string(nextState.second);
                if (visited.find(stateStr) != visited.end())
                    continue;
                visited.insert(stateStr);
                stateQueue.push(nextState);
            }
        }
        return false;
    }
};

// mathematical solution
class Solution {
public:
    bool canMeasureWater(int x, int y, int z) {
        if (x + y < z) {
            return false;
        }
        if (x == 0 || y == 0) {
            return z == 0 || x + y == z;
        }
        return z % gcd(x, y) == 0;
    }
};
