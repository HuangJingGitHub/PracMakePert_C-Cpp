#include <iostream>
#include <string>
#include <set>
#include <vector>
#include <algorithm>

using namespace std;

void solution(string input) {
    set<char> product_set;
    int idx = 0;
    while (input[idx] != '|') {
        idx++;
    }

    idx++;
    while (idx < input.size()) {
        if (input[idx] != ' ')
            product_set.insert(input[idx++]);
    }
        
    vector<char> product_vec;
    for (char product : product_set)
        product_vec.push_back(product);
    sort(product_vec.begin(), product_vec.end());

    int price_difference = 0;
    int dist = product_vec.back() - product_vec.front();
    int start_price_dif = product_vec.front() - 'A' + 1;
    for (int i = 0; i < dist; i++) {
        price_difference += (start_price_dif + i) * 10;
    }
    
    for (char product : product_vec)
        cout << product << " ";
    cout << price_difference;
}

int main() {
    string input = "5|F B B C D";
    solution(input);
    return 0;
}


#include <iostream>
#include <string>
using namespace std;

struct State; // x y theta
bool IsSafe(const State& state); // provided

/*
0				    																					  n-1
   a1 b1            a2 b2
   --------------------
   | | i i+1        | |
----- - - -------------------------------------------------
   |-|--------------|-| 
   a3 b3            a4 b4
                            mid
------|-------|------------|---------------|---------------

*/

/*
0																									  n-1
-------|------|------------|-------------|--------------

                          | mid
                          
              |
                                         |
                                         
       |
                     |       
                                  |
                                                |
                                                
*/




//---|---|-------|-------------|--------------------------

bool IsPathSafe(const std::vector<State> &path) {
  // for (auto state : path)
  //   if (!IsSafe(state))
  //     return false;
  // return true;

  if (path.size() == 1)
    return IsSafe(path.front());
  
  int mid_idx = path.size() / 2, len = path.size();
  queue<vector<int>> check_pair;
  check_pair.push({mid_idx, len});
  while (!check_pair.empty()) {
    vector<int> cur_pair = check_pair.front();
    mid_idx = cur_pair[0];
    len = cur_pair[1];

    State mid_state = path[mid_idx];
    if (!IsSafe(mid_state))
      return false;
    check_pair.push{mid_idx - len / 4， len / 2};
    check_pair.push{mid_idx + len / 4, len / 2};
  }
  
  return true;
}
/* 
  你必须定义一个 `main()` 函数入口。
  you must define a `main()` function entry.
*/
// given bicycle model,
// v_rear, angular velocity w, wheelbase L
// v_front = sqrt((v_rear^2 + L^2 * w^2)
// v_front = w x (r_rear + L) = v_rear + w x L
//
// m x n 棋盘格, (p,q)位置是obs.
// 左上到右下有多少种走法. （analytical expression）f(m, n) = f(m, n - 1) + f(m - 1, n)
//                                                      = C(m + n - 2, m - 1) 
// ---------
// Given F(m, n)
// G(m, n, (p, q)) = F(m, n) - F(p, q) * F(m - p + 1, n - q + 1)

#include <iostream>
#include <string>
#include <vector>
using namespace std;

int solution(int m, int n) {
  if (m <= 0 || n <= 0)
    return -1;
  
  vector<vector<int>> dp(m, vector<int>(n, 0));
  dp[0] = vector<int>(n, 1);
  for (int row = 1; row < m; row++) {
    dp[row][0] = 1;
  }
  /*if (p == 0) {
    for (int col = q; col < n; col++)
      dp[0][col] = 0;
  }
  if (q == 0) {
    for (int row = p; row < m; row++)
      dp[row][0] = 0;
  }*/

  // dp[p][q] = 0;
  for (int row = 1; row < m; row++) {
    for (int col = 1; col < n; col++) {
      // if (row == p && col == q)
      //    continue;
      
      dp[row][col] = dp[row][col - 1] + dp[row - 1][col];
    }
  }
  return dp.back().back();
}

int main() {
  int m = 3, n = 3;
  cout << solution(m, n) << endl;
  return 0;
}


#include <iostream>
#include <vector>

using namespace std;

class gomoku {
public:
    const int player_0 = 0;
    const int player_1 = 1;
    const int board_size = 15;
    vector<vector<int>> board;
    bool play_turn_0 = true;
    bool game_end = false;
    
    gomoku() {
        board = vector<vector<int>>(board_size, vector<int>(board_size, -1));
    }
    
    bool checkBoard(int row, int col) {
        if (row < 0 || row >= board_size 
            || col < 0 || col >= board_size) {
            cout << "Wrong arguments in checkBoard()";
            return false;
        }
        
        bool valid_layout = true;
        // check row
        if (col <= board_size - 5) {
            for (int i = 1; i < 5; i++) {
                if (board[row][col + i] != board[row][col]) {
                    valid_layout = false;
                    break;
                }
            }
        }
        if (valid_layout)
            return valid_layout;
        
        // check col
        if (row <= board_size - 5) {
            for (int i = 1; i < 5; i++) {
                if (board[row + i][col] != board[row][col]) {
                    valid_layout = false;
                    break;
                }
            }
        }        
        if (valid_layout)
            return valid_layout;
        
        // check left down diagonal 
        for (int i = 0; i < 5; i++) {
            int left_down_row = row + i, left_down_col = col - i;
            if (left_down_row >= board_size || left_down_row < 0
                || board[left_down_row][left_down_col] != board[row][col]) {
                valid_layout = false;
                break;
            }
        }
        if (valid_layout)
            return valid_layout;     

        // check right down diagonal 
        for (int i = 0; i < 5; i++) {
            int left_down_row = row + i, left_down_col = col + i;
            if (left_down_row >= board_size || left_down_row >= board_size
                || board[left_down_row][left_down_col] != board[row][col]) {
                valid_layout = false;
                break;
            }
        }
        return valid_layout;             
    }
    
    int checkWinner() {
        for (int row = 0; row < board_size; row++) {
            for (int col = 0; col < board_size; col++) {
                if (board[row][col] == -1)
                    continue;
                if (checkBoard(row, col))
                   return board[row][col]; 
            }
        }
        return -1;
    }
    
    void play() {
        cout << "Please input the position to place:\n";
        int place_row = -1, place_col = -1;
        cin >> place_row;
        cin >> place_col;
        // TODO: argument chcek- 1-arguments, 2-board position should be empty.
        
        // player 0 starts by default
        if (play_turn_0) {
            board[place_row][place_col] = player_0;
        }
        else {
            board[place_row][place_col] = player_1;
        }
        play_turn_0 = !play_turn_0;
        
        int winner = checkWinner();
        if (winner == -1)
            cout << "No winner appears.\n";
        else {
            game_end = true;
            cout << "The winner is " << winner << "\n";
        }
    }
    
    void printBoard() {
        for (auto& row : board) {
            for (auto& val : row)
                cout << val << ", ";
            cout << "\n";
        }
    }
};

int main() {
    gomoku test_gomoku;
    // cout << "Winner is player " << test_gomoku.checkWinner() << "\n";
    /*test_gomoku.printBoard();
    test_gomoku.play();
    test_gomoku.play();
    test_gomoku.printBoard(); */
    
    while (!test_gomoku.game_end) {
        test_gomoku.play();
    }
    
    return 0;
}
