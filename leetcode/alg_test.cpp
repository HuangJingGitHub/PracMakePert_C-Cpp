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
