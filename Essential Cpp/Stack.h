#include <string>
#include <vector>
#include <algorithm>
using namespace std;

class Stack
{
	public:
		bool push(const string&);
		bool pop(string &elem);
		bool peek(string &elem);
		bool find(const string &elem);
		bool empty() const { return _stack.empty();}
		bool full() const { return _stack.size() == _stack.max_size();}
		// string::max_size() or vector::max_size() is a quite large number.
		int size() const { return _stack.size();}
		int count(const string &elem);
	private:
		vector<string> _stack;
 };
