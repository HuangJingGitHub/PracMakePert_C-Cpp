#include "Stack.h"

bool Stack::pop(string &elem)
{
	if (empty())
		return false;
	elem = _stack.back();
	_stack.pop_back();
	return true;
}

bool Stack::peek(string &elem)
{
	if (empty())
		return false;
	elem = _stack.back();
	return true;
}

bool Stack::push(const string &elem)
{
	if (full())
		return false;
	_stack.push_back(elem);
	return true;
}

bool Stack::find(const string &elem)
{
	return ::find(_stack.begin(), _stack.end(), elem) != _stack.end();
}

int Stack::count(const string &elem)
{
	return ::count(_stack.begin(), _stack.end(), elem);
}
