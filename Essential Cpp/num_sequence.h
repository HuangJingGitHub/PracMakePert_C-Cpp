#include <iostream>
#include <vector>

using namespace std;

class num_sequence
{
	public:
		virtual ~num_sequence() { };
		
		virtual int         elem(int pos) const = 0;
		virtual const char* what_am_i() const = 0;
		static int          max_elems() { return _max_elems; };
		virtual ostream&    print(ostream &os = cout) const = 0;
		
	    friend ostream& operator<<(ostream &os, const num_sequence &ns);
	protected:
		virtual void        gen_elems(int pos) const = 0;
		bool                check_integrity(int pos, int size) const;
		
		const static int    _max_elems = 1024;
};

class Fibonacci : public num_sequence
{
	public:
		Fibonacci(int len = 1, int beg_pos = 1)
		: _length(len), _beg_pos(beg_pos){ }
		
		virtual int          elem(int pos) const;
		virtual const char*  what_am_i() const { return "Fibonacci";}
		virtual ostream&     print(ostream &os = cout) const; 
		int                  length() const { return _length;}
		int                  beg_pos() const { return _beg_pos;}
	protected:
		virtual void         gen_elems(int pos) const;
		int                  _length;
		int                  _beg_pos;
	    static vector<int>   _elems;
};

vector<int> Fibonacci::_elems;  // static varible in class must be initialized outside.
