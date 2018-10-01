
class num_sequence
{
	public:
		virtual ~num_sequence() { };
		
		virtual int         elem(int pos) const = 0;
		virtual const char* what_am_i const = 0;
		static int          max_elems(int pos) const = 0;
		virtual             ostream& print(ostream &os = cout) const = 0;
		
	    friend ostream& operator<<(ostream &os, const num_sequence &ns);
	protected:
		virtual void        gen_elems(int pos) const = 0;
		bool                check_integrity(int pos) const;
		
		const static int    _max_elems = 1024;
}
