#include <iostream> 
#include <string>

using namespace std;

enum Mon{Jan = 1, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec};
class Date
{
	public:
		Date(const int year, const Mon month, int day):
			_year(year), _month(month), _day(day)  {};
		friend ostream & operator << (ostream & os,const Date& date);
	protected:
		int _year;
		Mon _month;
		int _day;
};

ostream & operator << (ostream &os,const Date& date)
{
	os << "(" << date._year << "-" << date._month << "-" << date._day << ")";
}

class LibMat
{
	public:
		LibMat() 
		{
			cout << "LibMat::LibMat() default constructor£¡\n";
		}
		virtual ~LibMat()
		{
			cout << "LibMat::~LibMat() destructor!\n";
		}
		virtual void print() const
		{
			cout << "LibMat::print() -- A LibMat object!\n";
		}
		
};

void print(const LibMat &mat)
{
	cout << "in global print(): about to print mat.print()\n";
	mat.print();
}

class Book : public LibMat
{
	public:
		Book(const string &title, const string &author):
			_title(title), _author(author)
			{
				cout << "Book::Book(" << _title << " ," << _author << ") constructor\n";
			}
			
	    virtual ~Book()
	    {
	    	cout << "Book::~Book() destructor!\n";
		}
		
		virtual void print() const
		{
			cout << "Book::print() -- A Book object!\n"
				 << "Title: "   << _title  << "\n"
				 << "Author: "  << _author << endl;
		}
		
		const string& title() const
		{
			return _title;
		}
		const string& author() const
		{
			return _author;
		}
		
	protected:
		string _title;
		string _author;
};

class Magazine : public LibMat
{
	public:
		Magazine(const string& title,
		         const Date& date):
		         	_title(title), _date(date)
		         	{
		         		cout << "Magazine::Maginine(" << _title <<", " << _date << ") constructor\n";
					 }
		virtual ~Magazine()
		{
			cout << "Magazine::~Magazine destructor!\n";
		}
	protected:
		string _title;
		Date _date;
};

class AudioBook : public Book 
{
	public:
		AudioBook(const string& title, 
		          const string& author, 
				  const string& narrator):
		          Book(title, author), _narrator(narrator) 
				  {
				  	cout << "AudioBook::AudioBook(" 
					     << _title  << ", "
					     << _author << ", "
					     << _narrator << ")." << endl;
				  }
		~AudioBook()
		{
			cout << "AudioBook::~AudioBook() destructor!\n";
		}
        virtual void print() const 
        {
        	cout << "AudioBook::print() -- An AudioBook object!\n"
        		 << "Title: "   << _title
        		 << " Author: "  << _author
				 << " Narrator: " << _narrator << endl;  
		}
		
		const string& narrator() const
		{
			return _narrator;
		}
		
	protected:
		string _narrator;
};

int main()
{
	cout << '\n' << "Creating a LibMat object to print()\n";
	LibMat libmat;
	print(libmat);
	
	cout << "\n" << "Creating a Book object to print()\n";
	Book b("The Ordinary World", "LuYao");
	print(b);
	
	cout << "\n" << "Creating an AudioBook object to print()\n";
	AudioBook ab("Dream of the Red Chamber","Cao Xueqin","Someone");
	print(ab);
	
	cout << "\n" << "Creating a Magazine object to print()\n";
	Magazine m("Times",Date(2018,Jan,1));
	print(m);
}
