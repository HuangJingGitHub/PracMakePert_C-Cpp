class Circle
{
	public:
		Circle(double ri, double xi, double yi):
		        x (xi) , y (yi)
				{ r = ri > 0 ? ri : 0;
				 } 
		friend bool operator ==(const Circle &c1, const Circle &c2);
		friend bool operator !=(const Circle &c1, const Circle &c2);
		Circle & operator =(const Circle & c);
		Circle & operator +=(const Circle & c);
		Circle & operator *=(const Circle & c);
		Circle & operator *=(const int & k);
		Circle & operator ++();
		Circle operator ++(int);
		
		void GetOrigin(double *x, double *y);
		void SetOrigin(double x, double y);
		double GetRadius();
		void SetRadius(double r);
		double GetPerimeter();
		double GetArea();
	private:
		double r, x, y;
};
