//#include "circle.h"

const double pi = 3.141592653589793;

bool operator ==(const Circle &c1, const Circle &c2)
{
	return (c1.r == c2.r)&&(c1.x == c2.x)&&(c1.y == c2.y);
}

bool operator !=(const Circle &c1, const Circle &c2)
{
	return (c1.r != c2.r)||(c1.x != c2.x )||(c1.y != c2.y);
}

Circle & Circle::operator =(const Circle &c)
{
	r = c.r;
	x = c.x;
	y = c.y;
	return *this;
}

Circle & Circle::operator +=(const Circle & c)
{
	r += c.r;
	x += c.x;
	y += c.y;
	return *this;
}

Circle & Circle::operator *=(const Circle & c)
{
	r *= c.r;
	x *= c.x;
	y *= c.y;
	return *this;
 } 
 
 Circle & Circle::operator *=(const int & k)
 {
 	r *= k;
 	x *= k;
 	y *= k;
 	return *this;
 }
 
 Circle & Circle::operator ++()
 {
 	++ r;
 	return *this;
  } 
  
Circle Circle::operator ++(int)
{
	Circle _t(*this);
	++ r;
	return _t;
 } 
void Circle::GetOrigin(double *x, double *y)
{
	*x = this->x;
	*y = this->y;
 } 
 
void Circle::SetOrigin(double x, double y)
{
	this->x = x;
	this->y = y;
}

double Circle::GetRadius()
{
	return r;
}

void Circle::SetRadius(double r)
{
	this->r = r;
}

double Circle::GetPerimeter()
{
	return 2*pi*r;
}

double Circle::GetArea()
{
	return pi*r*r;
 } 
