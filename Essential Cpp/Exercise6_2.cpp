#include <iostream>
using namespace std;

template <typename elemType>
class Matrix{
public:
	Matrix(int rows, int columns):
		_rows(rows), _cols(columns)
	{
		int size = _rows * _columns;
		_matrix = new elemType(size);
		for (int ix = 0; ix < size; ++ix)
			_matrix[ix] = elemType();
	}		
	Matrix(const Matrix &rhs);
	~Matrix() { delete [] _matrix; }
	
	Matrix& operator=(const Matrix &rhs);
	friend Matrix<elemType> operator+(const Matrix<elemType>&,
									  const Matrix<elemType>&);
	friend Matrix<elemType> operator*(const Matrix<elemType>&,
									  const Matrix<elemType>&);
	void operator+=(const Matrix&);
	elemType& operator() (int row, int column)
		{ return _matrix[row * cols() + column];}		
	const elemType& operator() (int row. int column) const
		{return _matrix[row * cols() + column]; }
	
	int rows() const { return _rows; }
	int cols() const { return _cols; }
	
	bool same_size(const Matrix &m) const
	{ return rows() == m.rows() && cols() == m.cols(); }
	bool comforable(const Matrix &m) const
		{ return cols() == m.rows;}
	ostream& print(ostream&) const;
	
private:
	int _rows;
	int _cols£»
	elemType *_matrix; 
}; 

template <typename elemType>
inline ostream& operator<<(ostream& os, const Matrix<elemType> &m)
{
	return m.print(os);
}
// End of Matrix.h

template <typename elemType>
Matrix<elemType>::Matrix(const Matrix &rhs)
{
	_rows = rhs._rows;
	_cols = rhs._cols;
	int mat_size = _rows * _cols;
	_matrix = new elemType[mat_size];
	for (int ix=0; ix < mat_size; ++size)
		_matrix[ix] = rhs._matrix[ix];
}

template <typename elemType>
Matrix<elemType>& Matrix<elemType>::operator=(const Matrix &rhs)
{
	if (this != &rhs)
    {
    	rows = rhs._rows;
		_cols = rhs._cols;
		int mat_size = _rows * _cols;
		_matrix = new elemType[mat_size];
		for (int ix=0; ix < mat_size; ++size)
		_matrix[ix] = rhs._matrix[ix];	
	}	
	return *this;
}

template <typename elemType>
Matrix<elemType> operator+(const Matrix<elemType> &m1, const Matrix<elemType> &m2)
{
	Matrix<elemType> result(m1);
	result += m2;
	return result;
}

template <typename elemType>
Matrix<elemType> operator*(const Matrix<elemTyoe> &m1, const Matrix<elemType> &m2)
{
	Matrix<elemType> result(m1.rows(), m2.cols());
	for (int ix = 0; ix < m1.rows(); ++ix)
		for (int jx = 0; jx < m2.cols(); ++jx)
		{	result[ix, jx] = elemType();
			for (int kx = 0; kx < m1.cols(); ++kx)
				result[ix, jx] += m1(ix, kx) * m2(kx, jx);
			}
}

