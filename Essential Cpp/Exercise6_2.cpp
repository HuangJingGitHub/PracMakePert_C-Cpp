#include <iostream>
#include <fstream>
using namespace std;

template <typename elemType>
class Matrix{
public:
	Matrix(int rows, int columns):
		_rows(rows), _cols(columns)
	{
		int size = _rows * _cols;
		_matrix = new elemType(size);
		for (int ix = 0; ix < size; ++ix)
			_matrix[ix] = elemType();
	}		
	Matrix(const Matrix &rhs);
	~Matrix() { delete [] _matrix; }
	
	Matrix& operator=(const Matrix &rhs);
	template <typename elemType1>
	friend Matrix<elemType1> operator+(const Matrix<elemType1>&,
									  const Matrix<elemType1>&);
	template <typename elemType1>
	friend Matrix<elemType1> operator*(const Matrix<elemType1>&,
									  const Matrix<elemType1>&);
	void operator+=(const Matrix&);
	elemType& operator() (int row, int column)
		{ return _matrix[row * cols() + column];}		
	const elemType& operator() (int row, int column) const
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
	int _cols;
	elemType *_matrix; 
}; 

template <typename elemType>
inline ostream& operator<<(ostream& os, const Matrix<elemType> &m)
{
	return m.print(os);
}

template <typename elemType>
Matrix<elemType>::Matrix(const Matrix &rhs)
{
	_rows = rhs._rows;
	_cols = rhs._cols;
	int mat_size = _rows * _cols;
	_matrix = new elemType[mat_size];
	for (int ix = 0; ix < mat_size; ++ix)
		_matrix[ix] = rhs._matrix[ix];
}

template <typename elemType>
Matrix<elemType>& Matrix<elemType>::operator=(const Matrix &rhs)
{
	if (this != &rhs)
    {
    	_rows = rhs._rows;
		_cols = rhs._cols;
		int mat_size = _rows * _cols;
		_matrix = new elemType[mat_size];
		for (int ix=0; ix < mat_size; ++ix)
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
Matrix<elemType> operator*(const Matrix<elemType> &m1, const Matrix<elemType> &m2)
{
	Matrix<elemType> result(m1.rows(), m2.cols());
	for (int ix = 0; ix < m1.rows(); ++ix)
		for (int jx = 0; jx < m2.cols(); ++jx)
		{	result(ix, jx)= elemType();
			for (int kx = 0; kx < m1.cols(); ++kx)
				result(ix, jx)+= m1(ix, kx) * m2(kx, jx);
			}
	return result;
}

template <typename elemType>
void Matrix<elemType>::operator+=(const Matrix &m)
{
	int matrix_size = rows() * cols();
	for (int ix = 0; ix < matrix_size; ++ix)
		(*(_matrix + ix)) += (*(m._matrix + ix)); 
}

template <typename elemType>
ostream& Matrix<elemType>::print(ostream &os) const
{
//	cout << "Printing!\n";
	int col = cols();
	int matrix_size = col * rows();
	for (int ix = 0; ix < matrix_size; ++ix)
	{
		if (ix % col == 0)
			os << endl;
		os << (*(_matrix + ix)) << ' ';
	}
	os << endl;
	return os;
}

int main()
{
	ofstream log("Exercise6_2.txt");
	if(! log)
	{ cerr << "Can't open log file!\n"; return -1; }
	
	Matrix<float> identity(4,4);
	log << "identity_4x4: " << identity << endl;
	identity.print(cout);
	//cout << identity(0,0) << endl;
	float ar[16] = { 1., 0., 0., 0.,
					 0., 1., 0., 0.,
					 0., 0., 1., 0.,
					 0., 0., 0., 1.};
	for (int i = 0, k = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			identity(i,j) = ar[k++];
	log << "identity_4x4 after set: " << identity << endl;
	
	Matrix<float> m1(identity);
	log << "m1: memberwise initialized: " << m1 << endl;
	
	Matrix<float> m2(8, 12);
	log << "m2: 8x12: " << m2 << endl;
	m2 = m1;
	log << "m2 after memberwise assigned to m: " << m2 << endl;
	
	float ar2[16] = {1.3, 43.4, 5.6,  1.2,
					 7.8, 5.8,  45.3, 7.6,
					 8.8, 4.5,  6.7,  5.5,
					 3.4, 7.8,  9.0,  0.0};
	
	Matrix<float> m3(4,4);
	for (int ix = 0, kx = 0; ix < 4; ++ix)
		for (int jx = 0; jx < 4; ++jx)
			m3(ix, jx) = ar2[kx++];
	
	log << "m3: assigned random values: " << m3 << endl;
	
	Matrix<float> m4 = m3 * identity;
	log << "m4 = m3 * identity_4x4: " << m4 << endl;
	Matrix<float> m5 = m3 + m4;
	log << "m5 = m3 + m4: " << m5 << endl;
	
	m3 += m4;
	log << "m3 += m4: " << m3 << endl;
	log.close();
}

