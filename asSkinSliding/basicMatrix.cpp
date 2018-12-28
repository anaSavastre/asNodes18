//
// Copyright (C) 2003 NCCA-YXS 
// 
// File: BasicMatrix.cpp
//
// BasicMatrix Operation algorithm
//
// Author: Yang Xiaosong
//
#include "BasicMatrix.h"
#include <maya/MGlobal.h>

bool BasicMatrix::Inverse()
{
	if (m_numRows != m_numColumns)
	{
		MGlobal::displayError("This BasicMatrix can not be inversed\n");
		return false;
	}

	if (m_numRows == 0) // include a special situation
		return true;

	double *result = new double[m_numRows*m_numColumns];
	if (result == NULL)
	{
		MGlobal::displayError("Allocate memory failed\n");
		return false;
	}

	unsigned int count, i, j, k, index;
	index = 0;
	for (i = 0; i < m_numRows; i++)
	{
		for (j = 0; j < m_numColumns; j++)
		{
			if (i == j)
				result[index] = 1.0;
			else
				result[index] = 0.0;
			index++;
		}
	}

	double tempt;
	count = 0;
	do {
		/* Find the first row whose diagnal element is not zero */
		j = count;
		while (j < m_numRows)
		{
			if (m_data[j*m_numColumns + j] == 0)
				j++;
			else
				break;
		}

		if (j >= m_numRows)
		{
			MGlobal::displayError("The BasicMatrix can not be inversed\n");
			return false;
		}

		/* the diagnol element of j row is not zero */
		if (j > count)
		{
			/* Exchange the count row and j row */
			for (k = 0; k < m_numColumns; k++)
			{
				tempt = m_data[j*m_numColumns + k];
				m_data[j*m_numColumns + k] = m_data[count*m_numColumns + k];
				m_data[count*m_numColumns + k] = tempt;
			}

			for (k = 0; k < m_numColumns; k++)
			{
				tempt = result[j*m_numColumns + k];
				result[j*m_numColumns + k] = result[count*m_numColumns + k];
				result[count*m_numColumns + k] = tempt;
			}
		}

		/** Use the [count] row as the base to make BasicMatrix[*][count] be zero **/

		/** Make BasicMatrix[count][count]=1 **/
		tempt = m_data[count*m_numColumns + count];
		for (k = count; k < m_numColumns; k++)
			m_data[count*m_numColumns + k] /= tempt;
		for (k = 0; k < m_numColumns; k++)
			result[count*m_numColumns + k] /= tempt;

		/** The rows except the 'count' row of BasicMatrix are processed to let rows[count]=0 **/

		for (i = 0; i < m_numRows; i++)
		{
			if (i != count)
			{
				tempt = m_data[i*m_numColumns + count];
				if (tempt != 0.0)
				{
					for (k = count; k < m_numColumns; k++)
						m_data[i*m_numColumns + k] -= tempt * m_data[count*m_numColumns + k];

					for (k = 0; k < m_numColumns; k++)
						result[i*m_numColumns + k] -= tempt * result[count*m_numColumns + k];
				}
			}
		}

		/** The BasicMatrix is processed row by row to be a identity/unit BasicMatrix **/
		count += 1;
	} while (count < m_numRows);

	// finally assign the result to this BasicMatrix
	delete[] m_data;
	m_data = result;
	return true; // BasicMatrix inverse operation succeed
}

double BasicMatrix::operator() (unsigned _index) const
{
	if (_index >= m_numColumns * m_numRows)
	{
		MGlobal::displayError("The BasicMatrix index is out of range\n");
		return 0.0;
	}
	return(m_data[_index]);
}

double& BasicMatrix::operator() (unsigned _index)
{
	if (_index >= m_numColumns * m_numRows)
	{
		MGlobal::displayError("The BasicMatrix index is out of range\n");
		return m_robot;
	}
	return(m_data[_index]);
}

double BasicMatrix::operator() (unsigned _row, unsigned _col) const
{
	if (_col >= m_numColumns || _row >= m_numRows)
	{
		MGlobal::displayError("The BasicMatrix index is out of range\n");
		return 0.0;
	}
	return(m_data[_row*m_numColumns + _col]);
}

double& BasicMatrix::operator() (unsigned _row, unsigned _col)
{
	if (_col >= m_numColumns || _row >= m_numRows)
	{
		MGlobal::displayError("The BasicMatrix index is out of range\n");
		return m_robot;
	}
	return(m_data[_row*m_numColumns + _col]);
}

const BasicMatrix BasicMatrix::operator*(const BasicMatrix &_right)
{
	unsigned int i, j, k, numColumn2 = _right.NumberOfColumns();
	BasicMatrix result(m_numRows, numColumn2);
	if (m_numColumns != _right.NumberOfRows())
	{
		MGlobal::displayError("BasicMatrix dimension can not take the mutiply operation\n");
		return result;
	}
	double sum, tmp;
	for (i = 0; i < m_numRows; i++)
	{
		for (j = 0; j < numColumn2; j++)
		{
			sum = 0.0;
			for (k = 0; k < m_numColumns; k++)
			{
				tmp = _right(k, j);
				sum += m_data[i*m_numColumns + k] * tmp;
			}
			result(i, j) = sum;
		}
	}
	return result;
}
const BasicMatrix BasicMatrix::operator*(const double _coef)
{
	unsigned int i, j, k;
	BasicMatrix result(m_numRows, m_numColumns);
	for (i=0; i<m_numRows; i++)
		for (j = 0; j < m_numColumns; j++)
		{
			result(i, j) = _coef * m_data[i*m_numColumns + j];
		}
	return result;
}
const BasicMatrix BasicMatrix::operator-(const BasicMatrix &_right)
{
	unsigned int i;
	BasicMatrix result(m_numRows, m_numColumns);
	if (m_numRows != _right.NumberOfRows() || m_numColumns != _right.NumberOfColumns())
	{
		MGlobal::displayError("Different BasicMatrix dimension, can not take the minus operation\n");
		return result;
	}
	unsigned int TotalNumber = m_numRows * m_numColumns;
	for (i = 0; i < TotalNumber; i++)
		result(i) = m_data[i] - _right(i);
	return result;
}

const BasicMatrix BasicMatrix::operator+(const BasicMatrix &_right)
{
	unsigned int i;
	BasicMatrix result(m_numRows, m_numColumns);
	if (m_numRows != _right.NumberOfRows() || m_numColumns != _right.NumberOfColumns())
	{
		MGlobal::displayError("Different BasicMatrix dimension, can not take the add operation\n");
		return result;
	}
	unsigned int TotalNumber = m_numRows * m_numColumns;
	for (i = 0; i < TotalNumber; i++)
		result(i) = m_data[i] + _right(i);
	return result;
}

BasicMatrix& BasicMatrix::operator=(const BasicMatrix &_right)
{
	unsigned int NewRowsNum, NewColumnsNum;
	NewRowsNum = _right.NumberOfRows();
	NewColumnsNum = _right.NumberOfColumns();
	if (m_numRows*m_numColumns != NewRowsNum * NewColumnsNum)
	{
		// memory size is different, free old, and allocate according to the new size
		double *datatmp = new double[NewRowsNum*NewColumnsNum];
		if (datatmp == NULL)
		{
			MGlobal::displayError("Memory allocation error\n");
			return *this;
		}
		delete[] m_data;
		m_data = datatmp;
	}
	m_numRows = NewRowsNum;
	m_numColumns = NewColumnsNum;
	// Copy the content
	unsigned int i, j, index = 0;
	for (i = 0; i < m_numRows; i++)
	{
		for (j = 0; j < m_numColumns; j++)
		{
			m_data[index] = _right(i, j);
			index++;
		}
	}
	return *this;
}

const BasicMatrix BasicMatrix::Transpose()
{
	BasicMatrix result(m_numColumns, m_numRows);
	unsigned int i, j;
	for (i = 0; i < m_numRows; i++)
	{
		for (j = 0; j < m_numColumns; j++)
		{
			result(j, i) = m_data[i*m_numColumns + j];
		}
	}
	return result;
}

void BasicMatrix::SetToZero()
{
	unsigned int i, num;

	num = m_numRows * m_numColumns;
	for (i = 0; i < num; i++)
		m_data[i] = 0.0;
}

void BasicMatrix::SetToIndentity()
{
	unsigned int i, j;
	unsigned int index = 0;

	for (i = 0; i < m_numRows; i++)
	{
		for (j = 0; j < m_numColumns; j++)
		{
			if (i == j)
				m_data[index] = 1.0;
			else
				m_data[index] = 0.0;
			index++;
		}
	}
}

unsigned int BasicMatrix::NumberOfRows() const
{
	return m_numRows;
}

unsigned int BasicMatrix::NumberOfColumns() const
{
	return m_numColumns;
}

BasicMatrix::BasicMatrix(unsigned _numOfRows, unsigned _numOfCols)
{
	unsigned int i, j;

	m_numRows = _numOfRows;
	m_numColumns = _numOfCols;
	m_robot = 0.0;
	m_data = new double[m_numRows*m_numColumns];
	if (m_data == NULL)
	{
		MGlobal::displayError("Memory allocation error\n");
		return;
	}

	// initialize the BasicMatrix to identity BasicMatrix
	unsigned int index = 0;
	for (i = 0; i < m_numRows; i++)
	{
		for (j = 0; j < m_numColumns; j++)
		{
			if (i == j)
				m_data[index] = 1.0;
			else
				m_data[index] = 0.0;
			index++;
		}
	}
}

BasicMatrix::BasicMatrix(const BasicMatrix &_mat)
{
	unsigned int i, j;

	m_numRows = _mat.NumberOfRows();
	m_numColumns = _mat.NumberOfColumns();
	m_robot = 0.0;

	// Allocate the memory for the BasicMatrix
	m_data = new double[m_numRows*m_numColumns];
	if (m_data == NULL)
	{
		MGlobal::displayError("Memory allocation error\n");
		return;
	}
	// Copy the content
	unsigned int index = 0;
	for (i = 0; i < m_numRows; i++)
	{
		for (j = 0; j < m_numColumns; j++)
		{
			m_data[index] = _mat(i, j);
			index++;
		}
	}
}

BasicMatrix::~BasicMatrix()
{
	if (m_data != NULL)
		delete[] m_data;
	else
		MGlobal::displayError("The BasicMatrix data can not be empty\n");
	m_numRows = m_numColumns = 0;
}
