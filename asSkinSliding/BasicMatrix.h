#pragma once
#ifndef _BasicMatrix_H
#define _BasicMatrix_H
//
// Copyright (C) 2003 NCCA-YXS 
// 
// File: BasicMatrix.h
//
// BasicMatrix Operation algorithm
//
// Author: Yang Xiaosong
//
#include <stdio.h>

class BasicMatrix
{
public:
	BasicMatrix(unsigned _numOfRows, unsigned _numOfCols);
	BasicMatrix(const BasicMatrix &_mat);
	~BasicMatrix();
	// Returns the number of rows
	unsigned int NumberOfRows()const;
	// Returns the numb colums
	unsigned int NumberOfColumns() const;
	void SetToIndentity();
	void SetToZero();
	const BasicMatrix Transpose();
	bool Inverse();
	BasicMatrix& operator=(const BasicMatrix &_right);
	const BasicMatrix operator+(const BasicMatrix &_right);
	const BasicMatrix operator*(const BasicMatrix &_right);
	const BasicMatrix operator*(const double _coef);
	const BasicMatrix operator-(const BasicMatrix &_right);
	void PrintBasicMatrix();
	double& operator() (unsigned _row, unsigned _col);
	double operator() (unsigned _row, unsigned _col) const;
	double operator() (unsigned _index) const; // physically the BasicMatrix stored in one dimension
	double& operator() (unsigned _index); // physically the BasicMatrix stored in one dimension
private:
	unsigned int m_numRows;
	unsigned int m_numColumns;
	double *m_data;
	double m_robot;
};

#endif
