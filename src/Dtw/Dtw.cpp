
#include "Vector.h"
#include "Util.h"
#include <algorithm>
#include "Dtw.h"


#include <iostream>
using namespace std;

CDtw::CDtw( void )
{
    
}

CDtw::~CDtw( void )
{
    
}

Error_t CDtw::init( int iNumRows, int iNumCols )
{
    m_iNumRows = iNumRows;
    m_iNumCols = iNumCols;
    costMatrix = new float*[m_iNumRows];
    for (int i = 0; i < m_iNumRows; i++)
    {
        costMatrix[i] = new float[iNumCols];
        for (int j = 0; j < m_iNumCols; j++)
        {
            costMatrix[i][j] = 0;
        }
    }
    directionMatrix = new int*[m_iNumRows];
    for (int i = 0; i < m_iNumRows; i++)
    {
        directionMatrix[i] = new int[iNumCols];
        for (int j = 0; j < m_iNumCols; j++)
        {
            directionMatrix[i][j] = 0;
        }
    }
    
    
    
    return kNoError;
}

Error_t CDtw::reset()
{
    return kNoError;
}

Error_t CDtw::process(float **ppfDistanceMatrix)
{
    //start from (0,0)
    costMatrix[0][0] = ppfDistanceMatrix[0][0];
    //the cost of elements in first column is always come from up direction
    for (int i = 1; i < m_iNumRows; i++) {
        costMatrix[i][0] = ppfDistanceMatrix[i][0] + costMatrix[i-1][0];
        directionMatrix[i][0] = kVert;
    }
    //the cost of elements in first row is always come from left direction
    for (int i = 1; i < m_iNumCols; i++) {
        costMatrix[0][i] = ppfDistanceMatrix[0][i] + costMatrix[0][i-1];
        directionMatrix[0][i] = kHoriz;
    }
    //calculate the cost of remaining elements
    for (int i = 1; i < m_iNumRows; i++)
    {
        for (int j = 1; j < m_iNumCols; j++)
        {
            //find the min of three direction
            float minCost = std::min(costMatrix[i-1][j-1], std::min(costMatrix[i-1][j], costMatrix[i][j-1]));
            costMatrix[i][j] = ppfDistanceMatrix[i][j] + minCost;
            if (minCost == costMatrix[i-1][j-1])
                directionMatrix[i][j] = kDiag;
            if (minCost == costMatrix[i-1][j])
                directionMatrix[i][j] = kVert;
            if (minCost == costMatrix[i][j-1])
                directionMatrix[i][j] = kHoriz;
        }
    }
    
    
    int iRow = m_iNumRows - 1;
    int iColumn = m_iNumCols - 1;
    
    //calculate pathLength
    while ( iRow >= 0 && iColumn >= 0)
    {
        int iCurrentDirection = directionMatrix[iRow][iColumn];
        if (iCurrentDirection == kVert)
        {
            iRow--;
        }
        
        if (iCurrentDirection == kHoriz)
        {
            iColumn--;
        }
        if (iCurrentDirection == kDiag){
            iRow--;
            iColumn--;
        }
        m_iPathLength++;
    }
    
    return kNoError;
}

int CDtw::getPathLength()
{
    return m_iPathLength;
}

float CDtw::getPathCost() const
{
    return costMatrix[m_iNumRows-1][m_iNumCols-1];
}

Error_t CDtw::getPath( int **ppiPathResult ) const
{
    int iRow = m_iNumRows-1;
    int iColumn = m_iNumCols-1;
    int iPathResultLength = m_iPathLength;
    while ( iRow >= 0 && iColumn >= 0)
    {
        ppiPathResult[0][iPathResultLength-1] = iRow;
        ppiPathResult[1][iPathResultLength-1] = iColumn;
        
        int iCurrentDirection = directionMatrix[iRow][iColumn];
        if (iCurrentDirection == kVert)
        {
            iRow--;
        }
        
        if (iCurrentDirection == kHoriz)
        {
            iColumn--;
        }
        if (iCurrentDirection == kDiag){
            iRow--;
            iColumn--;
        }
        iPathResultLength--;
    }
    
    
    return kNoError;
}

