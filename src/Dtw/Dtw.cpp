
#include "Vector.h"
#include "Util.h"
#include <algorithm>
#include "Dtw.h"

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
    costMatrix[0][0] = ppfDistanceMatrix[0][0];
    for (int i = 1; i < m_iNumRows; i++)
    {
        for (int j = 1; j < m_iNumCols; j++)
        {
            float nextMinCost = std::min(costMatrix[i-1][j-1], std::min(costMatrix[i-1][j], costMatrix[i][j-1]));
            costMatrix[i][j] = nextMinCost + ppfDistanceMatrix[i][j];
            if (nextMinCost == costMatrix[i-1][j-1])
                directionMatrix[i][j] = kDiag;
            if (nextMinCost == costMatrix[i-1][j])
                directionMatrix[i][j] = kHoriz;
            if (nextMinCost == costMatrix[i][j-1])
                directionMatrix[i][j] = kVert;
        }
    }
    
    int iCurrentX = m_iNumRows - 1;
    int iCurrentY = m_iNumCols - 1;
    
    while ( iCurrentX >= 0 && iCurrentY >= 0)
    {
        int iCurrentDirection = directionMatrix[iCurrentX][iCurrentY];
        if (iCurrentDirection == kVert)
        {
            iCurrentY--;
            m_fPathCost += costMatrix[iCurrentX][iCurrentY];
        }
        
        if (iCurrentDirection == kHoriz)
        {
            iCurrentX--;
            m_fPathCost += costMatrix[iCurrentX][iCurrentY];
        }
        if (iCurrentDirection == kDiag){
            iCurrentY--;
            iCurrentX--;
            m_fPathCost += costMatrix[iCurrentX][iCurrentY];
            
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
    return m_fPathCost;
}

Error_t CDtw::getPath( int **ppiPathResult ) const
{
    return kNoError;
}

