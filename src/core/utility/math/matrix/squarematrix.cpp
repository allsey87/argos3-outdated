
#include "squarematrix.h"

namespace argos {
   template<>
   Real CSquareMatrix<1>::GetDeterminant() const {
      return m_pfValues[0];
   }

   template<>
   Real CSquareMatrix<2>::GetDeterminant() const {
      return m_pfValues[0] * m_pfValues[3] - m_pfValues[2] * m_pfValues[1];
   }

   template<>
   CSquareMatrix<2> CSquareMatrix<2>::GetCofactorMatrix() const {
      return CSquareMatrix<2>{m_pfValues[3], -m_pfValues[2], -m_pfValues[1], m_pfValues[0]};
   }
}
