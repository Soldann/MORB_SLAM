// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "sparse_block_matrix.h"
#include <iostream>


using namespace g2o;
using namespace Eigen;

typedef SparseBlockMatrix< MatrixXd >
SparseBlockMatrixX;

std::ostream& operator << (std::ostream& os, const SparseBlockMatrixX::SparseMatrixBlock& m) {
  for (int i=0; i<m.rows(); ++i){
    for (int j=0; j<m.cols(); ++j)
      std::cerr << m(i,j) << " ";
    std::cerr << std::endl;
  }
  return os;
}

int main (int argc, char** argv){
  int rcol[] = {3,6,8,12};
  int ccol[] = {2,4,13};
  std::cerr << "creation" << std::endl;
  SparseBlockMatrixX* M=new SparseBlockMatrixX(rcol, ccol, 4,3);

  std::cerr << "block access" << std::endl;

  SparseBlockMatrixX::SparseMatrixBlock* b=M->block(0,0, true);
  std::cerr << b->rows() << " " << b->cols() << std::endl;
  for (int i=0; i<b->rows(); ++i)
    for (int j=0; j<b->cols(); ++j){
      (*b)(i,j)=i*b->cols()+j;
    }


  std::cerr << "block access 2" << std::endl;
  b=M->block(0,2, true);
  std::cerr << b->rows() << " " << b->cols() << std::endl;
  for (int i=0; i<b->rows(); ++i)
    for (int j=0; j<b->cols(); ++j){
      (*b)(i,j)=i*b->cols()+j;
    }

  b=M->block(3,2, true);
  std::cerr << b->rows() << " " << b->cols() << std::endl;
  for (int i=0; i<b->rows(); ++i)
    for (int j=0; j<b->cols(); ++j){
      (*b)(i,j)=i*b->cols()+j;
    }

  std::cerr << *M << std::endl;

  std::cerr << "SUM" << std::endl;

  SparseBlockMatrixX* Ms=0;
  M->add(Ms);
  M->add(Ms);
  std::cerr << *Ms;
  
  SparseBlockMatrixX* Mt=0;
  M->transpose(Mt);
  std::cerr << *Mt << std::endl;

  SparseBlockMatrixX* Mp=0;
  M->multiply(Mp, Mt);
  std::cerr << *Mp << std::endl;
  
  int iperm[]={3,2,1,0};
  SparseBlockMatrixX* PMp=0;

  Mp->symmPermutation(PMp,iperm, false);
  std::cerr << *PMp << std::endl;

  PMp->clear(true);
  Mp->block(3,0)->fill(0.);
  Mp->symmPermutation(PMp,iperm, true);
  std::cerr << *PMp << std::endl;
  
  
  
}
