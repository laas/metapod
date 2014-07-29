// Copyright 2011, 2012, 2013, 2014
//
// Nuno Guedelha (LAAS, CNRS)
//
// This file is part of metapod.
// metapod is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// metapod is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.

// This test applies the permutation matrix Q computation on a test model with a reference configuration,
// then compares the result with the reference Q matrix.

// Common test tools
#include "common.hh"
#include <metapod/tools/qcalc.hh>
#include <metapod/tools/initnodeidconf.hh>

using namespace metapod;

typedef double LocalFloatType;

BOOST_AUTO_TEST_CASE (test_qcalc)
{
  typedef CURRENT_MODEL_ROBOT<LocalFloatType> CURRENT_MODEL_ROBOT_LFT;
  int confSize = CURRENT_MODEL_ROBOT_LFT::confVector::SizeAtCompileTime;

  // Apply the permutation matrix Q computation to the metapod multibody
  qcalc< CURRENT_MODEL_ROBOT_LFT >::run();
  std::cout << "Applied the permutation matrix Q computation to the metapod multibody.\n\n";

  // Prior tests on permutation vector: indexes are unique, i.e. each value occurs only once.
  CURRENT_MODEL_ROBOT_LFT::confVector occurenceCounter = CURRENT_MODEL_ROBOT_LFT::confVector::Zero();
  for(int i=0; i<confSize; i++)
  {
    occurenceCounter((int)CURRENT_MODEL_ROBOT_LFT::fdNodesFirst(i))++;
  }
  assert(occurenceCounter == CURRENT_MODEL_ROBOT_LFT::confVector::Constant(1));

  // test on permutation matrix Q. Check that Q has expected permutation matrix properties:
  // Q invertible and inv(Q) = Qt
  assert(CURRENT_MODEL_ROBOT_LFT::Q.toDenseMatrix() * CURRENT_MODEL_ROBOT_LFT::Qt.toDenseMatrix() == CURRENT_MODEL_ROBOT_LFT::MatrixNBDOFf::Identity());

  // set node IDs list vector nodeIdList as per node_id to q_idx mapping.
  CURRENT_MODEL_ROBOT_LFT::confVector nodeIdList;
  initNodeIdConf<CURRENT_MODEL_ROBOT_LFT>::run(nodeIdList);
  std::cout << "set node IDs list vector nodeIdList as per node_id to q_idx mapping ...\n\n";

  // compute re-ordered q_idx vector nodeIdList_sorted = Q * nodeIdList
  CURRENT_MODEL_ROBOT_LFT::confVector nodeIdList_sorted = CURRENT_MODEL_ROBOT_LFT::Q * nodeIdList;
  std::cout << "computed re-ordered node IDs list vector nodeIdList_sorted = Q * nodeIdList ...\n\n";

  // generate refence sorted node IDs list vector.
  CURRENT_MODEL_ROBOT_LFT::confVector nodeIdList_sorted_ref;
  initNodeIdConfReordRef<CURRENT_MODEL_ROBOT_LFT>::run(nodeIdList_sorted_ref);
  std::cout << "generated refence sorted node IDs list vector ...\n\n";

  // Print all results in a log file
  const char result_file_Q[] = "qcalc.log";
  std::ofstream log1(result_file_Q, std::ofstream::out);
  log1 << "permutation matrix Q\n" << CURRENT_MODEL_ROBOT_LFT::Q.toDenseMatrix() << std::endl;
  const char result_file_nodeIdList[] = "nodeIdList.log";
  std::ofstream log2(result_file_nodeIdList, std::ofstream::out);
  log2 << "initial vector nodeIdList\n" << nodeIdList << std::endl;
  const char result_file_nodeIdList_sorted[] = "nodeIdList_sorted.log";
  std::ofstream log3(result_file_nodeIdList_sorted, std::ofstream::out);
  log3 << "re-ordered vector nodeIdList\n" << nodeIdList_sorted << std::endl;
  const char result_file_nodeIdList_sorted_ref[] = "nodeIdList_sorted_ref.log";
  std::ofstream log4(result_file_nodeIdList_sorted_ref, std::ofstream::out);
  log4 << "re-ordered vector nodeIdList\n" << nodeIdList_sorted_ref << std::endl;
  log1.close();
  log2.close();
  log3.close();
  log4.close();
  std::cout << "Printed all results in a log file\n\n";

  // Compare results with reference file
  std::cout << "Compared re-ordered node IDs list vector with reference";
  compareLogs(result_file_nodeIdList_sorted, result_file_nodeIdList_sorted_ref, 1e-5);
}
