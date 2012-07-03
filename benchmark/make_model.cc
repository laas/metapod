#include "make_model.hh"
#include <fstream>

int main()
{
  metapod::benchmark::generate_model("model_3_dof", 1);
  metapod::benchmark::generate_model("model_7_dof", 2);
  metapod::benchmark::generate_model("model_15_dof", 3);
  metapod::benchmark::generate_model("model_31_dof", 4);
  metapod::benchmark::generate_model("model_63_dof", 5);
}
