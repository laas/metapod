#include <iostream>
#include <ginac/ginac.h>
#include <stdlib.h>

using namespace std;
using namespace GiNaC;

int main()
{
  symbol x("x"), y("y"),z("z");
  ex cx = cos(x); ex sx = sin(x);
  ex cy = cos(y); ex sy = sin(y);
  ex cz = cos(z); ex sz = sin(z);

  matrix Rx(3,3,lst(numeric(1),numeric(0),numeric(0),numeric(0),cx,sx,0,-sx,cx));
  matrix Ry(3,3,lst(cy,numeric(0),-sy,numeric(0),numeric(1),numeric(0),sy,numeric(0),cy));
  matrix Rz(3,3,lst(cz,sz,numeric(0),-sz,cz,numeric(0),numeric(0),numeric(0),numeric(1)));

  const matrix Refs[3] = {Rx,Ry,Rz};
  std::vector<matrix> m;
  m.resize(9);

  char Axes[3] = {'X','Y','Z'};
    for(unsigned int j=0;j<3;j++)
      for(unsigned int k=0;k<3;k++)
        {
          unsigned int idx= j*3+k;
          cout << "idx: "<< idx<< std::endl;
          m[idx] = Refs[j].mul(Refs[k]);
          cout << Axes[j] << " - " << Axes[k] << std::endl;
          cout << m[idx].evalm() << endl;
        }
  
  // 
  for(unsigned int i=0;i<9;i++)
    for(unsigned int j=i+1;j<9;j++)
      {
        if (m[i].evalm().is_equal(m[j].evalm()))
          {
            cout << i << " and " << j << " are identical !" << std::endl;
            exit(1);
          }
        else
          {
            //cout << i << " and " << j << " are different !" << std::endl;
          }
      }
  
  return 0;
}
