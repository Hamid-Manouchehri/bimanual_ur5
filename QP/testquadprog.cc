/*
 File main.cc
 => renamed to test.cpp
 
 This file contains just an example on how to set-up the matrices for using with
 the solve_quadprog() function.
 
 The test problem is the following:
 
 Given:
 G =  4 -2   g0^T = [6 0]
     -2  4       
 
 Solve:
 min f(x) = 1/2 x G x + g0 x
 s.t.
   x_1 + x_2 = 3
   x_1 >= 0
   x_2 >= 0
   x_1 + x_2 >= 2
 
 The solution is x^T = [1 2] and f(x) = 12
 
 Author: Luca Di Gaspero
 DIEGM - University of Udine, Italy
 l.digaspero@uniud.it
 http://www.diegm.uniud.it/digaspero/
 
 LICENSE
 
 Copyright 2006-2009 Luca Di Gaspero
 
 This file is part of QuadProg++.
 
 QuadProg++ is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 QuadProg++ is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with QuadProg++; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <iostream>
#include <sstream>
#include <string>
#include "QuadProg++.hh"

int main (int argc, char *const argv[]) {
  Matrix<double> G, CE, CI;
  Vector<double> g0, ce0, ci0, x;
  int n, m = 0, p = 0;
  double sum = 0.0;
  char ch;
  
  n = 2;
  G.resize(n, n);
  {
    std::istringstream is("4, -2,"
			  "-2, 4 ");
    
    for (int i = 0; i < n; i++)	
      for (int j = 0; j < n; j++)
	is >> G[i][j] >> ch;
  }
	
  g0.resize(n);
  {
    std::istringstream is("6.0, 0.0 ");
    
    for (int i = 0; i < n; i++)
      is >> g0[i] >> ch;
  }

  m = 0;
  CE.resize(n, m);
  ce0.resize(m);

  p = 0;
  CI.resize(n, p);
  ci0.resize(p);


#if 0
  m = 1;
  CE.resize(n, m);
  {
    std::istringstream is("1.0, "
			  "1.0 ");
    
    for (int i = 0; i < n; i++)
      for (int j = 0; j < m; j++)
	is >> CE[i][j] >> ch;
  } 
  
  ce0.resize(m);
  {
    std::istringstream is("-3.0 ");
    
    for (int j = 0; j < m; j++)
      is >> ce0[j] >> ch;
  }
  p = 3;
  CI.resize(n, p);
  {
    std::istringstream is("1.0, 0.0, 1.0, "
			  "0.0, 1.0, 1.0 ");
    
    for (int i = 0; i < n; i++)
      for (int j = 0; j < p; j++)
	is >> CI[i][j] >> ch;
  }
  ci0.resize(p);
  {
    std::istringstream is("0.0, 0.0, -2.0 ");
    
    for (int j = 0; j < p; j++)
      is >> ci0[j] >> ch;
  }
#endif  
  x.resize(n);


  std::cout << "f: " << solve_quadprog(G, g0, CE, ce0, CI, ci0, x) << std::endl;
  std::cout << "x: " << x << std::endl;
  /*  for (int i = 0; i < n; i++)
      std::cout << x[i] << ' ';
      std::cout << std::endl;	 */
  
  /* FOR DOUBLE CHECKING COST since in the solve_quadprog routine the matrix G is modified */
	
  {
    std::istringstream is("4, -2,"
			  "-2, 4 ");
	
    for (int i = 0; i < n; i++)
      for (int j = 0; j < n; j++)
	is >> G[i][j] >> ch;
  }
	
  std::cout << "Double checking cost: ";
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++)
      sum += x[i] * G[i][j] * x[j];
  sum *= 0.5;	
  
  for (int i = 0; i < n; i++)
    sum += g0[i] * x[i];
  std::cout << sum << std::endl;
}