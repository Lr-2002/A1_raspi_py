//
// Created by lr-2002 on 23-7-28.
//
#include <vector>
#include <algorithm>
#include <iostream>
#include <boost/bind.hpp>
//using namespace std;

void print(std::ostream *os, int i)
{
    *os << i << '\n';
}

int main()
{
  std::vector<int> a = {1,2,3};
    std::for_each(a.begin(), a.end(), boost::bind(print, &std::cout, _1));
}