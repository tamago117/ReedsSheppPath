#!/bin/sh
# get file path
cwd=`dirname "${0}"`
expr "${0}" : "/.*" > /dev/null || cwd=`(cd "${cwd}" && pwd)`

#g++ ${cwd}/main.cpp -lstdc++ -I/usr/include/python2.7 -lpython2.7 -std=c++14 && ${cwd}/a.out
g++ ${cwd}/main.cpp -lstdc++ -I/usr/include/python3.8 -Ilib -lpython3.8 -std=c++14 && ${cwd}/a.out

