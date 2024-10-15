mkdir build
rm build/png_mechanic
cd src

g++ png_mechanic.cpp png_mechanic.h png.cpp png.h utility.cpp utility.h deflate.cpp deflate.h recovery.cpp recovery.h -o ../build/png_mechanic -g

cd ..
