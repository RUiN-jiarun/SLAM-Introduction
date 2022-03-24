## Complile

1. install dependency for pangolin
```bash
sudo apt-get install libglew-dev
```

2. install pangolin
```bash
cd [path-to-pangolin]
mkdir build
cd build
cmake ..
make 
sudo make install 
sudo ldconfig
```

3. don't forget Eigen library which is in `home/usrname/eigen`