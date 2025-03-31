# Libigl example

## Run CMake
```
mkdir build && cd build
cmake .. && cmake --build . -- -j  && ./example
```

## Debug
```
gdb ./example
break main
run
step  # Step into functions
next  # Step over functions
print V  # Print the contents of V
print F  # Print the contents of F
backtrace
exit
```

## Format

```
sudo apt install clang-format
clang-format -i your_file.cpp
```
