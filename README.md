# Build requirements

See: http://mrcal.secretsauce.net/install.html

- Install cmake

```
sudo apt install cmake gcc g++ libmrcal-dev mrbuild libsuitesparse-dev
cmake -B build . # You may need to add JAVA_HOME=/path/to/java
cmake --build build
```

For windows: `cmake -B build -S . -T ClangCl -A x64 -G "Visual Studio 17 2022" -DCMAKE_TOOLCHAIN_FILE="vcpkg/scripts/buildsystems/vcpkg.cmake"`

and cvpkg install suitesparse:x64-windows
