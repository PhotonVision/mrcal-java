# Build requirements

See: http://mrcal.secretsauce.net/install.html

- Install cmake

```bash
sudo apt install cmake gcc g++ libmrcal-dev
cmake -B build . # You may need to add JAVA_HOME=/path/to/java
cmake --build build
```

For windows: `cmake -B build -S . -T ClangCl -A x64 -G "Visual Studio 17 2022"

For macOS:

```bash
cpan -i List::MoreUtils
cmake -B build -DOPENCV_ARCH=osxuniversal -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64"
cmake --build build
```

# Uncertainty testing

I configure with `cmake -B cmake_build`, build with `cmake --build cmake_build` and run with `time ./cmake_build/bin/mrcal_jni_test > /dev/null`

I benchmark with `perf record -F 99 -g ./cmake_build/bin/mrcal_jni_test`
