# The compiler for ./build.sh. Ubuntu 22.04 for glibc 2.35, the same as the board's.
FROM ubuntu:22.04
RUN apt-get update && apt-get install -y --no-install-recommends \
        g++ g++-arm-linux-gnueabihf make \
    && rm -rf /var/lib/apt/lists/*
