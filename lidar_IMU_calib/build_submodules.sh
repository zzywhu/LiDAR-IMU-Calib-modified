MYPWD=$(pwd)
set -x
set -e
NUM_CORES=`getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu || echo 1`
NUM_PARALLEL_BUILDS=$NUM_CORES

BUILD_PANGOLIN=thirdparty/build-pangolin

git submodule sync --recursive
git submodule update --init --recursive

rm -rf "$BUILD_PANGOLIN"

mkdir -p "$BUILD_PANGOLIN"
pushd "$BUILD_PANGOLIN"
cmake ../Pangolin
make -j$NUM_PARALLEL_BUILDS
popd

