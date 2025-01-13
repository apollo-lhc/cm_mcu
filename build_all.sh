#! /bin/sh
# wittich 8/2022
# this is similar to what is done in the CI for the build part.
# needs a recent CLANG installation

# exit on error
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
#trap 'echo "\"${last_command}\" command failed with exit code $?."' EXIT
trap 'echo "\"${last_command}\" command failed with exit code $?."' ERR


make clean
make -j REV1=1  > /dev/null
make clean
make -j REV2=1  > /dev/null
make clean
make -j REV3=1  > /dev/null
make COMPILER=clang clean 
make  -j REV1=1 COMPILER=clang > /dev/null
make COMPILER=clang clean
make -j REV2=1 COMPILER=clang > /dev/null
make COMPILER=clang clean
make -j REV3=1 COMPILER=clang > /dev/null
make format > /dev/null
echo "all build succeeded."

