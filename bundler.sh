#!/bin/bash -e
[ -n "$1" ] || set -- a.out
mkdir -p ./lib/ #<copy the libraries here
#use ldd to resolve the libs and use `patchelf --print-needed to filter out
# "magic" libs kernel-interfacing libs such as linux-vdso.so, ld-linux-x86-65.so or libpthread
# which you probably should not relativize anyway
ldd "$1" |awk '$3 ~ /^ *\// {print $1,$3}' | sort |cut -d\  -f2 |
#copy the lib selection to ./lib
xargs -d '\n' -I{} cp --copy-contents {} ./lib 
#make the relative lib paths override the system lib path
patchelf --set-rpath "\$ORIGIN/lib" "$1"


##
# ldd raspicam_cv_test |awk '$3 ~ /^ *\// {print $1,$3}' | sort |cut -d\  -f2
# join \
#     <(ldd "$1" |awk '$3 ~ /^ *\// {print $1,$3}' |sort) \
#     <(patchelf --print-needed "$1" |sort) |cut -d\  -f2 |
# ldd raspicam_cv_test |awk '$3 ~ /^ *\// {print $1,$3}' | grep har
# patchelf --print-needed raspicam_cv_test