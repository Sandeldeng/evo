#! /bin/bash

# by leatherwang on 2020-06-29

echo
echo "==========="

if (($#<1)) ; then
    echo "usage: euroc_2_tum.sh <input euroc_pose_file> [split char]"
    exit 1
fi

split_char=" "
input_euroc_pose_file=$1
output_tum_pose_file="data.tum"

if (($#==2)) ; then
    split_char=$2
fi

echo "input file is: ${input_euroc_pose_file}"
echo "out file is: ${output_tum_pose_file}"
echo "split char is: \"${split_char}\""

echo "doing awk transform..."

# " "
awk  \
    'substr($1,1,1) != "#" {printf "%f %s %s %s %s %s %s %s\n",$1/1000000000,$2,$3,$4,$6,$7,$8,$5}' \
    ${input_euroc_pose_file} > ${output_tum_pose_file}

#
#awk -F${split_char} \
#    'substr($1,1,1) != "#" {printf "%f %s %s %s %s %s %s %s\n",$1/1000000000,$2,$3,$4,$6,$7,$8,$5}' \
#    ${input_euroc_pose_file} > ${output_tum_pose_file}

echo "Successfully!"
