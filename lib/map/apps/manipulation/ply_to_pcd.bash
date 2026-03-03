#!/usr/bin/env bash

regex="[0-9]+_([a-z]+)_[0-9a-z]*"

for in_file in $1/*.ply; do
	sed -i -e 's/double/float/g' $in_file
	number=$(grep -Eo "[0-9]*" <<< "$(basename ${in_file})")
	number=$(printf '%02d' "$number")
	out_file="$1/cloud_$number.pcd"
	pcl_ply2pcd "$in_file" "$out_file"
done