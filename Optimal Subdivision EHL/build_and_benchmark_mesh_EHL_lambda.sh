#!/bin/bash

for lambda in 1 5 9; do
  rm -f dataset/result/Query/meshLambda${lambda}Comp0/${1}.csv
for file in dataset/merged-mesh/$1/* ;do
  echo Processing query for $file ..........
  array=(${file//// });
  directory_name=${array[2]};
  f="$(basename -- $file)";
  array2=(${f//-merged/ });
  map_name=${array2[0]};
  echo $map_name
  
  ./bin/build_mesh_based_hub_labelling $directory_name $map_name $lambda 0

./bin/testMeshEHL $directory_name $map_name $lambda 0

done
done