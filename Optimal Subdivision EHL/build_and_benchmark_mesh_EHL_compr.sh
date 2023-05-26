#!/bin/bash

for compr in 1, 3, 5, 6, 10; do
  rm -f dataset/result/Query/meshLambda${2}Comp${compr}/${1}.csv
for file in dataset/merged-mesh/$1/* ;do
  echo Processing query for $file ..........
  array=(${file//// });
  directory_name=${array[2]};
  f="$(basename -- $file)";
  array2=(${f//-merged/ });
  map_name=${array2[0]};
  echo $map_name
  
  ./bin/build_mesh_based_hub_labelling $directory_name $map_name $2 $compr
  ./bin/testMeshEHL $directory_name $map_name $2 $compr

done
done