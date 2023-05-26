#!/bin/bash

for file in dataset/merged-mesh/$1/* ;do
  echo Processing query for $file ..........
  array=(${file//// });
  directory_name=${array[2]};
  f="$(basename -- $file)";
  array2=(${f//-merged/ });
  map_name=${array2[0]};
  echo $map_name

./bin/testMeshEHL $directory_name $map_name

done