#!/bin/bash
mkdir -p dataset/ebhl-mesh/$1
rm -f temp_output.txt
rm -f temp_output2.txt
rm -f temp_output3.txt
for file in dataset/merged-mesh/$1/* ;do
  echo Creating hub labels for $file ..........
  array=(${file//// });
  directory_name=${array[2]};
  f="$(basename -- $file)";
  array2=(${f//-merged/ });
  map_name=${array2[0]};

  ./bin/build_mesh_based_hub_labelling $directory_name $map_name $2 $3

  echo Finish preprocessing .......................................................................

done