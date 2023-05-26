#!/bin/bash
res_name="tempRes/$1.txt"
rm -f $res_name
for file in dataset/merged-mesh/$1/* ;do
  echo Processing query for $file ..........
  array=(${file//// });
  directory_name=${array[2]};
  f="$(basename -- $file)";
  array2=(${f//-merged/ });
  map_name=${array2[0]};
  echo $map_name
  res=$(./bin/testPolyCon $directory_name $map_name);
  echo $res >> $res_name
done