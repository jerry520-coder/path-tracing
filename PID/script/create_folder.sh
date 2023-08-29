#!/bin/bash

#scp root@192.168.8.194:/orbbec/*.txt  .
# 创建文件夹并自动命名
folder_name=$(date +"%Y-%m-%d_%H-%M-%S")

mkdir "$folder_name"

mv *.txt "$folder_name"

echo "Created folder: $folder_name"
