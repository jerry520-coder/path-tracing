#!/bin/bash

read -p "创建文件(y/n): " choice
if [ "$choice" == "y" ] || [ "$choice" == "Y" ]; then

    # 创建文件夹并自动命名
    folder_name=$(date +"%Y-%m-%d_%H-%M-%S")
    mkdir "$folder_name"

    mv *.txt "$folder_name"

    echo "Created folder: $folder_name"
else
    scp root@192.168.8.194:/orbbec/*.txt .
    echo "File not moved."
fi

echo "完成"
