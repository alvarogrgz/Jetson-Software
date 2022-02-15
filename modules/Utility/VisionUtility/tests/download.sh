#!/bin/bash

pathToFile=${1:1:-1}
pathToFile+="/bin/testfiles/"

# Download process
cd $pathToFile
wget -nc https://cernbox.cern.ch/index.php/s/CW2Z6HlMulJRLuK/download -O OrganizedPointCloud.pcd
wget -nc https://cernbox.cern.ch/index.php/s/WSQ7fHVU3gFLqz7/download -O UnorganizedPointCloud.pcd
