#!/bin/bash

getPIDs() {
    pgrep $1
}

getUnixSockets() {
    sockets=$(ls -l /proc/$1/fd/ | grep -o -P '(?<=socket\:\[).*(?=\])')
    for socket in ${sockets}; do
        details=$(ss -a --unix -p | grep ${socket})
        if [ ! -z "$details" ]; then
            echo ${details} | awk '{print $5}'
        fi
    done
}

getTcpSockets() {
    sockets=$(netstat -tlpn | grep LISTEN | grep $1 | grep 'tcp')
    echo ${sockets} | awk '{print $4}'
}

pointsArraySize=1
pointNames=("UvcCameraPoint")

echo '{['
for ((i=0; i<${pointsArraySize}; i++)); do   
    for pid in $(getPIDs ${pointNames[${i}]}); do
        echo -e '\t{'
        echo -e '\t\t"program_name": "'${pointNames[${i}]}'",'
        echo -e '\t\t"pid": '${pid}','
        echo -e '\t\t"unixdomainsockets": ['
            for filename in $(getUnixSockets ${pid}); do
                echo -e '\t\t\t"'${filename}'",'
            done
        echo -e '\t\t],'
        echo -e '\t\t"tcpsockets": ['
            for filename in $(getTcpSockets ${pid}); do
                echo -e '\t\t\t"'${filename}'",'
            done
        echo -e '\t\t]'
        echo -e '\t},'
    done
    echo '}'
done
echo ']}'