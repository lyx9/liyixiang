#!/bin/bash
#set -x;
if [[  $# < 1 ]];then
    echo "a param to specify dir";
    exit 1;
fi

if [[ -d $1 ]];then
    dir=$1;
else
    echo "param $1 is not a effetive directory.";
    exit 1;
fi
files=(`ls $dir |grep tgz`);
echo "path $dir has ${#files[@]} core files"

for fl in ${files[@]};
do
    temp_path=${dir}/${fl%.tgz};
    goal_file=${dir}/${fl};
    if [[ -d ${temp_path} ]];then
        if [[ -n "$(ls ${temp_path}|grep core)" ]];then
            echo "${fl%.tgz} has comp core file $(ls ${temp_path}|grep core)"
            continue;
        fi
    else
        mkdir -p ${temp_path};
    fi
    tar -xz -C ${temp_path} -f ${goal_file};
    if [[ -n "$(ls ${temp_path}|grep core)" ]];then
        echo "comp core file $(ls ${temp_path}|grep core) success.";
    else
        echo "comp core file ${goal_file} failed.";
    fi
done
echo -e "result: jobid\tcore file"
for fl in ${files[@]};
do
    jobid=${fl%.tgz};
    core_file=$(ls ${dir}/${jobid} |grep core);
    echo -e "\t${jobid}\t${core_file}";
done
