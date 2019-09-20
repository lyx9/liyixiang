#!/bin/bash
#-*- coding: utf-8 -*-

#set -x

ROOT_DIR=$(cd $(dirname $0); pwd);
Chrome_Download_Dir="/home/caros/Downloads"
RECORD_DOWNLOAD_DIR=/home/caros/Downloads/cores/$(date "+%m%d_%H%M");

[[ $# < 1 ]] && echo -e "bash download_record.sh job_id_file" &&  exit 1
job_id_file=$1
data_type='tgz'

#record_file=${job_id}.${data_type}
[[ -f ${job_id_file} ]] || echo "no job id file" || exit 1
while read job_id
do
  #record_file=${job_id}.recorder
  record_file=${job_id}.${data_type}
  goal_file=${RECORD_DOWNLOAD_DIR}/${record_file}
  #clean and build dir befor download
  [[ -d ${RECORD_DOWNLOAD_DIR} ]] || mkdir -p ${RECORD_DOWNLOAD_DIR}

  wget -O  ${goal_file} http://dreamland.baidu.com/api/getobj/${job_id}/12/${record_file}
  echo -e "\e[1;32mdownload: wget -O "${RECORD_DOWNLOAD_DIR}/${record_file}" http://dreamland.baidu.com/api/getobj/${job_id}/12/${record_file}"

  echo -e "\e[1;34mwaiting less than 1 mins \c"

  count=0
  while [[ ! -f ${goal_file} ]]
  do
      if (( $count > 60 )); then
          echo -e "\e[1;31m timeout!\e[0m"
	  break
      else
          sleep 1
          echo -e ". \c"
          ((count++))
      fi
  done
  echo -e "\e[1;32m\nfinish dowload ${goal_file}\e[0m"
done < ${job_id_file}
echo "download core files cmd is:";

while read job_id
do
   echo "wget http://dreamland.baidu.com/api/getobj/${job_id}/12/${record_file}"
done < ${job_id_file}

